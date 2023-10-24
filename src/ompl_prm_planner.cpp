#include "ompl_ros_connector/ompl_prm_planner.h"

PLUGINLIB_EXPORT_CLASS(ompl_prm_planner::OmplPRMPlanner, nav_core::BaseGlobalPlanner)

namespace ompl_prm_planner
{
    OmplPRMPlanner::OmplPRMPlanner()
    {
        // Constructor implementation
    }

    void OmplPRMPlanner::reconfigureCallback(ompl_prm_planner::PlannerParamsConfig &config, uint32_t level)
    {
        _occupancy_threshold = config.occupancy_threshold;

        // Continue for other params...
    }

    void OmplPRMPlanner::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &costmap_msg)
    {
        _costmap = costmap_msg;
    }

    void OmplPRMPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = _frame_id;
        for (const auto& pose : plan)
        {
            path_msg.poses.push_back(pose);
        }
        _path_pub.publish(path_msg);
    }

    bool OmplPRMPlanner::worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my) const
    {
        double origin_x, origin_y, resolution;
        origin_x = _costmap->info.origin.position.x;
        origin_y = _costmap->info.origin.position.y;
        resolution = _costmap->info.resolution;
        unsigned int size_x, size_y;
        size_x = _costmap->info.width;
        size_y = _costmap->info.height;

        if (wx < origin_x || wy < origin_y)
            return false;

        mx = (int)((wx - origin_x) / resolution);
        my = (int)((wy - origin_y) / resolution);

        if (mx < size_x && my < size_y)
            return true;

        return false;
    }
    
    unsigned char OmplPRMPlanner::getCost(double wx, double wy) const
    {
        unsigned int mx, my;
        bool inBounds = worldToMap(wx, wy, mx, my);

        unsigned int width = _costmap->info.width;

        return inBounds ? _costmap->data[my * width + mx] : costmap_2d::NO_INFORMATION;
    }

    bool OmplPRMPlanner::isStateValid(const ob::State *state, const ob::SpaceInformation *si)
    {
        // OMPL state to ROS position
        const auto *se2state = state->as<ob::SE2StateSpace::StateType>();
        double x = se2state->getX(), y = se2state->getY();

        unsigned char cost = getCost(x, y);
        if (cost < _occupancy_threshold)
            return true;

        return false;
    }

    void OmplPRMPlanner::initOMPL(costmap_2d::Costmap2DROS *costmap_ros)
    {
        _space = std::make_shared<ob::SE2StateSpace>();

        ob::RealVectorBounds bounds(2);
        bounds.setLow(0, costmap_ros->getCostmap()->getOriginX());
        bounds.setHigh(0, costmap_ros->getCostmap()->getSizeInMetersX());
        bounds.setLow(1, costmap_ros->getCostmap()->getOriginY());
        bounds.setHigh(1, costmap_ros->getCostmap()->getSizeInMetersY());

        _space->as<ob::SE2StateSpace>()->setBounds(bounds);

        auto space_information = std::make_shared<ob::SpaceInformation>(_space);

        // set validity checker
        space_information->setStateValidityChecker(
            [this, space_information](const ob::State *state)
            { return isStateValid(state, space_information.get()); });
        // set sampler algorithm
        space_information->setValidStateSamplerAllocator([](const ob::SpaceInformation *si) -> std::shared_ptr<ob::ValidStateSampler>
                                                         { return std::make_shared<ob::BridgeTestValidStateSampler>(si); });

        // set planner type
        _planner = std::make_shared<og::PRM>(space_information);
        // _planner = std::make_shared<og::RRT>(space_information);

        _simple_setup = std::make_unique<ompl::geometric::SimpleSetup>(space_information);
        _simple_setup->setPlanner(_planner);

        // disable ompl info logs
        ompl::msg::setLogLevel(ompl::msg::LOG_WARN);
    }

    void OmplPRMPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!_initialized)
        {
            _frame_id = costmap_ros->getGlobalFrameID();

            ros::NodeHandle private_nh("~/" + name);
            
            // costmap subscriber
            _costmapSub = private_nh.subscribe("/move_base/global_costmap/costmap", 1, &OmplPRMPlanner::costmapCallback, this);

            // path publisher
            _path_pub = private_nh.advertise<nav_msgs::Path>("global_plan", 1, true);

            // dynamic reconfigure
            dynamic_reconfigure::Server<ompl_prm_planner::PlannerParamsConfig>::CallbackType cb;
            cb = boost::bind(&OmplPRMPlanner::reconfigureCallback, this, _1, _2);
            dsrv_ = new dynamic_reconfigure::Server<ompl_prm_planner::PlannerParamsConfig>(private_nh);
            dsrv_->setCallback(cb);

            // init ompl setup
            initOMPL(costmap_ros);

            _initialized = true;
            ROS_INFO("ompl_prm_planner::OmplPRMPlanner plugin initialized.");
        }
        else
            ROS_WARN("ompl_prm_planner::OmplPRMPlanner plugin has already been initialized, doing nothing.");
    }

    bool OmplPRMPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                              const geometry_msgs::PoseStamped &goal,
                              std::vector<geometry_msgs::PoseStamped> &plan)
    {
        if (!_initialized)
        {
            ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
            return false;
        }

        plan.clear(); // clear old plans

        // Create start and goal states for OMPL
        ob::ScopedState<> start_state(_space);
        start_state[0] = start.pose.position.x;
        start_state[1] = start.pose.position.y;
        start_state[2] = tf::getYaw(start.pose.orientation);

        ob::ScopedState<> goal_state(_space);
        goal_state[0] = goal.pose.position.x;
        goal_state[1] = goal.pose.position.y;
        goal_state[2] = tf::getYaw(goal.pose.orientation);

        _simple_setup->setStartAndGoalStates(start_state, goal_state);

        // set planning time
        ob::PlannerStatus status = _simple_setup->solve(3.0);

        // solution found
        if (status == ob::PlannerStatus::EXACT_SOLUTION)
        {
            og::PathGeometric path = _simple_setup->getSolutionPath();
            // path.interpolate(_costmap->info.resolution); // Fill the path with intermediate values according to given resolution

            // OMPL path to ROS plan
            for (std::size_t path_idx = 0; path_idx < path.getStateCount(); path_idx++)
            {
                const auto *state = path.getState(path_idx)->as<ob::SE2StateSpace::StateType>();
                geometry_msgs::PoseStamped new_pose;
                new_pose.header.stamp = ros::Time::now();
                new_pose.header.frame_id = _frame_id;
                new_pose.pose.position.x = state->getX();
                new_pose.pose.position.y = state->getY();
                new_pose.pose.orientation = tf::createQuaternionMsgFromYaw(state->getYaw());

                plan.push_back(new_pose);
            }

            // publish the path for visualization & debugging
            publishPlan(plan);

            return true;
        }
        else
        {
            ROS_WARN("No solution could be found for the given start and goal");
            return false;
        }
    }
} // namespace
