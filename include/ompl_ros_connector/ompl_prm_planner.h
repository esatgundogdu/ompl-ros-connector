#include <nav_core/base_global_planner.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <pluginlib/class_list_macros.h>

#include <dynamic_reconfigure/server.h>
#include <ompl_ros_connector/PlannerParamsConfig.h>

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/samplers/BridgeTestValidStateSampler.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include "ompl/util/Console.h"

#include <vector>
#include <memory>

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;

namespace ompl_prm_planner {

class OmplPRMPlanner : public nav_core::BaseGlobalPlanner {
public:
  OmplPRMPlanner();
  OmplPRMPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override;

  void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& costmap_msg);
  void reconfigureCallback(ompl_prm_planner::PlannerParamsConfig &config, uint32_t level);

  private:
    /**
     * @brief Initializes OMPL setup.
    */
    void initOMPL(costmap_2d::Costmap2DROS* costmap_ros);
    /**
     * @brief Checks given state and returns true if it is valid. Checks state cost, if it is smaller than occupancy_threshold param it is valid.
    */
    bool isStateValid(const ob::State *state, const ob::SpaceInformation *si);
    /*
     * @brief fills mx, my values. Returns true if coordinates are in legal bounds, false otherwise.
     */
    bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const;
    /*
     * @brief returns costmap value if given coordinates are in legal border. Otherwise, returns NO_INFORMATION
     */
    unsigned char getCost(double wx, double wy) const;

  private:
    bool _initialized = false;
    std::string _frame_id;
    unsigned int _occupancy_threshold; // 0-100 interval

    // costmap subscription
    ros::Subscriber _costmapSub;
    nav_msgs::OccupancyGrid::ConstPtr _costmap;

    // dynamic reconfigure
    dynamic_reconfigure::Server<ompl_prm_planner::PlannerParamsConfig>* dsrv_;

    // ompl specific variables
    ob::StateSpacePtr _space;
    ob::PlannerPtr _planner;
    std::unique_ptr<og::SimpleSetup> _simple_setup;
};
}