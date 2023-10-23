#include <nav_core/base_global_planner.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

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

class PRMPlanner : public nav_core::BaseGlobalPlanner {
public:
  PRMPlanner();
  PRMPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override;

  void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& costmap_msg);

  private:
    bool isStateValid(const ob::State *state, const ob::SpaceInformation *si);
    bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const;
    unsigned char getCost(double wx, double wy) const;

  private:
    bool _initialized = false;
    std::string _frame_id;
    const unsigned int _occupancy_threshold = 60; // 0-100 interval

    ros::Subscriber _costmapSub;
    nav_msgs::OccupancyGrid::ConstPtr _costmap;

    ob::StateSpacePtr _space;
    ob::PlannerPtr _planner;
    std::unique_ptr<og::SimpleSetup> _simple_setup;
};
}