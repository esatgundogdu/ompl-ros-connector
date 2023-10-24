cmake_minimum_required(VERSION 3.0.2)
project(ompl_ros_connector)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_core
  navfn
  nav_msgs
  pluginlib
  roscpp
  tf
  std_msgs
  dynamic_reconfigure
)

list(APPEND CMAKE_PREFIX_PATH "/home/saslab/omplapp/ompl/")
find_package(ompl REQUIRED)
include_directories(${OMPL_INCLUDE_DIRS})
link_directories(${OMPL_LIBRARY_DIRS})

generate_dynamic_reconfigure_options(
    cfg/PlannerParams.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ompl_global_planner_lib
  CATKIN_DEPENDS
    costmap_2d
    geometry_msgs
    nav_core
    navfn
    nav_msgs
    pluginlib
    roscpp
    tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(ompl_prm_planner
  src/ompl_prm_planner.cpp
)

target_link_libraries(ompl_prm_planner ${catkin_LIBRARIES} ${OMPL_LIBRARIES})

install(TARGETS ompl_prm_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# install(FILES ompl_gp_plugin.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  