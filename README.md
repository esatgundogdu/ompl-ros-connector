# ompl-ros-connector
This repo includes a move_base plugin to connect ompl planners with ROS.


## TODO
- DWA planner does not take all of the global plan, it gets cropped global plan. Rviz uses global planner that DWA took. That's why full global plan does not seem sometimes on Rviz.
- DWA planner takes cropped global plan for optimization purpose. To get full path visible on Rviz, we need to publish our global plan seperately.
