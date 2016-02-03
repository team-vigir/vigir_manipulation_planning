# vigir_manipulation_planning
This repository contains ROS packages related to manipulation motion planning. Some pointers to important packages follow:

* [vigir_lidar_octomap_updater](https://github.com/team-vigir/vigir_manipulation_planning/tree/master/vigir_lidar_octomap_updater) is a sensor updater plugin that updates the planning scene octomap from LIDAR data while simultaneously providing a low bandwidth representation of scan data.
* [vigir_move_group](https://github.com/team-vigir/vigir_manipulation_planning/tree/master/vigir_move_group) contains move_group plugins used for manipulation with humanoid robots.
* [vigir_ocs_robot_model](https://github.com/team-vigir/vigir_manipulation_planning/tree/master/vigir_ocs_robot_model) contains a node for translating standard ROS control commands (joy and cmd_vel) to commands for the Arduino.
