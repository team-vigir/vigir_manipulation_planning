# vigir_manipulation_planning
This repository contains ROS packages related to manipulation motion planning. See also talks from ROScon 2015 ([pdf](http://roscon.ros.org/2015/presentations/ViGIR_open_source_software_drc_pos_mortem.pdf), [video](https://vimeo.com/142149253)) and the 1st online [MoveIt! user meeting](https://youtu.be/D5rs9uLrveg?t=1h13s).
Some pointers to important packages follow:

* [vigir_lidar_octomap_updater](https://github.com/team-vigir/vigir_manipulation_planning/tree/master/vigir_lidar_octomap_updater) is a sensor updater plugin that updates the planning scene octomap from LIDAR data while simultaneously providing a low bandwidth representation of scan data.
* [vigir_move_group](https://github.com/team-vigir/vigir_manipulation_planning/tree/master/vigir_move_group) contains move_group plugins used for manipulation with humanoid robots.
* [vigir_ocs_robot_model](https://github.com/team-vigir/vigir_manipulation_planning/tree/master/vigir_ocs_robot_model) provides a "ghost robot" model that can be used in a UI for kinematic pre-planning of actions.
