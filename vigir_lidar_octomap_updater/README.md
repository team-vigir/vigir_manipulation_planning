This package provides similar functionality to the [point_cloud_octomap_updater](https://github.com/ros-planning/moveit_ros/tree/jade-devel/perception/pointcloud_octomap_updater)
in that it updates the octomap used inside move_group and used for collision
avoidance. It provides additional functionality as follows, however:
* Subscribes to [LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html) messages
* Processes scans using [laser_filters](http://wiki.ros.org/laser_filters) and performs a high fidelity projection provided with [laser_geometry](http://wiki.ros.org/laser_geometry)
* Publishes [FilteredLocalizedLaserScan](https://github.com/team-vigir/vigir_perception_msgs/blob/master/msg/FilteredLocalizedLaserScan.msg)
messages that provide information about non-filtered and self filtered scan points as well as the LIDAR's pose. This allows efficient compression, transmission over a restricted link and reconstruction. 

Examples of use can be found in [vigir_atlas_moveit_config](https://github.com/team-vigir/vigir_atlas_planning/blob/master/vigir_atlas_moveit_config/launch/atlas_moveit_sensor_manager.launch.xml) and [thor_mang_tud_moveit_config](https://github.com/thor-mang/thor_mang_tud_moveit_config/blob/master/launch/thor_mang_robot_moveit_sensor_manager.launch.xml)
