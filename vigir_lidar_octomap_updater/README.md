This package provides similar functionality to the [point_cloud_octomap_updater](https://github.com/ros-planning/moveit_ros/tree/jade-devel/perception/pointcloud_octomap_updater)
in that it updates the octomap used inside move_group and used for collision
avoidance. It provides additional functionality as follows, however:
* Subscribes to LaserScan messages
* Processes scans using [laser_filters](http://wiki.ros.org/laser_filters) and high fidelity projection provided by [laser_geometry]](http://wiki.ros.org/laser_geometry)
* Publishes [FilteredLocalizedLaserScan](https://github.com/team-vigir/vigir_perception_msgs/blob/master/msg/FilteredLocalizedLaserScan.msg)
messages that provide information about non-filtered and self filtered scan points as wellas the LIDAR's pose. This allows efficient compression, transmission over a restricted link and reconstruction.