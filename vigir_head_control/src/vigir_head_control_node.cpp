

#include <ros/ros.h>
#include <vigir_head_control/vigir_head_control.h>




int main(int argc, char **argv)
{
  ros::init(argc, argv, "head_control_node");

  ROS_INFO("Starting Head Control Node");
  head_control::HeadControl head_controller;
  ros::Rate loop_rate(30);
  while (ros::ok()) {
    head_controller.updateHeadPosition();
    ros::spinOnce();
  }
  ros::spin();
  exit(0);
}

