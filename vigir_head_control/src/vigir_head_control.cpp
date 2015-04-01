#include <ros/ros.h>
#include <vigir_head_control/vigir_head_control.h>


namespace head_control{
    HeadControl::HeadControl()
    {
        ros::NodeHandle nh_("");
        head_control_sub = nh_.subscribe("head_control_mode", 10, &HeadControl::HeadControlCb, this);
        joint_trajectory_pub = nh_.advertise<trajectory_msgs::JointTrajectory>("/thor_mang/head_traj_controller/command", 10, false);
    }

    HeadControl::~HeadControl()
    {}


    void HeadControl::HeadControlCb(const vigir_planning_msgs::HeadControlCommand &command){
     ///flor/l_arm_current_pose
     /// //trajectory_msgs/JointTrajectory
    }
}
