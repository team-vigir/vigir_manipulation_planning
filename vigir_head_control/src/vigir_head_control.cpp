#include <ros/ros.h>
#include <vigir_head_control/vigir_head_control.h>

namespace head_control{
    HeadControl::HeadControl()
    {
        ros::NodeHandle nh_;
        head_control_sub = nh_.subscribe("head_control_mode", 10, &HeadControl::HeadControlCb, this);
        joint_trajectory_pub = nh_.advertise<trajectory_msgs::JointTrajectory>("/thor_mang/head_traj_controller/command", 10, false);
    }

    HeadControl::~HeadControl()
    {}


    void HeadControl::HeadControlCb(const vigir_planning_msgs::HeadControlCommand &command){
     ///flor/l_arm_current_pose
     /// //trajectory_msgs/JointTrajectory
        if(command.motion_type == vigir_planning_msgs::HeadControlCommand::USE_PROVIDED_JOINTS){
            if(command.provided_joints.size() < 2){
                ROS_WARN("Head cannot be moved because values are missing. (%lu provided)", command.provided_joints.size());
            }else{
                double pan = command.provided_joints[0];
                double tilt = command.provided_joints[1];

                trajectory_msgs::JointTrajectory jointTrajectory;

                std::vector<std::string> joints;
                joints.push_back("head_pan");
                joints.push_back("head_tilt");
                jointTrajectory.joint_names = joints;

                std::vector <double> positions;
                positions.push_back(pan);
                positions.push_back(tilt);
                trajectory_msgs::JointTrajectoryPoint targetPoint;
                targetPoint.positions = positions;
                targetPoint.time_from_start = ros::Duration(0.5);

                joint_trajectory_pub.publish(jointTrajectory);
            }
        } else if (command.motion_type == vigir_planning_msgs::HeadControlCommand::TRACK_LEFT_HAND){
            // TODO: implement
        } else if (command.motion_type == vigir_planning_msgs::HeadControlCommand::TRACK_RIGHT_HAND){
            // TODO: implement
        }
    }
}
