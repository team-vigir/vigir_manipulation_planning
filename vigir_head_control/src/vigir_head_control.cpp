#include <ros/ros.h>
#include <vigir_head_control/vigir_head_control.h>

namespace head_control{
    HeadControl::HeadControl()
    {
        tracking_mode = head_tracking_mode::NONE;

        ROS_DEBUG("Creating Head Controler");
        ros::NodeHandle nh_("");

        head_control_sub = nh_.subscribe("/thor_mang/head_control_mode", 10, &HeadControl::HeadControlCb, this);
        left_hand_sub = nh_.subscribe("/flor/l_arm_current_pose", 1, &HeadControl::trackLeftHandCb, this);
        right_hand_sub = nh_.subscribe("/flor/r_arm_current_pose", 1, &HeadControl::trackRightHandCb, this);

        joint_trajectory_pub = nh_.advertise<trajectory_msgs::JointTrajectory>("/thor_mang/head_traj_controller/command", 10, false);
    }

    HeadControl::~HeadControl()
    {}


    void HeadControl::HeadControlCb(const vigir_planning_msgs::HeadControlCommand &command){
     ///flor/l_arm_current_pose
     /// //trajectory_msgs/JointTrajectory

        ROS_DEBUG("Setting Head Control Mode to %u", command.motion_type);

        if(command.motion_type == vigir_planning_msgs::HeadControlCommand::USE_PROVIDED_JOINTS){
            // TODO: We might need a mutex here for changing the node
            tracking_mode = head_tracking_mode::NONE;

            if(command.provided_joints.size() < 2){
                ROS_WARN("Head cannot be moved because values are missing. (%lu provided)", command.provided_joints.size());
            }else{
                double pan = command.provided_joints[0];
                double tilt = command.provided_joints[1];
                ROS_DEBUG("Moving Head to provided joint positions: (pan, tilt) = (%f, %f)", pan, tilt);
                setHeadJointPosition(pan, tilt);
            }

        } else if (command.motion_type == vigir_planning_msgs::HeadControlCommand::TRACK_LEFT_HAND){
            tracking_mode = head_tracking_mode::LEFT_HAND_TRACKING;
        } else if (command.motion_type == vigir_planning_msgs::HeadControlCommand::TRACK_RIGHT_HAND){
            tracking_mode = head_tracking_mode::RIGHT_HAND_TRACKING;
            std::vector<double> joints = computeJointsRightHandTracking();
            setHeadJointPosition(joints[0], joints[1]);
        }else{
            ROS_WARN("Received invalid Head Control Mode: %u", command.motion_type);
        }
    }

    void HeadControl::setHeadJointPosition(const double pan, const double tilt){
        trajectory_msgs::JointTrajectory jointTrajectory;

        // Create joint names list
        std::vector<std::string> joints;
        joints.push_back("head_pan");
        joints.push_back("head_tilt");
        jointTrajectory.joint_names = joints;

        // create point list
        std::vector <trajectory_msgs::JointTrajectoryPoint> points;

        trajectory_msgs::JointTrajectoryPoint targetPoint;
        std::vector <double> positions;
        positions.push_back(pan);
        positions.push_back(tilt);
        targetPoint.positions = positions;
        targetPoint.time_from_start = ros::Duration(0.5);

        points.push_back(targetPoint);
        jointTrajectory.points = points;

        ROS_DEBUG("Publishing Message for Head Joints (pan, tilt) = (%f, %f)", pan, tilt);

        joint_trajectory_pub.publish(jointTrajectory);
    }

    std::vector<double> HeadControl::computeJointsLeftHandTracking(){
        //TODO:: implement
        double pan = 2.0;
        double tilt = 1.0;
        std::vector<double> joints;
        joints.push_back(pan);
        joints.push_back(tilt);

        return joints;
    }

    std::vector<double> HeadControl::computeJointsRightHandTracking(){
        //TODO:: implement
        double pan = 1.0;
        double tilt = 1.0;
        std::vector<double> joints;
        joints.push_back(pan);
        joints.push_back(tilt);

        return joints;
    }

    void HeadControl::trackLeftHandCb(const geometry_msgs::PoseStamped &pose){
        if(tracking_mode == head_tracking_mode::LEFT_HAND_TRACKING){
            std::vector<double> joints = computeJointsLeftHandTracking();
            setHeadJointPosition(joints[0], joints[1]);
        }
    }

    void HeadControl::trackRightHandCb(const geometry_msgs::PoseStamped &pose){
        if(tracking_mode == head_tracking_mode::RIGHT_HAND_TRACKING){
            std::vector<double> joints = computeJointsRightHandTracking();
            setHeadJointPosition(joints[0], joints[1]);
        }
    }
}
