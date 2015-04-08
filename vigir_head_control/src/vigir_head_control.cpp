#include <ros/ros.h>
#include <vigir_head_control/vigir_head_control.h>

namespace head_control{
    HeadControl::HeadControl()
    {   ROS_INFO ("started!!!!!!!!!!!!!!");
        std::cout << "started node !!!!!!!" << std::endl;
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
            //std::vector<double> joints = computeJointsRightHandTracking();
            //setHeadJointPosition(joints[0], joints[1]);
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

    std::vector<double> HeadControl::computeJointsForTracking(const geometry_msgs::PoseStamped &pose){
        //TODO:: implement
        geometry_msgs::PointStamped lookat_point;
        lookat_point.header=pose.header;
        lookat_point.point=pose.pose.position;
        geometry_msgs::PointStamped lookat_camera;

            tf::StampedTransform base_camera_transform;

            try {
              tf.waitForTransform("utorso", lookat_point.header.frame_id, ros::Time(), ros::Duration(1.0));
              tf.transformPoint("utorso", ros::Time(), lookat_point, lookat_point.header.frame_id, lookat_camera);
            } catch (std::runtime_error& e) {
              ROS_WARN("Could not transform look_at position to target frame_id %s", e.what());
                 //TODO return  !!!!
            }

            try {
              tf.waitForTransform("utorso", "head_cam_link", ros::Time(), ros::Duration(1.0));
              tf.lookupTransform("utorso", "head_cam_link", ros::Time(), base_camera_transform);
            } catch (std::runtime_error& e) {
              ROS_WARN("Could not transform from base frame to camera_frame %s", e.what());
              //TODO return  !!!!
            }

            geometry_msgs::QuaternionStamped orientation;
            orientation.header = lookat_camera.header;
            orientation.header.frame_id = "world";
            tf::Vector3 dir(lookat_camera.point.x - base_camera_transform.getOrigin().x(), lookat_camera.point.y - base_camera_transform.getOrigin().y(), lookat_camera.point.z - base_camera_transform.getOrigin().z());

            //tf::Quaternion quaternion = tf::createQuaternionFromRPY(0.0, -atan2(dir.z(), sqrt(dir.x()*dir.x() + dir.y()*dir.y())), atan2(dir.y(), dir.x()));


            double pan = atan2(dir.y(), dir.x()); //yaw
            double tilt = -atan2(dir.z(), sqrt(dir.x()*dir.x() + dir.y()*dir.y()));  // pitch

            std::vector<double> joints;
            joints.push_back(pan);
            joints.push_back(tilt);

            return joints;
    }


    void HeadControl::trackLeftHandCb(const geometry_msgs::PoseStamped &pose){
        //std::cout << "In Callback !!!!!"<<std::endl;
        if(tracking_mode == head_tracking_mode::LEFT_HAND_TRACKING){
            std::vector<double> joints = computeJointsForTracking(pose);
            setHeadJointPosition(joints[0], joints[1]);
        }
    }

    void HeadControl::trackRightHandCb(const geometry_msgs::PoseStamped &pose){
        if(tracking_mode == head_tracking_mode::RIGHT_HAND_TRACKING){
            std::vector<double> joints = computeJointsForTracking(pose);
            setHeadJointPosition(joints[0], joints[1]);
        }
    }
}
