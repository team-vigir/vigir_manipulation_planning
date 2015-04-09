#ifndef VIGIR_HEAD_CONTROL_NODE_H
#define VIGIR_HEAD_CONTROL_NODE_H

#include <ros/ros.h>
#include <vigir_planning_msgs/HeadControlCommand.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2_msgs/TFMessage.h>

namespace head_control{

    class HeadControl {

    public:
      HeadControl();
      virtual ~HeadControl();
    protected:
      void HeadControlCb(const vigir_planning_msgs::HeadControlCommand &command);
      void trackLeftHandCb(const geometry_msgs::PoseStamped &pose);
      void trackRightHandCb(const geometry_msgs::PoseStamped &pose);
      void tfCb(const tf2_msgs::TFMessage &tfmsg);

      void setHeadJointPosition(const double pan, const double tilt);
      std::vector<double> computeJointsForTracking(const geometry_msgs::PoseStamped &pose);
      std::vector<double> computeJointsForTracking(const std::string &target_frame_id);

    private:
      unsigned char tracking_mode;
      ros::Publisher joint_trajectory_pub;
      ros::Subscriber head_control_sub;
      ros::Subscriber left_hand_sub;
      ros::Subscriber right_hand_sub;
      ros::Subscriber tf_sub;
      std::string tracking_frame;
      tf::TransformListener tf;
    };
}

namespace head_tracking_mode{
  const unsigned char NONE = 0;
  const unsigned char LEFT_HAND_TRACKING = 1;
  const unsigned char RIGHT_HAND_TRACKING = 2;
  const unsigned char FRAME_TRACKING = 3;
}
#endif // VIGIR_HEAD_CONTROL_NODE_H
