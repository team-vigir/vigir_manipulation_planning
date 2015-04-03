#ifndef VIGIR_HEAD_CONTROL_NODE_H
#define VIGIR_HEAD_CONTROL_NODE_H

#include <ros/ros.h>
#include <vigir_planning_msgs/HeadControlCommand.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>

namespace head_control{

    class HeadControl {

    public:
      HeadControl();
      virtual ~HeadControl();
    protected:
      void HeadControlCb(const vigir_planning_msgs::HeadControlCommand &command);
      void trackLeftHandCb(const geometry_msgs::PoseStamped &pose);
      void trackRightHandCb(const geometry_msgs::PoseStamped &pose);

      void setHeadJointPosition(const double pan, const double tilt);
      std::vector<double> computeJointsLeftHandTracking();
      std::vector<double> computeJointsRightHandTracking();
    private:
      unsigned char tracking_mode;
      ros::Publisher joint_trajectory_pub;
      ros::Subscriber head_control_sub;
      ros::Subscriber left_hand_sub;
      ros::Subscriber right_hand_sub;
    };
}

namespace head_tracking_mode{
  const unsigned char NONE = 0;
  const unsigned char LEFT_HAND_TRACKING = 1;
  const unsigned char RIGHT_HAND_TRACKING = 2;
}
#endif // VIGIR_HEAD_CONTROL_NODE_H
