#ifndef VIGIR_HEAD_CONTROL_NODE_H
#define VIGIR_HEAD_CONTROL_NODE_H

#include <ros/ros.h>
#include <vigir_planning_msgs/HeadControlCommand.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace head_control{
    class HeadControl {


    public:
      HeadControl();
      virtual ~HeadControl();
    protected:
      void HeadControlCb(const vigir_planning_msgs::HeadControlCommand &command);
    private:
      ros::Publisher joint_trajectory_pub;
      ros::Subscriber head_control_sub;

    };
}
#endif // VIGIR_HEAD_CONTROL_NODE_H
