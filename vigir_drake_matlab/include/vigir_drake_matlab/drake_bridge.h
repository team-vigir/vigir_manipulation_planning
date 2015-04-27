#ifndef VIGIR_DRAKE_BRIDGE_H__
#define VIGIR_DRAKE_BRIDGE_H__

#include "ros/ros.h"
#include <vigir_planning_msgs/RequestWholeBodyIK.h>
#include <vigir_planning_msgs/RequestWholeBodyTrajectory.h>
#include <vigir_planning_msgs/RequestWholeBodyCartesianTrajectory.h>

#include <boost/thread/mutex.hpp>

namespace vigir_drake_matlab {
  
class DrakeBridge {
public:
  DrakeBridge();
  ~DrakeBridge();
  
protected:
  bool handleWholeBodyIKRequest(vigir_planning_msgs::RequestWholeBodyIK::Request &request, vigir_planning_msgs::RequestWholeBodyIK::Response &response);
  bool handleWholeBodyTrajectoryRequest(vigir_planning_msgs::RequestWholeBodyTrajectory::Request &request, vigir_planning_msgs::RequestWholeBodyTrajectory::Response &response);
  bool handleWholeBodyCartesianTrajectoryRequest(vigir_planning_msgs::RequestWholeBodyCartesianTrajectory::Request &request, vigir_planning_msgs::RequestWholeBodyCartesianTrajectory::Response &response);

private:
  ros::NodeHandle node_handle_;
  ros::ServiceServer whole_body_ik_service_;
  ros::ServiceServer whole_body_trajectory_service_;
  ros::ServiceServer whole_body_cartesian_trajectory_service_;

  ros::Publisher ik_request_publisher_;
  ros::Publisher trajectory_request_publisher_;
  ros::Publisher cartesian_trajectory_request_publisher_;
};

};

#endif
