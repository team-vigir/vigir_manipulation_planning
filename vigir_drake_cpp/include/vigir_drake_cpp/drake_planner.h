#ifndef FLOR_DRAKE_PLANNER_H__
#define FLOR_DRAKE_PLANNER_H__

#include "ros/ros.h"
#include <vigir_planning_msgs/RequestWholeBodyIK.h>
#include <vigir_planning_msgs/RequestWholeBodyTrajectory.h>
#include <vigir_planning_msgs/RequestWholeBodyCartesianTrajectory.h>

#include <boost/thread/mutex.hpp>

class RigidBodyManipulator;

namespace vigir_drake_cpp {

class PositionIKPlannerModule;
class TrajectoryPlannerModule;
class CartesianTrajectoryPlannerModule;
  
class DrakePlanner {
public:
  DrakePlanner();
  ~DrakePlanner();
  
protected:
  bool handleWholeBodyIKRequest(vigir_planning_msgs::RequestWholeBodyIK::Request &request, vigir_planning_msgs::RequestWholeBodyIK::Response &response);
  bool handleWholeBodyTrajectoryRequest(vigir_planning_msgs::RequestWholeBodyTrajectory::Request &request, vigir_planning_msgs::RequestWholeBodyTrajectory::Response &response);
  bool handleWholeBodyCartesianTrajectoryRequest(vigir_planning_msgs::RequestWholeBodyCartesianTrajectory::Request &request, vigir_planning_msgs::RequestWholeBodyCartesianTrajectory::Response &response);

private:
  ros::NodeHandle node_handle_;
  ros::ServiceServer whole_body_ik_service_;
  ros::ServiceServer whole_body_trajectory_service_;
  ros::ServiceServer whole_body_cartesian_trajectory_service_;

  RigidBodyManipulator *robot_model_;

  PositionIKPlannerModule *position_ik_planner_;
  TrajectoryPlannerModule *trajectory_planner_;
  CartesianTrajectoryPlannerModule *cartesian_trajectory_planner_;
};

}

#endif
