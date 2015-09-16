/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, Team ViGIR ( TORC Robotics LLC, TU Darmstadt, Virginia Tech, Oregon State University, Cornell University, and Leibniz University Hanover )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
//@TODO_ADD_AUTHOR_INFO
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

  void handleIKResult(vigir_planning_msgs::ResultDrakeIKConstPtr result);
  void handleTrajectoryResult(vigir_planning_msgs::ResultDrakeTrajectoryConstPtr result);
  void handleCartesianTrajectoryResult(vigir_planning_msgs::ResultDrakeTrajectoryConstPtr result);

private:
  ros::NodeHandle node_handle_;

  // offer service interface to ROS side
  ros::ServiceServer whole_body_ik_service_;
  ros::ServiceServer whole_body_trajectory_service_;
  ros::ServiceServer whole_body_cartesian_trajectory_service_;

  // send requests to Drake (via rosmatlab)
  ros::Publisher ik_request_publisher_;
  ros::Publisher trajectory_request_publisher_;
  ros::Publisher cartesian_trajectory_request_publisher_;

  // receive results from Drake (via rosmatlab)
  ros::Subscriber ik_result_subscriber_;
  ros::Subscriber trajectory_result_subscriber_;
  ros::Subscriber cartesian_trajectory_result_subscriber_;

  // stores last received result message
  vigir_planning_msgs::ResultDrakeIKConstPtr last_ik_result_;
  vigir_planning_msgs::ResultDrakeTrajectoryConstPtr last_trajectory_result_;
  vigir_planning_msgs::ResultDrakeTrajectoryConstPtr last_cartesian_trajectory_result_;

  // maximum times to wait before returning failure
  const ros::Duration MaxIKTime = ros::Duration(0.4);
  const ros::Duration MaxTrajectoryTime = ros::Duration(2.0);
  const ros::Duration MaxCartesianTrajectoryTime = ros::Duration(10.0);

};

};

#endif
