/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, TORC Robotics, LLC ( Team ViGIR )
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
#include "vigir_drake_matlab/drake_bridge.h"
#include <vigir_planning_msgs/RequestDrakeIK.h>
#include <vigir_planning_msgs/ResultDrakeIK.h>
#include <vigir_planning_msgs/RequestDrakeTrajectory.h>
#include <vigir_planning_msgs/ResultDrakeTrajectory.h>

namespace vigir_drake_matlab {

DrakeBridge::DrakeBridge() 
{
  whole_body_ik_service_ = node_handle_.advertiseService("drake_planner/request_whole_body_ik", &DrakeBridge::handleWholeBodyIKRequest, this);
  ik_request_publisher_ = node_handle_.advertise<vigir_planning_msgs::RequestDrakeIK>("/drake_planner/request_drake_ik", 1, false);

  whole_body_trajectory_service_ = node_handle_.advertiseService("drake_planner/request_whole_body_trajectory", &DrakeBridge::handleWholeBodyTrajectoryRequest, this);
  trajectory_request_publisher_ = node_handle_.advertise<vigir_planning_msgs::RequestDrakeTrajectory>("/drake_planner/request_drake_trajectory", 1, false);

  whole_body_cartesian_trajectory_service_ = node_handle_.advertiseService("drake_planner/request_whole_body_cartesian_trajectory", &DrakeBridge::handleWholeBodyCartesianTrajectoryRequest, this);
  cartesian_trajectory_request_publisher_ = node_handle_.advertise<vigir_planning_msgs::RequestDrakeCartesianTrajectory>("/drake_planner/request_drake_cartesian_trajectory", 1, false);
}

DrakeBridge::~DrakeBridge() 
{
  
}

bool DrakeBridge::handleWholeBodyIKRequest(vigir_planning_msgs::RequestWholeBodyIK::Request &request, vigir_planning_msgs::RequestWholeBodyIK::Response &response)
{
  ROS_INFO("received IK request");

  if ( request.ik_request.target_link_names.size() != request.ik_request.target_poses.size() ) {
      ROS_WARN("number of target poses and names does not match => aborting");
      return false;
  }


  // send request via regular topic
  ik_request_publisher_.publish(request.ik_request);

  // wait for 0.5 secs at maximum
  vigir_planning_msgs::ResultDrakeIKConstPtr response_msg = ros::topic::waitForMessage<vigir_planning_msgs::ResultDrakeIK>("/drake_planner/ik_result", ros::Duration(0.5));

  // received a valid response?
  if (response_msg && response_msg->is_valid) {
    response.ik_result.result_state = response_msg->result_state;
    response.ik_result.is_valid = true;
    return true;
  }
  else {
    response.ik_result.is_valid = false;
    return false;
  }
}

bool DrakeBridge::handleWholeBodyTrajectoryRequest(vigir_planning_msgs::RequestWholeBodyTrajectory::Request &request, vigir_planning_msgs::RequestWholeBodyTrajectory::Response &response)
{
  // send request via regular topic
  trajectory_request_publisher_.publish(request.trajectory_request);

  // wait for 2 secs at maximum
  vigir_planning_msgs::ResultDrakeTrajectoryConstPtr response_msg = ros::topic::waitForMessage<vigir_planning_msgs::ResultDrakeTrajectory>("/drake_planner/trajectory_result", ros::Duration(2.0));

  // received a valid response?
  if (response_msg && response_msg->is_valid) {      
    response.trajectory_result.result_trajectory = response_msg->result_trajectory;
    response.trajectory_result.is_valid = true;
    return true;
  }
  else {
    response.trajectory_result.is_valid = false;
    return false;
  }
}

bool DrakeBridge::handleWholeBodyCartesianTrajectoryRequest(vigir_planning_msgs::RequestWholeBodyCartesianTrajectory::Request &request, vigir_planning_msgs::RequestWholeBodyCartesianTrajectory::Response &response)
{
  // send request via regular topic
  cartesian_trajectory_request_publisher_.publish(request.trajectory_request);

  // wait for 5 secs at maximum
  vigir_planning_msgs::ResultDrakeTrajectoryConstPtr response_msg = ros::topic::waitForMessage<vigir_planning_msgs::ResultDrakeTrajectory>("/drake_planner/cartesian_trajectory_result", ros::Duration(5.0));

  // received a valid response?
  if (response_msg && response_msg->is_valid) {
    response.trajectory_result.result_trajectory = response_msg->result_trajectory;
    response.trajectory_result.is_valid = true;
    return true;
  }
  else {
    response.trajectory_result.is_valid = false;
    return false;
  }
}

}
