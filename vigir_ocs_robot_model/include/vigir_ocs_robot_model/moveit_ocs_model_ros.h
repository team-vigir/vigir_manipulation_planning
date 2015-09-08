//=================================================================================================
// Copyright (c) 2013, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef MOVEIT_OCS_MODEL_ROS_H__
#define MOVEIT_OCS_MODEL_ROS_H__

#include <vigir_ocs_robot_model/moveit_ocs_model.h>

#include <vigir_teleop_planning_msgs/TargetConfigIkRequest.h>

#include <vigir_teleop_planning_msgs/PlanRequest.h>
#include <vigir_planning_msgs/PlannerConfiguration.h>
#include <vigir_teleop_planning_msgs/PlanToJointTargetRequest.h>
#include <std_msgs/Int8.h>

#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_state/conversions.h>

#include <boost/make_shared.hpp>

#include <vigir_visualization_utils/marker_utils.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>



class MoveItOcsModelRos{
public:
  MoveItOcsModelRos();

  void targetConfigCallback (const vigir_teleop_planning_msgs::TargetConfigIkRequest::ConstPtr& msg);

  void incomingJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);

  void realJointStatesCallback(const sensor_msgs::JointState::ConstPtr msg);
  void realPoseCallback(const geometry_msgs::PoseStampedConstPtr msg);
  void snapGhostToRobotCallback(const std_msgs::Bool& msg);


  // Sets global pose of model
  void rootPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  // Publishes endeffector poses at 30Hz
  void pubEndeffectorPosesTimerCallback(const ros::TimerEvent& event);

  // To be called when model changed
  void onModelUpdated();

  void plannerConfigurationCb(const vigir_planning_msgs::PlannerConfiguration::ConstPtr& msg);

  void incomingPlanToPoseRequestCallback(const std_msgs::String::ConstPtr& msg);

  void incomingPlanToJointRequestCallback(const std_msgs::String::ConstPtr& msg);

  void ghostStateCallback(const std_msgs::Bool::ConstPtr& msg);

  void updateRobotStateColors();

  void setLinkColor(double r, double g, double b, double a, size_t index);

  void setLinkColors(double r, double g, double b, double a);

protected:
  void setPoseWithWholeBodyIK( const std::vector< ::geometry_msgs::PoseStamped > goal_poses, const std::vector<std_msgs::String> target_link_names, const std::string& group_name);

  bool collision_avoidance_active_;
  bool use_drake_ik_;

  boost::shared_ptr<MoveItOcsModel> ocs_model_;

  moveit_msgs::DisplayRobotState display_state_msg_;

  sensor_msgs::JointStateConstPtr real_joint_states_;
  geometry_msgs::PoseStampedConstPtr real_robot_pose_;
  boost::shared_ptr<robot_state::RobotState> real_robot_state_;

  ros::Publisher pose_plan_request_pub_;
  ros::Publisher joint_plan_request_pub_;

  ros::Subscriber incoming_plan_to_pose_request_sub_;
  ros::Subscriber incoming_plan_to_joint_request_sub_;

  ros::Subscriber incoming_joint_states_sub_;
  ros::Subscriber incoming_real_joint_states_sub_;
  ros::Subscriber incoming_real_pose_sub_;
  ros::Subscriber ghost_snap_to_real_config_sub_;

  ros::Subscriber pose_sub_;
  ros::Subscriber root_pose_sub_;
  ros::Subscriber torso_joint_position_constraints_sub_;

  ros::Publisher robot_state_vis_pub_;
  ros::Publisher robot_state_diff_real_vis_pub_;


  ros::Publisher marker_array_pub_;
  ros::Publisher current_ghost_joint_states_pub_;
  ros::Publisher ghost_pelvis_pose_pub_;

  ros::Publisher left_hand_pose_pub_;
  ros::Publisher right_hand_pose_pub_;
  ros::Publisher pose_pub_;

  ros::Timer ee_pose_pub_timer_;

  ros::ServiceClient whole_body_ik_client_;
  ros::Subscriber ghost_state_sub_;
  std::string base_frame_;
  std::string l_hand_frame_;
  std::string r_hand_frame_;
  std::string pelvis_frame_;

  tf::TransformListener transform_listener_;
};

#endif
