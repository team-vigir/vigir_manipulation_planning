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

#ifndef INV_REACH_MANAGER_H__
#define INV_REACH_MANAGER_H__


//#include <flor_visualization_utils/marker_utils.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

//#include <flor_grasp_msgs/InverseReachabilityForGraspRequest.h>
//#include <flor_grasp_msgs/InverseReachabilityWithPoseRequest.h>

#include <flor_planning_msgs/CircularMotionRequest.h>
#include <flor_planning_msgs/CartesianMotionRequest.h>

//#include <simox_inv_reach_ros/IKResult.h>
#include <ros/ros.h>

class InvReachManager{
public:
  InvReachManager()
  {
    ros::NodeHandle nh("");

    //simox_pose_inverse_reachability_client_l_arm_and_yaw_ = nh.serviceClient<simox_inv_reach_ros::IKResult>("/simox_left_arm/inv_reach/get_robot_state_for_pose");
    //simox_pose_inverse_reachability_client_r_arm_and_yaw_ = nh.serviceClient<simox_inv_reach_ros::IKResult>("/simox_right_arm/inv_reach/get_robot_state_for_pose");

    //ghost_robot_pose_publisher_   = nh.advertise<geometry_msgs::PoseStamped>("/flor/ghost/set_root_pose",1);
    //ghost_robot_joints_publisher_ = nh.advertise<sensor_msgs::JointState>("/flor/ghost/set_joint_states",1);

    circular_plan_publisher_ = nh.advertise<flor_planning_msgs::CircularMotionRequest>("/flor/planning/upper_body/plan_circular_request",1);
    cartesian_plan_publisher_ = nh.advertise<flor_planning_msgs::CartesianMotionRequest>("/flor/planning/upper_body/plan_cartesian_request",1);

    
    rotation_center_pose_sub_ = nh.subscribe("/flor/ocs/planning/rotation_center_pose", 1, &InvReachManager::rotationCenterPoseCallback, this);
    trigger_rotation_motion_sub_ = nh.subscribe("/flor/ocs/planning/trigger_rotation_motion", 1, &InvReachManager::triggerRotationMotionCallback, this);

    cartesian_target_pose_sub_ = nh.subscribe("/flor/ocs/planning/cartesian_target_pose", 1, &InvReachManager::cartesianTargetPoseCallback, this);
    trigger_cartesian_motion_sub_ = nh.subscribe("/flor/ocs/planning/trigger_cartesian_motion", 1, &InvReachManager::triggerCartesianMotionCallback, this);

    /*
    inverse_reachability_target_pose_sub_ = nh.subscribe("/flor/ocs/planning/inverse_reachability_target", 1, &InvReachManager::invReachTargetPoseCallback, this);
    inverse_reachability_req_sub_ = nh.subscribe("/flor/ocs/planning/inverse_rechability_for_grasp", 1, &InvReachManager::poseInverseReachabilityQueryCallback, this);
    inverse_reachability_req_with_pose_sub_ = nh.subscribe("/flor/ocs/planning/inverse_reachability_with_target", 1, &InvReachManager::invReachQueryWithTargetPoseCallback, this);
    */
  }

  /*
  void poseInverseReachabilityQueryCallback(const flor_grasp_msgs::InverseReachabilityForGraspRequest req)
  {
    if (inv_reach_target_pose_ == 0){
      ROS_WARN("No inverse reachability target received so far, aborting inverse reachability request (hand=%d)!",req.hand_side);
      return;
    }

    const geometry_msgs::PoseStamped& target = *inv_reach_target_pose_;

    ros::ServiceClient* service_client = 0;

    if (req.hand_side == flor_grasp_msgs::InverseReachabilityForGraspRequest::HAND_LEFT){
      service_client = &simox_pose_inverse_reachability_client_l_arm_and_yaw_;
    }else{
      service_client = &simox_pose_inverse_reachability_client_r_arm_and_yaw_;
    }

    /*simox_inv_reach_ros::IKResult srv;

    srv.request.pose = target.pose;

    if (service_client->call(srv)){
      ghost_robot_joints_publisher_.publish(srv.response.jointStates);

      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "world";
      pose.pose = srv.response.poseRobot;
      ghost_robot_pose_publisher_.publish(pose);
    }else{
      ROS_WARN("No valid configuration to reach target pose found!");
    } 
  }
  */

  void invReachTargetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    inv_reach_target_pose_ = msg;
  }

  /*
  void invReachQueryWithTargetPoseCallback(const flor_grasp_msgs::InverseReachabilityWithPoseRequest::ConstPtr& msg)
  {
    invReachTargetPoseCallback((const geometry_msgs::PoseStamped::ConstPtr)&msg->target_pose);
    poseInverseReachabilityQueryCallback(msg->reachability_request);
  }
  */

  void rotationCenterPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    rotation_center_pose_ = msg;
  }

  void cartesianTargetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    cartesian_target_pose_ = msg;
  }

  void triggerRotationMotionCallback(const std_msgs::Float64& msg )
  {
    if (rotation_center_pose_ == 0){
      ROS_WARN("No rotation center pose received yet, cannot generate circular motion!");
      return;
    }

    flor_planning_msgs::CircularMotionRequest req;

    req.rotation_center_pose = *rotation_center_pose_;
    req.rotation_angle = msg.data;
    req.use_environment_obstacle_avoidance = false;
    req.planning_group = "l_arm_group";

    circular_plan_publisher_.publish(req);
  }

  void triggerCartesianMotionCallback(const std_msgs::Empty msg )
  {
    if (cartesian_target_pose_ == 0){
      ROS_WARN("No cartesian target pose received yet, cannot generate circular motion!");
      return;
    }

    flor_planning_msgs::CartesianMotionRequest req;
    req.header.frame_id = cartesian_target_pose_->header.frame_id;

    req.waypoints.push_back(cartesian_target_pose_->pose);
    req.use_environment_obstacle_avoidance = false;
    req.planning_group = "l_arm_group";

    cartesian_plan_publisher_.publish(req);
  }




protected:

  boost::shared_ptr<const geometry_msgs::PoseStamped> inv_reach_target_pose_;

  boost::shared_ptr<const geometry_msgs::PoseStamped> rotation_center_pose_;
  boost::shared_ptr<const geometry_msgs::PoseStamped> cartesian_target_pose_;

  ros::Publisher ghost_robot_pose_publisher_;
  ros::Publisher ghost_robot_joints_publisher_;

  ros::Publisher circular_plan_publisher_;
  ros::Publisher cartesian_plan_publisher_;

  ros::Subscriber inverse_reachability_req_sub_;
  ros::Subscriber inverse_reachability_target_pose_sub_;
  ros::Subscriber inverse_reachability_req_with_pose_sub_;

  ros::Subscriber rotation_center_pose_sub_;
  ros::Subscriber cartesian_target_pose_sub_;

  ros::Subscriber trigger_rotation_motion_sub_;
  ros::Subscriber trigger_cartesian_motion_sub_;

  ros::ServiceClient simox_pose_inverse_reachability_client_l_arm_and_yaw_;
  ros::ServiceClient simox_pose_inverse_reachability_client_l_arm_and_yaw_waypoints_;
  ros::ServiceClient simox_pose_inverse_reachability_client_r_arm_and_yaw_;
  ros::ServiceClient simox_pose_inverse_reachability_client_r_arm_and_yaw_waypoints_;

};

#endif
