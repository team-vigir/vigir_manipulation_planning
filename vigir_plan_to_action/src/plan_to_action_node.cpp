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

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <flor_ocs_msgs/OCSRobotStatus.h>
#include <flor_ocs_msgs/RobotStatusCodes.h>

#include <flor_planning_msgs/PlanRequest.h>
#include <flor_planning_msgs/PlanToJointTargetRequest.h>


#include <flor_planning_msgs/GetMotionPlanForPose.h>
#include <flor_planning_msgs/GetMotionPlanForJoints.h>
#include <flor_planning_msgs/GetMotionPlanForCircularMotion.h>
#include <flor_planning_msgs/GetMotionPlanForCartesianWaypoints.h>

#include <actionlib/client/simple_action_client.h>
#include <vigir_planning_msgs/MoveAction.h>

#include <vigir_planning_msgs/PlannerConfiguration.h>
#include <vigir_moveit_utils/joint_constraint_utils.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/kinematic_constraints/utils.h>

class PlanToAction
{
public:

  PlanToAction()
  {
    ros::NodeHandle nh("");

    // Trajectory Controllers
    /*
    l_arm_traj_pub_ = nh.advertise<trajectory_msgs::JointTrajectory>("l_arm_trajectory",1,false);
    r_arm_traj_pub_ = nh.advertise<trajectory_msgs::JointTrajectory>("r_arm_trajectory",1,false);
    torso_traj_pub_ = nh.advertise<trajectory_msgs::JointTrajectory>("torso_trajectory",1,false);
    neck_traj_pub_  = nh.advertise<trajectory_msgs::JointTrajectory>("neck_trajectory",1,false);
    */

    // Connection to planning service
    /*
    pose_planning_srv_client_ = nh.serviceClient<flor_planning_msgs::GetMotionPlanForPose>("get_plan");
    joints_planning_srv_client_ = nh.serviceClient<flor_planning_msgs::GetMotionPlanForJoints>("get_plan_joints");
    circular_planning_srv_client_ = nh.serviceClient<flor_planning_msgs::GetMotionPlanForCircularMotion>("get_plan_circular");
    cartesian_planning_srv_client_ = nh.serviceClient<flor_planning_msgs::GetMotionPlanForCartesianWaypoints>("get_plan_cartesian");
    */

    planner_configuration_.disable_collision_avoidance = false;
    planner_configuration_.robot_collision_padding = 0.0f;
    planner_configuration_.trajectory_time_factor = 1.0f;
    planner_configuration_.octomap_max_height = 2.3f;
    planner_configuration_.goal_cube_clearance = 0.105f;


    robot_model_loader_.reset(new robot_model_loader::RobotModelLoader());
    robot_model_ = robot_model_loader_->getModel();

    ros::Duration wait_for_server(5.0);

    move_action_client_.reset(new actionlib::SimpleActionClient<vigir_planning_msgs::MoveAction>(nh,
                                                                                              "vigir_move_group",
                                                                                              true));
    waitForAction(move_action_client_, wait_for_server, "move");

    planner_configuration_sub_ = nh.subscribe("/flor/planning/upper_body/configuration",
                                              10,
                                              &PlanToAction::plannerConfigurationCb,
                                              this);

    // General planning ROS API
    plan_request_sub_ = nh.subscribe("plan_request", 1, &PlanToAction::planRequestCallback, this);
    plan_joint_request_sub_ = nh.subscribe("plan_joint_request", 1, &PlanToAction::planJointRequestCallback, this);
    plan_circular_request_sub_ = nh.subscribe("plan_circular_request", 1, &PlanToAction::planCircularRequestCallback, this);
    plan_cartesian_request_sub_ = nh.subscribe("plan_cartesian_request", 1, &PlanToAction::planCartesianRequestCallback, this);
    plan_status_pub_ = nh.advertise<flor_ocs_msgs::OCSRobotStatus>("plan_status",1,false);

    // Subscriber for plans that sends them of to controller
    plan_execute_sub_ = nh.subscribe("execute_trajectory", 1, &PlanToAction::planExecuteCallback, this);

    // Subscribers to trajectory messages. For these, the final configuration of the trajectory will be used
    // and a new collision free and smooth path there will be planned and sent to controller
    general_trajectory_sub_ = nh.subscribe("refine_general_trajectory", 1, &PlanToAction::generalTrajectoryCallback, this);
    neck_joint_trajectory_sub_  = nh.subscribe("refine_neck_trajectory", 1, &PlanToAction::refineNeckTrajectoryCallback, this);
    torso_joint_trajectory_sub_ = nh.subscribe("refine_torso_trajectory", 1, &PlanToAction::refineTorsoTrajectoryCallback, this);
    l_arm_joint_trajectory_sub_ = nh.subscribe("refine_l_arm_trajectory", 1, &PlanToAction::refineLeftArmTrajectoryCallback, this);
    r_arm_joint_trajectory_sub_ = nh.subscribe("refine_r_arm_trajectory", 1, &PlanToAction::refineRightArmTrajectoryCallback, this);

    // Grasping ROS API
    l_grasp_status_pub_ = nh.advertise<flor_ocs_msgs::OCSRobotStatus>("l_grasp_status",1,false);
    r_grasp_status_pub_ = nh.advertise<flor_ocs_msgs::OCSRobotStatus>("r_grasp_status",1,false);
    l_grasp_plan_request_sub_ = nh.subscribe("l_grasp_plan_request", 1, &PlanToAction::lGraspRequestCallback, this);
    r_grasp_plan_request_sub_ = nh.subscribe("r_grasp_plan_request", 1, &PlanToAction::rGraspRequestCallback, this);

  }

  void plannerConfigurationCb(const vigir_planning_msgs::PlannerConfiguration::ConstPtr& msg)
  {
    ROS_INFO("Received planner configuration");
    planner_configuration_ = *msg;

    joint_constraint_utils::toMoveitConstraint(planner_configuration_.joint_position_constraints,
                                               *robot_model_,
                                               goal_.request.path_constraints.joint_constraints);
  }

  void planRequestCallback(const flor_planning_msgs::PlanRequest::ConstPtr& msg)
  {
    flor_ocs_msgs::OCSRobotStatus status;

    if (!planAndMove(*msg, &status)){
      //status.stamp  = ros::Time::now();
      //plan_status_pub_.publish(status); // publish original message first
      //status.status = RobotStatusCodes::status(RobotStatusCodes::PLANNER_FAILED, RobotStatusCodes::ERROR);
    }

    //plan_status_pub_.publish(status);
  }

  void lGraspRequestCallback(const flor_planning_msgs::PlanRequest::ConstPtr& msg)
  {
    flor_ocs_msgs::OCSRobotStatus status;

    if (!planAndMove(*msg, &status)){
      status.stamp = ros::Time::now();
      //plan_status_pub_.publish(status); // publish original message first
      status.status = RobotStatusCodes::status(RobotStatusCodes::PLANNER_FAILED, RobotStatusCodes::ERROR);
    }

    l_grasp_status_pub_.publish(status);
    //plan_status_pub_.publish(status);
  }

  void rGraspRequestCallback(const flor_planning_msgs::PlanRequest::ConstPtr& msg)
  {
    flor_ocs_msgs::OCSRobotStatus status;

    if (!planAndMove(*msg, &status)){
      status.stamp = ros::Time::now();
      //plan_status_pub_.publish(status); // publish original message first
      status.status = RobotStatusCodes::status(RobotStatusCodes::PLANNER_FAILED, RobotStatusCodes::ERROR);
    }

    r_grasp_status_pub_.publish(status);
    //plan_status_pub_.publish(status);
  }

  void planJointRequestCallback(const flor_planning_msgs::PlanToJointTargetRequest::ConstPtr& msg)
  {
    flor_ocs_msgs::OCSRobotStatus status;

    if (!planAndMoveToJoints(*msg, &status)){
      //status.stamp = ros::Time::now();
      //plan_status_pub_.publish(status); // publish original message first
      //status.status = RobotStatusCodes::status(RobotStatusCodes::PLANNER_FAILED, RobotStatusCodes::ERROR);
    }

    //plan_status_pub_.publish(status);
  }

  void planCircularRequestCallback(const flor_planning_msgs::CircularMotionRequest::ConstPtr& msg)
  {
    /*
    flor_planning_msgs::GetMotionPlanForCircularMotion plan_srv;

    plan_srv.request.plan_request = *msg;

    flor_ocs_msgs::OCSRobotStatus status;

    if (circular_planning_srv_client_.call(plan_srv))
    {
      if (plan_srv.response.status == flor_planning_msgs::GetMotionPlanForPose::Response::OK){
        splitAndSendTrajectory(plan_srv.response.trajectory, msg->planning_group);
        ROS_INFO("Succesfully planned and sent circular trajectory to planner.");
      }
      ROS_DEBUG("Called circular trajectory planning service with return status %d", (int)plan_srv.response.status);
      setStatus(plan_srv.response.status,status);
      plan_status_pub_.publish(status);
    }else{
      ROS_ERROR("Service call to circular trajectory planning service returned false.");
      setStatus(plan_srv.response.status,status);
      plan_status_pub_.publish(status);
    }
    */

  }

  void planCartesianRequestCallback(const flor_planning_msgs::CartesianMotionRequest::ConstPtr& msg)
  {
    /*
    flor_planning_msgs::GetMotionPlanForCartesianWaypoints plan_srv;

    plan_srv.request.plan_request = *msg;

    flor_ocs_msgs::OCSRobotStatus status;

    if (cartesian_planning_srv_client_.call(plan_srv))
    {
      if (plan_srv.response.status == flor_planning_msgs::GetMotionPlanForPose::Response::OK){
        splitAndSendTrajectory(plan_srv.response.trajectory, msg->planning_group);
        ROS_INFO("Succesfully planned and sent cartesian waypoint trajectory to planner.");
      }
      ROS_DEBUG("Called cartesian waypoint planning service with return status %d", (int)plan_srv.response.status);
      setStatus(plan_srv.response.status, status);
      plan_status_pub_.publish(status);
    }else{
      ROS_ERROR("Service call to circular trajectory planning service returned false.");
      //status.stamp = ros::Time::now();
      //status.status = RobotStatusCodes::status(RobotStatusCodes::PLANNER_UNKNOWN_FAILURE, RobotStatusCodes::ERROR);
      setStatus(plan_srv.response.status, status);
      plan_status_pub_.publish(status);
    }
    */

  }

  void planExecuteCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg) const
  {
    std::string group = this->inferGroupNameFromTrajectory(*msg);

    if (group.empty()){
      ROS_ERROR("Plan execute could not infer group name for given trajectory with number of joints %d, not sending trajectory", (int)msg->joint_names.size());
    }

    splitAndSendTrajectory(*msg, group);

  }

  void generalTrajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
  {
    replanToGivenTrajectoryConfig(msg);
  }

  void refineNeckTrajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
  {
    replanToGivenTrajectoryConfig(msg);
  }

  void refineTorsoTrajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
  {
    replanToGivenTrajectoryConfig(msg);
  }

  void refineLeftArmTrajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
  {
    replanToGivenTrajectoryConfig(msg);
  }

  void refineRightArmTrajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
  {
    replanToGivenTrajectoryConfig(msg);
  }

  bool replanToGivenTrajectoryConfig(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
  {
    std::string group_name = this->inferGroupNameFromTrajectory(*msg);

    if (group_name.empty()){
      ROS_WARN("Refine trajectory could not infer group name for given trajectory with number of joints %d, not sending trajectory", (int)msg->joint_names.size());

      if (msg->joint_names.size() == 7){
        ROS_WARN("Received trajectory with size 7!. This is intended for experimental calibration !");
        size_t size = msg->points.size();

        flor_planning_msgs::GetMotionPlanForJoints plan_srv;

        plan_srv.request.plan_request.planning_group = "l_arm_group";

        plan_srv.request.plan_request.position.resize(6);
        for(size_t i = 0; i < 6; ++i){
          plan_srv.request.plan_request.position[i] = msg->points.back().positions[i+1];
        }

        if (joints_planning_srv_client_.call(plan_srv))
        {
          ROS_DEBUG ("Planning service called succesfully. Status: %d Used group: %s", plan_srv.response.status, plan_srv.response.used_planning_group.data.c_str());
          if (plan_srv.response.status == flor_planning_msgs::GetMotionPlanForPose::Response::OK){
            splitAndSendTrajectory(plan_srv.response.trajectory, plan_srv.response.used_planning_group.data);
          }
        }

        plan_srv.request.plan_request.position.clear();
        plan_srv.request.plan_request.position.push_back(msg->points.back().positions[0]);
        plan_srv.request.plan_request.planning_group = "neck_group";

        if (joints_planning_srv_client_.call(plan_srv))
        {
          ROS_DEBUG ("Planning service called succesfully. Status: %d Used group: %s", plan_srv.response.status, plan_srv.response.used_planning_group.data.c_str());
          if (plan_srv.response.status == flor_planning_msgs::GetMotionPlanForPose::Response::OK){
            splitAndSendTrajectory(plan_srv.response.trajectory, plan_srv.response.used_planning_group.data);
          }
        }


      }
    }

    size_t size = msg->points.size();

    flor_planning_msgs::GetMotionPlanForJoints plan_srv;

    plan_srv.request.plan_request.planning_group = group_name;
    plan_srv.request.plan_request.position = msg->points.back().positions;


    if (joints_planning_srv_client_.call(plan_srv))
    {
      ROS_DEBUG ("Planning service called succesfully. Status: %d Used group: %s", plan_srv.response.status, plan_srv.response.used_planning_group.data.c_str());
      if (plan_srv.response.status == flor_planning_msgs::GetMotionPlanForPose::Response::OK){
        splitAndSendTrajectory(plan_srv.response.trajectory, plan_srv.response.used_planning_group.data);
      }

      return true;
    }else{
      return false;
    }

  }


  bool planAndMove(const flor_planning_msgs::PlanRequest& plan_request, flor_ocs_msgs::OCSRobotStatus* status = 0)
  {
    /*
    flor_planning_msgs::GetMotionPlanForPose plan_srv;

    plan_srv.request.plan_request = plan_request;

    if (pose_planning_srv_client_.call(plan_srv))
    {
      if (plan_srv.response.status == flor_planning_msgs::GetMotionPlanForPose::Response::OK){
        splitAndSendTrajectory(plan_srv.response.trajectory, plan_srv.response.used_planning_group.data);
      }

      if (status){
        this->setStatus(plan_srv.response.status, *status);
      }

      return true;
    }else{
      if (status){
          this->setStatus(plan_srv.response.status, *status);
      }
      return false;
    }
    */

    goal_.request.max_velocity_scaling_factor = static_cast<double>(this->planner_configuration_.trajectory_time_factor);

    //@TODO Convert from vigir to moveit constraints
    //goal_.request.path_constraints.joint_constraints  =this->planner_configuration_.joint_position_constraints;

    goal_.request.group_name = plan_request.planning_group.data;
    goal_.request.num_planning_attempts = 1;
    goal_.request.allowed_planning_time = 1.0;
    goal_.request.max_velocity_scaling_factor = 1.0;

    goal_.extended_planning_options.target_frame = plan_request.pose.header.frame_id;
    goal_.extended_planning_options.target_poses.clear();
    goal_.extended_planning_options.target_poses.push_back(plan_request.pose.pose);
    goal_.extended_planning_options.target_motion_type = vigir_planning_msgs::ExtendedPlanningOptions::TYPE_FREE_MOTION;

    move_action_client_->sendGoal(goal_,
                                  boost::bind(&PlanToAction::moveActionDoneCallback, this, _1, _2),
                                  boost::bind(&PlanToAction::moveActionActiveCallback, this),
                                  boost::bind(&PlanToAction::moveActionFeedbackCallback, this, _1));
  }

  bool planAndMoveToJoints(const flor_planning_msgs::PlanToJointTargetRequest plan_request, flor_ocs_msgs::OCSRobotStatus* status = 0)
  {
    /*
    flor_planning_msgs::GetMotionPlanForJoints plan_srv;

    plan_srv.request.plan_request = plan_request;

    if (joints_planning_srv_client_.call(plan_srv))
    {
      ROS_DEBUG ("Planning service called succesfully. Status: %d Used group: %s", plan_srv.response.status, plan_srv.response.used_planning_group.data.c_str());
      if (plan_srv.response.status == flor_planning_msgs::GetMotionPlanForPose::Response::OK || plan_srv.response.status == flor_planning_msgs::GetMotionPlanForPose::Response::OCTOMAP_WARNING){
        splitAndSendTrajectory(plan_srv.response.trajectory, plan_srv.response.used_planning_group.data);
      }

      if (status){
        this->setStatus(plan_srv.response.status, *status);
      }

      return true;
    }else{
      if (status){
         this->setStatus(plan_srv.response.status, *status);
      }
      return false;
    }*/
    goal_.request.group_name = plan_request.planning_group;
    goal_.request.num_planning_attempts = 1;
    goal_.request.max_velocity_scaling_factor = 1.0;
    goal_.request.allowed_planning_time = 1.0;

    //goal_.request.goal_constraints = plan_request.position;
    //kinematic_constraints::constructGoalConstraints()

    robot_state::RobotState tmp (robot_model_);

    std::string curr_group = plan_request.planning_group;

    const robot_state::JointModelGroup* joint_model_group = tmp.getJointModelGroup(curr_group);

    tmp.setVariablePositions(joint_model_group->getJointModelNames(), plan_request.position);

    goal_.request.goal_constraints.clear();
    goal_.request.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(tmp, joint_model_group));

    goal_.extended_planning_options.target_poses.clear();
    goal_.extended_planning_options.target_motion_type = vigir_planning_msgs::ExtendedPlanningOptions::TYPE_FREE_MOTION;

    move_action_client_->sendGoal(goal_,
                                  boost::bind(&PlanToAction::moveActionDoneCallback, this, _1, _2),
                                  boost::bind(&PlanToAction::moveActionActiveCallback, this),
                                  boost::bind(&PlanToAction::moveActionFeedbackCallback, this, _1));
  }

  void moveActionDoneCallback(const actionlib::SimpleClientGoalState& state, const vigir_planning_msgs::MoveResultConstPtr& result)
  {
    flor_ocs_msgs::OCSRobotStatus status;

    status.stamp = ros::Time::now();

    //@TODO: Fill status msg here
    ROS_INFO("Move Action done callback, error code: %d", result->error_code.val);
    switch (result->error_code.val)
    {
      case moveit_msgs::MoveItErrorCodes::SUCCESS:
        status.status = RobotStatusCodes::status(RobotStatusCodes::PLANNER_MOVEIT_PLAN_ACTIVE, RobotStatusCodes::OK);
        break;
      case moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION:
        status.status = RobotStatusCodes::status(RobotStatusCodes::PLANNER_IK_FAILED, RobotStatusCodes::ERROR);
        break;
      default:
        status.status = RobotStatusCodes::status(RobotStatusCodes::PLANNER_FAILED, RobotStatusCodes::ERROR);
    }

    plan_status_pub_.publish(status);
  }

  void moveActionActiveCallback()
  {
      ROS_INFO("Move Action active callback");

      flor_ocs_msgs::OCSRobotStatus status;
      status.status = RobotStatusCodes::status(RobotStatusCodes::PLANNER_MOVEIT_PLAN_ACTIVE, RobotStatusCodes::OK);
      plan_status_pub_.publish(status);
  }

  void moveActionFeedbackCallback(const vigir_planning_msgs::MoveFeedbackConstPtr& feedback)
  {
      ROS_INFO("Move Action feedback callback: %s", feedback->state.c_str());
  }

  // Assumes groups are named according to special conventions
  std::string inferGroupNameFromTrajectory(const trajectory_msgs::JointTrajectory& traj) const
  {
    const std::vector<std::string>& names = traj.joint_names;

    if (traj.joint_names.size() == 15){
      return "both_arms_with_torso_group";
    }else if(traj.joint_names.size() == 1){
      return "neck_group";
    }

    std::string test_string = names[3].substr(0,1);

    if (names.size() == 6){
      if (test_string == "r"){
        return "r_arm_group";
      }else if (test_string == "l"){
        return "l_arm_group";
      }
    }else if (traj.joint_names.size() == 9){
      if (test_string == "r"){
        return "r_arm_with_torso_group";
      }else if (test_string == "l"){
        return "l_arm_with_torso_group";
      }
    }

    return "";
  }

  // Assumes groups are named according to special conventions
  void splitAndSendTrajectory(const trajectory_msgs::JointTrajectory& traj, const std::string& group) const
  {
    if (group =="r_arm_with_torso_group"){
      torso_traj_pub_.publish(splitTrajectoryData(traj,0,3));
      r_arm_traj_pub_.publish(splitTrajectoryData(traj,3,9));
    }else if (group =="r_arm_group"){
      r_arm_traj_pub_.publish(traj);
    }else if (group =="l_arm_with_torso_group"){
      torso_traj_pub_.publish(splitTrajectoryData(traj,0,3));
      l_arm_traj_pub_.publish(splitTrajectoryData(traj,3,9));
    }else if (group =="l_arm_group"){
      l_arm_traj_pub_.publish(traj);
    }else if (group =="both_arms_with_torso_group"){
      torso_traj_pub_.publish(splitTrajectoryData(traj,0,3));
      l_arm_traj_pub_.publish(splitTrajectoryData(traj,3,9));
      r_arm_traj_pub_.publish(splitTrajectoryData(traj,9,15));
    }else if (group =="neck_group"){
      neck_traj_pub_.publish(traj);
    }

  }

  // Performs no checks, make sure you feed with correct data
  trajectory_msgs::JointTrajectory splitTrajectoryData(const trajectory_msgs::JointTrajectory& traj, size_t min, size_t max) const
  {
    trajectory_msgs::JointTrajectory traj_out;

    traj_out.header = traj.header;

    for (size_t i = min; i < max; ++i){
      traj_out.joint_names.push_back(traj.joint_names[i]);
    }

    size_t traj_length = traj.points.size();

    size_t num_joints = max - min;

    bool has_vel = (traj.points[0].velocities.size() > 0);
    bool has_acc = (traj.points[0].accelerations.size() > 0);

    traj_out.points.resize(traj_length);

    for (size_t i = 0; i < traj_length; ++i){
      traj_out.points[i].time_from_start = traj.points[i].time_from_start;
      traj_out.points[i].positions.reserve(num_joints);

      if (has_vel){
        traj_out.points[i].velocities.reserve(num_joints);
      }


      for (size_t j = min; j < max; ++j){
        traj_out.points[i].positions.push_back(traj.points[i].positions[j]);

        if (has_vel){
          traj_out.points[i].velocities.push_back(traj.points[i].velocities[j]);
        }

        if (has_acc){
          traj_out.points[i].accelerations.push_back(traj.points[i].accelerations[j]);
        }
      }
    }

    return traj_out;
  }

  //Sets status for OCS feedback
  void setStatus(uint8_t plan_status, flor_ocs_msgs::OCSRobotStatus& status) const
  {
    status.stamp = ros::Time::now();

    switch(plan_status)
    {

      case flor_planning_msgs::GetMotionPlanForPose::Response::OK:
        //ROS_WARN(" OK status (%d) but failed to plan in %s",plan_status, p_group_name_.c_str());
        status.status = RobotStatusCodes::status(RobotStatusCodes::PLANNER_MOVEIT_PLAN_ACTIVE, RobotStatusCodes::OK);
        break;
      case flor_planning_msgs::GetMotionPlanForPose::Response::OCTOMAP_WARNING:
        ROS_WARN(" Octomap warning (%d)",plan_status);
        status.status = RobotStatusCodes::status(RobotStatusCodes::PLANNER_OCTOMAP_WARNING, RobotStatusCodes::WARNING);
        break;
      case flor_planning_msgs::GetMotionPlanForPose::Response::IK_FAILED:
        ROS_WARN(" IK Failed status (%d)",plan_status);
        status.status = RobotStatusCodes::status(RobotStatusCodes::PLANNER_IK_FAILED, RobotStatusCodes::ERROR);
        break;
      case flor_planning_msgs::GetMotionPlanForPose::Response::PLANNER_EXCEPTION:
        ROS_WARN(" Planner exception status (%d)",plan_status);
        status.status = RobotStatusCodes::status(RobotStatusCodes::PLANNER_EXCEPTION, RobotStatusCodes::ERROR);
        break;
      case flor_planning_msgs::GetMotionPlanForPose::Response::PLANNING_FAILED:
        ROS_WARN(" Planner failed status (%d)",plan_status);
        status.status = RobotStatusCodes::status(RobotStatusCodes::PLANNER_FAILED, RobotStatusCodes::ERROR);
        break;
      case flor_planning_msgs::GetMotionPlanForPose::Response::PLANNING_INVALID_REQUEST:
        ROS_WARN(" PLANNING_INVALID_REQUEST status (%d)",plan_status);
        status.status = RobotStatusCodes::status(RobotStatusCodes::PLANNING_INVALID_REQUEST, RobotStatusCodes::ERROR);
        break;
      case flor_planning_msgs::GetMotionPlanForPose::Response::PLANNING_INVALID_START:
        ROS_WARN(" PLANNING_INVALID_START status (%d)",plan_status);
        status.status = RobotStatusCodes::status(RobotStatusCodes::PLANNING_INVALID_START, RobotStatusCodes::ERROR);
        break;
      case flor_planning_msgs::GetMotionPlanForPose::Response::PLANNING_INVALID_GOAL:
        ROS_WARN(" PLANNING_INVALID_GOAL status (%d)",plan_status);
        status.status = RobotStatusCodes::status(RobotStatusCodes::PLANNING_INVALID_GOAL, RobotStatusCodes::ERROR);
        break;
      case flor_planning_msgs::GetMotionPlanForPose::Response::PLANNING_INVALID_PLAN:
        ROS_WARN(" PLANNING_INVALID_PLAN status (%d)",plan_status);
        status.status = RobotStatusCodes::status(RobotStatusCodes::PLANNING_INVALID_PLAN, RobotStatusCodes::ERROR);
        break;
       default:
        ROS_INFO(" Mixed failure status (0x%x)",plan_status);
        if (plan_status & flor_planning_msgs::GetMotionPlanForPose::Response::PLANNER_EXCEPTION )
        {
          status.status = RobotStatusCodes::status(RobotStatusCodes::PLANNER_EXCEPTION, RobotStatusCodes::ERROR);
        }
        else if (plan_status & flor_planning_msgs::GetMotionPlanForPose::Response::PLANNING_FAILED)
        {
          status.status = RobotStatusCodes::status(RobotStatusCodes::PLANNER_FAILED, RobotStatusCodes::ERROR);
        }
        else if (plan_status & flor_planning_msgs::GetMotionPlanForPose::Response::OCTOMAP_WARNING)
        {
          status.status = RobotStatusCodes::status(RobotStatusCodes::PLANNER_OCTOMAP_WARNING, RobotStatusCodes::WARNING);
        }
        else if (plan_status & flor_planning_msgs::GetMotionPlanForPose::Response::IK_FAILED)
        {
              status.status = RobotStatusCodes::status(RobotStatusCodes::PLANNER_IK_FAILED, RobotStatusCodes::ERROR);
        }
        else
        {
          ROS_ERROR(" Unknown response = %d from planning service",plan_status);
          status.status = RobotStatusCodes::status(RobotStatusCodes::PLANNER_UNKNOWN_FAILURE, RobotStatusCodes::ERROR);
        }
        break;
    }
  }

  template<typename T>
  void waitForAction(const T &action, const ros::Duration &wait_for_server, const std::string &name)
  {
    ros::NodeHandle node_handle_("");
    ROS_DEBUG("Waiting for VigirMoveGroup action server (%s)...", name.c_str());

    // in case ROS time is published, wait for the time data to arrive
    ros::Time start_time = ros::Time::now();
    while (start_time == ros::Time::now())
    {
      ros::WallDuration(0.01).sleep();
      ros::spinOnce();
    }

    // wait for the server (and spin as needed)
    if (wait_for_server == ros::Duration(0, 0))
    {
      while (node_handle_.ok() && !action->isServerConnected())
      {
        ros::WallDuration(0.02).sleep();
        ros::spinOnce();
      }
    }
    else
    {
      ros::Time final_time = ros::Time::now() + wait_for_server;
      while (node_handle_.ok() && !action->isServerConnected() && final_time > ros::Time::now())
      {
        ros::WallDuration(0.02).sleep();
        ros::spinOnce();
      }
    }

    if (!action->isServerConnected())
      throw std::runtime_error("Unable to connect to move_group action server within allotted time (2)");
    else
      ROS_DEBUG("Connected to '%s'", name.c_str());
  }


protected:
  ros::Publisher l_arm_traj_pub_;
  ros::Publisher r_arm_traj_pub_;
  ros::Publisher torso_traj_pub_;
  ros::Publisher neck_traj_pub_;

  ros::Subscriber pose_sub_;
  ros::Subscriber plan_request_sub_;
  ros::Subscriber plan_joint_request_sub_;
  ros::Subscriber plan_circular_request_sub_;
  ros::Subscriber plan_cartesian_request_sub_;

  ros::Subscriber general_trajectory_sub_;
  ros::Subscriber neck_joint_trajectory_sub_;
  ros::Subscriber torso_joint_trajectory_sub_;
  ros::Subscriber l_arm_joint_trajectory_sub_;
  ros::Subscriber r_arm_joint_trajectory_sub_;

  ros::Subscriber plan_execute_sub_;

  ros::Publisher plan_status_pub_;

  // Below should go away in the future, use Actions instead
  ros::Publisher l_grasp_status_pub_;
  ros::Publisher r_grasp_status_pub_;
  ros::Subscriber l_grasp_plan_request_sub_;
  ros::Subscriber r_grasp_plan_request_sub_;

  ros::ServiceClient pose_planning_srv_client_;
  ros::ServiceClient joints_planning_srv_client_;
  ros::ServiceClient circular_planning_srv_client_;
  ros::ServiceClient cartesian_planning_srv_client_;

  boost::scoped_ptr<actionlib::SimpleActionClient<vigir_planning_msgs::MoveAction> > move_action_client_;

  vigir_planning_msgs::MoveGoal goal_;

  ros::Subscriber planner_configuration_sub_;
  vigir_planning_msgs::PlannerConfiguration planner_configuration_;

  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  robot_model::RobotModelPtr robot_model_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "flor_pose_to_plan_node");

  PlanToAction pp;

  ros::spin();
}
