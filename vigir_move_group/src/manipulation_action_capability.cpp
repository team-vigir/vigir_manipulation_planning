/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#include <vigir_move_group/manipulation_action_capability.h>

#include <ctime>
#include <algorithm>

#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/plan_execution/plan_with_sensing.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>


#include <vigir_planning_msgs/RequestWholeBodyTrajectory.h>
#include <vigir_planning_msgs/RequestWholeBodyCartesianTrajectory.h>

move_group::MoveGroupManipulationAction::MoveGroupManipulationAction() :
  MoveGroupCapability("VigirManipulationAction"),
  move_state_(IDLE)
{
}

void move_group::MoveGroupManipulationAction::initialize()
{  
  continuous_plan_execution_.reset(new plan_execution::ContinuousPlanExecution(context_));

  ros::NodeHandle pnh("~/visualization");

  planned_traj_vis_.reset(new trajectory_utils::TrajectoryVisualization(pnh));
  executed_traj_vis_.reset(new trajectory_utils::TrajectoryVisualization(pnh, "eef_traj_executed", 0.0, 0.0, 1.0));


  // start the move action server MOVE_ACTION
  move_action_server_.reset(new actionlib::SimpleActionServer<vigir_planning_msgs::MoveAction>(root_node_handle_, "vigir_move_group",
                                                                                            boost::bind(&MoveGroupManipulationAction::executeMoveCallback, this, _1), false));
  move_action_server_->registerPreemptCallback(boost::bind(&MoveGroupManipulationAction::preemptMoveCallback, this));
  move_action_server_->start();

  drake_trajectory_srv_client_ = root_node_handle_.serviceClient<vigir_planning_msgs::RequestWholeBodyTrajectory>("flor_drake_bridge/request_whole_body_trajectory");
  drake_cartesian_trajectory_srv_client_ = root_node_handle_.serviceClient<vigir_planning_msgs::RequestWholeBodyCartesianTrajectory>("flor_drake_bridge/request_whole_body_cartesian_trajectory");
  drake_trajectory_result_pub_ = root_node_handle_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 10);
}

void move_group::MoveGroupManipulationAction::executeMoveCallback(const vigir_planning_msgs::MoveGoalConstPtr& goal)
{
  setMoveState(PLANNING);
  context_->planning_scene_monitor_->updateFrameTransforms();

  vigir_planning_msgs::MoveResult action_res;

  if (goal->request.planner_id == "drake"){
    // @DRAKE Plan using Drake here. Alternatively, could also implement alternative callback below where @DRAKE is marked
    ROS_WARN("Planning using Drake requested, still work in progress!");

    if (goal->planning_options.plan_only || !context_->allow_trajectory_execution_)
    {
      if (!goal->planning_options.plan_only)
        ROS_WARN("This instance of MoveGroup is not allowed to execute trajectories but the goal request has plan_only set to false. Only a motion plan will be computed anyway.");

      // check for cartesian motion request
      if ( goal->extended_planning_options.target_motion_type == vigir_planning_msgs::ExtendedPlanningOptions::TYPE_CARTESIAN_MOTION)
      {
        executeMoveCallback_DrakeCartesianPlanOnly(goal, action_res);
      }
      else
      {
        executeMoveCallback_DrakePlanOnly(goal, action_res);
      }
    }
    else
    {
      executeMoveCallback_PlanAndExecute(goal, action_res);
    }

    action_res.error_code.val == moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
  }else if (goal->extended_planning_options.target_poses.size() != 0){

    const boost::shared_ptr<tf::Transformer>& tf = context_->planning_scene_monitor_->getTFClient();
    tf->waitForTransform(context_->planning_scene_monitor_->getRobotModel()->getModelFrame(), goal->extended_planning_options.target_frame, ros::Time::now(), ros::Duration(0.5));



  }else{


    if (goal->planning_options.plan_only || !context_->allow_trajectory_execution_)
    {
      if (!goal->planning_options.plan_only)
        ROS_WARN("This instance of MoveGroup is not allowed to execute trajectories but the goal request has plan_only set to false. Only a motion plan will be computed anyway.");
      executeMoveCallback_PlanOnly(goal, action_res);
    }
    else
    {
      executeMoveCallback_PlanAndExecute(goal, action_res);
    }

  }

  bool planned_trajectory_empty = trajectory_processing::isTrajectoryEmpty(action_res.planned_trajectory);
  std::string response = getActionResultString(action_res.error_code, planned_trajectory_empty, goal->planning_options.plan_only);
  if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    move_action_server_->setSucceeded(action_res, response);
  else
  {
    if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
      move_action_server_->setPreempted(action_res, response);
    else
      move_action_server_->setAborted(action_res, response);
  }

  setMoveState(IDLE);
}

void move_group::MoveGroupManipulationAction::executeMoveCallback_PlanAndExecute(const vigir_planning_msgs::MoveGoalConstPtr& goal, vigir_planning_msgs::MoveResult &action_res)
{
  ROS_INFO("Combined planning and execution request received for MoveGroup action. Forwarding to planning and execution pipeline.");

  if (planning_scene::PlanningScene::isEmpty(goal->planning_options.planning_scene_diff))
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(context_->planning_scene_monitor_);
    const robot_state::RobotState &current_state = lscene->getCurrentState();

    // check to see if the desired constraints are already met
    for (std::size_t i = 0 ; i < goal->request.goal_constraints.size() ; ++i)
      if (lscene->isStateConstrained(current_state, kinematic_constraints::mergeConstraints(goal->request.goal_constraints[i],
                                                                                            goal->request.path_constraints)))
      {
        ROS_INFO("Goal constraints are already satisfied. No need to plan or execute any motions");
        action_res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
        return;
      }
  }

  plan_execution::PlanExecution::Options opt;

  const moveit_msgs::MotionPlanRequest &motion_plan_request = planning_scene::PlanningScene::isEmpty(goal->request.start_state) ?
    goal->request : clearRequestStartState(goal->request);
  const moveit_msgs::PlanningScene &planning_scene_diff = planning_scene::PlanningScene::isEmpty(goal->planning_options.planning_scene_diff.robot_state) ?
    goal->planning_options.planning_scene_diff : clearSceneRobotState(goal->planning_options.planning_scene_diff);

  opt.replan_ = goal->planning_options.replan;
  opt.replan_attempts_ = goal->planning_options.replan_attempts;
  opt.replan_delay_ = goal->planning_options.replan_delay;
  opt.before_execution_callback_ = boost::bind(&MoveGroupManipulationAction::startMoveExecutionCallback, this);

  if (goal->extended_planning_options.continuous_replanning){
    ROS_WARN("Continuous replanning not integrated yet!");
  }else{    
    if (goal->request.planner_id == "drake") // plan using drake
    {
        // check if it is a cartesian request
        if ( goal->extended_planning_options.target_motion_type == vigir_planning_msgs::ExtendedPlanningOptions::TYPE_CARTESIAN_MOTION)
        {
            opt.plan_callback_ = boost::bind(&MoveGroupManipulationAction::planCartesianUsingDrake, this, boost::cref(goal), _1);
        }
        else { // normal joint-level planning
            opt.plan_callback_ = boost::bind(&MoveGroupManipulationAction::planUsingDrake, this, boost::cref(goal), _1);
        }
    }
    else {
        opt.plan_callback_ = boost::bind(&MoveGroupManipulationAction::planUsingPlanningPipeline, this, boost::cref(motion_plan_request), _1);
        
        if (goal->planning_options.look_around && context_->plan_with_sensing_)
        {
          opt.plan_callback_ = boost::bind(&plan_execution::PlanWithSensing::computePlan, context_->plan_with_sensing_.get(), _1, opt.plan_callback_, goal->planning_options.look_around_attempts, goal->planning_options.max_safe_execution_cost);
          context_->plan_with_sensing_->setBeforeLookCallback(boost::bind(&MoveGroupManipulationAction::startMoveLookCallback, this));
        }
    }
    
    plan_execution::ExecutableMotionPlan plan;
    context_->plan_execution_->planAndExecute(plan, planning_scene_diff, opt);


    //@TODO: We only consider first plan for visualization at the moment
    //if (plan.plan_components_.size() > 0){
    //  planned_traj_vis_->publishTrajectoryEndeffectorVis(*plan.plan_components_[0].trajectory_);
    //}

    convertToMsg(plan.plan_components_, action_res.trajectory_start, action_res.planned_trajectory);
    if (plan.executed_trajectory_){
      plan.executed_trajectory_->getRobotTrajectoryMsg(action_res.executed_trajectory);
      executed_traj_vis_->publishTrajectoryEndeffectorVis(*plan.executed_trajectory_);
    }
    action_res.error_code = plan.error_code_;
  }
}

void move_group::MoveGroupManipulationAction::executeMoveCallback_PlanOnly(const vigir_planning_msgs::MoveGoalConstPtr& goal, vigir_planning_msgs::MoveResult &action_res)
{
  ROS_INFO("Planning request received for MoveGroup action. Forwarding to planning pipeline.");

  planning_scene_monitor::LockedPlanningSceneRO lscene(context_->planning_scene_monitor_); // lock the scene so that it does not modify the world representation while diff() is called
  const planning_scene::PlanningSceneConstPtr &the_scene = (planning_scene::PlanningScene::isEmpty(goal->planning_options.planning_scene_diff)) ?
    static_cast<const planning_scene::PlanningSceneConstPtr&>(lscene) : lscene->diff(goal->planning_options.planning_scene_diff);
  planning_interface::MotionPlanResponse res;
  try
  {
    context_->planning_pipeline_->generatePlan(the_scene, goal->request, res);
  }
  catch(std::runtime_error &ex)
  {
    ROS_ERROR("Planning pipeline threw an exception: %s", ex.what());
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }
  catch(...)
  {
    ROS_ERROR("Planning pipeline threw an exception");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }

  convertToMsg(res.trajectory_, action_res.trajectory_start, action_res.planned_trajectory);

  if (res.trajectory_)
    planned_traj_vis_->publishTrajectoryEndeffectorVis(*res.trajectory_);

  action_res.error_code = res.error_code_;
  action_res.planning_time = res.planning_time_;
}

void move_group::MoveGroupManipulationAction::executeMoveCallback_DrakePlanOnly(const vigir_planning_msgs::MoveGoalConstPtr& goal, vigir_planning_msgs::MoveResult &action_res)
{
  ROS_INFO("Planning request received for MoveGroup action. Forwarding to Drake.");

  planning_scene_monitor::LockedPlanningSceneRO lscene(context_->planning_scene_monitor_); // lock the scene so that it does not modify the world representation while diff() is called
  const planning_scene::PlanningSceneConstPtr &the_scene = (planning_scene::PlanningScene::isEmpty(goal->planning_options.planning_scene_diff)) ?
    static_cast<const planning_scene::PlanningSceneConstPtr&>(lscene) : lscene->diff(goal->planning_options.planning_scene_diff);
  planning_interface::MotionPlanResponse res;

  const robot_model::RobotModelConstPtr& robot_model = context_->planning_pipeline_->getRobotModel();
  const planning_scene::PlanningScenePtr& planning_scene = context_->planning_scene_monitor_->getPlanningScene();
  robot_state::RobotState current_robot_state = planning_scene->getCurrentState();
  moveit_msgs::RobotState current_state_msg;
  moveit::core::robotStateToRobotStateMsg(current_robot_state, current_state_msg);

  vigir_planning_msgs::RequestWholeBodyTrajectory::Response drake_response_msg;

  try
  {
      //Everything OK in the beginning, this will be changed below if we encounter problems
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

      // build request message
      vigir_planning_msgs::RequestWholeBodyTrajectory::Request drake_request_msg;
      drake_request_msg.trajectory_request.current_state = current_state_msg;

      if ( goal->extended_planning_options.target_pose_times.size() > 0 ) {
        drake_request_msg.trajectory_request.duration = goal->extended_planning_options.target_pose_times[0];
      }

      drake_request_msg.trajectory_request.trajectory_sample_rate = goal->extended_planning_options.trajectory_sample_rate;
      drake_request_msg.trajectory_request.motion_plan_request = goal->request;

      // call service and process response
      struct timeval start_time;
      gettimeofday(&start_time, NULL);

      bool solved = drake_trajectory_srv_client_.call(drake_request_msg, drake_response_msg);

      if ( solved ) {
        res.trajectory_ = robot_trajectory::RobotTrajectoryPtr(new robot_trajectory::RobotTrajectory(robot_model, goal->request.group_name));
        res.trajectory_->setRobotTrajectoryMsg(current_robot_state, drake_response_msg.trajectory_result.result_trajectory);

        struct timeval end_time, diff;
        gettimeofday(&end_time, NULL);
        timersub(&end_time, &start_time, &diff);
        res.planning_time_ = (double)diff.tv_sec + (double)diff.tv_usec/1000000.0;
      }
      else {
          res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
      }
  }
  catch(std::runtime_error &ex)
  {
    ROS_ERROR("Planning pipeline threw an exception: %s", ex.what());
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }
  catch(...)
  {
    ROS_ERROR("Planning pipeline threw an exception");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }

  convertToMsg(res.trajectory_, action_res.trajectory_start, action_res.planned_trajectory);

  if (res.trajectory_) {
    planned_traj_vis_->publishTrajectoryEndeffectorVis(*res.trajectory_);

    moveit_msgs::DisplayTrajectory result_trajectory_display_msg;
    result_trajectory_display_msg.trajectory.push_back( drake_response_msg.trajectory_result.result_trajectory );
    result_trajectory_display_msg.trajectory_start = current_state_msg;
    result_trajectory_display_msg.model_id = robot_model->getName();
    drake_trajectory_result_pub_.publish(result_trajectory_display_msg);
  }

  action_res.error_code = res.error_code_;
  action_res.planning_time = res.planning_time_;
}


void move_group::MoveGroupManipulationAction::executeMoveCallback_DrakeCartesianPlanOnly(const vigir_planning_msgs::MoveGoalConstPtr& goal, vigir_planning_msgs::MoveResult &action_res)
{
  ROS_INFO("Planning request received for MoveGroup action. Forwarding to Drake.");

  planning_scene_monitor::LockedPlanningSceneRO lscene(context_->planning_scene_monitor_); // lock the scene so that it does not modify the world representation while diff() is called
  const planning_scene::PlanningSceneConstPtr &the_scene = (planning_scene::PlanningScene::isEmpty(goal->planning_options.planning_scene_diff)) ?
    static_cast<const planning_scene::PlanningSceneConstPtr&>(lscene) : lscene->diff(goal->planning_options.planning_scene_diff);
  planning_interface::MotionPlanResponse res;

  vigir_planning_msgs::RequestWholeBodyCartesianTrajectory::Response drake_response_msg;

  // get current robot state and model
  const robot_model::RobotModelConstPtr& robot_model = context_->planning_pipeline_->getRobotModel();  
  const planning_scene::PlanningScenePtr& planning_scene = context_->planning_scene_monitor_->getPlanningScene();
  robot_state::RobotState current_robot_state = planning_scene->getCurrentState();
  moveit_msgs::RobotState current_state_msg;
  moveit::core::robotStateToRobotStateMsg(current_robot_state, current_state_msg);



  // if global world position is not set, try to get robot orientation from tf transform (THOR!)
  bool has_world_virtual_joint = ( std::find( current_state_msg.multi_dof_joint_state.joint_names.begin(), current_state_msg.multi_dof_joint_state.joint_names.end(), "world_virtual_joint" ) != current_state_msg.multi_dof_joint_state.joint_names.end() );
  if ( has_world_virtual_joint == false )
  {
      try{
        ROS_INFO("No world virtual joint given - using tf-transform");
        tf::StampedTransform pelvis_tf;        
        transform_listener_.lookupTransform("/world", "/pelvis", ros::Time(0), pelvis_tf);

        geometry_msgs::Transform pelvis_pose_msg;
        tf::transformTFToMsg(pelvis_tf, pelvis_pose_msg);

        current_state_msg.multi_dof_joint_state.joint_names.push_back("world_virtual_joint");
        current_state_msg.multi_dof_joint_state.transforms.push_back(pelvis_pose_msg);
      }
      catch (tf::TransformException &ex) {
        ROS_WARN("%s",ex.what());
      }
  }

  try
  {
      //Everything OK in the beginning, this will be changed below if we encounter problems
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

      // build request message
      const moveit::core::JointModelGroup *joint_model_group = current_robot_state.getJointModelGroup(goal->request.group_name);

      vigir_planning_msgs::RequestWholeBodyCartesianTrajectory::Request drake_request_msg;
      drake_request_msg.trajectory_request.current_state = current_state_msg;
      drake_request_msg.trajectory_request.waypoints = goal->extended_planning_options.target_poses;
      drake_request_msg.trajectory_request.waypoint_times = goal->extended_planning_options.target_pose_times;
      drake_request_msg.trajectory_request.target_link_names = goal->extended_planning_options.target_link_names;
      drake_request_msg.trajectory_request.free_joint_names = joint_model_group->getJointModelNames();
      drake_request_msg.trajectory_request.target_orientation_type = goal->extended_planning_options.target_orientation_type;
      drake_request_msg.trajectory_request.trajectory_sample_rate = goal->extended_planning_options.trajectory_sample_rate;
      drake_request_msg.trajectory_request.check_self_collisions = goal->extended_planning_options.check_self_collisions;

      // call service and process response
      struct timeval start_time;
      gettimeofday(&start_time, NULL);

      bool solved = drake_cartesian_trajectory_srv_client_.call(drake_request_msg, drake_response_msg);

      if ( solved ) {

        // if the robot model has no world joint, remove info from result trajectory
        if ( has_world_virtual_joint == false ) {
            drake_response_msg.trajectory_result.result_trajectory.multi_dof_joint_trajectory.joint_names.clear();
            drake_response_msg.trajectory_result.result_trajectory.multi_dof_joint_trajectory.points.clear();
        }

        res.trajectory_ = robot_trajectory::RobotTrajectoryPtr(new robot_trajectory::RobotTrajectory(robot_model, goal->request.group_name));
        res.trajectory_->setRobotTrajectoryMsg(current_robot_state, drake_response_msg.trajectory_result.result_trajectory);

        struct timeval end_time, diff;
        gettimeofday(&end_time, NULL);
        timersub(&end_time, &start_time, &diff);
        res.planning_time_ = (double)diff.tv_sec + (double)diff.tv_usec/1000000.0;
      }
      else {
          res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
      }
  }
  catch(std::runtime_error &ex)
  {
    ROS_ERROR("Planning pipeline threw an exception: %s", ex.what());
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }
  catch(...)
  {
    ROS_ERROR("Planning pipeline threw an exception");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }

  convertToMsg(res.trajectory_, action_res.trajectory_start, action_res.planned_trajectory);

  if (res.trajectory_) {
    planned_traj_vis_->publishTrajectoryEndeffectorVis(*res.trajectory_);

    moveit_msgs::DisplayTrajectory result_trajectory_display_msg;
    result_trajectory_display_msg.trajectory.push_back( drake_response_msg.trajectory_result.result_trajectory );
    result_trajectory_display_msg.trajectory_start = current_state_msg;
    result_trajectory_display_msg.model_id = robot_model->getName();
    drake_trajectory_result_pub_.publish(result_trajectory_display_msg);
  }

  action_res.error_code = res.error_code_;
  action_res.planning_time = res.planning_time_;
}

void move_group::MoveGroupManipulationAction::executeCartesianMoveCallback_PlanAndExecute(const vigir_planning_msgs::MoveGoalConstPtr& goal, vigir_planning_msgs::MoveResult &action_res)
{
  if (goal->extended_planning_options.target_motion_type == vigir_planning_msgs::ExtendedPlanningOptions::TYPE_CARTESIAN_WAYPOINTS){
    ROS_WARN("Cartesian waypoints not implemented yet!");

  }else if (goal->extended_planning_options.target_motion_type == vigir_planning_msgs::ExtendedPlanningOptions::TYPE_CIRCULAR_MOTION){
    ROS_WARN("Circular waypoints not implemented yet!");
  }
}

bool move_group::MoveGroupManipulationAction::planUsingPlanningPipeline(const planning_interface::MotionPlanRequest &req, plan_execution::ExecutableMotionPlan &plan)
{

    setMoveState(PLANNING);

    planning_scene_monitor::LockedPlanningSceneRO lscene(plan.planning_scene_monitor_);
    bool solved = false;
    planning_interface::MotionPlanResponse res;
    try
    {
      solved = context_->planning_pipeline_->generatePlan(plan.planning_scene_, req, res);
    }
    catch(std::runtime_error &ex)
    {
      ROS_ERROR("Planning pipeline threw an exception: %s", ex.what());
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    }
    catch(...)
    {
      ROS_ERROR("Planning pipeline threw an exception");
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    }
    if (res.trajectory_)
    {
      plan.plan_components_.resize(1);
      plan.plan_components_[0].trajectory_ = res.trajectory_;
      plan.plan_components_[0].description_ = "plan";

      planned_traj_vis_->publishTrajectoryEndeffectorVis(*plan.plan_components_[0].trajectory_);
    }
    plan.error_code_ = res.error_code_;
    return solved;
}


bool move_group::MoveGroupManipulationAction::planUsingDrake(const vigir_planning_msgs::MoveGoalConstPtr& goal, plan_execution::ExecutableMotionPlan &plan)
{
    setMoveState(PLANNING);

    planning_scene_monitor::LockedPlanningSceneRO lscene(plan.planning_scene_monitor_);
    bool solved = false;
    planning_interface::MotionPlanResponse res;

    // get current robot state and model
    const robot_model::RobotModelConstPtr& robot_model = context_->planning_pipeline_->getRobotModel();
    const planning_scene::PlanningScenePtr& planning_scene = context_->planning_scene_monitor_->getPlanningScene();
    robot_state::RobotState current_robot_state = planning_scene->getCurrentState();
    moveit_msgs::RobotState current_state_msg;
    moveit::core::robotStateToRobotStateMsg(current_robot_state, current_state_msg);

    vigir_planning_msgs::RequestWholeBodyTrajectory::Response drake_response_msg;

    try
    {
        //Everything OK in the beginning, this will be changed below if we encounter problems
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

        // build request message
        vigir_planning_msgs::RequestWholeBodyTrajectory::Request drake_request_msg;
        drake_request_msg.trajectory_request.current_state = current_state_msg;

        if ( goal->extended_planning_options.target_pose_times.size() > 0 )
            drake_request_msg.trajectory_request.duration = goal->extended_planning_options.target_pose_times[0];

        drake_request_msg.trajectory_request.trajectory_sample_rate = goal->extended_planning_options.trajectory_sample_rate;
        drake_request_msg.trajectory_request.check_self_collisions = goal->extended_planning_options.check_self_collisions;
        drake_request_msg.trajectory_request.motion_plan_request = goal->request;

        // call service and process response
        struct timeval start_time;
        gettimeofday(&start_time, NULL);

        bool solved = drake_trajectory_srv_client_.call(drake_request_msg, drake_response_msg);

        if ( solved ) {
          res.trajectory_ = robot_trajectory::RobotTrajectoryPtr(new robot_trajectory::RobotTrajectory(robot_model, goal->request.group_name));
          res.trajectory_->setRobotTrajectoryMsg(current_robot_state, drake_response_msg.trajectory_result.result_trajectory);

          struct timeval end_time, diff;
          gettimeofday(&end_time, NULL);
          timersub(&end_time, &start_time, &diff);
          res.planning_time_ = (double)diff.tv_sec + (double)diff.tv_usec/1000000.0;
        }
        else {
            res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        }
    }
    catch(std::runtime_error &ex)
    {
      ROS_ERROR("Planning pipeline threw an exception: %s", ex.what());
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    }
    catch(...)
    {
      ROS_ERROR("Planning pipeline threw an exception");
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    }
    if (res.trajectory_)
    {
      plan.plan_components_.resize(1);
      plan.plan_components_[0].trajectory_ = res.trajectory_;
      plan.plan_components_[0].description_ = "plan";

      planned_traj_vis_->publishTrajectoryEndeffectorVis(*plan.plan_components_[0].trajectory_);

      // display preview in rviz
      moveit_msgs::DisplayTrajectory result_trajectory_display_msg;
      result_trajectory_display_msg.trajectory.push_back( drake_response_msg.trajectory_result.result_trajectory );
      result_trajectory_display_msg.trajectory_start = current_state_msg;
      result_trajectory_display_msg.model_id = robot_model->getName();
      drake_trajectory_result_pub_.publish(result_trajectory_display_msg);
    }
    plan.error_code_ = res.error_code_;
    return solved;
}

bool move_group::MoveGroupManipulationAction::planCartesianUsingDrake(const vigir_planning_msgs::MoveGoalConstPtr& goal, plan_execution::ExecutableMotionPlan &plan)
{
    setMoveState(PLANNING);

    planning_scene_monitor::LockedPlanningSceneRO lscene(plan.planning_scene_monitor_);
    bool solved = false;
    planning_interface::MotionPlanResponse res;
    vigir_planning_msgs::RequestWholeBodyTrajectory::Response drake_response_msg;

    const robot_model::RobotModelConstPtr& robot_model = context_->planning_pipeline_->getRobotModel();

    const planning_scene::PlanningScenePtr& planning_scene = context_->planning_scene_monitor_->getPlanningScene();
    robot_state::RobotState current_robot_state = planning_scene->getCurrentState();
    const moveit::core::JointModelGroup *joint_model_group = current_robot_state.getJointModelGroup(goal->request.group_name);

    moveit_msgs::RobotState current_state_msg;
    moveit::core::robotStateToRobotStateMsg(current_robot_state, current_state_msg);

    // if global world position is not set, try to get robot orientation from tf transform (THOR!)
    bool has_world_virtual_joint = ( std::find( current_state_msg.multi_dof_joint_state.joint_names.begin(), current_state_msg.multi_dof_joint_state.joint_names.end(), "world_virtual_joint" ) != current_state_msg.multi_dof_joint_state.joint_names.end() );
    if ( has_world_virtual_joint == false )
    {
        try{
          ROS_INFO("No world virtual joint given - using tf-transform");
          tf::StampedTransform pelvis_tf;
          transform_listener_.lookupTransform("/world", "/pelvis", ros::Time(0), pelvis_tf);

          geometry_msgs::Transform pelvis_pose_msg;
          tf::transformTFToMsg(pelvis_tf, pelvis_pose_msg);

          current_state_msg.multi_dof_joint_state.joint_names.push_back("world_virtual_joint");
          current_state_msg.multi_dof_joint_state.transforms.push_back(pelvis_pose_msg);
        }
        catch (tf::TransformException &ex) {
          ROS_WARN("%s",ex.what());
        }
    }

    try
    {
        //Everything OK in the beginning, this will be changed below if we encounter problems
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

        // build request message
        vigir_planning_msgs::RequestWholeBodyCartesianTrajectory::Request drake_request_msg;
        drake_request_msg.trajectory_request.current_state = current_state_msg;
        drake_request_msg.trajectory_request.waypoints = goal->extended_planning_options.target_poses;
        drake_request_msg.trajectory_request.waypoint_times = goal->extended_planning_options.target_pose_times;
        drake_request_msg.trajectory_request.target_link_names = goal->extended_planning_options.target_link_names;
        drake_request_msg.trajectory_request.free_joint_names = joint_model_group->getJointModelNames();
        drake_request_msg.trajectory_request.target_orientation_type = goal->extended_planning_options.target_orientation_type;
        drake_request_msg.trajectory_request.trajectory_sample_rate = goal->extended_planning_options.trajectory_sample_rate;
        drake_request_msg.trajectory_request.check_self_collisions = goal->extended_planning_options.check_self_collisions;

        // call service and process response
        struct timeval start_time;
        gettimeofday(&start_time, NULL);

        bool solved = drake_cartesian_trajectory_srv_client_.call(drake_request_msg, drake_response_msg);

        if ( solved ) {
          // if the robot model has no world joint, remove info from result trajectory
          if ( has_world_virtual_joint == false ) {
              drake_response_msg.trajectory_result.result_trajectory.multi_dof_joint_trajectory.joint_names.clear();
              drake_response_msg.trajectory_result.result_trajectory.multi_dof_joint_trajectory.points.clear();
          }

          res.trajectory_ = robot_trajectory::RobotTrajectoryPtr(new robot_trajectory::RobotTrajectory(robot_model, goal->request.group_name));
          res.trajectory_->setRobotTrajectoryMsg(current_robot_state, drake_response_msg.trajectory_result.result_trajectory);

          struct timeval end_time, diff;
          gettimeofday(&end_time, NULL);
          timersub(&end_time, &start_time, &diff);
          res.planning_time_ = (double)diff.tv_sec + (double)diff.tv_usec/1000000.0;
        }
        else {
            res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        }
    }
    catch(std::runtime_error &ex)
    {
      ROS_ERROR("Planning pipeline threw an exception: %s", ex.what());
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    }
    catch(...)
    {
      ROS_ERROR("Planning pipeline threw an exception");
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    }
    if (res.trajectory_)
    {
      plan.plan_components_.resize(1);
      plan.plan_components_[0].trajectory_ = res.trajectory_;
      plan.plan_components_[0].description_ = "plan";

      planned_traj_vis_->publishTrajectoryEndeffectorVis(*plan.plan_components_[0].trajectory_);

      // display preview in rviz
      moveit_msgs::DisplayTrajectory result_trajectory_display_msg;
      result_trajectory_display_msg.trajectory.push_back( drake_response_msg.trajectory_result.result_trajectory );
      result_trajectory_display_msg.trajectory_start = current_state_msg;
      result_trajectory_display_msg.model_id = robot_model->getName();
      drake_trajectory_result_pub_.publish(result_trajectory_display_msg);
    }
    plan.error_code_ = res.error_code_;
    return solved;
}

void move_group::MoveGroupManipulationAction::startMoveExecutionCallback()
{
  setMoveState(MONITOR);
}

void move_group::MoveGroupManipulationAction::startMoveLookCallback()
{
  setMoveState(LOOK);
}

void move_group::MoveGroupManipulationAction::preemptMoveCallback()
{
  context_->plan_execution_->stop();
}

void move_group::MoveGroupManipulationAction::setMoveState(MoveGroupState state)
{
  move_state_ = state;
  move_feedback_.state = stateToStr(state);
  move_action_server_->publishFeedback(move_feedback_);
}

#include <class_loader/class_loader.h>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupManipulationAction, move_group::MoveGroupCapability)
