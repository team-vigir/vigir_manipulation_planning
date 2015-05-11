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

#include <vigir_moveit_utils/constrained_motion_utils.h>
#include <vigir_moveit_utils/joint_constraint_utils.h>
#include <vigir_moveit_utils/group_utils.h>
#include <vigir_moveit_utils/planning_scene_utils.h>


namespace
{
bool isStateValid(const planning_scene::PlanningScene *planning_scene,
                  const kinematic_constraints::KinematicConstraintSet *constraint_set,
                  robot_state::RobotState *state,
                  const robot_state::JointModelGroup *group, const double *ik_solution)
{
  state->setJointGroupPositions(group, ik_solution);
  state->update();
  return (!planning_scene || !planning_scene->isStateColliding(*state, group->getName())) &&
    (!constraint_set || constraint_set->decide(*state).satisfied);
}
}



move_group::MoveGroupManipulationAction::MoveGroupManipulationAction() :
  MoveGroupCapability("VigirManipulationAction"),
  move_state_(IDLE)
{
  time_param_.reset(new trajectory_processing::IterativeParabolicTimeParameterization());
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

  drake_trajectory_srv_client_ = root_node_handle_.serviceClient<vigir_planning_msgs::RequestWholeBodyTrajectory>("drake_planner/request_whole_body_trajectory");
  drake_cartesian_trajectory_srv_client_ = root_node_handle_.serviceClient<vigir_planning_msgs::RequestWholeBodyCartesianTrajectory>("drake_planner/request_whole_body_cartesian_trajectory");
  trajectory_result_display_pub_ = root_node_handle_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 10);
}

void move_group::MoveGroupManipulationAction::executeMoveCallback(const vigir_planning_msgs::MoveGoalConstPtr& goal)
{
  setMoveState(PLANNING);
  context_->planning_scene_monitor_->updateFrameTransforms();

  vigir_planning_msgs::MoveResult action_res;

  if (goal->request.planner_id == "drake"){
    // @DRAKE Plan using Drake here. Alternatively, could also implement alternative callback below where @DRAKE is marked

    if (goal->planning_options.plan_only || !context_->allow_trajectory_execution_)
    {
      if (!goal->planning_options.plan_only)
        ROS_WARN("This instance of MoveGroup is not allowed to execute trajectories but the goal request has plan_only set to false. Only a motion plan will be computed anyway.");

      // check for cartesian motion request
      if ( goal->extended_planning_options.target_motion_type == vigir_planning_msgs::ExtendedPlanningOptions::TYPE_CARTESIAN_WAYPOINTS)
      {
        executeMoveCallback_DrakeCartesianPlanOnly(goal, action_res);
      }
      else if ( goal->extended_planning_options.target_motion_type == vigir_planning_msgs::ExtendedPlanningOptions::TYPE_FREE_MOTION)
      {
        executeMoveCallback_DrakePlanOnly(goal, action_res);
      }
      else if ( goal->extended_planning_options.target_motion_type == vigir_planning_msgs::ExtendedPlanningOptions::TYPE_CIRCULAR_MOTION)
      {
          executeMoveCallback_DrakeCircularMotionPlanOnly(goal, action_res);
      }
      else {
        ROS_WARN("Motion request type %d not implemented for Drake!", goal->extended_planning_options.target_motion_type);
    action_res.error_code.val == moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
      }
    }
    else
    {
      executeMoveCallback_PlanAndExecute(goal, action_res);
    }

    action_res.error_code.val == moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
  }
  
  // Below if not using Drake and not using copy of standard MoveIt! Action
  else if (goal->extended_planning_options.target_poses.size() != 0){


    if (goal->extended_planning_options.target_motion_type == vigir_planning_msgs::ExtendedPlanningOptions::TYPE_FREE_MOTION){

      // For free motion, do IK and plan.
      // Consider additional joint constraints/redundant joints

      moveit_msgs::Constraints goal_constraints;
      bool found_ik = false;

      {
        planning_scene_monitor::LockedPlanningSceneRO lscene(context_->planning_scene_monitor_);
        robot_state::RobotState tmp = lscene->getCurrentState();

        const robot_state::JointModelGroup* joint_model_group = tmp.getJointModelGroup(goal->request.group_name);

        found_ik = group_utils::setJointModelGroupFromIk(tmp,
                                                         joint_model_group,
                                                         goal->extended_planning_options.target_poses[0],
                                                         goal->request.path_constraints.joint_constraints);

        if (found_ik){
          goal_constraints = kinematic_constraints::constructGoalConstraints(tmp, joint_model_group);
        }
      }

      if (found_ik){
        vigir_planning_msgs::MoveGoalPtr updated_goal;
        updated_goal.reset(new vigir_planning_msgs::MoveGoal());
        *updated_goal = *goal;

        updated_goal->request.goal_constraints.push_back(goal_constraints);

        if (goal->planning_options.plan_only || !context_->allow_trajectory_execution_)
        {
          if (!goal->planning_options.plan_only)
            ROS_WARN("This instance of MoveGroup is not allowed to execute trajectories but the goal request has plan_only set to false. Only a motion plan will be computed anyway.");
          executeMoveCallback_PlanOnly(updated_goal, action_res);
        }
        else
        {
          executeMoveCallback_PlanAndExecute(updated_goal, action_res);
        }
      }else{
        ROS_WARN("No valid IK solution found, cannot generate goal constraints!");
        action_res.error_code.val == moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
      }



    }else{
      //Otherwise, perform cartesian motion
      executeCartesianMoveCallback_PlanAndExecute(goal, action_res);
    }

  // Below forwards to standard MoveIt Action
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
        if ( goal->extended_planning_options.target_motion_type == vigir_planning_msgs::ExtendedPlanningOptions::TYPE_CARTESIAN_WAYPOINTS)
        {
            opt.plan_callback_ = boost::bind(&MoveGroupManipulationAction::planCartesianUsingDrake, this, boost::cref(goal), _1);
        }
        else if ( goal->extended_planning_options.target_motion_type == vigir_planning_msgs::ExtendedPlanningOptions::TYPE_CIRCULAR_MOTION)
        {
            opt.plan_callback_ = boost::bind(&MoveGroupManipulationAction::planCircularMotionUsingDrake, this, boost::cref(goal), _1);
        }
        else if ( goal->extended_planning_options.target_motion_type == vigir_planning_msgs::ExtendedPlanningOptions::TYPE_FREE_MOTION) { // normal joint-level planning
            opt.plan_callback_ = boost::bind(&MoveGroupManipulationAction::planUsingDrake, this, boost::cref(goal), _1);
        }
        else {
            ROS_WARN("Selected motion type %d not implemented for Drake!", goal->extended_planning_options.target_motion_type);
        }
    }
    else {
    opt.plan_callback_ = boost::bind(&MoveGroupManipulationAction::planUsingPlanningPipeline, this, boost::cref(motion_plan_request), _1);

    //We normally don't plan with lookaround so the below can be ignored
    if (goal->planning_options.look_around && context_->plan_with_sensing_)
    {
      opt.plan_callback_ = boost::bind(&plan_execution::PlanWithSensing::computePlan, context_->plan_with_sensing_.get(), _1, opt.plan_callback_,
                                       goal->planning_options.look_around_attempts, goal->planning_options.max_safe_execution_cost);
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

    if (trajectory_result_display_pub_.getNumSubscribers() > 0){
      moveit_msgs::DisplayTrajectory result_trajectory_display_msg;
      result_trajectory_display_msg.trajectory.push_back( drake_response_msg.trajectory_result.result_trajectory );
      result_trajectory_display_msg.trajectory_start = current_state_msg;
      result_trajectory_display_msg.model_id = robot_model->getName();
      trajectory_result_display_pub_.publish(result_trajectory_display_msg);
    }
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
      drake_request_msg.trajectory_request.target_link_axis = goal->extended_planning_options.target_link_axis;
      drake_request_msg.trajectory_request.free_joint_names = joint_model_group->getJointModelNames();
      drake_request_msg.trajectory_request.target_orientation_type = goal->extended_planning_options.target_orientation_type;
      drake_request_msg.trajectory_request.trajectory_sample_rate = goal->extended_planning_options.trajectory_sample_rate;
      drake_request_msg.trajectory_request.check_self_collisions = goal->extended_planning_options.check_self_collisions;
      drake_request_msg.trajectory_request.execute_incomplete_cartesian_plans = goal->extended_planning_options.execute_incomplete_cartesian_plans;

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

    if (trajectory_result_display_pub_.getNumSubscribers() > 0){
      moveit_msgs::DisplayTrajectory result_trajectory_display_msg;
      result_trajectory_display_msg.trajectory.push_back( drake_response_msg.trajectory_result.result_trajectory );
      result_trajectory_display_msg.trajectory_start = current_state_msg;
      result_trajectory_display_msg.model_id = robot_model->getName();
      trajectory_result_display_pub_.publish(result_trajectory_display_msg);
    }
  }

  action_res.error_code = res.error_code_;
  action_res.planning_time = res.planning_time_;
}

void move_group::MoveGroupManipulationAction::executeMoveCallback_DrakeCircularMotionPlanOnly(const vigir_planning_msgs::MoveGoalConstPtr& goal, vigir_planning_msgs::MoveResult &action_res)
{
  ROS_INFO("Received circular cartesian motion request! Forwarding to Drake");

  try
  {
      //Everything OK in the beginning, this will be changed below if we encounter problems
      action_res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

      if (goal->extended_planning_options.target_poses.size() != 1){
        ROS_ERROR("There has to be exactly one target pose for circular motion requests!");
        return;
      }

      //Only used if keep endeffector orientation true or if circular motion requested
      Eigen::Affine3d eef_start_pose;

      if((goal->extended_planning_options.target_motion_type == vigir_planning_msgs::ExtendedPlanningOptions::TYPE_CIRCULAR_MOTION ||
         (goal->extended_planning_options.target_motion_type == vigir_planning_msgs::ExtendedPlanningOptions::TYPE_CARTESIAN_WAYPOINTS &&
          goal->extended_planning_options.keep_endeffector_orientation)) &&
         !planning_scene_utils::getEndeffectorTransform(goal->request.group_name,
                                                        context_->planning_scene_monitor_,
                                                        eef_start_pose))
      {
        ROS_ERROR("Cannot get endeffector transform, cartesian planning not possible!");
        action_res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        return;
      }

      std::string eef_link_name;
      if( ! planning_scene_utils::get_eef_link(goal->request.group_name, eef_link_name)) {
        ROS_ERROR("Cannot get endeffector link name, circular planning not possible!");
        action_res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        return;
      }

      geometry_msgs::PoseStamped rotation_pose;
      rotation_pose.pose = goal->extended_planning_options.target_poses[0];
      rotation_pose.header.frame_id = goal->extended_planning_options.target_frame;

      //Can easily transform goal pose to arbitrary target frame
      this->performTransform(rotation_pose, context_->planning_scene_monitor_->getRobotModel()->getModelFrame());


      Eigen::Affine3d rotation_center;
      tf::poseMsgToEigen(rotation_pose.pose, rotation_center);

      std::vector <geometry_msgs::Pose> pose_vec;
      constrained_motion_utils::getCircularArcPoses(rotation_center,
                                                      eef_start_pose,
                                                      pose_vec,
                                                      0.2,
                                                      goal->extended_planning_options.rotation_angle,
                                                      goal->extended_planning_options.keep_endeffector_orientation);

      // make a copy of goal, so I can modify it
      vigir_planning_msgs::MoveGoalPtr new_goal( new vigir_planning_msgs::MoveGoal( *goal ) );

      new_goal->extended_planning_options.target_poses = pose_vec;
      new_goal->extended_planning_options.target_link_names.assign(pose_vec.size(), eef_link_name);
      if ( new_goal->extended_planning_options.target_link_axis.empty() == false ) {
          new_goal->extended_planning_options.target_link_axis.assign(pose_vec.size(), goal->extended_planning_options.target_link_axis[0]);
      }

      // call default cartesian motion handler
      executeMoveCallback_DrakeCartesianPlanOnly(new_goal, action_res);
    }
  catch(std::runtime_error &ex)
  {
    ROS_ERROR("Planning pipeline threw an exception: %s", ex.what());
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }
  catch(...)
  {
    ROS_ERROR("Planning pipeline threw an exception");
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }
}

void move_group::MoveGroupManipulationAction::executeCartesianMoveCallback_PlanAndExecute(const vigir_planning_msgs::MoveGoalConstPtr& goal, vigir_planning_msgs::MoveResult &action_res)
{

  //Only used if keep endeffector orientation true or if circular motion requested
  Eigen::Affine3d eef_start_pose;

  if((goal->extended_planning_options.target_motion_type == vigir_planning_msgs::ExtendedPlanningOptions::TYPE_CIRCULAR_MOTION ||
     (goal->extended_planning_options.target_motion_type == vigir_planning_msgs::ExtendedPlanningOptions::TYPE_CARTESIAN_WAYPOINTS &&
      goal->extended_planning_options.keep_endeffector_orientation)) &&
     !planning_scene_utils::getEndeffectorTransform(goal->request.group_name,
                                                    context_->planning_scene_monitor_,
                                                    eef_start_pose))
  {
    ROS_ERROR("Cannot get endeffector transform, cartesian planning not possible!");
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    return;
  }

  moveit_msgs::GetCartesianPath cart_path;

  std::vector <geometry_msgs::Pose> pose_vec;

  if (goal->extended_planning_options.target_motion_type == vigir_planning_msgs::ExtendedPlanningOptions::TYPE_CARTESIAN_WAYPOINTS){
    ROS_INFO("Received %d cartesian waypoints with target frame %s",
             (int)goal->extended_planning_options.target_poses.size(),
             goal->extended_planning_options.target_frame.c_str());

    pose_vec.resize(goal->extended_planning_options.target_poses.size());
    geometry_msgs::PoseStamped tmp_pose;

    for (size_t i = 0; i < goal->extended_planning_options.target_poses.size(); ++i){
      tmp_pose.pose = goal->extended_planning_options.target_poses[i];
      tmp_pose.header.frame_id = goal->extended_planning_options.target_frame;
      this->performTransform(tmp_pose, context_->planning_scene_monitor_->getRobotModel()->getModelFrame());

      // Optionally set all poses to keep start orientation
      if(goal->extended_planning_options.keep_endeffector_orientation)
      {
        Eigen::Affine3d oriented_pose = Eigen::Translation3d(Eigen::Vector3d(tmp_pose.pose.position.x,tmp_pose.pose.position.y,tmp_pose.pose.position.z)) * eef_start_pose.rotation();
        tf::poseEigenToMsg(oriented_pose, tmp_pose.pose);
      }

      pose_vec[i] = tmp_pose.pose;
    }

  }else if (goal->extended_planning_options.target_motion_type == vigir_planning_msgs::ExtendedPlanningOptions::TYPE_CIRCULAR_MOTION){
    ROS_INFO("Received circular cartesian motion request!");

    if (goal->extended_planning_options.target_poses.size() != 1){
      ROS_ERROR("There has to be exactly one target pose for circular motion requests!");
      return;
    }

    geometry_msgs::PoseStamped rotation_pose;
    rotation_pose.pose = goal->extended_planning_options.target_poses[0];
    rotation_pose.header.frame_id = goal->extended_planning_options.target_frame;

    //Can easily transform goal pose to arbitrary target frame
    this->performTransform(rotation_pose, context_->planning_scene_monitor_->getRobotModel()->getModelFrame());


    Eigen::Affine3d rotation_center;
    tf::poseMsgToEigen(rotation_pose.pose, rotation_center);

    {
      constrained_motion_utils::getCircularArcPoses(rotation_center,
                                                    eef_start_pose,
                                                    pose_vec,
                                                    0.2,
                                                    goal->extended_planning_options.rotation_angle,
                                                    goal->extended_planning_options.keep_endeffector_orientation);
    }
  }

  cart_path.request.waypoints = pose_vec;

  cart_path.request.header.stamp = ros::Time::now();

  //We already converted to planning frame ("world" per default) in the preceding part, so use that.
  cart_path.request.header.frame_id = context_->planning_scene_monitor_->getRobotModel()->getModelFrame();

  cart_path.request.jump_threshold = 2.0;
  cart_path.request.max_step = 0.01;
  cart_path.request.avoid_collisions = goal->extended_planning_options.avoid_collisions;
  cart_path.request.group_name = goal->request.group_name;

  setMoveState(PLANNING);
  this->computeCartesianPath(cart_path.request, cart_path.response, goal->request.max_velocity_scaling_factor);

  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(context_->planning_scene_monitor_);
    const robot_state::RobotState& curr_state = lscene.getPlanningSceneMonitor()->getPlanningScene()->getCurrentState();

    robot_trajectory::RobotTrajectoryPtr tmp;
    tmp.reset(new robot_trajectory::RobotTrajectory(context_->planning_scene_monitor_->getRobotModel(), goal->request.group_name));
    tmp->setRobotTrajectoryMsg(curr_state, cart_path.response.solution);

    convertToMsg(tmp, action_res.trajectory_start, action_res.planned_trajectory);
  }

  action_res.extended_planning_result.plan_completion_fraction = cart_path.response.fraction;

  if (trajectory_result_display_pub_.getNumSubscribers() > 0){
    moveit_msgs::DisplayTrajectory result_trajectory_display_msg;
    result_trajectory_display_msg.trajectory.push_back( action_res.planned_trajectory );
    result_trajectory_display_msg.trajectory_start = action_res.trajectory_start;
    result_trajectory_display_msg.model_id = context_->planning_scene_monitor_->getRobotModel()->getName();
    trajectory_result_display_pub_.publish(result_trajectory_display_msg);
  }

  if ((cart_path.response.fraction < 1.0) && !goal->extended_planning_options.execute_incomplete_cartesian_plans){
    ROS_WARN("Incomplete cartesian plan computed, fraction: %f and goal specified to not execute in that case!", cart_path.response.fraction);
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    return;
  }

  //Past this point, we have either full path or allow noncomplete paths

  if (!goal->planning_options.plan_only){
    context_->trajectory_execution_manager_->clear();

    if (context_->trajectory_execution_manager_->push(cart_path.response.solution)){
      context_->trajectory_execution_manager_->execute();
      moveit_controller_manager::ExecutionStatus es = context_->trajectory_execution_manager_->waitForExecution();
      if (es == moveit_controller_manager::ExecutionStatus::SUCCEEDED)
        action_res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
      else
        if (es == moveit_controller_manager::ExecutionStatus::PREEMPTED)
          action_res.error_code.val = moveit_msgs::MoveItErrorCodes::PREEMPTED;
        else
          if (es == moveit_controller_manager::ExecutionStatus::TIMED_OUT)
            action_res.error_code.val = moveit_msgs::MoveItErrorCodes::TIMED_OUT;
          else
            action_res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
      ROS_INFO_STREAM("Execution completed: " << es.asString());
    }else{
      ROS_WARN("Could not push trajectory for execution!");
    }
  }else{
    action_res.planned_trajectory = cart_path.response.solution;
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  }
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
      if (trajectory_result_display_pub_.getNumSubscribers() > 0){
        moveit_msgs::DisplayTrajectory result_trajectory_display_msg;
        result_trajectory_display_msg.trajectory.push_back( drake_response_msg.trajectory_result.result_trajectory );
        result_trajectory_display_msg.trajectory_start = current_state_msg;
        result_trajectory_display_msg.model_id = robot_model->getName();
        trajectory_result_display_pub_.publish(result_trajectory_display_msg);
      }
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
        drake_request_msg.trajectory_request.target_link_axis = goal->extended_planning_options.target_link_axis;
        drake_request_msg.trajectory_request.free_joint_names = joint_model_group->getJointModelNames();
        drake_request_msg.trajectory_request.target_orientation_type = goal->extended_planning_options.target_orientation_type;
        drake_request_msg.trajectory_request.trajectory_sample_rate = goal->extended_planning_options.trajectory_sample_rate;
        drake_request_msg.trajectory_request.check_self_collisions = goal->extended_planning_options.check_self_collisions;
        drake_request_msg.trajectory_request.execute_incomplete_cartesian_plans = goal->extended_planning_options.execute_incomplete_cartesian_plans;

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
      if (trajectory_result_display_pub_.getNumSubscribers() > 0){
        moveit_msgs::DisplayTrajectory result_trajectory_display_msg;
        result_trajectory_display_msg.trajectory.push_back( drake_response_msg.trajectory_result.result_trajectory );
        result_trajectory_display_msg.trajectory_start = current_state_msg;
        result_trajectory_display_msg.model_id = robot_model->getName();
        trajectory_result_display_pub_.publish(result_trajectory_display_msg);
      }
    }
    plan.error_code_ = res.error_code_;
    return solved;
}

bool move_group::MoveGroupManipulationAction::planCircularMotionUsingDrake(const vigir_planning_msgs::MoveGoalConstPtr& goal, plan_execution::ExecutableMotionPlan &plan)
{
    ROS_INFO("Received circular cartesian motion request! Forwarding to Drake");

    try
    {
        //Everything OK in the beginning, this will be changed below if we encounter problems
        plan.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

        if (goal->extended_planning_options.target_poses.size() != 1){
          ROS_ERROR("There has to be exactly one target pose for circular motion requests!");
          plan.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
          return false;
        }

          //Only used if keep endeffector orientation true or if circular motion requested
          Eigen::Affine3d eef_start_pose;

          if((goal->extended_planning_options.target_motion_type == vigir_planning_msgs::ExtendedPlanningOptions::TYPE_CIRCULAR_MOTION ||
             (goal->extended_planning_options.target_motion_type == vigir_planning_msgs::ExtendedPlanningOptions::TYPE_CARTESIAN_WAYPOINTS &&
              goal->extended_planning_options.keep_endeffector_orientation)) &&
             !planning_scene_utils::getEndeffectorTransform(goal->request.group_name,
                                                            context_->planning_scene_monitor_,
                                                            eef_start_pose))
          {
            ROS_ERROR("Cannot get endeffector transform, cartesian planning not possible!");
            plan.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
            return false;
          }

          std::string eef_link_name;
          if( ! planning_scene_utils::get_eef_link(goal->request.group_name, eef_link_name)) {
            ROS_ERROR("Cannot get endeffector link name, circular planning not possible!");
            plan.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
            return false;
          }

          geometry_msgs::PoseStamped rotation_pose;
          rotation_pose.pose = goal->extended_planning_options.target_poses[0];
          rotation_pose.header.frame_id = goal->extended_planning_options.target_frame;

          //Can easily transform goal pose to arbitrary target frame
          this->performTransform(rotation_pose, context_->planning_scene_monitor_->getRobotModel()->getModelFrame());


          Eigen::Affine3d rotation_center;
          tf::poseMsgToEigen(rotation_pose.pose, rotation_center);

          std::vector <geometry_msgs::Pose> pose_vec;
          constrained_motion_utils::getCircularArcPoses(rotation_center,
                                                          eef_start_pose,
                                                          pose_vec,
                                                          0.2,
                                                          goal->extended_planning_options.rotation_angle,
                                                          goal->extended_planning_options.keep_endeffector_orientation);

          // make a copy of goal, so I can modify it
          vigir_planning_msgs::MoveGoalPtr new_goal( new vigir_planning_msgs::MoveGoal( *goal ) );

          new_goal->extended_planning_options.target_poses = pose_vec;
          new_goal->extended_planning_options.target_link_names.assign(pose_vec.size(), eef_link_name);
          if ( new_goal->extended_planning_options.target_link_axis.empty() == false ) {
              new_goal->extended_planning_options.target_link_axis.assign(pose_vec.size(), goal->extended_planning_options.target_link_axis[0]);
          }

          // call default cartesian motion handler
          return planCartesianUsingDrake(new_goal, plan);
        }
      catch(std::runtime_error &ex)
      {
        ROS_ERROR("Planning pipeline threw an exception: %s", ex.what());
        plan.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        return false;
      }
      catch(...)
      {
        ROS_ERROR("Planning pipeline threw an exception");
        plan.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        return false;
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

// This is basically a copy of the original MoveIt! cartesian planner with minor mods
bool move_group::MoveGroupManipulationAction::computeCartesianPath(moveit_msgs::GetCartesianPath::Request &req,
                                                                   moveit_msgs::GetCartesianPath::Response &res,
                                                                   double max_velocity_scaling_factor)
{
  /*
  if (marker_array_pub_.getNumSubscribers() > 0){
    visualization_msgs::MarkerArray markers;
    flor_visualization_utils::drawPoses(req.waypoints, markers, req.header.frame_id, ros::Time::now(), 0.1, 1.0);
    marker_array_pub_.publish(markers);
  }
  */


  ROS_INFO("Received request to compute Cartesian path");
  context_->planning_scene_monitor_->updateFrameTransforms();

  const planning_scene::PlanningScenePtr& planning_scene = context_->planning_scene_monitor_->getPlanningScene();

  geometry_msgs::PoseStamped goal_pose;

  uint8_t status;

  /*
  planningSceneCommonSetup(planning_scene,
                           //req.plan_request.use_environment_obstacle_avoidance.data,
                           true,
                           //res.status,
                           status,
                           goal_pose);
  */



  robot_state::RobotState start_state = planning_scene_monitor::LockedPlanningSceneRO(context_->planning_scene_monitor_)->getCurrentState();
  robot_state::robotStateMsgToRobotState(req.start_state, start_state);
  if (const robot_model::JointModelGroup *jmg = start_state.getJointModelGroup(req.group_name))
  {
    std::string link_name = req.link_name;
    if (link_name.empty() && !jmg->getLinkModelNames().empty())
      link_name = jmg->getLinkModelNames().back();

    bool ok = true;
    EigenSTL::vector_Affine3d waypoints(req.waypoints.size());
    const std::string &default_frame = context_->planning_scene_monitor_->getRobotModel()->getModelFrame();
    bool no_transform = req.header.frame_id.empty() || robot_state::Transforms::sameFrame(req.header.frame_id, default_frame) ||
      robot_state::Transforms::sameFrame(req.header.frame_id, link_name);

    for (std::size_t i = 0 ; i < req.waypoints.size() ; ++i)
    {
      if (no_transform)
        tf::poseMsgToEigen(req.waypoints[i], waypoints[i]);
      else
      {
        geometry_msgs::PoseStamped p;
        p.header = req.header;
        p.pose = req.waypoints[i];
        if (performTransform(p, default_frame))
          tf::poseMsgToEigen(p.pose, waypoints[i]);
        else
        {
          ROS_ERROR("Error encountered transforming waypoints to frame '%s'", default_frame.c_str());
          ok = false;
          break;
        }
      }
    }

    if (ok)
    {
      if (req.max_step < std::numeric_limits<double>::epsilon())
      {
        ROS_ERROR("Maximum step to take between consecutive configrations along Cartesian path was not specified (this value needs to be > 0)");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
      }
      else
      {
        if (waypoints.size() > 0)
        {
          robot_state::GroupStateValidityCallbackFn constraint_fn;
          boost::scoped_ptr<planning_scene_monitor::LockedPlanningSceneRO> ls;
          boost::scoped_ptr<kinematic_constraints::KinematicConstraintSet> kset;
          if (req.avoid_collisions || !kinematic_constraints::isEmpty(req.path_constraints))
          {
            ls.reset(new planning_scene_monitor::LockedPlanningSceneRO(context_->planning_scene_monitor_));
            kset.reset(new kinematic_constraints::KinematicConstraintSet((*ls)->getRobotModel()));
            kset->add(req.path_constraints, (*ls)->getTransforms());
            constraint_fn = boost::bind(&isStateValid, req.avoid_collisions ? static_cast<const planning_scene::PlanningSceneConstPtr&>(*ls).get() : NULL, kset->empty() ? NULL : kset.get(), _1, _2, _3);
          }
          bool global_frame = !robot_state::Transforms::sameFrame(link_name, req.header.frame_id);
          ROS_INFO("Attempting to follow %u waypoints for link '%s' using a step of %lf m and jump threshold %lf (in %s reference frame)",
                   (unsigned int)waypoints.size(), link_name.c_str(), req.max_step, req.jump_threshold, global_frame ? "global" : "link");
          std::vector<robot_state::RobotStatePtr> traj;

          std::vector<std::string> locked_joints = joint_constraint_utils::getLockedJoints(jmg, req.path_constraints.joint_constraints);

          ROS_INFO("Using %d locked torso joints", (int)locked_joints.size());

          if (locked_joints.size() != 0){
            ROS_INFO("Locking joints for cartesian planning");

            robot_model::JointModelGroup group_cpy = *jmg;

            const kinematics::KinematicsBasePtr& solver = group_cpy.getSolverInstance();
            solver->setRedundantJoints(locked_joints);

            kinematics::KinematicsQueryOptions options;
            options.lock_redundant_joints = true;

            res.fraction = start_state.computeCartesianPath(&group_cpy, traj, start_state.getLinkModel(link_name), waypoints, global_frame, req.max_step, req.jump_threshold, constraint_fn, options);
          }else{         
            res.fraction = start_state.computeCartesianPath(jmg, traj, start_state.getLinkModel(link_name), waypoints, global_frame, req.max_step, req.jump_threshold, constraint_fn);
          }

          robot_state::robotStateToRobotStateMsg(start_state, res.start_state);

          std::vector<robot_state::RobotStatePtr> traj_filtered;
          trajectory_utils::removeDuplicateStates(traj, traj_filtered);

          robot_trajectory::RobotTrajectory rt(context_->planning_scene_monitor_->getRobotModel(), req.group_name);
          for (std::size_t i = 0 ; i < traj_filtered.size() ; ++i)
            rt.addSuffixWayPoint(traj_filtered[i], 0.2); // \todo make 0.2 a param; better: compute time stemps based on eef distance and param m/s speed for eef;


          if (!time_param_->computeTimeStamps(rt, max_velocity_scaling_factor)){
            ROS_WARN("Time parametrization for the solution path failed.");
          }

          //trajectory_utils::removeZeroDurationJointTrajectoryPoints(rt);

          rt.getRobotTrajectoryMsg(res.solution);
          ROS_INFO("Computed Cartesian path with %u points (followed %lf%% of requested trajectory)", (unsigned int)traj.size(), res.fraction * 100.0);
          ROS_INFO("Reduced to %u points by removing duplicate states", (unsigned int)traj_filtered.size());
          /*
          if (plan_vis_pub_.getNumSubscribers() > 0 && rt.getWayPointCount() > 0)
          {
            moveit_msgs::DisplayTrajectory disp;
            disp.model_id = context_->planning_scene_monitor_->getRobotModel()->getName();
            disp.trajectory.resize(1, res.solution);
            robot_state::robotStateToRobotStateMsg(rt.getFirstWayPoint(), disp.trajectory_start);
            plan_vis_pub_.publish(disp);
          }
          */
        }
        res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
      }
    }
    else
      res.error_code.val = moveit_msgs::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE;
  }
  else
    res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;

  return true;
}

#include <class_loader/class_loader.h>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupManipulationAction, move_group::MoveGroupCapability)
