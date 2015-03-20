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

#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/plan_execution/plan_with_sensing.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group/capability_names.h>

#include <moveit/robot_state/conversions.h>

#include <vigir_moveit_utils/constrained_motion_utils.h>


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
}

void move_group::MoveGroupManipulationAction::executeMoveCallback(const vigir_planning_msgs::MoveGoalConstPtr& goal)
{
  setMoveState(PLANNING);
  context_->planning_scene_monitor_->updateFrameTransforms();

  vigir_planning_msgs::MoveResult action_res;
  //goal->extended_planning_options;

  if (goal->request.planner_id == "drake"){
    // @DRAKE Plan using Drake here. Alternatively, could also implement alternative callback below where @DRAKE is marked
    ROS_WARN("Planning using Drake requested, but not implemented yet!");
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
    // @DRAKE: Could implement callback for Drake and use here
    opt.plan_callback_ = boost::bind(&MoveGroupManipulationAction::planUsingPlanningPipeline, this, boost::cref(motion_plan_request), _1);

    //We normally don't plan with lookaround so the below can be ignored
    if (goal->planning_options.look_around && context_->plan_with_sensing_)
    {
      opt.plan_callback_ = boost::bind(&plan_execution::PlanWithSensing::computePlan, context_->plan_with_sensing_.get(), _1, opt.plan_callback_,
                                       goal->planning_options.look_around_attempts, goal->planning_options.max_safe_execution_cost);
      context_->plan_with_sensing_->setBeforeLookCallback(boost::bind(&MoveGroupManipulationAction::startMoveLookCallback, this));
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

void move_group::MoveGroupManipulationAction::executeCartesianMoveCallback_PlanAndExecute(const vigir_planning_msgs::MoveGoalConstPtr& goal, vigir_planning_msgs::MoveResult &action_res)
{
  moveit_msgs::GetCartesianPath cart_path;

  std::vector <geometry_msgs::Pose> pose_vec;

  if (goal->extended_planning_options.target_motion_type == vigir_planning_msgs::ExtendedPlanningOptions::TYPE_CARTESIAN_WAYPOINTS){
    ROS_WARN("Cartesian waypoints not implemented yet!");

  }else if (goal->extended_planning_options.target_motion_type == vigir_planning_msgs::ExtendedPlanningOptions::TYPE_CIRCULAR_MOTION){
    ROS_WARN("Circular waypoints not implemented yet!");

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
      planning_scene_monitor::LockedPlanningSceneRO lscene(context_->planning_scene_monitor_);
      const robot_state::RobotState& curr_state = lscene.getPlanningSceneMonitor()->getPlanningScene()->getCurrentState();

      std::string start_pose_link;

      std::string first_char = goal->request.group_name.substr(0,1);

      if (first_char == "r"){
        start_pose_link = "r_hand";
      }else if(first_char == "l"){
        start_pose_link = "l_hand";
      }else{
        ROS_ERROR("Group name %s does not start with l or r. Cannot infer endeffector to use, aborting", goal->request.group_name.c_str());
        //res.status += flor_planning_msgs::GetMotionPlanForPose::Response::PLANNING_INVALID_REQUEST;
        return;
      }

      Eigen::Affine3d start (curr_state.getGlobalLinkTransform(start_pose_link));

      constrained_motion_utils::getCircularArcPoses(rotation_center,
                                                    start,
                                                    pose_vec,
                                                    0.2,
                                                    goal->extended_planning_options.rotation_angle,
                                                    goal->extended_planning_options.keep_endeffector_orientation);


    }

    //std::string start_pose_link;
  }

  cart_path.request.waypoints = pose_vec;

  cart_path.request.header.stamp = ros::Time::now();
  cart_path.request.header.frame_id = goal->extended_planning_options.target_frame;

  cart_path.request.jump_threshold = 10.0;
  cart_path.request.max_step = 0.01;
  cart_path.request.avoid_collisions = goal->extended_planning_options.avoid_collisions;
  cart_path.request.group_name = goal->request.group_name;

  moveit_msgs::GetCartesianPath::Response result;
  //this->computeCartesianPathService(cart_path.request, cart_path.response);
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

// This is basically a copy of the original MoveIt!
bool move_group::MoveGroupManipulationAction::computeCartesianPath(moveit_msgs::GetCartesianPath::Request &req, moveit_msgs::GetCartesianPath::Response &res)
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

          //std::vector<std::string> locked_joints = group_utils::getLockedLinks(jmg, planner_configuration_.joint_position_constraints);

          /*
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
          */
            res.fraction = start_state.computeCartesianPath(jmg, traj, start_state.getLinkModel(link_name), waypoints, global_frame, req.max_step, req.jump_threshold, constraint_fn);
          //}

          robot_state::robotStateToRobotStateMsg(start_state, res.start_state);

          std::vector<robot_state::RobotStatePtr> traj_filtered;
          trajectory_utils::removeDuplicateStates(traj, traj_filtered);

          robot_trajectory::RobotTrajectory rt(context_->planning_scene_monitor_->getRobotModel(), req.group_name);
          for (std::size_t i = 0 ; i < traj_filtered.size() ; ++i)
            rt.addSuffixWayPoint(traj_filtered[i], 0.2); // \todo make 0.2 a param; better: compute time stemps based on eef distance and param m/s speed for eef;

          /*
          if (!time_param_.computeTimeStamps(rt, planner_configuration_.trajectory_time_factor.data)){
            ROS_WARN("Time parametrization for the solution path failed.");
          }
          */

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
