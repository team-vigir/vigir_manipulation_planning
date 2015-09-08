/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Stefan Kohlbrecher, TU Darmstadt ( Team ViGIR )
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
 *   * Neither the name of TU Darmstadt, Team ViGIR, nor the names of its
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

/* Author: Stefan Kohlbrecher */

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

#include <vigir_moveit_utils/joint_constraint_utils.h>
#include <vigir_moveit_utils/group_utils.h>
#include <vigir_moveit_utils/robot_model_utils.h>
#include <vigir_moveit_utils/collision_utils.h>

#include <vigir_moveit_utils/planning_scene_utils.h>
#include <vigir_moveit_utils/constrained_motion_utils.h>
#include <vigir_moveit_utils/trajectory_utils.h>

#include <nav_msgs/Path.h>

#include <vigir_move_group/helper/drake_planning_helper.h>


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
  drake_planning_helper_.reset(new DrakePlanningHelper(this));

  ros::NodeHandle pnh("~/visualization");

  planned_traj_vis_.reset(new trajectory_utils::TrajectoryVisualization(pnh));
  executed_traj_vis_.reset(new trajectory_utils::TrajectoryVisualization(pnh, "eef_traj_executed", 0.0, 0.0, 1.0));

  this->setupHandData();

  // start the move action server MOVE_ACTION
  move_action_server_.reset(new actionlib::SimpleActionServer<vigir_planning_msgs::MoveAction>(root_node_handle_, "vigir_move_group",
                                                                                            boost::bind(&MoveGroupManipulationAction::executeMoveCallback, this, _1), false));
  move_action_server_->registerPreemptCallback(boost::bind(&MoveGroupManipulationAction::preemptMoveCallback, this));
  move_action_server_->start();


  trajectory_result_display_pub_ = root_node_handle_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 10);
  circular_target_path_pub_ = root_node_handle_.advertise<nav_msgs::Path>("/move_group/circular_target_path", 1);
}

void move_group::MoveGroupManipulationAction::setupHandData()
{
  const robot_model::RobotModelConstPtr& robot_model = context_->planning_pipeline_->getRobotModel();

  left_hand_links_vector_ = robot_model_utils::getSubLinks(*robot_model,"l_hand");
  right_hand_links_vector_ = robot_model_utils::getSubLinks(*robot_model,"r_hand");
}

void move_group::MoveGroupManipulationAction::setCollisionOptions(bool all_env_collision_allow,
                         bool left_hand_collision_allow,
                         bool right_hand_collision_allow)
{
  planning_scene_monitor::LockedPlanningSceneRW ps(context_->planning_scene_monitor_);

  //Set all collision allow directly (allowed or not allowed)
  collision_utils::setAllowedCollisions(context_->planning_scene_monitor_->getRobotModel()->getLinkModelNames(),
                                          context_->planning_scene_monitor_->getPlanningScene(),
                                          all_env_collision_allow);

  //If all env collisions allowed, ignore hand settings as they are implied in allowing all
  if(!all_env_collision_allow){
    collision_utils::setAllowedCollisions(left_hand_links_vector_,
                                          context_->planning_scene_monitor_->getPlanningScene(),
                                          left_hand_collision_allow);

    collision_utils::setAllowedCollisions(right_hand_links_vector_,
                                          context_->planning_scene_monitor_->getPlanningScene(),
                                          right_hand_collision_allow);
  }
}

bool move_group::MoveGroupManipulationAction::checkGroupStateSelfCollisionFree(robot_state::RobotState *robot_state, const robot_state::JointModelGroup *joint_group, const double *joint_group_variable_values)
{
    collision_detection::CollisionRequest request;
    collision_detection::CollisionResult result;
    robot_state->setJointGroupPositions(joint_group, joint_group_variable_values);
    context_->planning_scene_monitor_->getPlanningScene()->checkSelfCollision(request, result, *robot_state);
    return !result.collision;
}

void move_group::MoveGroupManipulationAction::executeMoveCallback(const vigir_planning_msgs::MoveGoalConstPtr& goal)
{
  setMoveState(PLANNING);
  context_->planning_scene_monitor_->updateFrameTransforms();

  //planning_scene_monitor::LockedPlanningSceneRO lscene(context_->planning_scene_monitor_); // lock the scene so that it does not modify the world representation while diff() is called
  //const planning_scene::PlanningSceneConstPtr &the_scene = (planning_scene::PlanningScene::isEmpty(goal->planning_options.planning_scene_diff)) ?
  //  static_cast<const planning_scene::PlanningSceneConstPtr&>(lscene) : lscene->diff(goal->planning_options.planning_scene_diff);


  /*
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(context_->planning_scene_monitor_);

    std::vector<std::string> object_strings = context_->planning_scene_monitor_->getPlanningScene()->getCollisionWorld()->getWorld()->getObjectIds();

    size_t size = object_strings.size();

    const robot_model::RobotModelConstPtr& robot_model = context_->planning_pipeline_->getRobotModel();

    for (size_t i = 0; i < size; ++i){
      context_->planning_scene_monitor_->getPlanningScene()->getAllowedCollisionMatrixNonConst().setEntry(object_strings[i], robot_model->getLinkModelNames(), !goal->extended_planning_options.avoid_collisions);
    }

    context_->planning_scene_monitor_->getPlanningScene()->getAllowedCollisionMatrixNonConst().setEntry("<octomap>", robot_model->getLinkModelNames(), !goal->extended_planning_options.avoid_collisions);
  }
  */

  vigir_planning_msgs::MoveResult action_res;

  this->setCollisionOptions(goal->extended_planning_options.allow_environment_collisions,
                            goal->extended_planning_options.extended_planning_scene_diff.allow_left_hand_environment_collision,
                            goal->extended_planning_options.extended_planning_scene_diff.allow_right_hand_environment_collision);

  if (goal->request.planner_id == "drake")
  {
    if (goal->planning_options.plan_only || !context_->allow_trajectory_execution_)
    {
      if (!goal->planning_options.plan_only)
        ROS_WARN("This instance of MoveGroup is not allowed to execute trajectories but the goal request has plan_only set to false. Only a motion plan will be computed anyway.");

      // check for cartesian motion
      if ( goal->extended_planning_options.target_poses.size() != 0 && ( goal->extended_planning_options.target_motion_type == vigir_planning_msgs::ExtendedPlanningOptions::TYPE_CARTESIAN_WAYPOINTS || goal->extended_planning_options.target_motion_type == vigir_planning_msgs::ExtendedPlanningOptions::TYPE_FREE_MOTION))
      {
        drake_planning_helper_->planCartesianMotionAction(goal, action_res);
      }
      else if ( goal->extended_planning_options.target_poses.size() == 0) // non-cartesian planning
      {
        drake_planning_helper_->planAction(goal, action_res);
      }
      else if ( goal->extended_planning_options.target_motion_type == vigir_planning_msgs::ExtendedPlanningOptions::TYPE_CIRCULAR_MOTION)
      {
        drake_planning_helper_->planCircularMotionAction(goal, action_res);
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
  else if (goal->extended_planning_options.target_poses.size() != 0)
  {

    /* Implement reference point from the request
       * Output needed: targetFrame_T_wrist.
       * Input needed:  targetFrame_T_referencePoint. This is given in the request as targetPose, but in this case it refers to the reference point, not the wrist.
       *                wrist_T_referencePoint.   This is the reference point in wrist frame (need to use inverse transform)
       * targetFrame_T_wrist = targetFrame_T_referencePoint * referencePoint_T_wrist.
       * For n waypoints:
       *    waypoint[i] = waypoint[i] * reference_point.inverse;  //User gives point of reference in wrist frame, but need to use inverse: */

    std::vector<geometry_msgs::Pose> new_target_poses;
    vigir_planning_msgs::MoveGoalPtr new_goal;
    //Copy directly from goal
    new_goal.reset(new vigir_planning_msgs::MoveGoal());
    *new_goal = *goal;

    if(goal->extended_planning_options.reference_point.orientation.x != 0.0 ||
       goal->extended_planning_options.reference_point.orientation.y != 0.0 ||
       goal->extended_planning_options.reference_point.orientation.z != 0.0 ||
       goal->extended_planning_options.reference_point.orientation.w != 0.0 )
    {
      ROS_INFO("Using reference point in the request");
      tf::Transform wrist_T_referencePoint;
      tf::Transform targetFrame_T_referencePoint;
      tf::Transform targetFrame_T_wrist;

      new_target_poses = goal->extended_planning_options.target_poses;

      if(goal->extended_planning_options.target_motion_type == vigir_planning_msgs::ExtendedPlanningOptions::TYPE_FREE_MOTION ||
         goal->extended_planning_options.target_motion_type == vigir_planning_msgs::ExtendedPlanningOptions::TYPE_CARTESIAN_WAYPOINTS)
      {
        ROS_INFO("Calculating new waypoints given reference point for FREE MOTIONS and CATRTESIAN WAYPOINTS");

        tf::poseMsgToTF(goal->extended_planning_options.reference_point, wrist_T_referencePoint);

        for (size_t i = 0; i < goal->extended_planning_options.target_poses.size(); ++i)
        {
          tf::poseMsgToTF(goal->extended_planning_options.target_poses[i], targetFrame_T_referencePoint);
          targetFrame_T_wrist = targetFrame_T_referencePoint * wrist_T_referencePoint.inverse();
          tf::poseTFToMsg(targetFrame_T_wrist,new_target_poses[i]);
        }
      }
      else  //Circular motions behave different when reference point is given. Waypoint is translated but not rotated.
      {
        if(goal->extended_planning_options.target_motion_type == vigir_planning_msgs::ExtendedPlanningOptions::TYPE_CIRCULAR_MOTION &&
           goal->extended_planning_options.keep_endeffector_orientation)
        {
          ROS_INFO("Calculating new waypoints given reference point for CIRCULAR MOTIONS");
          //Only used if keep endeffector orientation true or if circular motion requested
          Eigen::Affine3d eef_start_pose;

          if(planning_scene_utils::getEndeffectorTransform(goal->request.group_name,
                                                            context_->planning_scene_monitor_,
                                                            eef_start_pose))
          {
            geometry_msgs::PoseStamped reference_pose;

            reference_pose.header.frame_id = goal->extended_planning_options.reference_point_frame;
            reference_pose.pose            = goal->extended_planning_options.reference_point;

            if (this->performTransform(reference_pose, context_->planning_scene_monitor_->getRobotModel()->getModelFrame()))//Gets reference point in world frame
            {
              geometry_msgs::Pose wristPose;
              tf::poseEigenToMsg(eef_start_pose,wristPose);  //Converts eef eigen pose to geometry pose

              // calculate the difference between them
              tf::Vector3 diff_vector;
              diff_vector.setX(wristPose.position.x - reference_pose.pose.position.x);
              diff_vector.setY(wristPose.position.y - reference_pose.pose.position.y);
              diff_vector.setZ(wristPose.position.z - reference_pose.pose.position.z);

              // apply the difference to the circular center
              ROS_INFO("Applying difference vector to rotation axis (x: %f, y: %f, z: %f)", diff_vector.getX(), diff_vector.getY(), diff_vector.getZ());
              new_target_poses[0].position.x += diff_vector.getX();
              new_target_poses[0].position.y += diff_vector.getY();
              new_target_poses[0].position.z += diff_vector.getZ();
              ROS_INFO("New rotation axis (x: %f, y: %f, z: %f)", new_target_poses[0].position.x, new_target_poses[0].position.y, new_target_poses[0].position.z);

              new_goal->extended_planning_options.target_poses = new_target_poses;
            }else{
              ROS_ERROR("Could not get reference pose (%s) into %s frame, resetting target poses.", reference_pose.header.frame_id.c_str(),
                        context_->planning_scene_monitor_->getRobotModel()->getModelFrame().c_str());
              action_res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
            }
          }else{
            //Under normal operating conditions we always can get the endeffector transform
            ROS_ERROR("Cannot get endeffector transform, cartesian planning not possible!");
            action_res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
            return;
          }
        }else{  //Using circular motion without keeping end effector orientation
          ROS_WARN("Resetting target poses since no modification is needed when circular and not keeping end effector orientation");
        }
      }

    }


    if (new_goal->extended_planning_options.target_motion_type == vigir_planning_msgs::ExtendedPlanningOptions::TYPE_FREE_MOTION)
    {

      // For free motion, do IK and plan.
      // Consider additional joint constraints/redundant joints

      if (new_goal->extended_planning_options.target_poses.size() > 1)
      {
        ROS_ERROR("For FREE MOTION, only a single target pose is supported, but I got %d", (int)new_goal->extended_planning_options.target_poses.size());
        action_res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
      }else{

        geometry_msgs::PoseStamped goal_pose_planning_frame;

        goal_pose_planning_frame.pose = new_goal->extended_planning_options.target_poses[0];
        goal_pose_planning_frame.header.frame_id = new_goal->extended_planning_options.target_frame;

        if (this->performTransform(goal_pose_planning_frame, context_->planning_scene_monitor_->getRobotModel()->getModelFrame()))
        {

          moveit_msgs::Constraints goal_constraints;
          bool found_ik = false;

          {
            planning_scene_monitor::LockedPlanningSceneRO lscene(context_->planning_scene_monitor_);
            robot_state::RobotState tmp = lscene->getCurrentState();

            const robot_state::JointModelGroup* joint_model_group = tmp.getJointModelGroup(new_goal->request.group_name);

            found_ik = group_utils::setJointModelGroupFromIk(tmp,
                                                             joint_model_group,
                                                             goal_pose_planning_frame.pose,
                                                             new_goal->request.path_constraints.joint_constraints,
                                                             boost::bind(&MoveGroupManipulationAction::checkGroupStateSelfCollisionFree, this, _1, _2, _3));

            if (found_ik){
              goal_constraints = kinematic_constraints::constructGoalConstraints(tmp, joint_model_group);
            }
          }

          if (found_ik)
          {
            vigir_planning_msgs::MoveGoalPtr updated_goal;
            updated_goal.reset(new vigir_planning_msgs::MoveGoal());
            *updated_goal = *new_goal;

            updated_goal->request.goal_constraints.push_back(goal_constraints);

            if (new_goal->planning_options.plan_only || !context_->allow_trajectory_execution_)
            {
              if (!new_goal->planning_options.plan_only)
                ROS_WARN("This instance of MoveGroup is not allowed to execute trajectories but the goal request has plan_only set to false. Only a motion plan will be computed anyway.");
              executeMoveCallback_PlanOnly(updated_goal, action_res);
            }
            else
            {
              executeMoveCallback_PlanAndExecute(updated_goal, action_res);
            }
          }else{
            ROS_WARN("No valid IK solution found, cannot generate goal constraints!");
            action_res.error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
          }
        }else{
          ROS_ERROR("Invalid target frame %s requested for cartesian planning!", new_goal->extended_planning_options.target_frame.c_str());
          action_res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        }
      }



    }else{
      //Otherwise, perform cartesian motion
      executeCartesianMoveCallback_PlanAndExecute(new_goal, action_res);
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

  this->setCollisionOptions(false, false, false);

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
        action_res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        return;
    }

    if (goal->request.planner_id == "drake") // plan using drake
    {
        // check if it is a cartesian request
        if ( goal->extended_planning_options.target_poses.size() != 0 && (goal->extended_planning_options.target_motion_type == vigir_planning_msgs::ExtendedPlanningOptions::TYPE_CARTESIAN_WAYPOINTS || goal->extended_planning_options.target_motion_type == vigir_planning_msgs::ExtendedPlanningOptions::TYPE_FREE_MOTION))
            opt.plan_callback_ = boost::bind(&DrakePlanningHelper::planCartesianMotion, drake_planning_helper_.get(), boost::cref(goal), _1);
        else if ( goal->extended_planning_options.target_motion_type == vigir_planning_msgs::ExtendedPlanningOptions::TYPE_CIRCULAR_MOTION)
            opt.plan_callback_ = boost::bind(&DrakePlanningHelper::planCircularMotion, drake_planning_helper_.get(), boost::cref(goal), _1);
        else if ( goal->extended_planning_options.target_poses.size() == 0 ) // normal joint-level planning
            opt.plan_callback_ = boost::bind(&DrakePlanningHelper::plan, drake_planning_helper_.get(), boost::cref(goal), _1);
        else {
            ROS_WARN("Selected motion type %d not implemented for Drake!", goal->extended_planning_options.target_motion_type);
            action_res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
            return;
        }
    }
    else
    {
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
                                                    goal->extended_planning_options.keep_endeffector_orientation,
                                                    goal->extended_planning_options.pitch);
    }

    if ( circular_target_path_pub_.getNumSubscribers() > 0 ) {
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = context_->planning_scene_monitor_->getRobotModel()->getModelFrame();
        path_msg.header.stamp = ros::Time::now();

        for ( int i = 0; i < pose_vec.size(); i++ ) {
            geometry_msgs::PoseStamped current_pose;
            current_pose.header.frame_id = context_->planning_scene_monitor_->getRobotModel()->getModelFrame();
            current_pose.header.stamp = ros::Time::now();
            current_pose.pose = pose_vec[i];

            path_msg.poses.push_back(current_pose);
        }

        circular_target_path_pub_.publish(path_msg);
    }
  }

  cart_path.request.waypoints = pose_vec;

  cart_path.request.header.stamp = ros::Time::now();

  //We already converted to planning frame ("world" per default) in the preceding part, so use that.
  cart_path.request.header.frame_id = context_->planning_scene_monitor_->getRobotModel()->getModelFrame();

  cart_path.request.jump_threshold = 2.0;
  cart_path.request.max_step = 0.01;
  cart_path.request.avoid_collisions = false;//goal->extended_planning_options.allow_environment_collisions;
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
    vigir_visualization_utils::drawPoses(req.waypoints, markers, req.header.frame_id, ros::Time::now(), 0.1, 1.0);
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

planning_scene::PlanningSceneConstPtr move_group::MoveGroupManipulationAction::getCollisionSettingsPlanningSceneDiff(const vigir_planning_msgs::MoveGoalConstPtr& goal,
                                                                                                                     planning_scene_monitor::LockedPlanningSceneRO& lscene) const
{
  const planning_scene::PlanningSceneConstPtr &the_scene = (planning_scene::PlanningScene::isEmpty(goal->planning_options.planning_scene_diff)) ?
        static_cast<const planning_scene::PlanningSceneConstPtr&>(lscene) : lscene->diff(goal->planning_options.planning_scene_diff);

  const planning_scene::PlanningScenePtr extended_scene = the_scene->diff();

  // We only modify extended scene if required, otherwise no copies are performed, which is preferable.
  if ( ( goal->extended_planning_options.allow_environment_collisions) ||
       ( goal->extended_planning_options.extended_planning_scene_diff.allowed_collision_pairs.size() > 0) ||
       ( goal->extended_planning_options.extended_planning_scene_diff.forbidden_collision_pairs.size() > 0) ||
       ( goal->extended_planning_options.extended_planning_scene_diff.allow_left_hand_environment_collision) ||
       ( goal->extended_planning_options.extended_planning_scene_diff.allow_right_hand_environment_collision) ||
       ( goal->extended_planning_options.extended_planning_scene_diff.allow_hands_collision_with_object_id.size() > 0))
  {
    collision_detection::AllowedCollisionMatrix& acm = extended_scene->getAllowedCollisionMatrixNonConst();

    // Completely disable complete environment collision checks
    if (goal->extended_planning_options.allow_environment_collisions){
      std::vector<std::string> object_strings = context_->planning_scene_monitor_->getPlanningScene()->getCollisionWorld()->getWorld()->getObjectIds();

      size_t size = object_strings.size();

      const robot_model::RobotModelConstPtr& robot_model = context_->planning_pipeline_->getRobotModel();

      for (size_t i = 0; i < size; ++i){
        acm.setEntry(object_strings[i], robot_model->getLinkModelNames(), true);
      }

      acm.setEntry("<octomap>", robot_model->getLinkModelNames(), true);
    }


    // Selectively disable collision checks
    const std::vector<vigir_planning_msgs::CollisionPair>& allowed_vec = goal->extended_planning_options.extended_planning_scene_diff.allowed_collision_pairs;

    for (size_t i = 0; i < allowed_vec.size();++i)
    {
      acm.setEntry(allowed_vec[i].collision_entities[0], allowed_vec[i].collision_entities[1], true);
    }

    // Selectively enable collision checks
    const std::vector<vigir_planning_msgs::CollisionPair>& forbidden_vec = goal->extended_planning_options.extended_planning_scene_diff.forbidden_collision_pairs;

    for (size_t i = 0; i < forbidden_vec.size();++i)
    {
      acm.setEntry(forbidden_vec[i].collision_entities[0], forbidden_vec[i].collision_entities[1], false);
    }

    // Selectively switch off hand collision checks
    if ((goal->extended_planning_options.extended_planning_scene_diff.allow_left_hand_environment_collision) ||
        (goal->extended_planning_options.extended_planning_scene_diff.allow_right_hand_environment_collision))
    {

      std::vector<std::string> object_strings = context_->planning_scene_monitor_->getPlanningScene()->getCollisionWorld()->getWorld()->getObjectIds();

      size_t size = object_strings.size();

      if (goal->extended_planning_options.extended_planning_scene_diff.allow_left_hand_environment_collision){
        for (size_t i = 0; i < size; ++i){
          acm.setEntry(object_strings[i], left_hand_links_vector_, true);
        }

        acm.setEntry("<octomap>", left_hand_links_vector_, true);
      }

      if (goal->extended_planning_options.extended_planning_scene_diff.allow_right_hand_environment_collision){
        for (size_t i = 0; i < size; ++i){
          acm.setEntry(object_strings[i], right_hand_links_vector_, true);
        }

        acm.setEntry("<octomap>", right_hand_links_vector_, true);
      }
    }

    if (goal->extended_planning_options.extended_planning_scene_diff.allow_hands_collision_with_object_id.size() > 0){
      acm.setEntry(left_hand_links_vector_, goal->extended_planning_options.extended_planning_scene_diff.allow_hands_collision_with_object_id, true);
      acm.setEntry(right_hand_links_vector_, goal->extended_planning_options.extended_planning_scene_diff.allow_hands_collision_with_object_id, true);
    }
  }

  return extended_scene;
}



#include <class_loader/class_loader.h>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupManipulationAction, move_group::MoveGroupCapability)
