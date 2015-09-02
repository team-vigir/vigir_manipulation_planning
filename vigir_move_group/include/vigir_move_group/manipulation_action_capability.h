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

/* Author: Ioan Sucan */

#ifndef VIGIR_MOVE_GROUP_MANIPULATION_ACTION_CAPABILITY_
#define VIGIR_MOVE_GROUP_MANIPULATION_ACTION_CAPABILITY_

#include <moveit/move_group/move_group_capability.h>
#include <actionlib/server/simple_action_server.h>
#include <vigir_planning_msgs/MoveAction.h>


#include <vigir_plan_execution/continuous_plan_execution.h>
#include <moveit_msgs/GetCartesianPath.h>

#include <vigir_move_group/helper/drake_planning_helper.h>

namespace move_group
{

class MoveGroupManipulationAction : public MoveGroupCapability
{
    friend class DrakePlanningHelper;

public:

  MoveGroupManipulationAction();

  virtual void initialize();

private:
  bool checkGroupStateSelfCollisionFree(robot_state::RobotState *robot_state, const robot_state::JointModelGroup *joint_group, const double *joint_group_variable_values);

  void setupHandData();

  void setCollisionOptions(bool all_env_collision_allow,
                           bool left_hand_collision_allow,
                           bool right_hand_collision_allow);

  void executeMoveCallback(const vigir_planning_msgs::MoveGoalConstPtr& goal);
  void executeMoveCallback_PlanAndExecute(const vigir_planning_msgs::MoveGoalConstPtr& goal, vigir_planning_msgs::MoveResult &action_res);
  void executeMoveCallback_PlanOnly(const vigir_planning_msgs::MoveGoalConstPtr& goal, vigir_planning_msgs::MoveResult &action_res);

  void executeCartesianMoveCallback_PlanAndExecute(const vigir_planning_msgs::MoveGoalConstPtr& goal, vigir_planning_msgs::MoveResult &action_res);
  void startMoveExecutionCallback();
  void startMoveLookCallback();
  void preemptMoveCallback();
  void setMoveState(MoveGroupState state);
  bool planUsingPlanningPipeline(const planning_interface::MotionPlanRequest &req, plan_execution::ExecutableMotionPlan &plan);

  // Mostly copy of MoveGroup CartesianPath service with modifications
  bool computeCartesianPath(moveit_msgs::GetCartesianPath::Request &req,
                            moveit_msgs::GetCartesianPath::Response &res,
                            double max_velocity_scaling_factor);

  planning_scene::PlanningSceneConstPtr getCollisionSettingsPlanningSceneDiff(const vigir_planning_msgs::MoveGoalConstPtr& goal,
                                                                              planning_scene_monitor::LockedPlanningSceneRO& lscene) const;

  boost::scoped_ptr<actionlib::SimpleActionServer<vigir_planning_msgs::MoveAction> > move_action_server_;
  vigir_planning_msgs::MoveFeedback move_feedback_;

  MoveGroupState move_state_;

  plan_execution::ContinuousPlanExecutionPtr continuous_plan_execution_;

  boost::shared_ptr<trajectory_utils::TrajectoryVisualization> planned_traj_vis_;
  boost::shared_ptr<trajectory_utils::TrajectoryVisualization> executed_traj_vis_;

  boost::shared_ptr<DrakePlanningHelper> drake_planning_helper_;

  ros::Publisher trajectory_result_display_pub_;
  ros::Publisher circular_target_path_pub_;

  tf::TransformListener transform_listener_;

  boost::shared_ptr<trajectory_processing::IterativeParabolicTimeParameterization> time_param_;

  std::vector<std::string> left_hand_links_vector_;
  std::vector<std::string> right_hand_links_vector_;
};

}

#endif
