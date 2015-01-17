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
*   * Neither the name of the Willow Garage nor the names of its
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

//#include <moveit/move_group/names.h>
#include <vigir_move_group/continuous_replanning_capability.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>
#include <moveit_msgs/PlanningScene.h>

//#include <eigen_conversions/eigen_msg.h>






move_group::ContinuousReplanningCapability::ContinuousReplanningCapability():
  MoveGroupCapability("ContinuousReplanningCapability")
{}

void move_group::ContinuousReplanningCapability::initialize()
{  
  const planning_scene::PlanningScenePtr& planning_scene = context_->planning_scene_monitor_->getPlanningScene();

  trigger_sub_ = root_node_handle_.subscribe("/trigger_cont", 1, &ContinuousReplanningCapability::triggerCb, this);
}

void move_group::ContinuousReplanningCapability::triggerCb(const std_msgs::Empty::ConstPtr& msg)
{
  ROS_INFO("Received trigger");

  const robot_model::RobotModelConstPtr& robot_model = context_->planning_pipeline_->getRobotModel();
  const planning_scene::PlanningScenePtr& planning_scene = context_->planning_scene_monitor_->getPlanningScene();


  std::string group_name = "l_arm_group";

  planning_interface::MotionPlanRequest motion_plan_request;

  motion_plan_request.allowed_planning_time = 1.0;
  motion_plan_request.group_name = group_name;
  motion_plan_request.goal_constraints.resize(1);

  planning_interface::MotionPlanResponse mp_res;



  size_t count = 1000;
  ros::WallTime start = ros::WallTime::now();

  for (size_t i = 0; i < count; ++i){
      context_->planning_scene_monitor_->updateFrameTransforms();

      robot_state::RobotState tmp = planning_scene->getCurrentState();

      const robot_state::JointModelGroup* jmg = tmp.getJointModelGroup(group_name);

      tmp.setToRandomPositions(jmg);

      motion_plan_request.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(tmp, jmg);

      planning_scene_monitor::LockedPlanningSceneRO ps(context_->planning_scene_monitor_);

      bool solved = context_->planning_pipeline_->generatePlan(ps, motion_plan_request, mp_res);

  }

  ROS_INFO("Elapsed time %f seconds, %f seconds per planning attempt", (ros::WallTime::now()-start).toSec(), (ros::WallTime::now()-start).toSec()/static_cast<double>(count));
}


#include <class_loader/class_loader.h>
CLASS_LOADER_REGISTER_CLASS(move_group::ContinuousReplanningCapability, move_group::MoveGroupCapability)
