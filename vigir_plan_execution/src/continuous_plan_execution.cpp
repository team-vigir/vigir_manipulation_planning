//=================================================================================================
// Copyright (c) 2014, Stefan Kohlbrecher, TU Darmstadt
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


#include <vigir_plan_execution/continuous_plan_execution.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>

//#include <octomap_msgs/GetOctomap.h>
//#include <octomap_msgs/conversions.h>
//#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>

//#include <eigen_conversions/eigen_msg.h>




namespace plan_execution{

ContinuousPlanExecution::ContinuousPlanExecution(const move_group::MoveGroupContextPtr context)
  : context_(context)
{}

void ContinuousPlanExecution::initialize()
{
  //const planning_scene::PlanningScenePtr& planning_scene = context_->planning_scene_monitor_->getPlanningScene();

  //trigger_sub_ = root_node_handle_.subscribe("/trigger_cont", 1, &ContinuousPlanExecution::triggerCb, this);
  //abort_sub_ = root_node_handle_.subscribe("/abort_cont", 1, &ContinuousPlanExecution::abortCb, this);
}

void ContinuousPlanExecution::startExecution()
{
  continuous_replanning_thread_.reset(new boost::thread(boost::bind(&ContinuousPlanExecution::continuousReplanningThread, this)));
}


void ContinuousPlanExecution::stopExecution()
{
  ROS_INFO("Received abort");

  stop_continuous_replanning_ = true;

  continuous_replanning_thread_->join();
  continuous_replanning_thread_.reset();
}


void ContinuousPlanExecution::continuousReplanningThread()
{
  stop_continuous_replanning_ = false;

  const robot_model::RobotModelConstPtr& robot_model = context_->planning_pipeline_->getRobotModel();
  const planning_scene::PlanningScenePtr& planning_scene = context_->planning_scene_monitor_->getPlanningScene();


  std::string group_name = "l_arm_group";

  planning_interface::MotionPlanRequest motion_plan_request;

  motion_plan_request.allowed_planning_time = 1.0;
  motion_plan_request.group_name = group_name;
  motion_plan_request.goal_constraints.resize(1);

  planning_interface::MotionPlanResponse mp_res;

  size_t count = 100;
  ros::WallTime start = ros::WallTime::now();

  context_->planning_scene_monitor_->updateFrameTransforms();

  robot_state::RobotState tmp = planning_scene->getCurrentState();

  const robot_state::JointModelGroup* jmg = tmp.getJointModelGroup(group_name);

  tmp.setToRandomPositions(jmg);

  motion_plan_request.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(tmp, jmg);
  motion_plan_request.start_state.is_diff = true;

  for (size_t i = 0; i < count; ++i){
      context_->planning_scene_monitor_->updateFrameTransforms();

      if (stop_continuous_replanning_){
        context_->trajectory_execution_manager_->stopExecution();
        return;
      }

      bool solved = false;
      {
        planning_scene_monitor::LockedPlanningSceneRO ps(context_->planning_scene_monitor_);
        solved = context_->planning_pipeline_->generatePlan(ps, motion_plan_request, mp_res);
      }

      if (stop_continuous_replanning_){
        context_->trajectory_execution_manager_->stopExecution();
        return;
      }

      if (solved){
        moveit_msgs::RobotTrajectory robot_traj;
        mp_res.trajectory_->getRobotTrajectoryMsg(robot_traj);
        robot_traj.joint_trajectory.header.stamp = ros::Time::now() + ros::Duration(0.1);
        //context_->trajectory_execution_manager_->ensureActiveControllers();
        context_->trajectory_execution_manager_->pushAndExecute(robot_traj);
        sleep(0.1);
      }

  }
  ROS_INFO("Elapsed time %f seconds, %f seconds per planning attempt", (ros::WallTime::now()-start).toSec(), (ros::WallTime::now()-start).toSec()/static_cast<double>(count));
}

}
