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

#ifndef MOVEIT_OCS_MODEL_H__
#define MOVEIT_OCS_MODEL_H__

#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>

#include <vigir_planning_msgs/JointPositionConstraint.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>


class MoveItOcsModel{
public:
  MoveItOcsModel();

  /**
   * Set given group from IK. If IK fails, keep previous configuration.
   */
  bool setByIk(const geometry_msgs::PoseStamped& goal_pose, const std::string& group_name);

  bool getManipulationMetrics(const std::string& group_name,
                              Eigen::MatrixXcd &eigen_values,
                              Eigen::MatrixXcd &eigen_vectors) const;

  /**
   * Get group endeffector link pose
   */
  bool getLinkPose(const std::string& link_name, geometry_msgs::Pose& pose) const;
  bool getJointPose(const std::string& joint_name, geometry_msgs::Pose& pose) const;
  void setJointStates(const sensor_msgs::JointState& msg);
  void setFromState(const robot_state::RobotState& new_state);
  void getJointStates(sensor_msgs::JointState& msg) const;
  bool getGroupJointPositions(const std::string& group_name, std::vector<double>& joint_positions);

  /**
   * Set the virtual world joint values to given pose (i.e. set floating base pose)
   */
  void setRootTransform(const geometry_msgs::PoseStamped& pose);

  const std::vector<std::string>& getLinkNames() const;
  const robot_state::RobotStateConstPtr getState();

  void setJointPositionConstraints(const std::vector<moveit_msgs::JointConstraint>& constraints);

  void getCollidingLinks(std::vector<std::string>& colliding_links) const;

  //Retrieve links for which ghost state is different from provided joint state
  void getDifferingLinks(const sensor_msgs::JointState state,
                         std::vector<std::string>& differing_links);

  size_t getLinkIndex(const std::string& link_name) const;

  const moveit::core::JointModel* getJointModel(const std::string& joint_name) const;

  double getMinDistanceToPositionBounds(const moveit::core::JointModel* joint) const;

  double getJointEffortLimit(const std::string& joint_name) const;

  const std::string getRobotName() const;

  std::vector<srdf::Model::Group>  getGroups();

  const robot_model::RobotModel& getModel() const { return *robot_model_; };
  const robot_model::RobotModelConstPtr getModelConstPtr() const { return robot_model_; };

protected:

  bool checkGroupStateSelfCollisionFree(robot_state::RobotState *robot_state, const robot_state::JointModelGroup *joint_group, const double *joint_group_variable_values);

  //std::vector <vigir_planning_msgs::JointPositionConstraints> torso_joint_position_constraints_;

  std::vector<moveit_msgs::JointConstraint> torso_joint_position_constraints_;

  planning_scene::PlanningScenePtr planning_scene_;

  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  robot_model::RobotModelPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;

  kinematics_metrics::KinematicsMetricsPtr kinematics_metrics_;

  // Used to make setting virtual joint positions (-> pelvis pose) easier
  sensor_msgs::JointState virtual_link_joint_states_;

};

#endif
