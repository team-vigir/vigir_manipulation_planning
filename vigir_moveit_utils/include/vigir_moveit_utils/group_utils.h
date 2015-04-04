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

#ifndef GROUP_UTILS_H__
#define GROUP_UTILS_H__

#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/joint_model_group.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/JointConstraint.h>

#include <flor_planning_msgs/JointPositionConstraints.h>

#include <eigen_conversions/eigen_msg.h>

namespace group_utils{

  static void setJointIkConstraint(const moveit_msgs::JointConstraint& constraint,
                                   const robot_model::JointModelGroup* group,
                                   robot_state::RobotState &state,
                                   std::vector<double>& consistency_limits,
                                   std::vector<std::string>& redundant_joints_vector)
  {

    //const robot_model::JointModelGroup* group = joint_state_group->getJointModelGroup();

    if (!group->hasJointModel(constraint.joint_name)){
      return;
    }

    //const moveit::core::JointModel* joint_model  = group->getJointModel(link_name);

    int idx = group->getVariableGroupIndex(constraint.joint_name);

    if ((constraint.tolerance_above < 0.0) || (constraint.tolerance_below < 0.0)){
      ROS_ERROR("Negative joint tolerance, not using!");
    }else if (constraint.tolerance_above <= std::numeric_limits<double>::epsilon()){
      redundant_joints_vector.push_back (constraint.joint_name);
    }else if ((fabs(constraint.tolerance_above) < 4.0) && (fabs(constraint.tolerance_below) < 4.0)){

      //We cannot be sure that constraints are always given with
      //position mean of max and min, so consider that.
      //Per default mean will be equal to constraint.position though.
      double max = constraint.position + constraint.tolerance_above;
      double min = constraint.position - constraint.tolerance_below;

      double mean = (max - min) * 0.5;

      consistency_limits[idx] = max - mean;

      state.setVariablePosition(constraint.joint_name, mean);
    }
  }



  static bool setJointPlanningConstraint(moveit_msgs::JointConstraint& constraint, const std::string& name, const robot_state::RobotState& robot_state, double min_angle, double max_angle, double weight)
  {
    double limit_range = (max_angle - min_angle) * 0.5;
    double seed_angle = min_angle + limit_range;

    constraint.joint_name = name;
    constraint.position = seed_angle;
    constraint.tolerance_above =  limit_range;
    constraint.tolerance_below =  limit_range;
    constraint.weight = weight;

    //This means joint should be locked
    if ((min_angle > 99.0) && (max_angle > 99.0)){
      double angle = robot_state.getVariablePosition(name);
      constraint.position = angle;
      constraint.tolerance_above = 0.001;
      constraint.tolerance_below = 0.001;
    }


    return true;
  }

  static bool setJointStateGroupFromIk(robot_state::RobotState &state,
                                       const robot_model::JointModelGroup* group,
                                       const geometry_msgs::Pose& goal_pose,
                                       const std::vector<moveit_msgs::JointConstraint>& torso_joint_position_constraints_)
  {

    if (group == NULL){
      ROS_WARN("invalid group name, cannot plan");
      return false;
    }
    // Can now call IK on the group
    // Could query multiple groups or poses here
    ROS_DEBUG_STREAM("----- pre IK tx: " << goal_pose.position.x << " ty: " << goal_pose.position.y << " tz: " << goal_pose.position.z << "\n");
    //ROS_DEBUG_STREAM("----- pre IK frame: " << goal_pose.header.frame_id << "\n");

    //const robot_model::JointModelGroup* group = joint_state_group->getJointModelGroup();

    const kinematics::KinematicsBaseConstPtr& solver = group->getSolverInstance();

    if (!solver){
      ROS_ERROR("No IK solver loaded for group %s, cannot set group configuration via IK.", group->getName().c_str());
      return false;
    }

    const std::string& tip_frame = group->getSolverInstance()->getTipFrame();

    //Remember state of joints to reapply later in case IK fails
    sensor_msgs::JointState original_joint_state_msg;
    robotStateToJointStateMsg(state, original_joint_state_msg);

    std::vector<double> consistency_limits;
    consistency_limits.resize(group->getVariableCount(), 1000.0);

    std::vector<std::string> redundant_joints_vector;

    for (size_t i = 0; i < torso_joint_position_constraints_.size(); ++i){
      group_utils::setJointIkConstraint(torso_joint_position_constraints_[i],
                                        group,
                                        state,
                                        consistency_limits,
                                        redundant_joints_vector);
    }

    Eigen::Affine3d mat;
    tf::poseMsgToEigen(goal_pose, mat);

    bool success = false;

    if (redundant_joints_vector.size() == 0){
      success = state.setFromIK(group, mat, tip_frame, consistency_limits, 1, 0.1);
    }else{

      robot_model::JointModelGroup group_cpy = *group;

      const kinematics::KinematicsBasePtr& solver = group_cpy.getSolverInstance();

      if (!solver->setRedundantJoints(redundant_joints_vector)){
        ROS_ERROR("Failure when setting redundant joints!");
      }

      kinematics::KinematicsQueryOptions options;
      options.lock_redundant_joints = true;
      success = state.setFromIK(&group_cpy, mat, tip_frame, consistency_limits, 1, 0.1, 0, options);

      //Reset redundant joints to make sure we don't alter solver settings
      redundant_joints_vector.clear();

      if (!solver->setRedundantJoints(redundant_joints_vector)){
        ROS_ERROR("Failure when resetting redundant joints!");
      }
    }

    if (!success){
      //Set joints back to original state if IK didn't succeed
      state.setVariableValues(original_joint_state_msg);
      return false;
    }

    return true;
  }

}

#endif
