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

#include <vigir_ocs_robot_model/moveit_ocs_model.h>

//#include <flor_moveit_tools/group_utils.h>
//#include <flor_moveit_tools/transform_utils.h>
//#include <flor_moveit_tools/planner_setting_utils.h>

#include <eigen_conversions/eigen_msg.h>
#include <vigir_moveit_utils/group_utils.h>

MoveItOcsModel::MoveItOcsModel()
{
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader());
  robot_model_ = robot_model_loader_->getModel();
  robot_state_.reset(new robot_state::RobotState(robot_model_));

  kinematics_metrics_.reset(new kinematics_metrics::KinematicsMetrics(robot_model_));

  planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));

  // Can print info about links, joints, planning groups etc. to console
  //robot_model_->printModelInfo();

  virtual_link_joint_states_.name.push_back("world_virtual_joint/trans_x");
  virtual_link_joint_states_.name.push_back("world_virtual_joint/trans_y");
  virtual_link_joint_states_.name.push_back("world_virtual_joint/trans_z");
  virtual_link_joint_states_.name.push_back("world_virtual_joint/rot_x");
  virtual_link_joint_states_.name.push_back("world_virtual_joint/rot_y");
  virtual_link_joint_states_.name.push_back("world_virtual_joint/rot_z");
  virtual_link_joint_states_.name.push_back("world_virtual_joint/rot_w");

  virtual_link_joint_states_.position.resize(7);

  for (size_t i = 0; i < 6; ++i){
    virtual_link_joint_states_.position[i] = 0.0;
  }
  virtual_link_joint_states_.position[6] = 1.0;

  robot_state_->setToDefaultValues();

  //planner_setting_utils::setDefaultPlanningConstraints(torso_joint_position_constraints_);

}

/**
 * Set given group from IK. If IK fails, keep previous configuration.
 */
bool MoveItOcsModel::setByIk(const geometry_msgs::PoseStamped& goal_pose, const std::string& group_name)
{
  const robot_state::JointModelGroup* joint_model_group = robot_state_->getJointModelGroup(group_name);

  return group_utils::setJointModelGroupFromIk(*robot_state_,joint_model_group, goal_pose.pose, torso_joint_position_constraints_, boost::bind(&MoveItOcsModel::checkGroupStateSelfCollisionFree, this, _1, _2, _3));
}

bool MoveItOcsModel::getManipulationMetrics(const std::string& group_name,
                            Eigen::MatrixXcd &eigen_values,
                            Eigen::MatrixXcd &eigen_vectors) const
{
  const robot_state::JointModelGroup* joint_model_group = robot_state_->getJointModelGroup(group_name);

  kinematics_metrics_->getManipulabilityEllipsoid(*robot_state_,
                                                  joint_model_group,
                                                  eigen_values,
                                                  eigen_vectors);
  return true;
}

/**
 * Get group endeffector link pose
 */
bool MoveItOcsModel::getLinkPose(const std::string& link_name, geometry_msgs::Pose& pose) const
{
  if (robot_model_->hasLinkModel(link_name)){
    const Eigen::Affine3d& link_state = robot_state_->getGlobalLinkTransform(link_name);


    //transform_utils::eigenPose2Msg (link_state, pose);
    tf::poseEigenToMsg(link_state, pose);
    return true;
  }else{
    ROS_ERROR("Tried to lookup pose for link %s which doesn't exist", link_name.c_str());
    return false;
  }
}

bool MoveItOcsModel::getJointPose(const std::string& joint_name, geometry_msgs::Pose& pose) const
{
  const Eigen::Affine3d& joint_state = robot_state_->getJointTransform(robot_state_->getJointModel(joint_name));

  //transform_utils::eigenPose2Msg (joint_state, pose);
  tf::poseEigenToMsg(joint_state, pose);

  return true;
}

void MoveItOcsModel::setJointStates(const sensor_msgs::JointState& msg)
{
  for (size_t i = 0; i < msg.name.size();++i){
    if (!robot_model_->hasJointModel(msg.name[i])){
      ROS_WARN("Tried to update ocs ghost model with message containing non-supported joint %s", msg.name[i].c_str());
      return;
    }
  }
  robot_state_->setVariableValues(msg);
}

void MoveItOcsModel::setFromState(const robot_state::RobotState& new_state)
{
  *robot_state_ = new_state;
}

void MoveItOcsModel::getJointStates(sensor_msgs::JointState& msg) const
{
  robotStateToJointStateMsg(*robot_state_, msg);
}


bool MoveItOcsModel::getGroupJointPositions(const std::string& group_name, std::vector<double>& joint_positions)
{
  const robot_state::JointModelGroup* joint_model_group = robot_state_->getJointModelGroup(group_name);

  if (joint_model_group == NULL){
    return false;
  }

  joint_positions.resize(joint_model_group->getVariableCount());

  robot_state_->copyJointGroupPositions(joint_model_group, &joint_positions[0]);

  return true;
}

/**
 * Set the virtual world joint values to given pose (i.e. set floating base pose)
 */
void MoveItOcsModel::setRootTransform(const geometry_msgs::PoseStamped& pose)
{
  virtual_link_joint_states_.position[0] = pose.pose.position.x;
  virtual_link_joint_states_.position[1] = pose.pose.position.y;
  virtual_link_joint_states_.position[2] = pose.pose.position.z;
  virtual_link_joint_states_.position[3] = pose.pose.orientation.x;
  virtual_link_joint_states_.position[4] = pose.pose.orientation.y;
  virtual_link_joint_states_.position[5] = pose.pose.orientation.z;
  virtual_link_joint_states_.position[6] = pose.pose.orientation.w;

  moveit::core::jointStateToRobotState(virtual_link_joint_states_, *robot_state_);
}

const std::vector<std::string>& MoveItOcsModel::getLinkNames() const
{
  return robot_model_->getLinkModelNames();
}

const robot_state::RobotStateConstPtr MoveItOcsModel::getState() { return robot_state_; }

void MoveItOcsModel::setJointPositionConstraints(const std::vector<moveit_msgs::JointConstraint>& constraints)
{
  torso_joint_position_constraints_ = constraints;
}

void MoveItOcsModel::getCollidingLinks(std::vector<std::string>& colliding_links) const
{
  colliding_links.clear();
  planning_scene_->getCollidingLinks(colliding_links, *robot_state_);
}

void MoveItOcsModel::getDifferingLinks(const sensor_msgs::JointState state,
                                       std::vector<std::string>& differing_links)
{
  const std::vector<std::string>& link_names = robot_model_->getLinkModelNames();

  //tmp var used below
  robot_state::RobotState robot_state_real(robot_model_);
  jointStateToRobotState(state, robot_state_real);

  for(int i = 0; i < link_names.size(); ++i)
  {
     const std::string& link_name = link_names[i];

     const Eigen::Affine3d& link_state_ghost = robot_state_->getGlobalLinkTransform(link_name);
     const Eigen::Affine3d& link_state_real  = robot_state_real.getGlobalLinkTransform(link_name);

     //Checking for translation is enough
     if((link_state_ghost.translation() - link_state_real.translation()).norm() > 0.01)
         differing_links.push_back(link_name);
  }
}

size_t MoveItOcsModel::getLinkIndex(const std::string& link_name) const
{
  return robot_model_->getLinkModel(link_name)->getLinkIndex();
}

const moveit::core::JointModel* MoveItOcsModel::getJointModel(const std::string& joint_name) const
{
  return robot_state_->getJointModel(joint_name);
}

double MoveItOcsModel::getMinDistanceToPositionBounds(const moveit::core::JointModel* joint) const
{
  std::vector<const moveit::core::JointModel*> jointContainer;
  jointContainer.push_back(joint);
  return robot_state_->getMinDistanceToPositionBounds(jointContainer).first;
}

double MoveItOcsModel::getJointEffortLimit(const std::string& joint_name) const
{
  return robot_model_->getURDF()->getJoint(joint_name)->limits->effort;
}

const std::string MoveItOcsModel::getRobotName() const
{
  return robot_model_->getURDF()->getName();
}

std::vector<srdf::Model::Group>  MoveItOcsModel::getGroups()
{
    return robot_model_->getSRDF()->getGroups();
}

bool MoveItOcsModel::checkGroupStateSelfCollisionFree(robot_state::RobotState *robot_state, const robot_state::JointModelGroup *joint_group, const double *joint_group_variable_values)
{
    collision_detection::CollisionRequest request;
    collision_detection::CollisionResult result;
    robot_state->setJointGroupPositions(joint_group, joint_group_variable_values);
    planning_scene_->checkSelfCollision(request, result, *robot_state);
    return !result.collision;
}


