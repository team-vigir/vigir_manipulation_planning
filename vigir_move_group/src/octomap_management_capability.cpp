/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, SRI, Inc.
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

/* Author: David Hershberger */

#include <vigir_move_group/octomap_management_capability.h>
#include <moveit/move_group/capability_names.h>
#include <octomap_msgs/conversions.h>

move_group::OctomapManagementCapability::OctomapManagementCapability():
  MoveGroupCapability("OctomapManagementCapability")
{

}

void move_group::OctomapManagementCapability::initialize()
{

  octree.reset(new octomap::OcTree(0.05));
  octree->readBinary("/home/kohlbrecher/argo/src/argo_scenarios_gazebo/argo_scenario_data/maps/argo_taurob_test_arena_v1/octomap.bt");
  //octree->read("/home/kohlbrecher/logs/argos/obstacles/taurob_near_stairs.ot");


  if (!octree.get()){
    ROS_WARN("Could not read octree from file, not using as prior map");
  }else{
    ROS_INFO("Successfully loaded octomap from file. Mem usage: %d", static_cast<int>(octree->memoryUsage()));

    octomap_msg.reset(new octomap_msgs::Octomap);
    if (!octomap_msgs::binaryMapToMsg(*octree, *octomap_msg)){
      ROS_ERROR("Failed conversion to octomap msg.");
      octomap_msg.reset();
    }else{
      octomap_msg->header.frame_id = "world";
      //octomap_msg->id = "blaa";

      octomap_prior_pub_ = node_handle_.advertise<octomap_msgs::Octomap>("octomap_prior_debug", 1, true);


      //planning_scene_monitor::LockedPlanningSceneRW ls (context_->planning_scene_monitor_);
      octomap_prior_pub_.publish(octomap_msg);

      //this->resetOctomapToPrior();

    }
  }

  initial_pose_sub_ = node_handle_.subscribe("/initialpose", 1, &move_group::OctomapManagementCapability::initialPoseCallback, this);

  // Reset and others
  sys_command_sub_ = node_handle_.subscribe("/syscommand", 1, &move_group::OctomapManagementCapability::sysCommandCallback, this);
}


void move_group::OctomapManagementCapability::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped pose)
{
  {
    ROS_INFO("Received intialpose, resetting planning scene octomap");

    // Important: Locking octree performed in below call, doing it again here results
    // in hanging
    context_->planning_scene_monitor_->clearOctomap();

    //planning_scene_monitor::LockedPlanningSceneRW ls (context_->planning_scene_monitor_);

    //ls.planning_scene_monitor_->getPlanningScene()->processOctomapMsg(*octomap_msgs);

    //context_->planning_scene_monitor_->getPlanningScene()->processOctomapMsg(*octomap_msg);

    this->resetOctomapToPrior();

    ROS_INFO("Finished clearing octomap");

  }
}


void move_group::OctomapManagementCapability::sysCommandCallback(const std_msgs::String::ConstPtr& msg)
{
  if (msg->data == "save_octomap"){
    std::string file_name;

    file_name =  "/tmp/octo.bt";
    {
      planning_scene_monitor::LockedPlanningSceneRO ls (context_->planning_scene_monitor_);
      collision_detection::CollisionWorld::ObjectConstPtr map = ls.getPlanningSceneMonitor()->getPlanningScene()->getWorld()->getObject("<octomap>");
      const shapes::OcTree* octree_shape = static_cast<const shapes::OcTree*>(map->shapes_[0].get());
      const boost::shared_ptr<const octomap::OcTree> octree_ = octree_shape->octree;

      ROS_INFO("Writing octomap to %s", file_name.c_str());
      octree_->write(file_name);
    }
  }
}

void move_group::OctomapManagementCapability::resetOctomapToPrior()
{
  //ls.planning_scene_monitor_->getPlanningScene()->processOctomapMsg(*octomap_msgs);
  ROS_INFO("Resetting octomap to pre-loaded state");

  //planning_scene_monitor::LockedPlanningSceneRW ls (context_->planning_scene_monitor_);

  const planning_scene::PlanningScenePtr& planning_scene = context_->planning_scene_monitor_->getPlanningScene();

  //context_->planning_scene_monitor_->octomap_monitor_->tree_->lockWrite();
  const occupancy_map_monitor::OccMapTreePtr& octree_ptr = context_->planning_scene_monitor_->octomap_monitor_->getOcTreePtr();

  octree_ptr->lockWrite();
  //octree_ptr = this->octree;

  octree_ptr->readBinary("/home/kohlbrecher/argo/src/argo_scenarios_gazebo/argo_scenario_data/maps/argo_taurob_test_arena_v1/octomap.bt");
      //octree->read("/home/kohlbrecher/logs/argos/obstacles/taurob_near_stairs.ot");

  //octomap::OcTree* tree_mon = dynamic_cast<octomap::OcTree*>(octree_ptr.get());

  //tree_mon = this->octree.get();
  //octree_ptr->re

  //octree_ptr
  octree_ptr->unlockWrite();

  //planning_scene->processOctomapMsg(*octomap_msg);



  //context_->planning_scene_monitor_->getPlanningScene()->processOctomapPtr(octree, Eigen::Affine3d::Identity());
}



#include <class_loader/class_loader.h>
CLASS_LOADER_REGISTER_CLASS(move_group::OctomapManagementCapability, move_group::MoveGroupCapability)
