//=================================================================================================
// Copyright (c) 2015, Alberto Romay, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the names of TU Darmstadt, Virginia Tech, Oregon State, nor TORC Robotics,
//       nor the names of its contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.

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

#ifndef VIGIR_MANIPULATION_CONTROLLER_H__
#define VIGIR_MANIPULATION_CONTROLLER_H__

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/algorithm/string.hpp>
#include <vector>

#include <math.h>
#include <iostream>
#include <boost/thread/locks.hpp>
#include <fstream>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sandia_hand_msgs/RawTactile.h>

#include <flor_grasp_msgs/GraspSelection.h>
#include <flor_grasp_msgs/TemplateSelection.h>
#include <flor_grasp_msgs/GraspState.h>
#include <flor_grasp_msgs/HandStatus.h>
#include "flor_ocs_msgs/OCSRobotStatus.h"
#include "flor_ocs_msgs/OCSGhostControl.h"
#include "flor_ocs_msgs/RobotStatusCodes.h"
#include "flor_control_msgs/FlorControlMode.h"
#include <flor_planning_msgs/PlanRequest.h>
#include <flor_atlas_msgs/AtlasHandMass.h>
#include <flor_control_msgs/FlorControlModeCommand.h>

#include <boost/thread.hpp>
#include <vector>
#include <tf/tf.h>
#include <tf/transform_listener.h>


#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>

#include "geometric_shapes/mesh_operations.h"
#include "shape_msgs/Mesh.h"
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/shape_messages.h"
#include "geometric_shapes/shape_operations.h"
#include <moveit_msgs/GripperTranslation.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <vigir_object_template_msgs/GetInstantiatedGraspInfo.h>
#include <vigir_object_template_msgs/SetAttachedObjectTemplate.h>
#include <vigir_object_template_msgs/DetachObjectTemplate.h>

#include <moveit/vigir_move_group_interface/move_group.h>


namespace vigir_manipulation_controller {

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryActionClient;

  // This is the generic grasp controller
  class VigirManipulationController
  {
  public:

      VigirManipulationController();
      virtual ~VigirManipulationController();

      void initializeManipulationController(ros::NodeHandle &nh, ros::NodeHandle &nhp);

     /** This function is called whenever the template needs to be stitched to the real object.
      * assump template pose is given in world frame
      */
     void  templateStitchCallback(const flor_grasp_msgs::TemplateSelection& template_pose);

     /** called to update the latest wrist pose */
     void  wristPoseCallback(const geometry_msgs::PoseStamped& wrist_pose);

     void moveToPoseCallback(const flor_grasp_msgs::GraspSelection& grasp);

     /** called to update the latest hand offset pose */
     void handOffsetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

     /** called whenever a new template/grasp pair is selected.
      *  Reset grasping state machine, and wait for updated matching template data with new pose.
      */
     void  graspSelectionCallback(const flor_grasp_msgs::GraspSelection& grasp);

     /** Set the current planning group to "arm only" or "arm + torso"           */
     void  graspPlanningGroupCallback(const flor_ocs_msgs::OCSGhostControl& planning_group);

    /** Release any currently active grasp, and return to the NONE state after opening.
     *  This requires a new grasp selection to restart.
     */
     void  releaseGraspCallback(const flor_grasp_msgs::GraspSelection& grasp);

    /**
     * This function must be called to publish the updated wrist target after the template is updated.
     */
     void updateWristTarget();

     void updateGraspStatus(); // call to publish latest grasp data

     void updateHandMass(); // call to publish latest grasp data

     void gripperTranslationToPreGraspPose(geometry_msgs::Pose& pose, moveit_msgs::GripperTranslation& trans);

     void requestTemplateService(const uint16_t& requested_template_type);

     void setAttachingObject(const flor_grasp_msgs::TemplateSelection& last_template_data);
     void setDetachingObject(const flor_grasp_msgs::TemplateSelection& last_template_data);
     void setStitchingObject(const flor_grasp_msgs::TemplateSelection& last_template_data);


  protected:

   inline int16_t  getGraspStatus() { return RobotStatusCodes::status(grasp_status_code_, grasp_status_severity_);}
   void            setGraspStatus(const RobotStatusCodes::StatusCode& status, const RobotStatusCodes::StatusLevel& severity);



    // Variables to store sensor data received from ROS interface
    flor_grasp_msgs::GraspSelection         last_grasp_msg_;
    flor_grasp_msgs::TemplateSelection      last_template_msg_;
    flor_grasp_msgs::GraspState             last_mode_msg_;
    flor_ocs_msgs::OCSRobotStatus           last_planner_status_msg_;
    flor_control_msgs::FlorControlMode      last_controller_mode_msg_;
    geometry_msgs::PoseStamped              last_wrist_pose_msg_;
    geometry_msgs::PoseStamped              last_wrist_error;
    double                                  last_position_error;
    double                                  last_orientation_error;
    bool                                    update_error_calc;

    // Variables to store locally sensor data recieved from ROS interface
    geometry_msgs::WrenchStamped            local_force_torque_msg_;


    // Internal data set by the controller (specifies right or left hand)
    std::string                             hand_name_;       // l_hand or r_hand
    std::string                             hand_side_;       // left or right
    int                                     hand_id_;         // -1=left, 1=right
    std::string                             planning_group_;

    // Active data specified by graspSelection/templateSelection commands
    int16_t                                 grasp_id_;
    int16_t                                 template_id_;
    int16_t                                 template_type_;
    uint8_t                                 grasp_type_;
    bool                                    template_updated_;
    bool                                    stitch_updated_;


    // Internal variables used by active controllers
    vigir_object_template_msgs::GetInstantiatedGraspInfoResponse last_grasp_res_;
    geometry_msgs::Pose                     final_wrist_pose_;
    geometry_msgs::Pose                     pregrasp_wrist_pose_;
    tf::Transform                           stitch_template_pose_;
    tf::Transform                           hand_offset_pose_;
    tf::Transform                           gp_T_hand_;
    tf::Transform                           hand_T_template_;
    tf::Transform                           hand_T_palm_;
//    tf::TransformListener                   listener_;
    flor_planning_msgs::PlanRequest         wrist_target_pose_;
    flor_atlas_msgs::AtlasHandMass          hand_mass_msg_;
    geometry_msgs::PoseStamped              com_;

    //Grasp status message
    flor_ocs_msgs::OCSRobotStatus      grasp_status_;
    RobotStatusCodes::StatusCode       grasp_status_code_;      // Using RobotStatusCodes with severity
    RobotStatusCodes::StatusLevel      grasp_status_severity_;

    double                             within_range_timer_threshold_;

    // Need to define error limits in terms of position and orientation error separately
    double                             pregrasp_position_error_threshold_;
    double                             final_grasp_position_error_threshold_;
    double                             pregrasp_orientation_error_threshold_;
    double                             final_grasp_orientation_error_threshold_;

    boost::mutex                       write_data_mutex_;

    //Trajectory Action
    TrajectoryActionClient*            trajectory_client_;

    moveit::planning_interface::VigirMoveGroup l_arm_group_;
    moveit::planning_interface::VigirMoveGroup r_arm_group_;

  private:
    ros::Publisher wrist_target_pub_ ;
    ros::Publisher template_stitch_pose_pub_ ;
    ros::Publisher wrist_plan_pub_   ;
    ros::Publisher grasp_status_pub_ ;
    ros::Publisher hand_mass_pub_ ;

    ros::Subscriber grasp_selection_sub_;       ///< Current template and grasp selection message
    ros::Subscriber release_grasp_sub_;         ///< Releasgrasp_joint_controller.e grasp and reset the initial finger positions
    ros::Subscriber template_selection_sub_;    ///< Current template pose update
    ros::Subscriber hand_offset_sub_;           ///< Current hand offset pose update
    ros::Subscriber template_stitch_sub_;       ///< Current template pose to be stitched

    ros::Subscriber force_torque_sub_;          ///< Force torque including wrists
    ros::Subscriber current_wrist_sub_;         ///< Current wrist pose (same frame as target)
    ros::Subscriber planner_status_sub_;        ///< Planner status (for reporting bundled error messages)
    ros::Subscriber controller_mode_sub_;       ///< Controller mode (verify we can control appendages)
    ros::Subscriber grasp_planning_group_sub_;


    ros::ServiceClient inst_grasp_info_client_;
    ros::ServiceClient attach_object_client_;
    ros::ServiceClient stitch_object_client_;
    ros::ServiceClient detach_object_client_;

    bool evaluateWristError(const uint8_t& current_state);       // true if within limits

    // Calculate the wrist target in world frame given wrist pose in template frame
    int calcWristTarget(const geometry_msgs::Pose& wrist_pose,const geometry_msgs::PoseStamped& template_pose);

    int staticTransform(geometry_msgs::Pose& palm_pose);


  };


}
#endif
