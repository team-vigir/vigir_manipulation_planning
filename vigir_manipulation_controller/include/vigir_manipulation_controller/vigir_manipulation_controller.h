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

#include <math.h>
#include <iostream>
#include <vector>
#include <fstream>

#include <boost/algorithm/string.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread.hpp>

#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>

#include <flor_grasp_msgs/GraspSelection.h>
#include <flor_grasp_msgs/TemplateSelection.h>
#include <flor_grasp_msgs/GraspState.h>
#include <flor_grasp_msgs/HandStatus.h>
#include "flor_ocs_msgs/OCSRobotStatus.h"
#include "flor_ocs_msgs/RobotStatusCodes.h"
#include "flor_control_msgs/FlorControlMode.h"
#include <flor_planning_msgs/PlanRequest.h>
#include <flor_planning_msgs/CircularMotionRequest.h>
#include <flor_planning_msgs/CartesianMotionRequest.h>
#include <flor_atlas_msgs/AtlasHandMass.h>
#include <flor_control_msgs/FlorControlModeCommand.h>

#include <actionlib/client/simple_action_client.h>
#include <vigir_planning_msgs/MoveAction.h>
#include <vigir_planning_msgs/ExtendedPlanningOptions.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/tfMessage.h>

#include <trajectory_msgs/JointTrajectory.h>

#include <vigir_object_template_msgs/GetInstantiatedGraspInfo.h>
#include <vigir_object_template_msgs/GetTemplateStateAndTypeInfo.h>
#include <vigir_object_template_msgs/SetAttachedObjectTemplate.h>
#include <vigir_object_template_msgs/Affordance.h>
#include <vigir_object_template_msgs/GetAffordanceInWristFrame.h>

//#include <vigir_manipulation_planning/vigir_planning_interface/vigir_move_group_interface/include/moveit/vigir_move_group_interface/move_group.h>
#include <moveit/vigir_move_group_interface/move_group.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/link_model.h>

#include <urdf/model.h>

namespace vigir_manipulation_controller {

    typedef enum
    {
        NO_GRASP_QUALITY            = 0,
        PALM_AND_ALL_FINGERS        ,  // palm contact with all fingers including thumb
        PALM_AND_THUMB_PLUS_ONE     ,
        PALM_AND_THUMB_PLUS_TWO     ,  // only relevant on Sandia (use ALL for iRobot)
        PALM_AND_NO_THUMB           ,  // all other fingers except thumb
        PALM_AND_NO_THUMB_LESS_ONE  ,  // no thumb and not all fingers
        NO_PALM_AND_ALL_FINGERS     ,  // No palm contact, but otherwise all fingers (including thumb) in contact
        NO_PALM_AND_NO_THUMB        ,  // all fingers except thumb
        NO_PALM_AND_THUMB_PLUS_ONE  ,
        NO_PALM_AND_THUMB_PLUS_TWO  ,  // only relevant on Sandia (use ALL for iRobot)
        NUM_GRASP_QUALITIES
    } GraspQuality;

  // This is the generic grasp controller
  class VigirManipulationController
  {
  public:

      VigirManipulationController();
      virtual ~VigirManipulationController();

      void initializeManipulationController(ros::NodeHandle &nh, ros::NodeHandle &nhp);

  protected:

   inline int16_t  getGraspStatus() { return RobotStatusCodes::status(grasp_status_code_, grasp_status_severity_);}
   void            setGraspStatus(const RobotStatusCodes::StatusCode& status, const RobotStatusCodes::StatusLevel& severity);

   void                        setLinkState(flor_grasp_msgs::LinkState link_state);
   flor_grasp_msgs::HandStatus getHandStatus() { return last_hand_status_msg_; }

   //Specific hand functions

   /* This funtion receives a grasp command.
    *
    * Grasp command has a grip in the range [0,200] where range [0,100) means percentage closure and
    * range [100,200] means percentage effort
    * */
   virtual void graspCommandCallback(const flor_grasp_msgs::GraspState &grasp)  = 0;

   /* This function needs to set the tactile information of the hand and will also return
    * a GraspQualtity evaluation    *
    */
   virtual GraspQuality processHandTactileData()                                = 0;

    flor_grasp_msgs::HandStatus                last_hand_status_msg_;
    flor_grasp_msgs::LinkState                 link_tactile_;

    flor_ocs_msgs::OCSRobotStatus              last_planner_status_msg_;
    flor_control_msgs::FlorControlMode         last_controller_mode_msg_;
    geometry_msgs::PoseStamped                 last_wrist_pose_msg_;

    // Variables to store locally sensor data recieved from ROS interface
    geometry_msgs::WrenchStamped               local_force_torque_msg_;

    std::string                                robot_description_;

    // Internal data set by the controller (specifies right or left hand)
    std::string                                wrist_name_;      // l_hand or r_hand
    std::string                                joint_group_;     // left_hand or right_hand
    std::string                                hand_link_;       // left_palm or right_palm
    std::string                                hand_side_;       // left or right
    int                                        hand_id_;         // -1=left, 1=right
    std::string                                planning_group_;

    // Internal variables used by active controllers
    vigir_object_template_msgs::GetInstantiatedGraspInfoResponse last_grasp_res_;
    vigir_object_template_msgs::GetTemplateStateAndTypeInfo      template_srv_;

    tf::Transform                              palmStitch_T_hand_;
    tf::Transform                              hand_T_palm_;
    tf::Transform                              hand_T_marker_;
//    tf::TransformListener                      listener_;
    flor_planning_msgs::PlanRequest            wrist_target_pose_;
    flor_atlas_msgs::AtlasHandMass             template_mass_msg_;
    geometry_msgs::PoseStamped                 com_;
    geometry_msgs::PoseStamped                 wrist_T_template_;

    //Grasp status message
    flor_ocs_msgs::OCSRobotStatus              grasp_status_;
    RobotStatusCodes::StatusCode               grasp_status_code_;      // Using RobotStatusCodes with severity
    RobotStatusCodes::StatusLevel              grasp_status_severity_;

    boost::mutex                               write_data_mutex_;

    moveit::planning_interface::VigirMoveGroup l_arm_group_;
    moveit::planning_interface::VigirMoveGroup r_arm_group_;

    robot_model_loader::RobotModelLoaderPtr    robot_model_loader_;
    robot_model::RobotModelPtr                 robot_model_;
    std::vector<std::string>                   hand_joint_names_;
    std::vector<float>                         hand_upper_limits_;
    std::vector<float>                         hand_lower_limits_;

  private:
    ros::Publisher     wrist_target_pub_ ;
    ros::Publisher     template_stitch_pose_pub_ ;
    ros::Publisher     wrist_plan_pub_   ;
    ros::Publisher     grasp_status_pub_ ;
    ros::Publisher     template_mass_pub_ ;
    ros::Publisher     tactile_feedback_pub_;
    ros::Publisher     circular_plan_request_pub_;
    ros::Publisher     cartesian_plan_request_pub_;

    ros::Subscriber    hand_status_sub_;
    ros::Subscriber    moveToPose_sub_;       ///< Current template and grasp selection message
    ros::Subscriber    grasp_command_sub_;         ///< Releasgrasp_joint_controller.e grasp and reset the initial finger positions
    ros::Subscriber    template_stitch_sub_;       ///< Current template pose to be stitched
    ros::Subscriber    detach_object_sub_;         ///< Detach current template
    ros::Subscriber    affordance_command_sub_;    ///< Circulat and Cartesian affordance
    ros::Subscriber    update_hand_marker_sub_;    ///< Update the pose of the marker to control the end-effector

    ros::Subscriber    force_torque_sub_;          ///< Force torque including wrists
    ros::Subscriber    current_wrist_sub_;         ///< Current wrist pose (same frame as target)
    ros::Subscriber    planner_status_sub_;        ///< Planner status (for reporting bundled error messages)
    ros::Subscriber    grasp_planning_group_sub_;


    ros::ServiceClient inst_grasp_info_client_;
    ros::ServiceClient template_info_client_;
    ros::ServiceClient stitch_object_client_;
    ros::ServiceClient detach_object_client_;

    ros::ServiceServer wrist_affordance_server_;

    /** This function is called whenever the template needs to be stitched to the real object.
     * assump template pose is given in world frame
     */
    void  templateStitchCallback(const flor_grasp_msgs::GraspSelection& grasp_msg);

    /** called to update the latest wrist pose */
    void  wristPoseCallback(const geometry_msgs::PoseStamped& wrist_pose);

    void moveToPoseCallback(const flor_grasp_msgs::GraspSelection& grasp);

    /** Set the current planning group to "arm only" or "arm + torso"           */
    void  graspPlanningGroupCallback(const std_msgs::Bool::ConstPtr& msg);
    void  affordanceCommandCallback(const vigir_object_template_msgs::Affordance &affordance);
    void  updateHandMarkerCallback(const std_msgs::Int8 &usability_id);

   /**
    * This function must be called to publish the updated wrist target after the template is updated.
    */
    void updateWristTarget();

    void updateGraspStatus(); // call to publish latest grasp data
    void updateTemplateMass(); // call to publish latest grasp data
    void processTemplateMassData(geometry_msgs::PoseStamped &template_pose, float &template_mass, geometry_msgs::Point &template_com);
    void handStatusCallback(const flor_grasp_msgs::HandStatus msg);

    void requestInstantiatedGraspService(const uint16_t& requested_template_type);

    void setDetachingObject(const flor_grasp_msgs::TemplateSelection& template_data);
    void setStitchingObject(const flor_grasp_msgs::TemplateSelection& template_data);

    void sendCircularAffordance(vigir_object_template_msgs::Affordance affordance);
    void sendCartesianAffordance(vigir_object_template_msgs::Affordance affordance);
    void sendFinalGrasp(geometry_msgs::PoseStamped final_grasp);

    bool affordanceInWristFrame(vigir_object_template_msgs::GetAffordanceInWristFrame::Request& req,
                                vigir_object_template_msgs::GetAffordanceInWristFrame::Response& res);

    // Calculate the wrist target in world frame given wrist pose in template frame
    int calcWristTarget(const geometry_msgs::Pose& wrist_pose);
    int poseTransform(geometry_msgs::Pose& input_pose, tf::Transform transform);
    int poseTransform(tf::Transform transform, geometry_msgs::Pose& input_pose);

  };


}
#endif
