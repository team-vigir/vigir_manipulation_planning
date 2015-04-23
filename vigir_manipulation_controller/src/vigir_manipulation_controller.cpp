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

#include <vigir_manipulation_controller/vigir_manipulation_controller.h>

namespace vigir_manipulation_controller{


VigirManipulationController::VigirManipulationController():
    wrist_name_( "unknown"),
    hand_side_( "unknown"),
    hand_id_(0),
    l_arm_group_("l_arm_group"),
    r_arm_group_("r_arm_group")
{
    //Stitch template to hand transformation initialization
    this->palmStitch_T_hand_.setIdentity();
}

VigirManipulationController::~VigirManipulationController()
  {
  std::cout << "Shutting down the manipulation controller ..." << std::endl;
  }


void VigirManipulationController::initializeManipulationController(ros::NodeHandle &nh, ros::NodeHandle &nhp)
  {
    ROS_INFO("Entering Initialize manipulation controller under private namespace: %s", nhp.getNamespace().c_str());

    if (!nhp.getParam("/robot_description", this->robot_description_)){
        ROS_ERROR(" Did not find parameter /robot_description");
        return;
    }

    // which hand are we controlling
    if (!nhp.getParam("wrist_name", this->wrist_name_)){
        ROS_ERROR(" Did not find wrist_name parameter - using r_hand as default");
        this->wrist_name_ = "r_hand";
    }else
        ROS_INFO("Hand parameters received, hand: %s", this->wrist_name_.c_str());

    // which joint_group are we controlling
    if (!nhp.getParam("hand_link", this->hand_link_)){
        ROS_WARN(" Did not find hand_link parameter, assuming no end-effector is being used.");
        this->hand_link_ = "right_palm";
    }else
        ROS_INFO("Hand link parameters received, hand: %s", this->hand_link_.c_str());

    // which joint_group are we controlling
    if (!nhp.getParam("joint_group", this->joint_group_)){
        ROS_WARN(" Did not find joint_group parameter - assuming end-effector has no joints");
        this->joint_group_ = "right_hand";
    }else
        ROS_INFO("Joint group parameters received, hand: %s", this->joint_group_.c_str());



    this->hand_id_            = 1;
    this->hand_side_          = "right";
    this->planning_group_     = "r_arm_group";
    if ("l_hand" == this->wrist_name_){
        this->hand_id_        = -1;
        this->hand_side_      = "left";
        this->planning_group_ = "l_arm_group";
    }

    // Not sure why this waiting is performed in the original joint control tutorial
    // Maybe related to setting transport to UDP and waiting before Publishers are online?
    ros::Time last_ros_time_;
    bool wait = true;
    while (wait)
    {
      last_ros_time_ = ros::Time::now();
      if (last_ros_time_.toSec() > 0)
        wait = false;
    }

    ROS_INFO("Setup communications for the %s manipulation controller ... ", wrist_name_.c_str());

    wrist_target_pub_           = nh.advertise<geometry_msgs::PoseStamped>("wrist_target",          1, true);
    template_stitch_pose_pub_   = nh.advertise<geometry_msgs::PoseStamped>("template_stitch_pose",  1, true);
    wrist_plan_pub_             = nh.advertise<flor_planning_msgs::PlanRequest>("wrist_plan",       1, true);
    grasp_status_pub_           = nh.advertise<flor_ocs_msgs::OCSRobotStatus>("grasp_status",       1, true);
    hand_mass_pub_              = nh.advertise<flor_atlas_msgs::AtlasHandMass>("hand_mass",         1, true);
    tactile_feedback_pub_       = nh.advertise<flor_grasp_msgs::LinkState>("link_states",           1, true);
    circular_plan_request_pub_  = nh.advertise<flor_planning_msgs::CircularMotionRequest>( "/flor/planning/upper_body/plan_circular_request",  1, false );
    cartesian_plan_request_pub_ = nh.advertise<flor_planning_msgs::CartesianMotionRequest>("/flor/planning/upper_body/plan_cartesian_request", 1, false );

    hand_status_sub_           = nh.subscribe("hand_status",        1, &VigirManipulationController::handStatusCallback,         this);
    grasp_command_sub_         = nh.subscribe("grasp_command",      1, &VigirManipulationController::graspCommandCallback,       this);
    template_stitch_sub_       = nh.subscribe("template_stitch",    1, &VigirManipulationController::templateStitchCallback,     this);
    current_wrist_sub_         = nh.subscribe("wrist_pose",         1, &VigirManipulationController::wristPoseCallback,          this);
    grasp_planning_group_sub_  = nh.subscribe("use_torso",          1, &VigirManipulationController::graspPlanningGroupCallback, this);
    moveToPose_sub_            = nh.subscribe("move_to_pose",       1, &VigirManipulationController::moveToPoseCallback,         this);
    attach_object_sub_         = nh.subscribe("attach_object",      1, &VigirManipulationController::setAttachingObject,         this);
    detach_object_sub_         = nh.subscribe("detach_object",      1, &VigirManipulationController::setDetachingObject,         this);
    affordance_command_sub_    = nh.subscribe("affordance_command", 1, &VigirManipulationController::affordanceCommandCallback,  this);
    update_hand_marker_sub_    = nh.subscribe("hand_marker",        1, &VigirManipulationController::updateHandMarkerCallback,   this);

    inst_grasp_info_client_    = nh.serviceClient<vigir_object_template_msgs::GetInstantiatedGraspInfo>("/instantiated_grasp_info");
    template_info_client_      = nh.serviceClient<vigir_object_template_msgs::GetTemplateStateAndTypeInfo>("/template_info");

    attach_object_client_      = nh.serviceClient<vigir_object_template_msgs::SetAttachedObjectTemplate>("/attach_object_template");
    stitch_object_client_      = nh.serviceClient<vigir_object_template_msgs::SetAttachedObjectTemplate>("/stitch_object_template");
    detach_object_client_      = nh.serviceClient<vigir_object_template_msgs::SetAttachedObjectTemplate>("/detach_object_template");

    //LOADING ROBOT MODEL FOR JOINT NAMES
    robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));
    robot_model_ = robot_model_loader_->getModel();

    urdf::Model robot_urdf_model_;

    //Initializing hand joint names
    hand_joint_names_.clear();

    if(robot_model_->hasJointModelGroup(joint_group_))
    {
        hand_joint_names_ = robot_model_->getJointModelGroup(joint_group_)->getActiveJointModelNames();

        ROS_INFO("%s %s hand model gotten, #actuated joints: %ld ",hand_side_.c_str(), robot_model_->getName().c_str(),hand_joint_names_.size() );

        //Getting joint limits from URDF
        if (!robot_urdf_model_.initString(this->robot_description_)){
          ROS_ERROR("Failed to parse robot description");
          return;
        }else{
            for(int i = 0; i < hand_joint_names_.size(); i++){
                hand_upper_limits_.push_back(robot_urdf_model_.getJoint(hand_joint_names_[i])->limits->upper);
                hand_lower_limits_.push_back(robot_urdf_model_.getJoint(hand_joint_names_[i])->limits->lower);
            }
        }
    }else{
        ROS_WARN("NO JOINTS FOUND FOR %s HAND",joint_group_.c_str());
    }

    //Initializing hand transformation in palm frame to identity (no end-effector used)
    hand_T_palm_.setIdentity();

    if(!robot_model_->hasLinkModel(hand_link_)){
        ROS_WARN("Hand model does not contain %s_palm link",hand_side_.c_str());
    }else{
        robot_model::LinkTransformMap hand_palm_tf_map = robot_model_->getLinkModel(hand_link_)->getAssociatedFixedTransforms();
        ROS_INFO("Requested linktransform for %s_palm",hand_side_.c_str());

        Eigen::Affine3d hand_palm_aff;
        bool found = false;

        for(robot_model::LinkTransformMap::iterator it = hand_palm_tf_map.begin(); it != hand_palm_tf_map.end(); ++it){
            ROS_INFO("Getting links in map: %s and comparing to: %s", it->first->getName().c_str(), (wrist_name_).c_str());
            if(it->first->getName() == wrist_name_){
                ROS_INFO("Wrist %s found!!!",wrist_name_.c_str());
                hand_palm_aff = it->second;
                found = true;
                break;
            }
        }

        if(!found){
            ROS_WARN("Wrist %c_hand NOT found!!!, setting to identity",hand_side_[0]);
        }else{
            //Initializing palm transformation in hand frame
            tf::transformEigenToTF( hand_palm_aff,hand_T_palm_);
            //We got palm_T_hand, need to invert
            hand_T_palm_ = hand_T_palm_.inverse();
        }
    }
}

///////////////////////////////////////////////////////
// Class Callback functions for ros subscribers


void VigirManipulationController::graspPlanningGroupCallback(const std_msgs::Bool::ConstPtr& msg)
{
    bool use_torso = msg->data;
    if (use_torso)
    {
        if (hand_id_>0)
            planning_group_ = "r_arm_with_torso_group";
        else
            planning_group_ = "l_arm_with_torso_group";
    }
    else
    {
        if (hand_id_>0)
            planning_group_ = "r_arm_group";
        else
            planning_group_ = "l_arm_group";
    }
}

void VigirManipulationController::templateStitchCallback(const flor_grasp_msgs::GraspSelection& grasp_msg)
{


    //Call service for template info
    vigir_object_template_msgs::GetTemplateStateAndTypeInfo template_srv_;
    template_srv_.request.template_id = grasp_msg.template_id.data;
    template_srv_.request.hand_side = template_srv_.request.BOTH_HANDS;
    if (!template_info_client_.call(template_srv_))
    {
        ROS_ERROR("Failed to call service request grasp info");
    }else{

        geometry_msgs::PoseStamped stitch_template_pose;

        //Set static transform to identity unstitching the template
        this->palmStitch_T_hand_.setIdentity();

        if(grasp_msg.final_pose && template_srv_.response.template_state_information.pose.header.frame_id == "/world") //using final pose as is_stitched?
        {
            //Calculate static transform to stitch template to hand
            tf::Transform world_T_hand;
            tf::Transform world_T_template;
            tf::Transform world_T_palm;

            world_T_hand.setIdentity();
            world_T_template.setIdentity();
            world_T_palm.setIdentity();


            int index;

            for(index = 0; index < template_srv_.response.template_type_information.grasps.size(); index++)
            {
                if(std::atoi(template_srv_.response.template_type_information.grasps[index].id.c_str()) == grasp_msg.grasp_id.data){
                    //Grasp pose in template frame
                    world_T_palm.setRotation(tf::Quaternion(template_srv_.response.template_type_information.grasps[index].grasp_pose.pose.orientation.x,
                                                               template_srv_.response.template_type_information.grasps[index].grasp_pose.pose.orientation.y,
                                                               template_srv_.response.template_type_information.grasps[index].grasp_pose.pose.orientation.z,
                                                               template_srv_.response.template_type_information.grasps[index].grasp_pose.pose.orientation.w));
                    world_T_palm.setOrigin(tf::Vector3(template_srv_.response.template_type_information.grasps[index].grasp_pose.pose.position.x,
                                                          template_srv_.response.template_type_information.grasps[index].grasp_pose.pose.position.y,
                                                          template_srv_.response.template_type_information.grasps[index].grasp_pose.pose.position.z));
                }
            }

            //Wrist pose in world frame
            world_T_hand.setRotation(tf::Quaternion(this->last_wrist_pose_msg_.pose.orientation.x,
                                                    this->last_wrist_pose_msg_.pose.orientation.y,
                                                    this->last_wrist_pose_msg_.pose.orientation.z,
                                                    this->last_wrist_pose_msg_.pose.orientation.w));
            world_T_hand.setOrigin(tf::Vector3(this->last_wrist_pose_msg_.pose.position.x,
                                               this->last_wrist_pose_msg_.pose.position.y,
                                               this->last_wrist_pose_msg_.pose.position.z));

            //Template pose in world frame
            world_T_template.setRotation(tf::Quaternion(template_srv_.response.template_state_information.pose.pose.orientation.x,
                                                        template_srv_.response.template_state_information.pose.pose.orientation.y,
                                                        template_srv_.response.template_state_information.pose.pose.orientation.z,
                                                        template_srv_.response.template_state_information.pose.pose.orientation.w));
            world_T_template.setOrigin(tf::Vector3(template_srv_.response.template_state_information.pose.pose.position.x,
                                                   template_srv_.response.template_state_information.pose.pose.position.y,
                                                   template_srv_.response.template_state_information.pose.pose.position.z));

            this->palmStitch_T_hand_ = (world_T_template.inverse() * world_T_palm).inverse() * world_T_template.inverse() * world_T_hand;


            stitch_template_pose.header.frame_id = template_srv_.response.template_state_information.pose.header.frame_id;
            stitch_template_pose.header.seq++;
            stitch_template_pose.header.stamp = template_srv_.response.template_state_information.pose.header.stamp;
        }

        //Publish to OCS
        if (template_stitch_pose_pub_)
        {
            flor_grasp_msgs::TemplateSelection last_template_data;
            last_template_data.template_id = grasp_msg.template_id;
            this->setStitchingObject(last_template_data); //Stitching collision object to robot

            stitch_template_pose.pose.position.x = this->palmStitch_T_hand_.getOrigin().getX();
            stitch_template_pose.pose.position.y = this->palmStitch_T_hand_.getOrigin().getY();
            stitch_template_pose.pose.position.z = this->palmStitch_T_hand_.getOrigin().getZ();
            stitch_template_pose.pose.orientation.w = this->palmStitch_T_hand_.getRotation().getW();
            stitch_template_pose.pose.orientation.x = this->palmStitch_T_hand_.getRotation().getX();
            stitch_template_pose.pose.orientation.y = this->palmStitch_T_hand_.getRotation().getY();
            stitch_template_pose.pose.orientation.z = this->palmStitch_T_hand_.getRotation().getZ();

            template_stitch_pose_pub_.publish(stitch_template_pose);
        }
        else
            ROS_WARN("Invalid template stitch pose publisher");

    }
}

/** called to update the latest wrist pose */
void  VigirManipulationController::wristPoseCallback(const geometry_msgs::PoseStamped& wrist_pose)
{
    // Store the latest wrist data, and update at next calculation loop
    {
        boost::lock_guard<boost::mutex> guard(this->write_data_mutex_);
        this->last_wrist_pose_msg_ = wrist_pose;
    }
     return;
}

void VigirManipulationController::moveToPoseCallback(const flor_grasp_msgs::GraspSelection& grasp)
{
    // Store the latest grasp command, and update at next calculation loop
    {
        boost::lock_guard<boost::mutex> guard(this->write_data_mutex_);
        requestInstantiatedGraspService(grasp.template_id.data);
        geometry_msgs::PoseStamped grasp_pose;
        geometry_msgs::PoseStamped pre_grasp_pose;

        size_t size = last_grasp_res_.grasp_information.grasps.size();

        if(size == 0)
            ROS_ERROR_STREAM("No grasps found for this template");
        else{
            size_t index = 0;
            for(; index < size; ++index)
            {
                if(std::atoi(last_grasp_res_.grasp_information.grasps[index].id.c_str()) == grasp.grasp_id.data){
                    grasp_pose     = last_grasp_res_.grasp_information.grasps[index].grasp_pose;
                    pre_grasp_pose = last_grasp_res_.pre_grasp_information.grasps[index].grasp_pose;
                    break;
                }
            }

            if(index >= size)
                ROS_ERROR_STREAM("Template server response id: " << last_grasp_res_.grasp_information.grasps[index].id << " while searching for id: " << grasp.grasp_id.data);
            else{
                this->wrist_target_pose_.planning_group.data                     = planning_group_;
                this->wrist_target_pose_.use_environment_obstacle_avoidance.data = true;
                if(grasp.final_pose){
                    this->wrist_target_pose_.pose = grasp_pose;
                    calcWristTarget(grasp_pose.pose);  //Applies stitch transform in hand frame
                    poseTransform(this->wrist_target_pose_.pose.pose, hand_T_palm_);  //Transform back to palm frame
                    vigir_object_template_msgs::Affordance affordance;
                    affordance.keep_orientation = true;
                    affordance.waypoints.push_back(this->wrist_target_pose_.pose);
                    sendCartesianAffordance(affordance);
                    wrist_target_pub_.publish(wrist_target_pose_.pose);
                }else{
                    this->wrist_target_pose_.pose = pre_grasp_pose;
                    calcWristTarget(pre_grasp_pose.pose);
                    this->updateWristTarget();
                }
            }
        }
    }
     return;
}

void VigirManipulationController::affordanceCommandCallback(const vigir_object_template_msgs::Affordance& affordance)
{
    //Need to get latest affordance pose
    if(affordance.type == "circular")
        sendCircularAffordance(affordance);
    else
        sendCartesianAffordance(affordance);
}

void VigirManipulationController::updateHandMarkerCallback(const geometry_msgs::Pose& hand_T_marker)
{
    hand_T_palm_.setOrigin(tf::Vector3(hand_T_marker.position.x,
                                       hand_T_marker.position.y,
                                       hand_T_marker.position.z));

    hand_T_palm_.setRotation(tf::Quaternion(hand_T_marker.orientation.x,
                                            hand_T_marker.orientation.y,
                                            hand_T_marker.orientation.z,
                                            hand_T_marker.orientation.w));
}

// Called because of stitching functionality
int VigirManipulationController::calcWristTarget(const geometry_msgs::Pose& wrist_pose)
{
    tf::Transform world_T_hand;
    tf::Transform world_T_hand_target;
    world_T_hand.setRotation(tf::Quaternion(wrist_pose.orientation.x,wrist_pose.orientation.y,wrist_pose.orientation.z,wrist_pose.orientation.w));
    world_T_hand.setOrigin(tf::Vector3(wrist_pose.position.x,wrist_pose.position.y,wrist_pose.position.z) );
    world_T_hand_target =  world_T_hand * this->palmStitch_T_hand_;  //palm_T_hand is identity when no stitching
    tf::Quaternion tg_quat;
    tf::Vector3    tg_vector;
    tg_quat   = world_T_hand_target.getRotation();
    tg_vector = world_T_hand_target.getOrigin();

    this->wrist_target_pose_.pose.pose.orientation.w = tg_quat.getW();
    this->wrist_target_pose_.pose.pose.orientation.x = tg_quat.getX();
    this->wrist_target_pose_.pose.pose.orientation.y = tg_quat.getY();
    this->wrist_target_pose_.pose.pose.orientation.z = tg_quat.getZ();

    this->wrist_target_pose_.pose.pose.position.x = tg_vector.getX();
    this->wrist_target_pose_.pose.pose.position.y = tg_vector.getY();
    this->wrist_target_pose_.pose.pose.position.z = tg_vector.getZ();

    return 0;
}

int VigirManipulationController::poseTransform(geometry_msgs::Pose& input_pose, tf::Transform transform)
{
    tf::Transform output_transform;    //describes hand in object's frame
    tf::Transform input_transform;       //describes palm_from_graspit in object's frame

    input_transform.setRotation(tf::Quaternion(input_pose.orientation.x,input_pose.orientation.y,input_pose.orientation.z,input_pose.orientation.w));
    input_transform.setOrigin(tf::Vector3(input_pose.position.x,input_pose.position.y,input_pose.position.z) );

    output_transform = input_transform * transform;

    tf::Quaternion output_quat;
    tf::Vector3    output_vector;
    output_quat   = output_transform.getRotation();
    output_vector = output_transform.getOrigin();

    input_pose.position.x    = output_vector.getX();
    input_pose.position.y    = output_vector.getY();
    input_pose.position.z    = output_vector.getZ();
    input_pose.orientation.x = output_quat.getX();
    input_pose.orientation.y = output_quat.getY();
    input_pose.orientation.z = output_quat.getZ();
    input_pose.orientation.w = output_quat.getW();
    return 0;
}

void VigirManipulationController::setGraspStatus(const RobotStatusCodes::StatusCode &status, const RobotStatusCodes::StatusLevel &severity)
{
    if ((RobotStatusCodes::NO_ERROR == this->grasp_status_code_)  || (RobotStatusCodes::GRASP_CONTROLLER_OK == this->grasp_status_code_))
    {
        this->grasp_status_code_      = status;
        this->grasp_status_severity_  = severity;
    }
    else
    {
        uint16_t current_code;
        uint8_t  current_severity;
        RobotStatusCodes::codes(this->grasp_status_code_, current_code, current_severity);
        if (this->grasp_status_severity_ < severity)
        {
            ROS_DEBUG(" Overwriting grasp controller error code %d:%d with %d:%d", this->grasp_status_code_, this->grasp_status_severity_, status, severity);
            this->grasp_status_code_      = status;
            this->grasp_status_severity_  = severity;
            return;
        }
    }
}

void VigirManipulationController::setLinkState(flor_grasp_msgs::LinkState link_state){
    link_tactile_ = link_state;
}

void VigirManipulationController::updateWristTarget()
{
    if (wrist_target_pub_ && wrist_plan_pub_)
    {
        wrist_target_pub_.publish(wrist_target_pose_.pose);
        wrist_plan_pub_.publish(wrist_target_pose_);
    }
    else
        ROS_WARN("Invalid wrist target publisher");
}

/**
 * This function must be called to publish the grasp state machine status.
 */
inline void VigirManipulationController::updateGraspStatus()
{

    uint16_t current_status = RobotStatusCodes::status(this->grasp_status_code_,this->grasp_status_severity_);
    if (this->grasp_status_code_ == RobotStatusCodes::NO_ERROR)
    {
        // Assign a meaningful message to robot_status for annunciator window
        this->grasp_status_code_ = RobotStatusCodes::GRASP_CONTROLLER_OK;
    }
    if (current_status != grasp_status_.status)
    {

        ROS_INFO("   Update Grasp Status %d:%d  for %s", this->grasp_status_code_,this->grasp_status_severity_, this->wrist_name_.c_str());
        grasp_status_.stamp  = ros::Time::now();
        grasp_status_.status = current_status;

        if (grasp_status_pub_)
            grasp_status_pub_.publish(grasp_status_);
        else
            ROS_WARN("Invalid grasp status publisher");
    }
}

inline void VigirManipulationController::updateHandMass()
{

    if (hand_mass_pub_)
        hand_mass_pub_.publish(this->hand_mass_msg_);
    else
        ROS_WARN("Invalid hand_mass_pub_");
}

void VigirManipulationController::processHandMassData(const tf::Transform& hand_T_template, float &template_mass, tf::Vector3 &template_com)
{

}

void VigirManipulationController::handStatusCallback(const flor_grasp_msgs::HandStatus msg)
{
    {
        boost::lock_guard<boost::mutex> guard(this->write_data_mutex_);
        last_hand_status_msg_ = msg;
    }

    this->processHandTactileData();

    if(tactile_feedback_pub_)
        tactile_feedback_pub_.publish(link_tactile_);
}

void VigirManipulationController::requestInstantiatedGraspService(const uint16_t& requested_template_id){
    //CALLING THE TEMPLATE SERVER
    vigir_object_template_msgs::GetInstantiatedGraspInfo srv;
    srv.request.template_id = requested_template_id;
    srv.request.hand_side   = srv.request.BOTH_HANDS;
    if (!inst_grasp_info_client_.call(srv))
    {
        ROS_ERROR("Failed to call service request grasp info");
    }
    last_grasp_res_ = srv.response;
}

void VigirManipulationController::setAttachingObject(const flor_grasp_msgs::TemplateSelection& template_data){
    //Add collision object with template pose and bounding box

    ROS_INFO("Attaching collision object :%s started",(boost::to_string(int16_t(template_data.template_id.data))).c_str());
    vigir_object_template_msgs::SetAttachedObjectTemplate srv;
    srv.request.template_id          = int16_t(template_data.template_id.data);
    srv.request.pose                 = last_wrist_pose_msg_;
    srv.request.pose.header.frame_id = this->wrist_name_;
    if (!attach_object_client_.call(srv))
        ROS_ERROR("Failed to call service request SetAttachedObjectTemplate");
}

void VigirManipulationController::setStitchingObject(const flor_grasp_msgs::TemplateSelection& template_data){
    //Add collision object with template pose and bounding box

    ROS_INFO("Stitching collision object :%s started",(boost::to_string(int16_t(template_data.template_id.data))).c_str());
    vigir_object_template_msgs::SetAttachedObjectTemplate srv;
    srv.request.template_id          = int16_t(template_data.template_id.data);
    srv.request.pose                 = last_wrist_pose_msg_;
    srv.request.pose.header.frame_id = this->wrist_name_;
    if (!stitch_object_client_.call(srv))
        ROS_ERROR("Failed to call service request SetStitchedObjectTemplate");
}

void VigirManipulationController::setDetachingObject(const flor_grasp_msgs::TemplateSelection& template_data){
    //Add collision object with template pose and bounding box

    ROS_INFO("Removing collision object :%s started",(boost::to_string(int16_t(template_data.template_id.data))).c_str());
    vigir_object_template_msgs::SetAttachedObjectTemplate srv;
    srv.request.template_id          = int16_t(template_data.template_id.data);
    srv.request.pose                 = last_wrist_pose_msg_;
    srv.request.pose.header.frame_id = this->wrist_name_;
    if (!detach_object_client_.call(srv))
        ROS_ERROR("Failed to call service request DetachObjectTemplate");
}

void VigirManipulationController::sendCircularAffordance(vigir_object_template_msgs::Affordance affordance)
{
    flor_planning_msgs::CircularMotionRequest cmd;

    // calculating the rotation based on position of the markers
    if(affordance.keep_orientation)
    {
        // get position of the wrist in world coordinates
        geometry_msgs::Pose hand = last_wrist_pose_msg_.pose;

        // get position of the marker in world coordinates
        poseTransform(hand, hand_T_palm_);

        // calculate the difference between them
        tf::Vector3 diff_vector;
        diff_vector.setX(last_wrist_pose_msg_.pose.position.x - hand.position.x);
        diff_vector.setY(last_wrist_pose_msg_.pose.position.y - hand.position.y);
        diff_vector.setZ(last_wrist_pose_msg_.pose.position.z - hand.position.z);

        // apply the difference to the circular center
        affordance.waypoints[0].pose.position.x += diff_vector.getX();
        affordance.waypoints[0].pose.position.y += diff_vector.getY();
        affordance.waypoints[0].pose.position.z += diff_vector.getZ();
    }

    cmd.rotation_center_pose = affordance.waypoints[0];

    cmd.rotation_angle = affordance.displacement;

    cmd.use_environment_obstacle_avoidance = false;

    cmd.keep_endeffector_orientation = affordance.keep_orientation;

    cmd.planning_group = this->planning_group_;

    circular_plan_request_pub_.publish(cmd);
}

void VigirManipulationController::sendCartesianAffordance(vigir_object_template_msgs::Affordance affordance)
{
    flor_planning_msgs::CartesianMotionRequest cmd;

    actionlib::SimpleActionClient<vigir_planning_msgs::MoveAction> move_action_client("/vigir_move_group",true);



    ROS_INFO("Waiting for move action server to start.");
    if(!move_action_client.waitForServer(ros::Duration(5))){
        ROS_ERROR("Move group client timed out");
        return;
    }

    ROS_INFO("Action server started, sending goal.");
    vigir_planning_msgs::MoveGoal move_goal;

    move_goal.extended_planning_options.target_motion_type                 = vigir_planning_msgs::ExtendedPlanningOptions::TYPE_CARTESIAN_WAYPOINTS;
    move_goal.extended_planning_options.avoid_collisions                   = false;
    move_goal.extended_planning_options.keep_endeffector_orientation       = affordance.keep_orientation;
    move_goal.extended_planning_options.execute_incomplete_cartesian_plans = true;
    move_goal.request.group_name                                           = this->planning_group_;
    move_goal.request.allowed_planning_time                                = 1.0;
    move_goal.request.num_planning_attempts                                = 1;





    cmd.header.frame_id = "/world";
    cmd.header.stamp = ros::Time::now();

    for(int waypoint=0; waypoint< affordance.waypoints.size(); waypoint++){
        cmd.waypoints.push_back(affordance.waypoints[waypoint].pose);
        move_goal.extended_planning_options.target_frame = affordance.waypoints[waypoint].header.frame_id;
        move_goal.extended_planning_options.target_poses.push_back(affordance.waypoints[waypoint].pose);
    }

    // get position of the wrist in world coordinates
    geometry_msgs::Pose hand = last_wrist_pose_msg_.pose;

    // get position of the marker in world coordinates
    poseTransform(hand, hand_T_palm_);

    // calculate the difference between them
    tf::Vector3 diff_vector;
    diff_vector.setX(last_wrist_pose_msg_.pose.position.x - hand.position.x);
    diff_vector.setY(last_wrist_pose_msg_.pose.position.y - hand.position.y);
    diff_vector.setZ(last_wrist_pose_msg_.pose.position.z - hand.position.z);

    for(int i = 0; i < cmd.waypoints.size(); i++)
    {
        // apply the difference to each one of the waypoints
        if(affordance.keep_orientation)
        {
            cmd.waypoints[i].position.x = cmd.waypoints[i].position.x + diff_vector.getX();
            cmd.waypoints[i].position.y = cmd.waypoints[i].position.y + diff_vector.getY();
            cmd.waypoints[i].position.z = cmd.waypoints[i].position.z + diff_vector.getZ();
            cmd.waypoints[i].orientation.x = last_wrist_pose_msg_.pose.orientation.x;
            cmd.waypoints[i].orientation.y = last_wrist_pose_msg_.pose.orientation.y;
            cmd.waypoints[i].orientation.z = last_wrist_pose_msg_.pose.orientation.z;
            cmd.waypoints[i].orientation.w = last_wrist_pose_msg_.pose.orientation.w;

            ROS_ERROR_STREAM("cmd waypoints in " << i << " = " << std::endl << cmd.waypoints[i] );

            move_goal.extended_planning_options.target_poses[i].position.x = cmd.waypoints[i].position.x + diff_vector.getX();
            move_goal.extended_planning_options.target_poses[i].position.y = cmd.waypoints[i].position.y + diff_vector.getY();
            move_goal.extended_planning_options.target_poses[i].position.z = cmd.waypoints[i].position.z + diff_vector.getZ();
            move_goal.extended_planning_options.target_poses[i].orientation.x = last_wrist_pose_msg_.pose.orientation.x;
            move_goal.extended_planning_options.target_poses[i].orientation.y = last_wrist_pose_msg_.pose.orientation.y;
            move_goal.extended_planning_options.target_poses[i].orientation.z = last_wrist_pose_msg_.pose.orientation.z;
            move_goal.extended_planning_options.target_poses[i].orientation.w = last_wrist_pose_msg_.pose.orientation.w;

            ROS_ERROR_STREAM("target poses in " << i << " = " << std::endl << move_goal.extended_planning_options.target_poses[i] );


        }
        else
        {
            geometry_msgs::Pose waypoint;
            waypoint.position.x = cmd.waypoints[i].position.x;
            waypoint.position.y = cmd.waypoints[i].position.y;
            waypoint.position.z = cmd.waypoints[i].position.z;
            waypoint.orientation.x = cmd.waypoints[i].orientation.x;
            waypoint.orientation.y = cmd.waypoints[i].orientation.y;
            waypoint.orientation.z = cmd.waypoints[i].orientation.z;
            waypoint.orientation.w = cmd.waypoints[i].orientation.w;

            poseTransform(waypoint, hand_T_palm_.inverse());

            cmd.waypoints[i] = waypoint;
            move_goal.extended_planning_options.target_poses[i] = waypoint;
        }

    }

    cmd.use_environment_obstacle_avoidance = false;

    cmd.planning_group = this->planning_group_;

    //cartesian_plan_request_pub_.publish(cmd);


    move_action_client.sendGoal(move_goal);

    //wait for the action to return
    bool finished_before_timeout = move_action_client.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = move_action_client.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_ERROR("Action did not finish before the time out.");
}


} // end of vigir_manipulation_controller namespace

//int main(int argc, char** argv)
//{
//  ros::init(argc, argv, "vigir_manipualtion_controller");

//  ros::NodeHandle nh;
//  ros::NodeHandle nhp("~");

//  vigir_manipulation_controller::VigirManipulationController manipulation_controller;

//  ROS_WARN(" Initialize the Vigir Manipulaiton controller ...");
//  manipulation_controller.initializeManipulationController(nh, nhp);

//  ROS_WARN(" Start the ros spinner ...");
//  ros::spin();
//  return 0;
//}
