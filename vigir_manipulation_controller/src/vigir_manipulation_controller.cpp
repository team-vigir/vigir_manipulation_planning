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
    hand_id_(0)
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
    template_mass_pub_          = nh.advertise<flor_atlas_msgs::AtlasHandMass>("hand_mass",         1, true);
    tactile_feedback_pub_       = nh.advertise<flor_grasp_msgs::LinkState>("link_states",           1, true);
    circular_plan_request_pub_  = nh.advertise<flor_planning_msgs::CircularMotionRequest>( "/flor/planning/upper_body/plan_circular_request",  1, false );
    cartesian_plan_request_pub_ = nh.advertise<flor_planning_msgs::CartesianMotionRequest>("/flor/planning/upper_body/plan_cartesian_request", 1, false );

    hand_status_sub_           = nh.subscribe("hand_status",        1, &VigirManipulationController::handStatusCallback,         this);
    grasp_command_sub_         = nh.subscribe("grasp_command",      1, &VigirManipulationController::graspCommandCallback,       this);
    template_stitch_sub_       = nh.subscribe("template_stitch",    1, &VigirManipulationController::templateStitchCallback,     this);
    current_wrist_sub_         = nh.subscribe("wrist_pose",         1, &VigirManipulationController::wristPoseCallback,          this);
    grasp_planning_group_sub_  = nh.subscribe("use_torso",          1, &VigirManipulationController::graspPlanningGroupCallback, this);
    moveToPose_sub_            = nh.subscribe("move_to_pose",       1, &VigirManipulationController::moveToPoseCallback,         this);
    detach_object_sub_         = nh.subscribe("detach_object",      1, &VigirManipulationController::setDetachingObject,         this);
    affordance_command_sub_    = nh.subscribe("affordance_command", 1, &VigirManipulationController::affordanceCommandCallback,  this);
    update_hand_marker_sub_    = nh.subscribe("hand_marker",        1, &VigirManipulationController::updateHandMarkerCallback,   this);

    inst_grasp_info_client_    = nh.serviceClient<vigir_object_template_msgs::GetInstantiatedGraspInfo>("/instantiated_grasp_info");
    template_info_client_      = nh.serviceClient<vigir_object_template_msgs::GetTemplateStateAndTypeInfo>("/template_info");

    stitch_object_client_      = nh.serviceClient<vigir_object_template_msgs::SetAttachedObjectTemplate>("/stitch_object_template");
    detach_object_client_      = nh.serviceClient<vigir_object_template_msgs::SetAttachedObjectTemplate>("/detach_object_template");

    //Affordance Services
    wrist_affordance_server_   = nh.advertiseService("wrist_affordance", &VigirManipulationController::affordanceInWristFrame, this);

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
    hand_T_marker_.setIdentity();
    hand_T_usability_.setIdentity();

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
            hand_T_palm_      = hand_T_palm_.inverse();
            hand_T_marker_    = hand_T_palm_;
        }
    }

    XmlRpc::XmlRpcValue   hand_T_marker;

    if (!nhp.getParam("/"+this->wrist_name_+"_tf/hand_T_marker", hand_T_marker))
        ROS_ERROR(" Did not find /%s_tf/hand_T_marker parameter, setting to left palm ", this->wrist_name_.c_str());
    else{
        hand_T_marker_.setOrigin(tf::Vector3(static_cast<double>(hand_T_marker[0]),static_cast<double>(hand_T_marker[1]),static_cast<double>(hand_T_marker[2])));
        hand_T_marker_.setRotation(tf::Quaternion(static_cast<double>(hand_T_marker[3]),static_cast<double>(hand_T_marker[4]),static_cast<double>(hand_T_marker[5]),static_cast<double>(hand_T_marker[6])));
    }

    hand_T_usability_ = hand_T_marker_;

    //initializing grasp status
    grasp_status_.status = RobotStatusCodes::status(RobotStatusCodes::NO_ERROR, RobotStatusCodes::OK);

    wrist_T_template_.pose.orientation.x = 0;
    wrist_T_template_.pose.orientation.y = 0;
    wrist_T_template_.pose.orientation.z = 0;
    wrist_T_template_.pose.orientation.w = 1.0;
    wrist_T_template_.pose.position.x = 0;
    wrist_T_template_.pose.position.y = 0;
    wrist_T_template_.pose.position.z = 0;

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
                    break;
                }
            }

            if(index >= template_srv_.response.template_type_information.grasps.size()){
                ROS_ERROR("Grasp id:%d not found while request attaching. Not Attaching", grasp_msg.grasp_id.data);
            }else{
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
        }else{
            ROS_ERROR("Palm not in /world frame, need to detach. Not Attaching.");
        }        
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
                    wrist_target_pub_.publish(wrist_target_pose_.pose);
                    sendFinalGrasp(wrist_target_pose_.pose);
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
    else if (affordance.type == "cartesian")
        sendCartesianAffordance(affordance);
    else if (affordance.type == "fixed")
        sendFixedPoseAffordance(affordance);
    else{
        ROS_ERROR("Affordance type not recognized, ignoring affordance command");
    }
}

void VigirManipulationController::updateHandMarkerCallback(const std_msgs::Int8& usability_id)
{
    int index;

    tf::Transform template_T_marker;
    tf::Transform hand_T_template;



    if(usability_id.data >= 0){

        if (!template_info_client_.call(template_srv_))
        {
            ROS_ERROR("Failed to call service request grasp info");
        }else{

            for(index = 0; index < template_srv_.response.template_type_information.usabilities.size(); index++)
            {
                if(int(template_srv_.response.template_type_information.usabilities[index].id) == int(usability_id.data)){
                    template_T_marker.setRotation(tf::Quaternion(template_srv_.response.template_type_information.usabilities[index].pose.pose.orientation.x,
                                                               template_srv_.response.template_type_information.usabilities[index].pose.pose.orientation.y,
                                                               template_srv_.response.template_type_information.usabilities[index].pose.pose.orientation.z,
                                                               template_srv_.response.template_type_information.usabilities[index].pose.pose.orientation.w));
                    template_T_marker.setOrigin(tf::Vector3(template_srv_.response.template_type_information.usabilities[index].pose.pose.position.x,
                                                          template_srv_.response.template_type_information.usabilities[index].pose.pose.position.y,
                                                          template_srv_.response.template_type_information.usabilities[index].pose.pose.position.z));

                    ROS_INFO("template_T_marker x:%f, y:%f, z:%f, qx:%f, qy:%f, qz:%f, qw:%f",
                              template_T_marker.getOrigin().getX(),
                              template_T_marker.getOrigin().getY(),
                              template_T_marker.getOrigin().getZ(),
                              template_T_marker.getRotation().getX(),
                              template_T_marker.getRotation().getY(),
                              template_T_marker.getRotation().getZ(),
                              template_T_marker.getRotation().getW());

                    hand_T_template.setRotation(tf::Quaternion(wrist_T_template_.pose.orientation.x,
                                                               wrist_T_template_.pose.orientation.y,
                                                               wrist_T_template_.pose.orientation.z,
                                                               wrist_T_template_.pose.orientation.w));
                    hand_T_template.setOrigin(tf::Vector3(wrist_T_template_.pose.position.x,
                                                          wrist_T_template_.pose.position.y,
                                                          wrist_T_template_.pose.position.z));

                    ROS_INFO("hand_T_template x:%f, y:%f, z:%f, qx:%f, qy:%f, qz:%f, qw:%f",
                              hand_T_template.getOrigin().getX(),
                              hand_T_template.getOrigin().getY(),
                              hand_T_template.getOrigin().getZ(),
                              hand_T_template.getRotation().getX(),
                              hand_T_template.getRotation().getY(),
                              hand_T_template.getRotation().getZ(),
                              hand_T_template.getRotation().getW());

                    hand_T_usability_ = hand_T_template * template_T_marker;
                    break;
                }
            }
        }

        if(index >= template_srv_.response.template_type_information.usabilities.size()){
            ROS_ERROR("Usability id:%d not found, setting marker to palm", usability_id.data);
            hand_T_usability_ = hand_T_marker_;
        }
    }else{
        if(usability_id.data == -2)
            hand_T_usability_ = hand_T_marker_;
        else if(usability_id.data == -1){
            hand_T_usability_.setRotation(tf::Quaternion(0,0,0,1));
            hand_T_usability_.setOrigin(tf::Vector3(0,-0.095,0));
        }
    }

    ROS_INFO("New hand_T_marker x:%f, y:%f, z:%f, qx:%f, qy:%f, qz:%f, qw:%f",
              hand_T_usability_.getOrigin().getX(),
              hand_T_usability_.getOrigin().getY(),
              hand_T_usability_.getOrigin().getZ(),
              hand_T_usability_.getRotation().getX(),
              hand_T_usability_.getRotation().getY(),
              hand_T_usability_.getRotation().getZ(),
              hand_T_usability_.getRotation().getW());
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

int VigirManipulationController::poseTransform(geometry_msgs::Pose& input_pose, const tf::Transform& transform)
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

int VigirManipulationController::poseTransform(const tf::Transform &transform, geometry_msgs::Pose& input_pose)
{
    tf::Transform output_transform;    //describes hand in object's frame
    tf::Transform input_transform;       //describes palm_from_graspit in object's frame

    input_transform.setRotation(tf::Quaternion(input_pose.orientation.x,input_pose.orientation.y,input_pose.orientation.z,input_pose.orientation.w));
    input_transform.setOrigin(tf::Vector3(input_pose.position.x,input_pose.position.y,input_pose.position.z) );

    output_transform = transform * input_transform;

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

inline void VigirManipulationController::updateTemplateMass()
{

    if (template_mass_pub_)
        template_mass_pub_.publish(this->template_mass_msg_);
    else
        ROS_WARN("Invalid template_mass_pub_");
}

void VigirManipulationController::processTemplateMassData(const geometry_msgs::PoseStamped& template_pose, const float &template_mass, const geometry_msgs::Point& template_com)
{
    tf::Transform wrist_T_template;
    tf::Vector3   template_P;

    wrist_T_template.setOrigin(tf::Vector3(template_pose.pose.position.x,template_pose.pose.position.y,template_pose.pose.position.z));
    wrist_T_template.setRotation(tf::Quaternion(template_pose.pose.orientation.x,template_pose.pose.orientation.y,template_pose.pose.orientation.z,template_pose.pose.orientation.w));

    template_P.setX(template_com.x);
    template_P.setY(template_com.y);
    template_P.setZ(template_com.z);

    template_P = wrist_T_template * template_P; //Convert template CoM to wrist frame

    this->template_mass_msg_.hand  = this->hand_id_ < 1 ? 0: 1;  //TO KEEP CONSISTENCY WITH BDI LEFT=0/RIGHT=1
    this->template_mass_msg_.mass = template_mass;

    if(this->template_mass_msg_.mass < 0 || this->template_mass_msg_.mass >= 10.0) //Mass sanity check, we don't consider masses over 10 Kg and mass shouldn't be less than zero
    {   //If sanity check failed, reset the mass and CoM to the corresponding hand.
        ROS_WARN("Template mass sanity check failed, resetting to 0 (template mass= %f).",template_mass);
        this->template_mass_msg_.mass = 0.0;
        template_P.setX(0.0);
        template_P.setY(0.0);
        template_P.setZ(0.0);
    }
    com_.header.frame_id    = "/"+this->wrist_name_;
    com_.header.stamp       = ros::Time::now();
    com_.header.seq++;
    com_.pose.orientation.w = 1.0;

    if (fabs(template_P.getX()) < 1.0 && fabs(template_P.getY()) < 1.0 && fabs(template_P.getZ()) < 1.0) //CoM sanity check, we don't consider positions over 1 meter.
    {
        com_.pose.position.x = template_P.getX();
        com_.pose.position.y = template_P.getY();
        com_.pose.position.z = template_P.getZ();
    }
    else{//CoM sanity check failed, reset the CoM to the corresponding hand.
        ROS_WARN("Template CoM sanity check failed, resetting to 0.");
        this->template_mass_msg_.mass = 0.0;
        com_.pose.position.x = 0.0;
        com_.pose.position.y = 0.0;
        com_.pose.position.z = 0.0;
    }
    this->template_mass_msg_.com = com_;

}

void VigirManipulationController::handStatusCallback(const flor_grasp_msgs::HandStatus &msg)
{
    {
        boost::lock_guard<boost::mutex> guard(this->write_data_mutex_);
        last_hand_status_msg_ = msg;
    }

    switch(last_hand_status_msg_.hand_status ){
    case 0: setGraspStatus(RobotStatusCodes::NO_ERROR , RobotStatusCodes::OK);
        break;
    case 1: setGraspStatus(RobotStatusCodes::GRASP_NO_APPENDAGE_CONTROL , RobotStatusCodes::WARNING);
        break;
    default: setGraspStatus(RobotStatusCodes::NO_ERROR , RobotStatusCodes::OK);
        break;
    }

    this->updateGraspStatus();
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

void VigirManipulationController::setStitchingObject(const flor_grasp_msgs::TemplateSelection& template_data){
    //Add collision object with template pose and bounding box

    ROS_INFO("Stitching collision object :%s started",(boost::to_string(int16_t(template_data.template_id.data))).c_str());
    vigir_object_template_msgs::SetAttachedObjectTemplate srv;
    srv.request.template_id          = int16_t(template_data.template_id.data);
    srv.request.pose                 = last_wrist_pose_msg_;
    srv.request.pose.header.frame_id = this->wrist_name_;
    if (!stitch_object_client_.call(srv))
        ROS_ERROR("Failed to call service request SetStitchedObjectTemplate");

    //Manage template mass
    wrist_T_template_ = srv.response.template_pose;
    processTemplateMassData(srv.response.template_pose, srv.response.template_mass,  srv.response.template_com);
    updateTemplateMass();

}

void VigirManipulationController::setDetachingObject(const flor_grasp_msgs::TemplateSelection& template_data){
    //Set static transform to identity unstitching the template
    this->palmStitch_T_hand_.setIdentity();

    ROS_INFO("Removing collision object :%s started",(boost::to_string(int16_t(template_data.template_id.data))).c_str());
    vigir_object_template_msgs::SetAttachedObjectTemplate srv;
    srv.request.template_id          = int16_t(template_data.template_id.data);
    srv.request.pose                 = last_wrist_pose_msg_;
    srv.request.pose.header.frame_id = this->wrist_name_;
    if (!detach_object_client_.call(srv))
        ROS_ERROR("Failed to call service request SetAttachedObjectTemplate");

    //Manage template mass
    geometry_msgs::PoseStamped tmp_pose;
    tmp_pose.pose.orientation.w = 1;
    geometry_msgs::Point tmp_point;
    float tmp_mass;
    processTemplateMassData(tmp_pose, tmp_mass,  tmp_point);
    updateTemplateMass();
}

void VigirManipulationController::sendFinalGrasp(const geometry_msgs::PoseStamped& final_grasp)
{
    actionlib::SimpleActionClient<vigir_planning_msgs::MoveAction> move_action_client("/vigir_move_group",true);

    ROS_INFO("Waiting for move action server to start.");
    if(!move_action_client.waitForServer(ros::Duration(5))){
        ROS_ERROR("Move group client timed out");
        return;
    }

    ROS_INFO("Action server started, sending goal.");
    vigir_planning_msgs::MoveGoal move_goal;

    move_goal.extended_planning_options.target_motion_type                 = vigir_planning_msgs::ExtendedPlanningOptions::TYPE_CARTESIAN_WAYPOINTS;
    move_goal.extended_planning_options.allow_environment_collisions       = true;
    move_goal.extended_planning_options.keep_endeffector_orientation       = false;  //Final Grasps are always sent to the correct orientation
    move_goal.extended_planning_options.execute_incomplete_cartesian_plans = true;
    move_goal.request.group_name                                           = this->planning_group_;
    move_goal.request.allowed_planning_time                                = 1.0;
    move_goal.request.num_planning_attempts                                = 1;
    move_goal.request.max_velocity_scaling_factor                          = 0.1;

    move_goal.extended_planning_options.target_frame = final_grasp.header.frame_id;
    move_goal.extended_planning_options.target_poses.push_back(final_grasp.pose);

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

void VigirManipulationController::sendCircularAffordance(const vigir_object_template_msgs::Affordance& affordance)
{
//    //Create message for manipulation controller
//    template_srv_.request.template_id = last_template_stitch_id_;
//    template_srv_.request.hand_side   = template_srv_.request.BOTH_HANDS;
//    if (!template_info_client_.call(template_srv_))
//    {
//        ROS_ERROR("Failed to call service request template info");
//    }else{
//        affordance.waypoints = template_srv_.response.template_type_information.affordances[affordance.id].waypoints;
//    }



    actionlib::SimpleActionClient<vigir_planning_msgs::MoveAction> move_action_client("/vigir_move_group",true);

    ROS_INFO("Waiting for move action server to start.");
    if(!move_action_client.waitForServer(ros::Duration(5))){
        ROS_ERROR("Move group client timed out");
        return;
    }

    ROS_INFO("Action server started, sending goal.");
    vigir_planning_msgs::MoveGoal move_goal;

    move_goal.extended_planning_options.target_motion_type                 = vigir_planning_msgs::ExtendedPlanningOptions::TYPE_CIRCULAR_MOTION;
    move_goal.extended_planning_options.allow_environment_collisions       = true;
    move_goal.extended_planning_options.keep_endeffector_orientation       = affordance.keep_orientation;
    move_goal.extended_planning_options.rotation_angle                     = affordance.displacement;
    move_goal.extended_planning_options.pitch                              = affordance.pitch;
    move_goal.extended_planning_options.execute_incomplete_cartesian_plans = true;
    move_goal.request.group_name                                           = this->planning_group_;
    move_goal.request.allowed_planning_time                                = 1.0;
    move_goal.request.num_planning_attempts                                = 1;
    move_goal.request.max_velocity_scaling_factor                          = affordance.speed/100.0;

    move_goal.extended_planning_options.target_frame = affordance.waypoints[0].header.frame_id;

    if(affordance.keep_orientation){
        geometry_msgs::Pose temp;
        tf::poseTFToMsg(hand_T_usability_, temp);
        move_goal.extended_planning_options.reference_point       = temp;
        move_goal.extended_planning_options.reference_point_frame = this->wrist_name_;
        ROS_INFO("Setting reference frame to %s and reference point to x: %f, y: %f, z: %f, qx: %f, qy: %f, qz: %f, qw:%f ",
                  move_goal.extended_planning_options.reference_point_frame.c_str(),
                  move_goal.extended_planning_options.reference_point.position.x,
                  move_goal.extended_planning_options.reference_point.position.y,
                  move_goal.extended_planning_options.reference_point.position.z,
                  move_goal.extended_planning_options.reference_point.orientation.x,
                  move_goal.extended_planning_options.reference_point.orientation.y,
                  move_goal.extended_planning_options.reference_point.orientation.z,
                  move_goal.extended_planning_options.reference_point.orientation.w);
    }
    wrist_target_pub_.publish(affordance.waypoints[0]);

    move_goal.extended_planning_options.target_poses.push_back(affordance.waypoints[0].pose);

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

void VigirManipulationController::sendCartesianAffordance(vigir_object_template_msgs::Affordance affordance)
{
    actionlib::SimpleActionClient<vigir_planning_msgs::MoveAction> move_action_client("/vigir_move_group",true);

    ROS_INFO("Waiting for move action server to start.");
    if(!move_action_client.waitForServer(ros::Duration(5))){
        ROS_ERROR("Move group client timed out");
        return;
    }

    ROS_INFO("Action server started, sending goal.");
    vigir_planning_msgs::MoveGoal move_goal;

    move_goal.extended_planning_options.target_motion_type                 = vigir_planning_msgs::ExtendedPlanningOptions::TYPE_CARTESIAN_WAYPOINTS;
    move_goal.extended_planning_options.allow_environment_collisions       = true;
    move_goal.extended_planning_options.keep_endeffector_orientation       = true;  //Cartesian Affordances don't care about end effector orientation
    move_goal.extended_planning_options.execute_incomplete_cartesian_plans = true;
    move_goal.request.group_name                                           = this->planning_group_;
    move_goal.request.allowed_planning_time                                = 1.0;
    move_goal.request.num_planning_attempts                                = 1;
    move_goal.request.max_velocity_scaling_factor                          = affordance.speed/100.0;

    float norm = sqrt((affordance.waypoints[0].pose.position.x * affordance.waypoints[0].pose.position.x) +
                      (affordance.waypoints[0].pose.position.y * affordance.waypoints[0].pose.position.y) +
                      (affordance.waypoints[0].pose.position.z * affordance.waypoints[0].pose.position.z));
    if(norm != 0){
        affordance.waypoints[0].pose.position.x /= norm;
        affordance.waypoints[0].pose.position.y /= norm;
        affordance.waypoints[0].pose.position.z /= norm;
    }else{
        ROS_INFO("Norm is ZERO!");
        affordance.waypoints[0].pose.position.x = 0 ;
        affordance.waypoints[0].pose.position.y = 0 ;
        affordance.waypoints[0].pose.position.z = 0 ;
    }

    affordance.waypoints[0].pose.position.x *= affordance.displacement;
    affordance.waypoints[0].pose.position.y *= affordance.displacement;
    affordance.waypoints[0].pose.position.z *= affordance.displacement;


    if(affordance.waypoints[0].header.frame_id == "/world"){

        geometry_msgs::Pose wrist_pose = last_wrist_pose_msg_.pose;

        tf::Transform world_T_wrist;       //describes wrist in world frame

        world_T_wrist.setRotation(tf::Quaternion(wrist_pose.orientation.x,wrist_pose.orientation.y,wrist_pose.orientation.z,wrist_pose.orientation.w));
        world_T_wrist.setOrigin(tf::Vector3(0.0,0.0,0.0) ); //we are only using the rotation part

        // get affordance in wrist frame
        poseTransform(world_T_wrist.inverse(), affordance.waypoints[0].pose);

        affordance.waypoints[0].header.frame_id = this->wrist_name_;
    }

    wrist_target_pub_.publish(affordance.waypoints[0]);

    move_goal.extended_planning_options.target_frame = this->wrist_name_;
    move_goal.extended_planning_options.target_poses.push_back(affordance.waypoints[0].pose);

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

void VigirManipulationController::sendFixedPoseAffordance(const vigir_object_template_msgs::Affordance &affordance)
{
    actionlib::SimpleActionClient<vigir_planning_msgs::MoveAction> move_action_client("/vigir_move_group",true);

    ROS_INFO("Waiting for move action server to start.");
    if(!move_action_client.waitForServer(ros::Duration(5))){
        ROS_ERROR("Move group client timed out");
        return;
    }

    ROS_INFO("Action server started, sending goal.");
    vigir_planning_msgs::MoveGoal move_goal;

    move_goal.extended_planning_options.target_motion_type                 = vigir_planning_msgs::ExtendedPlanningOptions::TYPE_CIRCULAR_MOTION;
    move_goal.extended_planning_options.allow_environment_collisions       = true;
    move_goal.extended_planning_options.keep_endeffector_orientation       = false;
    move_goal.extended_planning_options.rotation_angle                     = affordance.displacement;
    move_goal.extended_planning_options.execute_incomplete_cartesian_plans = true;
    move_goal.request.group_name                                           = this->planning_group_;
    move_goal.request.allowed_planning_time                                = 1.0;
    move_goal.request.num_planning_attempts                                = 1;
    move_goal.request.max_velocity_scaling_factor                          = affordance.speed/100.0;

    move_goal.extended_planning_options.target_frame = affordance.waypoints[0].header.frame_id;

    wrist_target_pub_.publish(affordance.waypoints[0]);

    move_goal.extended_planning_options.target_poses.push_back(affordance.waypoints[0].pose);

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

bool VigirManipulationController::affordanceInWristFrame(vigir_object_template_msgs::GetAffordanceInWristFrame::Request &req,
                                                         vigir_object_template_msgs::GetAffordanceInWristFrame::Response &res){

    if(req.affordance.waypoints[0].header.frame_id == "/world"){
        float norm = sqrt((req.affordance.waypoints[0].pose.position.x * req.affordance.waypoints[0].pose.position.x) +
                          (req.affordance.waypoints[0].pose.position.y * req.affordance.waypoints[0].pose.position.y) +
                          (req.affordance.waypoints[0].pose.position.z * req.affordance.waypoints[0].pose.position.z));
        if(norm != 0){
            req.affordance.waypoints[0].pose.position.x /= norm;
            req.affordance.waypoints[0].pose.position.y /= norm;
            req.affordance.waypoints[0].pose.position.z /= norm;
        }else{
            ROS_INFO("Norm is ZERO!");
            req.affordance.waypoints[0].pose.position.x = 0 ;
            req.affordance.waypoints[0].pose.position.y = 0 ;
            req.affordance.waypoints[0].pose.position.z = 0 ;
        }

        req.affordance.waypoints[0].pose.position.x *= req.affordance.displacement;
        req.affordance.waypoints[0].pose.position.y *= req.affordance.displacement;
        req.affordance.waypoints[0].pose.position.z *= req.affordance.displacement;

        geometry_msgs::Pose wrist_pose = last_wrist_pose_msg_.pose;

        tf::Transform world_T_wrist;       //describes wrist in world frame

        world_T_wrist.setRotation(tf::Quaternion(wrist_pose.orientation.x,wrist_pose.orientation.y,wrist_pose.orientation.z,wrist_pose.orientation.w));
        world_T_wrist.setOrigin(tf::Vector3(0.0,0.0,0.0) ); //we are only using the rotation part

        // get affordance in wrist frame
        poseTransform(world_T_wrist.inverse(), req.affordance.waypoints[0].pose);

        res.wrist_affordance.header.frame_id = this->wrist_name_;
        res.wrist_affordance.header.stamp    = ros::Time::now();
        res.wrist_affordance.pose            = req.affordance.waypoints[0].pose;
    }else{
        ROS_ERROR("GetAffordanceInWristFrame Service failed, affordance is in %s frame, not in /world frame", req.affordance.waypoints[0].header.frame_id.c_str());
        return false;
    }
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
