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

//#include <vigir_manipulation_planning/vigir_manipulation_controller/include/vigir_manipulation_controller/vigir_manipulation_controller.h>
#include <vigir_manipulation_controller/vigir_manipulation_controller.h>

namespace vigir_manipulation_controller{


VigirManipulationController::VigirManipulationController():
    update_error_calc(false),
    hand_name_( "unknown"), hand_side_( "unknown"), hand_id_(0), grasp_type_(0),
    template_updated_(false),stitch_updated_(true)/*,
    l_arm_group_("l_arm_group"),
    r_arm_group_("r_arm_group")*/
{

}

VigirManipulationController::~VigirManipulationController()
  {
  std::cout << "Shutting down the manipulation controller ..." << std::endl;
  }


void VigirManipulationController::initializeManipulationController(ros::NodeHandle &nh, ros::NodeHandle &nhp)
  {
    ROS_INFO("Entering Initialize manipulation controller under private namespace: %s", nhp.getNamespace().c_str());

    // which hand are we controlling
    if (!nhp.hasParam("hand"))
    {
        ROS_WARN(" Did not find HAND parameter - using right hand as default");
    }

    nhp.param<std::string>("hand", this->hand_name_,"r_hand");

    ROS_INFO("Hand parameters received, hand: %s", this->hand_name_.c_str());

    this->hand_id_   = 1;
    this->hand_side_ = "right";
    if ("l_hand" == this->hand_name_){
        this->hand_id_   = -1;
        this->hand_side_ = "left";
    }

    XmlRpc::XmlRpcValue   gp_T_hand;
    XmlRpc::XmlRpcValue   hand_T_palm;

    ROS_INFO("Selecting L/R Transformations");

    if (hand_id_>0)
    {
        planning_group_ = "r_arm_group";
        nh.getParam("/r_hand_tf/gp_T_hand", gp_T_hand);
        ROS_ASSERT(gp_T_hand.getType() == XmlRpc::XmlRpcValue::TypeArray);
        nh.getParam("/r_hand_tf/hand_T_palm", hand_T_palm);
        ROS_INFO("Transformations selected for right");

    }
    else
    {
        planning_group_ = "l_arm_group";
        nh.getParam("/l_hand_tf/gp_T_hand", gp_T_hand);
        ROS_ASSERT(gp_T_hand.getType() == XmlRpc::XmlRpcValue::TypeArray);
        nh.getParam("/l_hand_tf/hand_T_palm", hand_T_palm);
        ROS_INFO("Transformations selected for left");
    }

    gp_T_hand_.setOrigin(tf::Vector3(static_cast<double>(gp_T_hand[0]),static_cast<double>(gp_T_hand[1]),static_cast<double>(gp_T_hand[2])));
    gp_T_hand_.setRotation(tf::Quaternion(static_cast<double>(gp_T_hand[3]),static_cast<double>(gp_T_hand[4]),static_cast<double>(gp_T_hand[5]),static_cast<double>(gp_T_hand[6])));

    ROS_INFO("Graspit Transformations set");

    hand_T_palm_.setOrigin(tf::Vector3(static_cast<double>(hand_T_palm[0]),static_cast<double>(hand_T_palm[1]),static_cast<double>(hand_T_palm[2])));
    hand_T_palm_.setRotation(tf::Quaternion(static_cast<double>(hand_T_palm[3]),static_cast<double>(hand_T_palm[4]),static_cast<double>(hand_T_palm[5]),static_cast<double>(hand_T_palm[6])));

    ROS_INFO("Initialize manipulation controller for the %s ...", this->hand_name_.c_str());

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

    ROS_INFO("Setup communications for the %s manipulation controller ... ", hand_name_.c_str());

    wrist_target_pub_         = nh.advertise<geometry_msgs::PoseStamped>("wrist_target",          1, true);
    template_stitch_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("template_stitch_pose",  1, true);
    wrist_plan_pub_           = nh.advertise<flor_planning_msgs::PlanRequest>("wrist_plan",       1, true);
    grasp_status_pub_         = nh.advertise<flor_ocs_msgs::OCSRobotStatus>("grasp_status",       1, true);
    hand_mass_pub_            = nh.advertise<flor_atlas_msgs::AtlasHandMass>("hand_mass",         1, true);

    // These publishers should be remapped in launch file
    grasp_command_sub_        = nh.subscribe("grasp_command",      1, &VigirManipulationController::graspCommandCallback,       this);
    template_stitch_sub_      = nh.subscribe("template_stitch",    1, &VigirManipulationController::templateStitchCallback,     this);
    current_wrist_sub_        = nh.subscribe("wrist_pose",         1, &VigirManipulationController::wristPoseCallback,          this);
    grasp_planning_group_sub_ = nh.subscribe("planning_group",     1, &VigirManipulationController::graspPlanningGroupCallback, this);
    grasp_selection_sub_      = nh.subscribe("move_to_pose",       1, &VigirManipulationController::moveToPoseCallback,         this);
    attach_object_sub_        = nh.subscribe("attach_object",      1, &VigirManipulationController::setAttachingObject,         this);
    detach_object_sub_        = nh.subscribe("detach_object",      1, &VigirManipulationController::setDetachingObject,         this);

    inst_grasp_info_client_   = nh.serviceClient<vigir_object_template_msgs::GetInstantiatedGraspInfo>("/instantiated_grasp_info");
    template_info_client_     = nh.serviceClient<vigir_object_template_msgs::GetTemplateStateAndTypeInfo>("/template_info");

    attach_object_client_     = nh.serviceClient<vigir_object_template_msgs::SetAttachedObjectTemplate>("/attach_object_template");
    stitch_object_client_     = nh.serviceClient<vigir_object_template_msgs::SetAttachedObjectTemplate>("/stitch_object_template");
    detach_object_client_     = nh.serviceClient<vigir_object_template_msgs::SetAttachedObjectTemplate>("/detach_object_template");

    //Stitch template to hand transformation initialization
    this->palm_T_hand_.setIdentity();

    trajectory_client_ = new  TrajectoryActionClient("/"+this->hand_side_+"_robotiq/"+this->hand_side_+"_hand_traj_controller/follow_joint_trajectory", true);
    while(!trajectory_client_->waitForServer(ros::Duration(5.0)))
       ROS_INFO("Waititing for %s TrajectoryActionServer", this->hand_side_.c_str());

}

///////////////////////////////////////////////////////
// Class Callback functions for ros subscribers


void VigirManipulationController::graspPlanningGroupCallback(const flor_ocs_msgs::OCSGhostControl& planning_group)
{
    if (planning_group.planning_group[2] == 1)
    {
        if (hand_id_>0)
            planning_group_ = "r_arm_with_torso_group";
        else
            planning_group_ = "l_arm_with_torso_group";
    }
    else if (planning_group.planning_group[2] == 0)
    {
        if (hand_id_>0)
            planning_group_ = "r_arm_group";
        else
            planning_group_ = "l_arm_group";
    }
}

void VigirManipulationController::templateStitchCallback(const flor_grasp_msgs::GraspSelection& grasp_msg)
{
    geometry_msgs::PoseStamped stitch_template_pose;

    //Set static transform to identity unstitching the template
    this->palm_T_hand_.setIdentity();

    if(grasp_msg.final_pose) //using final pose as is_stitched?
    {
        //Calculate static transform to stitch template to hand
        tf::Transform world_T_hand;
        tf::Transform world_T_template;
        tf::Transform world_T_palm;

        world_T_hand.setIdentity();
        world_T_template.setIdentity();
        world_T_palm.setIdentity();

        //Call service for template info
        vigir_object_template_msgs::GetTemplateStateAndTypeInfo template_srv_;
        template_srv_.request.template_id = grasp_msg.template_id.data;
        if (!template_info_client_.call(template_srv_))
        {
            ROS_ERROR("Failed to call service request grasp info");
        }
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

        this->palm_T_hand_ = (world_T_template.inverse() * world_T_palm).inverse() * world_T_template.inverse() * world_T_hand;


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

        stitch_template_pose.pose.position.x = this->palm_T_hand_.getOrigin().getX();
        stitch_template_pose.pose.position.y = this->palm_T_hand_.getOrigin().getY();
        stitch_template_pose.pose.position.z = this->palm_T_hand_.getOrigin().getZ();
        stitch_template_pose.pose.orientation.w = this->palm_T_hand_.getRotation().getW();
        stitch_template_pose.pose.orientation.x = this->palm_T_hand_.getRotation().getX();
        stitch_template_pose.pose.orientation.y = this->palm_T_hand_.getRotation().getY();
        stitch_template_pose.pose.orientation.z = this->palm_T_hand_.getRotation().getZ();

        template_stitch_pose_pub_.publish(stitch_template_pose);
    }
    else
        ROS_WARN("Invalid template stitch pose publisher");
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


void VigirManipulationController::graspCommandCallback(const flor_grasp_msgs::GraspState& grasp)
{
    // Store the latest grasp command, and update at next calculation loop
    {
        boost::lock_guard<boost::mutex> guard(this->write_data_mutex_);

        control_msgs::FollowJointTrajectoryGoal trajectory_action;
        trajectory_action.goal_time_tolerance = ros::Duration(5.0);
        trajectory_action.trajectory.joint_names.resize(11);
        trajectory_action.trajectory.joint_names[0]  = this->hand_side_+"_f0_j1";
        trajectory_action.trajectory.joint_names[1]  = this->hand_side_+"_f1_j1";
        trajectory_action.trajectory.joint_names[2]  = this->hand_side_+"_f2_j1";
        trajectory_action.trajectory.joint_names[3]  = this->hand_side_+"_f1_j0";
        trajectory_action.trajectory.joint_names[4]  = this->hand_side_+"_f2_j0";
        trajectory_action.trajectory.joint_names[5]  = this->hand_side_+"_f0_j2";
        trajectory_action.trajectory.joint_names[6]  = this->hand_side_+"_f1_j2";
        trajectory_action.trajectory.joint_names[7]  = this->hand_side_+"_f2_j2";
        trajectory_action.trajectory.joint_names[8]  = this->hand_side_+"_f0_j3";
        trajectory_action.trajectory.joint_names[9]  = this->hand_side_+"_f1_j3";
        trajectory_action.trajectory.joint_names[10] = this->hand_side_+"_f2_j3";
        trajectory_action.trajectory.points.resize(1);
        trajectory_action.trajectory.points[0].positions.resize(11);
        trajectory_action.trajectory.points[0].positions[0]  = float(grasp.grip.data > 100 ? 100 : grasp.grip.data)*0.0122+float(grasp.finger_effort[0].data)*0.0122;
        trajectory_action.trajectory.points[0].positions[1]  = float(grasp.grip.data > 100 ? 100 : grasp.grip.data)*0.0113+float(grasp.finger_effort[1].data)*0.0113;
        trajectory_action.trajectory.points[0].positions[2]  = float(grasp.grip.data > 100 ? 100 : grasp.grip.data)*0.0113+float(grasp.finger_effort[2].data)*0.0113;
        trajectory_action.trajectory.points[0].positions[3]  = float(grasp.finger_effort[3].data)*0.0028;
        trajectory_action.trajectory.points[0].positions[4]  = 0.0;
        trajectory_action.trajectory.points[0].positions[5]  = 0.0;
        trajectory_action.trajectory.points[0].positions[6]  = 0.0;
        trajectory_action.trajectory.points[0].positions[7]  = 0.0;
        trajectory_action.trajectory.points[0].positions[8]  = 0.0;
        trajectory_action.trajectory.points[0].positions[9]  = 0.0;
        trajectory_action.trajectory.points[0].positions[10] = 0.0;
        trajectory_action.trajectory.points[0].time_from_start = ros::Duration(0.05);

        //Create ROS trajectory and publish
        if(trajectory_client_->isServerConnected())
        {
            trajectory_action.trajectory.header.stamp = ros::Time::now();
            trajectory_client_->sendGoal(trajectory_action,
                                         //function that inside updates if(hand_id_>0)
                                         //this->setGraspStatus(RobotStatusCodes::GRASP_R_CLOSURE_FAILURE, RobotStatusCodes::WARNING);
                                         boost::bind(&VigirManipulationController::trajectoryDoneCb, this, _1, _2),
                                         boost::bind(&VigirManipulationController::trajectoryActiveCB, this),
                                         boost::bind(&VigirManipulationController::trajectoryFeedbackCB, this, _1));
        }
        else
        {
            ROS_ERROR("TrajectoryActionClient: Server not connected!");
        }
    }

     return;
}

    void VigirManipulationController::trajectoryActiveCB()
    {
        //ROS_INFO("TrajectoryActionClient: Status changed to active.");
    }

    void VigirManipulationController::trajectoryFeedbackCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
    {
        ROS_INFO("TrajectoryActionClient: Feedback received.");// pos[0]= %f", feedback->actual.positions[0]);
    }

    void VigirManipulationController::trajectoryDoneCb(const actionlib::SimpleClientGoalState& state,
                                                      const control_msgs::FollowJointTrajectoryResultConstPtr& result)
    {
        ROS_INFO("Fingers Trajectory finished in state [%s]", state.toString().c_str());
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
                    calcWristTarget(grasp_pose.pose);
                }else{
                    this->wrist_target_pose_.pose = pre_grasp_pose;
                    calcWristTarget(pre_grasp_pose.pose);
                }


                this->updateWristTarget();

            }
        }
    }
     return;
}

// assume this function is called within mutex block
int VigirManipulationController::calcWristTarget(const geometry_msgs::Pose& wrist_pose)
{
    // Transform wrist_pose into the template pose frame
    //   @TODO        "wrist_target_pose.pose   = T(template_pose)*wrist_pose";
    tf::Transform world_T_hand;
    tf::Transform world_T_hand_target;
    world_T_hand.setRotation(tf::Quaternion(wrist_pose.orientation.x,wrist_pose.orientation.y,wrist_pose.orientation.z,wrist_pose.orientation.w));
    world_T_hand.setOrigin(tf::Vector3(wrist_pose.position.x,wrist_pose.position.y,wrist_pose.position.z) );
    world_T_hand_target =  world_T_hand * this->palm_T_hand_;  //palm_T_hand is identity when no stitching
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

        ROS_INFO("   Update Grasp Status %d:%d  for %s", this->grasp_status_code_,this->grasp_status_severity_, this->hand_name_.c_str());
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

void VigirManipulationController::requestInstantiatedGraspService(const uint16_t& requested_template_id){
    //CALLING THE TEMPLATE SERVER
    vigir_object_template_msgs::GetInstantiatedGraspInfo srv;
    srv.request.template_id = requested_template_id;
    if (!inst_grasp_info_client_.call(srv))
    {
        ROS_ERROR("Failed to call service request grasp info");
    }
    last_grasp_res_ = srv.response;
}

void VigirManipulationController::setAttachingObject(const flor_grasp_msgs::TemplateSelection& last_template_data){
    //Add collision object with template pose and bounding box

    ROS_INFO("Attaching collision object :%s started",(boost::to_string(int16_t(last_template_data.template_id.data))).c_str());
    vigir_object_template_msgs::SetAttachedObjectTemplate srv;
    srv.request.template_id          = int16_t(last_template_data.template_id.data);
    srv.request.pose                 = last_wrist_pose_msg_;
    srv.request.pose.header.frame_id = this->hand_name_;
    if (!attach_object_client_.call(srv))
        ROS_ERROR("Failed to call service request SetAttachedObjectTemplate");
}

void VigirManipulationController::setStitchingObject(const flor_grasp_msgs::TemplateSelection& last_template_data){
    //Add collision object with template pose and bounding box

    ROS_INFO("Stitching collision object :%s started",(boost::to_string(int16_t(last_template_data.template_id.data))).c_str());
    vigir_object_template_msgs::SetAttachedObjectTemplate srv;
    srv.request.template_id          = int16_t(last_template_data.template_id.data);
    srv.request.pose                 = last_wrist_pose_msg_;
    srv.request.pose.header.frame_id = this->hand_name_;
    if (!stitch_object_client_.call(srv))
        ROS_ERROR("Failed to call service request SetStitchedObjectTemplate");
}

void VigirManipulationController::setDetachingObject(const flor_grasp_msgs::TemplateSelection& last_template_data){
    //Add collision object with template pose and bounding box

    ROS_INFO("Removing collision object :%s started",(boost::to_string(int16_t(last_template_data.template_id.data))).c_str());
    vigir_object_template_msgs::SetAttachedObjectTemplate srv;
    srv.request.template_id          = int16_t(last_template_data.template_id.data);
    srv.request.pose                 = last_wrist_pose_msg_;
    srv.request.pose.header.frame_id = this->hand_name_;
    if (!detach_object_client_.call(srv))
        ROS_ERROR("Failed to call service request DetachObjectTemplate");
}

int VigirManipulationController::staticTransform(geometry_msgs::Pose& palm_pose)
{
    tf::Transform o_T_palm;    //describes palm in object's frame
    tf::Transform o_T_pg;       //describes palm_from_graspit in object's frame

    o_T_pg.setRotation(tf::Quaternion(palm_pose.orientation.x,palm_pose.orientation.y,palm_pose.orientation.z,palm_pose.orientation.w));
    o_T_pg.setOrigin(tf::Vector3(palm_pose.position.x,palm_pose.position.y,palm_pose.position.z) );

    o_T_palm = o_T_pg * gp_T_hand_;

    tf::Quaternion hand_quat;
    tf::Vector3    hand_vector;
    hand_quat   = o_T_palm.getRotation();
    hand_vector = o_T_palm.getOrigin();

    palm_pose.position.x = hand_vector.getX();
    palm_pose.position.y = hand_vector.getY();
    palm_pose.position.z = hand_vector.getZ();
    palm_pose.orientation.x = hand_quat.getX();
    palm_pose.orientation.y = hand_quat.getY();
    palm_pose.orientation.z = hand_quat.getZ();
    palm_pose.orientation.w = hand_quat.getW();

    return 0;
}


} // end of vigir_manipulation_controller namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vigir_manipualtion_controller");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  vigir_manipulation_controller::VigirManipulationController manipulation_controller;

  ROS_WARN(" Initialize the Vigir Manipulaiton controller ...");
  manipulation_controller.initializeManipulationController(nh, nhp);

  ROS_WARN(" Start the ros spinner ...");
  ros::spin();
  return 0;
}
