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
    template_updated_(false),stitch_updated_(true),
    within_range_timer_threshold_(1.0),              // seconds
    pregrasp_position_error_threshold_(0.05),        // meters
    final_grasp_position_error_threshold_(0.05),     // meters
    pregrasp_orientation_error_threshold_(0.1),      // quaternion mumble
    final_grasp_orientation_error_threshold_(0.1)    // quaternion mumble
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
    release_grasp_sub_        = nh.subscribe("release_grasp",      1, &VigirManipulationController::releaseGraspCallback,       this);
    hand_offset_sub_          = nh.subscribe("hand_offset",        1, &VigirManipulationController::handOffsetCallback,         this);
    template_stitch_sub_      = nh.subscribe("template_stitch",    1, &VigirManipulationController::templateStitchCallback,     this);
    current_wrist_sub_        = nh.subscribe("wrist_pose",         1, &VigirManipulationController::wristPoseCallback,          this);
    grasp_planning_group_sub_ = nh.subscribe("planning_group",     1, &VigirManipulationController::graspPlanningGroupCallback, this);
    grasp_selection_sub_      = nh.subscribe("move_to_pose",       1, &VigirManipulationController::moveToPoseCallback,         this);

    template_info_client_     = nh.serviceClient<vigir_object_template_msgs::GetTemplateStateAndTypeInfo>("/template_info");

    attach_object_client_     = nh.serviceClient<vigir_object_template_msgs::SetAttachedObjectTemplate>("/attach_object_template");
    stitch_object_client_     = nh.serviceClient<vigir_object_template_msgs::SetAttachedObjectTemplate>("/stitch_object_template");
    detach_object_client_     = nh.serviceClient<vigir_object_template_msgs::DetachObjectTemplate>(     "/detach_object_template");

    //Stitch template to hand transformation initialization
    this->stitch_template_pose_.setIdentity();

    this->hand_offset_pose_.setIdentity();

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

void VigirManipulationController::templateStitchCallback(const flor_grasp_msgs::TemplateSelection& template_pose)
{
    ROS_INFO("Confidence : %d",template_pose.confidence.data);
    if(template_pose.confidence.data >= 0)
    {
        //Calculate static transform to stitch template to hand
        tf::Transform wTh;
        tf::Transform wTt;
        tf::Transform tTg;

        wTh.setRotation(tf::Quaternion(this->last_wrist_pose_msg_.pose.orientation.x,this->last_wrist_pose_msg_.pose.orientation.y,this->last_wrist_pose_msg_.pose.orientation.z,this->last_wrist_pose_msg_.pose.orientation.w));
        wTh.setOrigin(tf::Vector3(this->last_wrist_pose_msg_.pose.position.x,this->last_wrist_pose_msg_.pose.position.y,this->last_wrist_pose_msg_.pose.position.z));

        wTt.setRotation(tf::Quaternion(template_pose.pose.pose.orientation.x,template_pose.pose.pose.orientation.y,template_pose.pose.pose.orientation.z,template_pose.pose.pose.orientation.w));
        wTt.setOrigin(tf::Vector3(template_pose.pose.pose.position.x,template_pose.pose.pose.position.y,template_pose.pose.pose.position.z));

        tTg.setRotation(tf::Quaternion(this->final_wrist_pose_.orientation.x,this->final_wrist_pose_.orientation.y,this->final_wrist_pose_.orientation.z,this->final_wrist_pose_.orientation.w));
        tTg.setOrigin(tf::Vector3(this->final_wrist_pose_.position.x,this->final_wrist_pose_.position.y,this->final_wrist_pose_.position.z));

        this->stitch_template_pose_ = hand_T_palm_ * hand_offset_pose_.inverse() * hand_T_palm_.inverse() * tTg.inverse() * wTt.inverse() * wTh;
    }else
    {
        //Set static transform to identity unstitching the template
        this->stitch_template_pose_.setIdentity();
    }
    //Publish to OCS
    if (template_stitch_pose_pub_)
    {
        flor_grasp_msgs::TemplateSelection last_template_data;
        last_template_data = this->last_template_msg_;
        this->setStitchingObject(last_template_data); //Stitching collision object to robot

        geometry_msgs::PoseStamped stitch_template_pose;
        stitch_template_pose.header.frame_id = template_pose.pose.header.frame_id;
        stitch_template_pose.header.seq++;
        stitch_template_pose.header.stamp = template_pose.pose.header.stamp;
        stitch_template_pose.pose.position.x = this->stitch_template_pose_.getOrigin().getX();
        stitch_template_pose.pose.position.y = this->stitch_template_pose_.getOrigin().getY();
        stitch_template_pose.pose.position.z = this->stitch_template_pose_.getOrigin().getZ();
        stitch_template_pose.pose.orientation.w = this->stitch_template_pose_.getRotation().getW();
        stitch_template_pose.pose.orientation.x = this->stitch_template_pose_.getRotation().getX();
        stitch_template_pose.pose.orientation.y = this->stitch_template_pose_.getRotation().getY();
        stitch_template_pose.pose.orientation.z = this->stitch_template_pose_.getRotation().getZ();
        template_stitch_pose_pub_.publish(stitch_template_pose);
    }
    else
        ROS_WARN("Invalid template stitch pose publisher");
}

void VigirManipulationController::handOffsetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

    this->hand_offset_pose_.setRotation(tf::Quaternion(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w));
    this->hand_offset_pose_.setOrigin(tf::Vector3(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z) );
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


void VigirManipulationController::releaseGraspCallback(const flor_grasp_msgs::GraspSelection& grasp)
{
    // Store the latest grasp command, and update at next calculation loop
    {
        boost::lock_guard<boost::mutex> guard(this->write_data_mutex_);
    }

     return;
}

void VigirManipulationController::moveToPoseCallback(const flor_grasp_msgs::GraspSelection& grasp)
{
    // Store the latest grasp command, and update at next calculation loop
    {
        boost::lock_guard<boost::mutex> guard(this->write_data_mutex_);
        requestTemplateService(grasp.template_type.data);
        geometry_msgs::PoseStamped grasp_pose;
        moveit_msgs::GripperTranslation trans;

        size_t size = last_template_res_.template_type_information.grasps.size();

        if(size == 0)
            ROS_ERROR_STREAM("No grasps found for this template");
        else{
            size_t index = 0;
            for(; index < size; ++index)
            {
                if(std::atoi(last_template_res_.template_type_information.grasps[index].id.c_str()) == grasp.grasp_id.data){
                    grasp_pose = last_template_res_.template_type_information.grasps[index].grasp_pose;
                    trans = last_template_res_.template_type_information.grasps[index].pre_grasp_approach;
                    break;
                }
            }

            if(index >= size)
                ROS_ERROR_STREAM("Template server response id: " << last_template_res_.template_type_information.grasps[index].id << " while searching for id: " << grasp.grasp_id.data);
            else{
                geometry_msgs::Pose template_T_palm;//geometry_msgs::PoseStamped grasp_transform;
                template_T_palm = grasp_pose.pose;//grasp_transform.pose = grasp_db_[grasp_index].final_pose;

                if(!grasp.final_pose)  //Pre-Grasp pose
                    gripperTranslationToPreGraspPose(template_T_palm,trans);

                staticTransform(template_T_palm);

                this->wrist_target_pose_.planning_group.data = planning_group_;
                this->wrist_target_pose_.use_environment_obstacle_avoidance.data = true;
                this->wrist_target_pose_.pose = last_template_res_.template_state_information.pose;

                this->calcWristTarget(template_T_palm,last_template_res_.template_state_information.pose);

                this->updateWristTarget();

            }
        }
    }
     return;
}

bool VigirManipulationController::evaluateWristError(const uint8_t& current_state)
{
    if (this->update_error_calc)
    {
        this->update_error_calc = false; // flag no new updates

        if (this->wrist_target_pose_.pose.header.frame_id != this->last_wrist_pose_msg_.header.frame_id)
        {
            ROS_WARN("Mis-matched headers  %s vs. %s", this->wrist_target_pose_.pose.header.frame_id.c_str(), this->last_wrist_pose_msg_.header.frame_id.c_str());// TODO remove after testing
        }

        // Position error vector calculation
        this->last_wrist_error.pose.position.x = this->wrist_target_pose_.pose.pose.position.x - this->last_wrist_pose_msg_.pose.position.x;
        this->last_wrist_error.pose.position.y = this->wrist_target_pose_.pose.pose.position.y - this->last_wrist_pose_msg_.pose.position.y;
        this->last_wrist_error.pose.position.z = this->wrist_target_pose_.pose.pose.position.z - this->last_wrist_pose_msg_.pose.position.z;

        // Basic Euclidean distance as norm of error vector
        // might want to consider projection on to hand frame for different grasp types in future
        this->last_position_error = sqrt(this->last_wrist_error.pose.position.x*this->last_wrist_error.pose.position.x +
                                         this->last_wrist_error.pose.position.y*this->last_wrist_error.pose.position.y +
                                         this->last_wrist_error.pose.position.z*this->last_wrist_error.pose.position.z );

        // Calculate orientation error
        tf::Quaternion Q, Qd, Qe;

        Q.setW(this->last_wrist_pose_msg_.pose.orientation.w);
        Q.setX(this->last_wrist_pose_msg_.pose.orientation.x);
        Q.setY(this->last_wrist_pose_msg_.pose.orientation.y);
        Q.setZ(this->last_wrist_pose_msg_.pose.orientation.z);

        Qd.setW(this->wrist_target_pose_.pose.pose.orientation.w);
        Qd.setX(this->wrist_target_pose_.pose.pose.orientation.x);
        Qd.setY(this->wrist_target_pose_.pose.pose.orientation.y);
        Qd.setZ(this->wrist_target_pose_.pose.pose.orientation.z);

        Qe = Qd * Q.inverse();

        this->last_orientation_error = sqrt(Qe.getX()*Qe.getX() +
                                            Qe.getY()*Qe.getY() +
                                            Qe.getZ()*Qe.getZ());
    }
        if (this->last_position_error    <= this->final_grasp_position_error_threshold_ &&
            this->last_orientation_error <= this->final_grasp_orientation_error_threshold_ ) return true;
    // true if within limits
    //ROS_INFO("Last position error: %f ; orientation error: %f", this->last_position_error, this->last_orientation_error);
    return false;
}




// assume this function is called within mutex block
int VigirManipulationController::calcWristTarget(const geometry_msgs::Pose& wrist_pose,const geometry_msgs::PoseStamped& template_pose)
{
    // Transform wrist_pose into the template pose frame
    //   @TODO        "wrist_target_pose.pose   = T(template_pose)*wrist_pose";
    tf::Transform wt_pose;
    tf::Transform tp_pose;
    tf::Transform target_pose;
    wt_pose.setRotation(tf::Quaternion(wrist_pose.orientation.x,wrist_pose.orientation.y,wrist_pose.orientation.z,wrist_pose.orientation.w));
    wt_pose.setOrigin(tf::Vector3(wrist_pose.position.x,wrist_pose.position.y,wrist_pose.position.z) );
    tp_pose.setRotation(tf::Quaternion(template_pose.pose.orientation.x,template_pose.pose.orientation.y,template_pose.pose.orientation.z,template_pose.pose.orientation.w));
    tp_pose.setOrigin(tf::Vector3(template_pose.pose.position.x,template_pose.pose.position.y,template_pose.pose.position.z) );
    hand_T_template_ = (wt_pose * this->stitch_template_pose_).inverse();
    target_pose = tp_pose * this->hand_offset_pose_ * wt_pose * this->hand_T_palm_ * this->hand_T_palm_.inverse() * this->stitch_template_pose_;  //I assume this works
    tf::Quaternion tg_quat;
    tf::Vector3    tg_vector;
    tg_quat   = target_pose.getRotation();
    tg_vector = target_pose.getOrigin();

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
        //else
        //{
        //    ROS_DEBUG(" Ignore grasp controller error code %d:%d due to more severe existing code %d:%d",status, severity, this->grasp_status_code_,this->grasp_status_severity_);
        //}
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
//        else
//        {
//            ROS_INFO("   Maintain Grasp Status %d:%d  for %s", this->grasp_status_code_,this->grasp_status_severity_, this->hand_name_.c_str());

//        }

}

inline void VigirManipulationController::updateHandMass()
{

    if (hand_mass_pub_)
        hand_mass_pub_.publish(this->hand_mass_msg_);
    else
        ROS_WARN("Invalid hand_mass_pub_");
}


//void VigirManipulationController::controllerLoop()
//{

//    // local variables used within control loop
//    int8_t    requested_mode;
//    int16_t   requested_grasp_id, requested_template_id, requested_template_type;
//    bool      release_grasp        = false;
//    double    grasp_fraction       = 0.0;
//    uint8_t   manual_type          =   0;
//    uint8_t   close_percentage     =   0;
//    uint8_t   grip_percentage      =   0;
//    int8_t    finger_effort[FINGER_EFFORTS]  =   {0,0,0,0};
//    double    close_fraction       = 0.0;
//    double    grasp_effort         = 0.0;
//    uint16_t  planner_status_code  = RobotStatusCodes::GRASP_CONTROLLER_OK;
//    uint8_t   planner_status_level = RobotStatusCodes::OK;
//    uint8_t   reaching_tries       =   0;

//    flor_grasp_msgs::TemplateSelection last_template_data;
//    flor_control_msgs::FlorControlMode controller_mode;

//   // prevent shutdown
//   boost::lock_guard<boost::mutex> running_lock(this->thread_running_mutex_);
//   //ROS_INFO("Start the worker thread with run_flag = %d", this->run_flag);
//   while (this->run_flag) // continue running until shutdown
//   {
//       {
//       // Wait until we have new data worth processing
//       //  and convert the sensor data to internal format
//           boost::unique_lock<boost::mutex> sensor_data_lock(this->write_data_mutex_);
//           while (!(this->sensor_data_ready_))
//           {
//               // unlock until signal received, then lock again
//               // http://www.boost.org/doc/libs/1_41_0/doc/html/thread/synchronization.html#thread.synchronization.condvar_ref
//               this->data_ready_condition_.wait(sensor_data_lock);
//           }
//           //ROS_WARN("Control loop sensor_data_ready = %d",this->sensor_data_ready_);
//           this->sensor_data_ready_ = false;




////           tf::StampedTransform transform;
////           try{
////             listener_.lookupTransform("/template_tf_0", "/pelvis",
////                                      ros::Time(0), transform);
////           }
////           catch (tf::TransformException &ex) {
////             ROS_ERROR("%s",ex.what());
////             ros::Duration(1.0).sleep();
////           }
////           ROS_INFO("%f, %f, %f, %f, %f, %f, %f, ",
////                    transform.getOrigin().x(),
////                    transform.getOrigin().y(),
////                    transform.getOrigin().z(),
////                    transform.getRotation().w(),
////                    transform.getRotation().x(),
////                    transform.getRotation().y(),
////                    transform.getRotation().z());

//           //Here is where the requested mode is set
//           requested_mode   = (this->last_mode_msg_.grasp_state.data & 0xF0)>>4;
//           manual_type      = (this->last_mode_msg_.grasp_state.data & 0x0F);
//           release_grasp    = this->release_grasp_;

//           // Main controller mode
//           controller_mode = this->last_controller_mode_msg_;

//           // Get planner status
//           RobotStatusCodes::codes(this->last_planner_status_msg_.status,planner_status_code, planner_status_level);

//           if (requested_mode > MANUAL_GRASP_MODE)
//           {
//               ROS_ERROR(" Invalid mode requested = %d - reset to none!", requested_mode);
//               requested_mode = GRASP_MODE_NONE;
//           }


//           // This was the last request that we have
//           requested_grasp_id      = this->last_grasp_msg_.grasp_id.data;
//           requested_template_id   = this->last_grasp_msg_.template_id.data;
//           requested_template_type = this->last_grasp_msg_.template_type.data;
//           last_template_data      = this->last_template_msg_;

//           // For manual control or feedforward grip strength
//           close_percentage = this->last_mode_msg_.grip.data;
//           grip_percentage  = 0;
//           finger_effort[0]     = this->last_mode_msg_.finger_effort[0].data;
//           finger_effort[1]     = this->last_mode_msg_.finger_effort[1].data;
//           finger_effort[2]     = this->last_mode_msg_.finger_effort[2].data;
//           finger_effort[3]     = this->last_mode_msg_.finger_effort[3].data;
//           if(close_percentage > 100)
//           {
//               close_percentage  = 100;
//               close_fraction    = 1.0;
//               grip_percentage   = this->last_mode_msg_.grip.data - 100 ;
//               grasp_effort      = (double)(grip_percentage);
//           }
//           else
//           {
//               close_fraction    = double(close_percentage)/100.0;
//               grip_percentage   = 0;
//               grasp_effort      = 0.0;
//           }

//           //ROS_WARN(" requested mode = %d current state=%d", requested_mode, this->active_state_);
//           this->convertSensorData(); // this is protected by the scoped lock and calls derived class processHandSensorData()
//       }
//       // END OF DATA LOCK AND DATA PROCESSING

//       // Reset the status codes
//       grasp_status_code_     = RobotStatusCodes::GRASP_CONTROLLER_OK;
//       grasp_status_severity_ = RobotStatusCodes::OK;


//       const uint8_t prior_state   = this->active_state_.grasp_state.data & 0x0F;
//       const uint8_t prior_mode    = (this->active_state_.grasp_state.data & 0xF0)>>4;
//       uint8_t current_mode = requested_mode;
//       uint8_t current_state = prior_state;
//       float    template_mass        = 0.0;
//       tf::Vector3 template_com       = tf::Vector3(0.0,0.0,0.0);
//       tf::Transform hand_T_template;
//       hand_T_template.setIdentity();

//       //ROS_WARN(" current mode = %d current state=%d (0x%x) prior mode=%d state=%d", current_mode, current_state, this->active_state_, prior_mode, prior_state);

//       if(release_grasp)
//       {
//           grip_percentage   = 0;
//           close_percentage  = 0;
//           close_fraction = 0;
//           grasp_effort = 0;
//           if (current_state > GRASP_INIT)
//           {
//               // Set initial values to current, and set grasp flag to true
//               // Initial finger values are used in the setHandOpeningData() function.
//               if(!start_grasp_flag)
//               {
//                   ROS_WARN("Release command received, abort grasp for %s!", hand_name_.c_str());
//                   start_grasp_time = ros::Time::now();  //assuming simulation time is set
//                   start_grasp_flag = true;    //grasping initiated
//                   grasp_fraction   = 0.0;

//                   setInitialJointToCurrentPositions();

//                   this->setDetachingObject(last_template_data);

//               }



//               current_state = OPENING; // make this opening, and do timer at start
//           }


//           // Don't care about planner status if not in template mode
//           planner_status_code  = RobotStatusCodes::GRASP_CONTROLLER_OK;
//           planner_status_level = RobotStatusCodes::OK;
//       }
//       /////////////////// END RELEASE GRASP, BEGIN TEMPLATE GRASP /////////////////////////////////////////

//       if (grasp_updated_)
//       {

//           //if the Grasp button is pressed and the new grasp ID's do not match (i.e. new grasp)
//           if ( (this->grasp_id_      != requested_grasp_id) ||
//                (this->template_id_   != requested_template_id) ||
//                (this->template_type_ != requested_template_type))
//           {
//               // Grasp has changed, so reset grasp flag
//               start_grasp_flag = false;
//               grasp_fraction = 0;
//               close_percentage = 0;
//               close_fraction = 0;
//               grasp_effort = 0;

//               // Try to get target data for selected grasp from potential_grasps_
//               //ROS_INFO("current state: %d ",current_state);
//               //ROS_INFO("requested: GraspID: %d , TemplateID: %d , Template Type: %d ",requested_grasp_id, requested_template_id, requested_template_type);
//               //ROS_INFO("GraspID: %d , TemplateID: %d , Template Type: %d ",grasp_id_, template_id_, template_type_);
//               requestTemplateService(requested_template_type);
//               updateGraspTemplate(requested_grasp_id, requested_template_id, requested_template_type);

//               // If grasp was found in database, update wrist_target_pose_ and set to INIT
//               if ( (this->grasp_id_ == requested_grasp_id) &&
//                    (this->template_id_ == requested_template_id)
//                    && current_mode == TEMPLATE_GRASP_MODE  )
//               {
//                   // set this->grasp_id and template_id if we find a valid match
//                   //  otherwise , GRASP_STATE_NONE, id=-1, and no control
//                   current_state = GRASP_INIT;
//                   this->last_mode_msg_.grip.data = 0;
//                   ROS_INFO("%s: grasp selection Init(): grasp controller mode = %d current state=%d",
//                            hand_name_.c_str(), current_mode, current_state);

//                   if ( (last_template_data.template_id.data   == this->template_id_)   &&
//                        (last_template_data.template_type.data == this->template_type_) )
//                   {
//                        boost::lock_guard<boost::mutex> sensor_data_lock(this->write_data_mutex_);
//                        this->template_updated_ = true;
//                        //templateUpdateCallback(last_template_data);

//                        this->wrist_target_pose_.pose.header.frame_id = last_template_data.pose.header.frame_id;
//                        this->wrist_target_pose_.pose.header.seq++;
//                        this->wrist_target_pose_.pose.header.stamp = last_template_data.pose.header.stamp; // time stampe of valid data

//                        this->wrist_target_pose_.planning_group.data = planning_group_;
//                        this->wrist_target_pose_.use_environment_obstacle_avoidance.data = true;

//                        this->calcWristTarget(this->pregrasp_wrist_pose_,last_template_data.pose);
//                        this->updateWristTarget();

//                   }
//                   start_grasp_flag = false;
//                   grasp_status_timer = ros::Time::now();


//               }
//               else
//               {
//                   current_state   = GRASP_INIT;

//                   //If a manual grasp is selected, then set grasp_id_=0
//                   if(requested_grasp_id == 0)
//                       grasp_id_ = 0;

//                   //If a template grasp is selected in manual mode, open the hand on template grasp change
//                   if(this->grasp_id_ > 0)
//                   {
//                        this->last_mode_msg_.grip.data = 0;
//                   }

//                       if(hand_id_>0)
//                       {
//                           this->setGraspStatus(RobotStatusCodes::GRASP_R_NO_CONTROLLER, RobotStatusCodes::WARNING);
//                       } else {
//                           this->setGraspStatus(RobotStatusCodes::GRASP_L_NO_CONTROLLER, RobotStatusCodes::WARNING);
//                       }
//               }
//           }
//           else // If the Grasp button is pressed but the ID's are still the same (same grasp, new location)
//           {
//               if(current_state < GRASP_INIT)
//               {
//                   current_state = GRASP_INIT;
//                   start_grasp_flag = false;
//                   grasp_status_timer = ros::Time::now();
//                   //ROS_INFO("%s: initialize grasp state machine mode = %d current state=%d", hand_name_.c_str(), current_mode, current_state);
//               }
//               //ROS_INFO("%s: maintain current grasp mode = %d current state=%d grasp id=%d template=%d",
//               //         hand_name_.c_str(), current_mode, current_state, this->grasp_id_,this->template_id_);
//           }
//       }
//        ///////////////// END TEMPLATE MODE BLOCK ////////////////////////////////////////////

//       // If entering manual grasp mode for the first time, set finger values?
//       if (current_mode == MANUAL_GRASP_MODE  && !release_grasp)
//       {
//           // Manual mode
//           start_grasp_flag  = false;
//           current_state     = MONITORING;
//           grasp_fraction    = close_fraction;

//           // Don't care about planner status if not in template mode
//           planner_status_code  = RobotStatusCodes::GRASP_CONTROLLER_OK;
//           planner_status_level = RobotStatusCodes::OK;

//           // TODO: need to remove manual type and have it accept any grasp including templates
//           if (grasp_updated_ ) // && manual_type < GRASP_NONE
//           {
//               boost::lock_guard<boost::mutex> guard(this->write_data_mutex_);
//               // Use CYLINDRICAL, SPHERICAL, PRISMATIC
//               // otherwise use the prior terminal state from template grasp

//               ROS_INFO(" Manual grasp mode  - grasp id (%d)  close_fraction=%f  type=%d",
//                        grasp_id_, close_fraction, manual_type);

//               grasp_updated_ = false;
//               //If a manual grasp is selected, set the finger values,
//               //else, use the finger values previously loaded from the template
//               if (this->grasp_id_ <= 0 && manual_type < 4)
//               {
//                    setFinalFingerPoses(manual_type);
//                    //Open hand if mode or grasp type has changed since previous command
//                    if(this->grasp_type_ != manual_type || current_mode != prior_mode)
//                    {
//                        this->last_mode_msg_.grip.data = 0;
//                        this->grasp_type_ = manual_type;
//                    }

//               }
//           }
//       }

//        ////////////////////// END MANUAL MODE BLOCK ////////////////////////////////////

//       // handle else condition, and also check controller mode if grasp control is active
//       if (current_mode == GRASP_MODE_NONE)
//       { //    idle
//           start_grasp_flag  = false;
//           current_state     = GRASP_STATE_NONE;
//           grasp_fraction    = 0;
//           if(hand_id_>0)
//           {
//               this->setGraspStatus(RobotStatusCodes::GRASP_R_NO_CONTROLLER, RobotStatusCodes::WARNING);
//           } else {
//               this->setGraspStatus(RobotStatusCodes::GRASP_L_NO_CONTROLLER, RobotStatusCodes::WARNING);
//           }
//       }
//       else
//       {
//           // Verify controller in proper mode
//           switch(controller_mode.control_mode)
//           {
//           // No appendage control
//           case atlas_msgs::AtlasSimInterfaceCommand::STAND: // @todo - this should check against allowed_control_modes, but for now they are the same
//           case atlas_msgs::AtlasSimInterfaceCommand::STEP:
//           case atlas_msgs::AtlasSimInterfaceCommand::WALK:
//           //case flor_control_msgs::FlorControlModeCommand::STAND_PREP: no longer valid mode command from OCS
//               this->setGraspStatus(RobotStatusCodes::GRASP_NO_APPENDAGE_CONTROL, RobotStatusCodes::WARNING);
//               break;
//           default:
//               // All other controller modes allow arm control
//               break;
//           }
//       }

//       if (start_grasp_flag)
//       {
//           ros::Time current_time = ros::Time::now();
//           grasp_fraction = this->sigmoid_squashing_function((current_time - start_grasp_time).toSec(), grasp_period, grasp_gain);
//           //ROS_INFO("Setting grasp fraction to: %f",grasp_fraction);
//       }

//       if (grasp_updated_ && current_mode == TEMPLATE_GRASP_MODE && !release_grasp)
//       {
//           bool within_range = this->evaluateWristError(current_state);
//           if(within_range)
//           {
//               if((ros::Time::now() - within_range_time).toSec() < within_range_timer_threshold_) //waits until hand is stabilized in within_range pose
//                   within_range = false;
//           }else
//           {
//               within_range_time = ros::Time::now();
//           }
//           switch(current_state)
//           {
//           case GRASP_INIT:
//               //close_percentage = 0;
//               if(within_range && grasp_fraction > finger_open_threshold) // It is in pre-grasp pose and has already set the fingers to inital position (transition to SURROUNDING)
//               {
//                   ROS_INFO("%s: Trigger transition from INIT to SURROUNDING with threshold", hand_name_.c_str());
//                   start_grasp_flag = false;
//                   {
//                       boost::lock_guard<boost::mutex> sensor_data_lock(this->write_data_mutex_);
//                       // Update the wrist target on transition
//                       if (this->last_template_msg_.template_id.data == this->template_id_)
//                       {
//                           this->wrist_target_pose_.planning_group.data = planning_group_;
//                           this->wrist_target_pose_.use_environment_obstacle_avoidance.data = true;
//                           this->calcWristTarget(this->final_wrist_pose_, this->last_template_msg_.pose);
//                           this->updateWristTarget();
//                       }
//                   }
//                   current_state = SURROUNDING;
//               }else                          // It is not in pre-grasp pose or fingers are not in intial position (transition to APPROACHING)
//               {
//                   ROS_INFO(" %s: Trigger transition from INIT to APPROACHING with threshold", hand_name_.c_str());
//                   start_grasp_time = ros::Time::now(); //resetting time for moving to initial finger positions
//                   start_grasp_flag = true;
//                   grasp_fraction   = 0.0;
//                   setInitialJointToCurrentPositions();
//                   current_state = APPROACHING;
//               }
//               grasp_status_timer = ros::Time::now();
//               reaching_tries= 0;
//               break;
//           case APPROACHING:
//               if(within_range && grasp_fraction > finger_open_threshold)  // It is already in pre-grasp pose with fingers in intial positions (transition to SURROUNDING)
//               {
//                   ROS_INFO(" %s: Trigger transition from APPROACHING to SURROUNDING with threshold", hand_name_.c_str() );
//                   start_grasp_flag = false;
//                   {
//                       boost::lock_guard<boost::mutex> sensor_data_lock(this->write_data_mutex_);
//                       // Update the wrist target on transition
//                       if (this->last_template_msg_.template_id.data == this->template_id_)
//                       {
//                           this->wrist_target_pose_.planning_group.data = planning_group_;
//                           this->wrist_target_pose_.use_environment_obstacle_avoidance.data = true;
//                           this->calcWristTarget(this->final_wrist_pose_, this->last_template_msg_.pose);
//                           this->updateWristTarget();
//                       }
//                   }
//                   current_state = SURROUNDING;
//                   grasp_status_timer = ros::Time::now();
//                   reaching_tries = 0;
//               }
//               else
//               {
//                   //ROS_INFO( "%s: APPROACHING within_range=%d  grasp_fraction=%f >? %f", hand_name_.c_str(), within_range, grasp_fraction, finger_open_threshold);
//               }
//               if((ros::Time::now()-grasp_status_timer).toSec() > approaching_timer_threshold_)
//               {  //Status message if stock in APPROACHING
//                   if(reaching_tries<5)//trying again for 5 times
//                   {
//                       this->templateUpdateCallback(this->last_template_msg_);
//                       grasp_status_timer = ros::Time::now();
//                       reaching_tries++;
//                   }else
//                       if(hand_id_>0)
//                       {
//                           this->setGraspStatus(RobotStatusCodes::GRASP_R_REACH_TIMEOUT, RobotStatusCodes::WARNING);
//                       } else {
//                           this->setGraspStatus(RobotStatusCodes::GRASP_L_REACH_TIMEOUT, RobotStatusCodes::WARNING);
//                       }
//               }
//               break;
//           case SURROUNDING:
//               if(!within_range) //It is not near final pose. Maintain state, but relax grasp
//               {
//                   //ROS_INFO( "%s: SURROUNDING within_range=%d  grasp_fraction=%f ", hand_name_.c_str(), within_range, grasp_fraction);
//                   grasp_fraction   = 0.0;
//                   start_grasp_flag = false;
//               }else             //It is already in final pose (transition to GRASPING)
//               {
//                   if(!start_grasp_flag)
//                   {
//                       ROS_INFO( "%s: SURROUNDING start GRASPING within_range=%d  grasp_fraction=%f ", hand_name_.c_str(), within_range, grasp_fraction);
//                       ROS_INFO("%s: start grasp timer - mode = %d current state=%d", hand_name_.c_str(), current_mode, current_state);
//                       start_grasp_time = ros::Time::now();  //assuming simulation time is set
//                       start_grasp_flag = true;    //grasping initiated
//                       grasp_fraction   = 0.0;
//                   }
//                   current_state = GRASPING;
//                   grasp_status_timer = ros::Time::now();
//               }
//               if((ros::Time::now()-grasp_status_timer).toSec() > surrounding_timer_threshold_){  //Status message if stock in SURROUNDING
//                   if(reaching_tries<5)//trying again for 5 times
//                   {
//                       this->templateUpdateCallback(this->last_template_msg_);
//                       grasp_status_timer = ros::Time::now();
//                       reaching_tries++;
//                   }else
//                       if(hand_id_>0)
//                       {
//                           this->setGraspStatus(RobotStatusCodes::GRASP_R_REACH_TIMEOUT, RobotStatusCodes::WARNING);
//                       } else {
//                           this->setGraspStatus(RobotStatusCodes::GRASP_L_REACH_TIMEOUT, RobotStatusCodes::WARNING);
//                       }
//               }
//               break;
//           case GRASPING:
//               if(!within_range)   // Not near final pose. (transition to SURROUNDING)
//               {
//                   ROS_INFO( "%s: GRASPING revert to SURROUNDING within_range=%d  grasp_fraction=%f ", hand_name_.c_str(), within_range, grasp_fraction);
//                   grasp_fraction   = 0.0;
//                   start_grasp_flag = false;
//                   current_state = SURROUNDING;
//                   grasp_status_timer = ros::Time::now();
//                   reaching_tries = 0;
//               }
//               else
//               {
//                   if(grasp_fraction > finger_close_threshold) // It is already in final pose and closed fingers (transition to MONITORING)
//                   {
//                       ROS_INFO( "%s: GRASPING transition to MONITORING within_range=%d  grasp_fraction=%f ", hand_name_.c_str(), within_range, grasp_fraction);
//                       current_state = MONITORING;
//                       grasp_fraction = 1.0;
//                       close_percentage = uint8_t(100.0*grasp_fraction);
//                       grip_percentage  = uint8_t(grasp_effort);
//                       this->setAttachingObject(last_template_data); //Attaching collision object to robot
//                   }
//                   if (!start_grasp_flag)
//                       ROS_WARN("Logic error in %s - ready to monitor but not started grasp flag", hand_name_.c_str());
//               }
//               if((ros::Time::now()-grasp_status_timer).toSec() > closure_timer_threshold_){  //Status message if stuck in GRASPING
//                   if(hand_id_>0)
//                       this->setGraspStatus(RobotStatusCodes::GRASP_R_CLOSURE_FAILURE, RobotStatusCodes::WARNING);
//                   else{
//                       this->setGraspStatus(RobotStatusCodes::GRASP_L_CLOSURE_FAILURE, RobotStatusCodes::WARNING);
//                   }
//               }
//               break;
//           case MONITORING:
//               //Maintaining grasp
//               {
//                   // @todo - fix quality handling and status codes
//                   switch (grasp_quality_)
//                   {
//                   case PALM_AND_ALL_FINGERS:
//                   case PALM_AND_THUMB_PLUS_ONE:
//                   case PALM_AND_THUMB_PLUS_TWO:
//                       if(hand_id_>0)
//                           this->setGraspStatus(RobotStatusCodes::GRASP_R_OBJECT_IN_PALM, RobotStatusCodes::OK);
//                       else{
//                           this->setGraspStatus(RobotStatusCodes::GRASP_L_OBJECT_IN_PALM, RobotStatusCodes::OK);
//                       }
//                       break;
//                   case NO_PALM_AND_ALL_FINGERS:
//                   case NO_PALM_AND_THUMB_PLUS_ONE:
//                   case NO_PALM_AND_THUMB_PLUS_TWO:
//                       if(hand_id_>0)
//                           this->setGraspStatus(RobotStatusCodes::GRASP_R_OBJECT_IN_FINGERS, RobotStatusCodes::OK);
//                       else{
//                           this->setGraspStatus(RobotStatusCodes::GRASP_L_OBJECT_IN_FINGERS, RobotStatusCodes::OK);
//                       }
//                       break;
//                   case NO_GRASP_QUALITY:
//                       if(hand_id_>0)
//                           this->setGraspStatus(RobotStatusCodes::GRASP_R_NO_OBJECT, RobotStatusCodes::WARNING);
//                       else{
//                           this->setGraspStatus(RobotStatusCodes::GRASP_L_NO_OBJECT, RobotStatusCodes::WARNING);
//                       }
//                   default:
//                       //ROS_WARN("grasp quality = %d for hand=%d ",grasp_quality_,hand_id_);
//                       if(hand_id_>0)
//                           this->setGraspStatus(RobotStatusCodes::GRASP_R_NO_OBJECT, RobotStatusCodes::WARNING);
//                       else{
//                           this->setGraspStatus(RobotStatusCodes::GRASP_L_NO_OBJECT, RobotStatusCodes::WARNING);
//                       }
//                       break;
//                   }
//               }
//               break;
//           case OPENING:
//               break;
//           default:
//               break;
//           }
//       }


//       // Do the control calculations as relevant
//       //bool update_joint_commands = true;

//       switch(current_state)
//       {
//       case APPROACHING:
//           //sets the fingers in initial position for grasp type.
//           update_joint_commands = true;
//           {
//               double one_minus_fraction = 1.0 - grasp_fraction;
//               close_percentage = uint8_t(100.0*one_minus_fraction + 0.5);
//               grip_percentage  = 0;
//               this->setHandApproachingData(grasp_fraction);
//               this->setGraspStatus( RobotStatusCodes::StatusCode(planner_status_code),RobotStatusCodes::StatusLevel(planner_status_level)); // require active trajectory during approaching
//           }
//           break;
//       case SURROUNDING:
//           //maintains fingers on initial finger pose
//           close_percentage = 0;
//           grip_percentage  = 0;
//           this->setHandSurroundingData();
//           this->setGraspStatus(RobotStatusCodes::StatusCode(planner_status_code),RobotStatusCodes::StatusLevel(planner_status_level)); // require active trajectory during approaching
//           break;

//       case GRASPING:
//           update_joint_commands = true;
//           //sends controls to fingers
//           {
//               close_percentage = uint8_t(100.0*grasp_fraction + 0.5);
//               grip_percentage  = uint8_t(grasp_effort);

//               //ROS_INFO(" Grasping action  - close_percentage=%d (%f)  grasp_fraction=%f grip percentage=%d grasp_type=%d ",
//               //         close_percentage, close_fraction, grasp_fraction, grip_percentage,this->grasp_type_);
//               int8_t finger_effort_tmp[FINGER_EFFORTS] = {0,0,0,0};
//               this->setHandGraspingData(grasp_fraction, finger_effort_tmp);
//               this->setGraspStatus(RobotStatusCodes::StatusCode(planner_status_code),RobotStatusCodes::StatusLevel(planner_status_level)); // require active trajectory during approaching
//           }
//           //publish current finger poses here
//           break;
//       case MONITORING:
//           update_joint_commands = true;
//           template_mass   = last_template_data.mass.data;//Mass from template library.
//           template_com    = tf::Vector3(last_template_data.com.x,last_template_data.com.y,last_template_data.com.z);
//           hand_T_template = hand_T_template_;

//           //sends FF controls to fingers
//           if (close_percentage < 100)
//              setHandGraspingData(grasp_fraction, finger_effort);
//           else
//              setHandMonitoringData(grasp_effort, finger_effort);

//           close_percentage = uint8_t(100.0*grasp_fraction);
//           grip_percentage  = uint8_t(grasp_effort);
//           break;
//       case OPENING:
//           update_joint_commands = true;
//           //sets the fingers in initial position for release type.
//           {
//               this->grasp_updated_=false;
//               double one_minus_fraction = 1.0 - grasp_fraction;
//               close_percentage = uint8_t(100.0*one_minus_fraction + 0.5);
//               grip_percentage  = 0;
//               this->last_template_msg_.mass.data  = 0.0;
//               this->last_template_msg_.com.x = 0.0;
//               this->last_template_msg_.com.y = 0.0;
//               this->last_template_msg_.com.z = 0.0;
//               this->last_template_msg_.pose.pose.orientation.w = 1;
//               this->last_template_msg_.pose.pose.orientation.x = 0;
//               this->last_template_msg_.pose.pose.orientation.y = 0;
//               this->last_template_msg_.pose.pose.orientation.z = 0;
//               this->last_template_msg_.pose.pose.position.x = 0;
//               this->last_template_msg_.pose.pose.position.y = 0;
//               this->last_template_msg_.pose.pose.position.z = 0;

//               setHandOpeningData(grasp_fraction);

//               if(grasp_fraction > finger_open_threshold) // Wait until hand is open
//               {
//                   boost::lock_guard<boost::mutex> guard(this->write_data_mutex_);
//                   ROS_INFO( "%s: OPENING transition to INIT grasp_fraction=%f ", hand_name_.c_str(), grasp_fraction);
//                   current_state = GRASP_INIT;
//                   this->release_grasp_=false;
//               }
//           }
//           break;
//       case GRASP_INIT:
//           update_joint_commands = false;
//           current_state = GRASP_INIT;
//           break;
//       case GRASP_STATE_NONE:
//           //moves fingers to none grasp pose
//           update_joint_commands = true;
//           close_percentage = 0;
//           grip_percentage  = 0;
//           setHandNoneData();
//           current_state = GRASP_INIT;
//           break;

//       default:
//           update_joint_commands = false;
//           //ROS_WARN(" Grasp controller is not active");
//       }

//       //Process tactile information
//       grasp_quality_ = this->processHandTactileData();

//       this->processHandMassData(hand_T_template, template_mass, template_com);


//       //Publisher for jointStates including tactile information
//       this->publishHandStates(update_joint_commands);

//       // Update the active state if any change
//       close_percentage += grip_percentage;

//       // If state has changed or finger closed value has changed, update active state and publish so UI can update
//       if ((prior_state != current_state) || (prior_mode != current_mode) || (close_percentage != this->active_state_.grip.data)||
//               (finger_effort[0] != this->active_state_.finger_effort[0].data) ||
//               (finger_effort[1] != this->active_state_.finger_effort[1].data) ||
//               (finger_effort[2] != this->active_state_.finger_effort[2].data) ||
//               (finger_effort[3] != this->active_state_.finger_effort[3].data) )
//       {
//           //ROS_INFO("close_percentage: %d  grip percentage: %d  active state grip data: %d", close_percentage, grip_percentage, active_state_.grip.data );
//           { // lock data that may be accessed in main
//               boost::lock_guard<boost::mutex> sensor_data_lock(this->write_data_mutex_);
//               this->active_state_.grasp_state.data = (current_mode << 4) + current_state;
//               this->active_state_.grip.data        = close_percentage;
//               this->active_state_.finger_effort[0].data= finger_effort[0];
//               this->active_state_.finger_effort[1].data= finger_effort[1];
//               this->active_state_.finger_effort[2].data= finger_effort[2];
//               this->active_state_.finger_effort[3].data= finger_effort[3];
//           }

//           if ((prior_state != current_state) || (prior_mode != current_mode))
//           { // don't bother to display if only change is grip
//               ROS_WARN(" %s: active state changed: mode = %d=?= %d current state=%d=?=%d (Active state = 0x%x) grip=%d", hand_name_.c_str(),
//                        current_mode, prior_mode, current_state, prior_state, this->active_state_.grasp_state.data,close_percentage);
//           }
//           this->updateActiveState(); // update on any change of state (or mode)
//       }

//       // Publish any changes to the grasp status
//       this->updateGraspStatus();
//       this->updateHandMass();

//   } // end of while run
//}


/////////////////////////////////////////////////////////////////////////
// END MAIN CONTROL LOOP
/////////////////////////////////////////////////////////////////////////
// Helper functions
void VigirManipulationController::requestTemplateService(const uint16_t& requested_template_type){
    //CALLING THE TEMPLATE SERVER
    vigir_object_template_msgs::GetTemplateStateAndTypeInfo srv;
    srv.request.template_type = requested_template_type;
    if (!template_info_client_.call(srv))
    {
        ROS_ERROR("Failed to call service request grasp info");
    }
    last_template_res_ = srv.response;
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
    vigir_object_template_msgs::DetachObjectTemplate srv;
    srv.request.template_id          = int16_t(last_template_data.template_id.data);
    if (!detach_object_client_.call(srv))
        ROS_ERROR("Failed to call service request DetachObjectTemplate");
}

void VigirManipulationController::gripperTranslationToPreGraspPose(geometry_msgs::Pose& pose, moveit_msgs::GripperTranslation& trans){
    geometry_msgs::Vector3Stamped direction = trans.direction;
    tf::Transform template_T_hand, vec_in, vec_out;
    ROS_INFO("receiving trans distance: %f; dx: %f, dy: %f, dz: %f", trans.desired_distance, direction.vector.x, direction.vector.y, direction.vector.z);
    float norm = sqrt((direction.vector.x * direction.vector.x) +(direction.vector.y * direction.vector.y) +(direction.vector.z * direction.vector.z));
    if(norm != 0){
        direction.vector.x /= norm;
        direction.vector.y /= norm;
        direction.vector.z /= norm;
    }else{
        ROS_INFO("Norm is ZERO!");
        direction.vector.x = 0 ;
        direction.vector.y = -1;
        direction.vector.z = 0 ;
    }

    direction.vector.x *= hand_side_ == "left" ? -trans.desired_distance : trans.desired_distance;  //Change due to Atlas specifics, hand axis are reflected
    direction.vector.y *= hand_side_ == "left" ? -trans.desired_distance : trans.desired_distance;  //Change due to Atlas specifics, hand axis are reflected
    direction.vector.z *= -trans.desired_distance;

    ROS_INFO("setting trans; dx: %f, dy: %f, dz: %f", direction.vector.x, direction.vector.y, direction.vector.z);

    template_T_hand.setRotation(tf::Quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w));
    template_T_hand.setOrigin(tf::Vector3(0,0,0));

    vec_in.setOrigin(tf::Vector3(direction.vector.x,direction.vector.y,direction.vector.z));
    vec_in.setRotation(tf::Quaternion(0,0,0,1));

    vec_out = template_T_hand * vec_in;

    ROS_INFO("setting result; dx: %f, dy: %f, dz: %f", vec_out.getOrigin().getX(), vec_out.getOrigin().getY(), vec_out.getOrigin().getZ());

    pose.position.x += vec_out.getOrigin().getX();
    pose.position.y += vec_out.getOrigin().getY();
    pose.position.z += vec_out.getOrigin().getZ();
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
