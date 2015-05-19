
#include <vigir_ocs_robot_model/moveit_ocs_model_ros.h>

#include <vigir_planning_msgs/constraints_conversion.h>
#include "vigir_planning_msgs/RequestWholeBodyIK.h"

#include <vigir_moveit_utils/joint_constraint_utils.h>

MoveItOcsModelRos::MoveItOcsModelRos()
{
    ocs_model_.reset(new MoveItOcsModel());

    ros::NodeHandle nh("");

    collision_avoidance_active_ = true;

    use_drake_ik_ = false;
    whole_body_ik_client_ = nh.serviceClient<vigir_planning_msgs::RequestWholeBodyIK>("drake_planner/request_whole_body_ik");

    //flor_visualization_utils::test();

    robot_state_vis_pub_ = nh.advertise<moveit_msgs::DisplayRobotState>("/flor/ghost/robot_state_vis",1, true);
    robot_state_diff_real_vis_pub_ = nh.advertise<moveit_msgs::DisplayRobotState>("/flor/ghost/robot_state_diff_vis",1, true);

    marker_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/flor/ghost/marker",1, true);
    current_ghost_joint_states_pub_ = nh.advertise<sensor_msgs::JointState>("/flor/ghost/get_joint_states",1, true);

    // Publishing endeffector and ghost poses
    left_hand_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/flor/ghost/pose/left_hand",1);
    right_hand_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/flor/ghost/pose/right_hand",1);
    pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/flor/ghost/pose/robot",1);

    // Receive changes in ghost robot state (use Drake?)
    ghost_state_sub_ = nh.subscribe( "/flor/ocs/ghost/state_use_drake_ik", 1, &MoveItOcsModelRos::ghostStateCallback, this);

    // Planning requests
    pose_plan_request_pub_ = nh.advertise<flor_planning_msgs::PlanRequest>("/flor/planning/upper_body/plan_request",1);
    joint_plan_request_pub_ = nh.advertise<flor_planning_msgs::PlanToJointTargetRequest>("/flor/planning/upper_body/plan_joint_request",1);
    incoming_plan_to_pose_request_sub_ = nh.subscribe("/flor/ocs/planning/plan_to_pose_state", 5, &MoveItOcsModelRos::incomingPlanToPoseRequestCallback, this);
    incoming_plan_to_joint_request_sub_ = nh.subscribe("/flor/ocs/planning/plan_to_joint_state", 5, &MoveItOcsModelRos::incomingPlanToJointRequestCallback, this);


    ghost_pelvis_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>( "/flor/ocs/ghost/set_pose", 1 );
    pose_sub_ = nh.subscribe("/flor/ghost/set_appendage_poses", 1, &MoveItOcsModelRos::targetConfigCallback, this);
    root_pose_sub_ = nh.subscribe("/flor/ghost/set_root_pose", 1, &MoveItOcsModelRos::rootPoseCallback, this);
    incoming_joint_states_sub_ = nh.subscribe("/flor/ghost/set_joint_states", 5, &MoveItOcsModelRos::incomingJointStatesCallback, this);
    incoming_real_joint_states_sub_ = nh.subscribe("/atlas/joint_states", 5, &MoveItOcsModelRos::realJointStatesCallback, this);


    torso_joint_position_constraints_sub_ = nh.subscribe("/flor/planning/upper_body/configuration",
                                                         10,
                                                         &MoveItOcsModelRos::plannerConfigurationCb,
                                                         this);


    ee_pose_pub_timer_ =  nh.createTimer(ros::Duration(0.033), &MoveItOcsModelRos::pubEndeffectorPosesTimerCallback, this, false);

    const std::vector<std::string>& link_names = ocs_model_->getLinkNames();

    for (size_t i = 0; i < link_names.size(); ++i){
      moveit_msgs::ObjectColor tmp;
      tmp.id = link_names[i];
      tmp.color.a = 0.5;
      tmp.color.r = 0.0;
      tmp.color.g = 1.0;
      tmp.color.b = 0.0;
      display_state_msg_.highlight_links.push_back(tmp);
    }

    this->updateRobotStateColors();

    ros::param::param<std::string>("~base_frame", base_frame_, "/world");
    ros::param::param<std::string>("~l_hand_frame", l_hand_frame_, "l_hand");
    ros::param::param<std::string>("~r_hand_frame", r_hand_frame_, "r_hand");
    ros::param::param<std::string>("~pelvis_frame", pelvis_frame_, "pelvis");
}

void MoveItOcsModelRos::targetConfigCallback (const flor_planning_msgs::TargetConfigIkRequest::ConstPtr& msg)
{
  if ( use_drake_ik_ == false ) { // continue with tried and true classic MoveIt IK
    if (msg->target_poses.size() == 1){

      size_t found_index = msg->planning_group.data.find_last_of("_position_only_ik");

      if (found_index < msg->planning_group.data.size()){
        std::string group_no_pos_ik = msg->planning_group.data.substr(0, found_index);

        if (!ocs_model_->setByIk(msg->target_poses[0], group_no_pos_ik)){
          ocs_model_->setByIk(msg->target_poses[0], msg->planning_group.data);
        }
      }else{
        ocs_model_->setByIk(msg->target_poses[0], msg->planning_group.data);
      }

    }else{
      ROS_WARN("Multiple target poses currently not supported!");
    }
  }
  else {
    std::vector< ::geometry_msgs::PoseStamped > testVec;
    testVec = msg->target_poses;
    setPoseWithWholeBodyIK(msg->target_poses, msg->target_link_names, msg->planning_group.data);
  }
  this->onModelUpdated();
}

void MoveItOcsModelRos::incomingJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    ocs_model_->setJointStates(*msg);
    this->onModelUpdated();
}

void MoveItOcsModelRos::realJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    real_joint_states_ = msg;
}

// Sets global pose of model
void MoveItOcsModelRos::rootPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ocs_model_->setRootTransform(*msg);
    this->onModelUpdated();
}

// Publishes endeffector poses at 30Hz
void MoveItOcsModelRos::pubEndeffectorPosesTimerCallback(const ros::TimerEvent& event)
{
    geometry_msgs::PoseStamped tmp;
    tmp.header.stamp = ros::Time::now();
    tmp.header.frame_id = base_frame_;

    ocs_model_->getLinkPose(l_hand_frame_,tmp.pose);
    left_hand_pose_pub_.publish(tmp);

    ocs_model_->getLinkPose(r_hand_frame_,tmp.pose);
    right_hand_pose_pub_.publish(tmp);

    ocs_model_->getLinkPose(pelvis_frame_,tmp.pose);
    pose_pub_.publish(tmp);

    //this->onModelUpdated();
}

// To be called when model changed
void MoveItOcsModelRos::onModelUpdated()
{
    ros::Time now = ros::Time::now();

    robot_state::robotStateToRobotStateMsg(*ocs_model_->getState(), display_state_msg_.state);
    display_state_msg_.state.joint_state.header.stamp = now;
    display_state_msg_.state.multi_dof_joint_state.header.stamp = now;

    this->updateRobotStateColors();

    std::vector<std::string> colliding_links;
    ocs_model_->getCollidingLinks(colliding_links);
    for (size_t i = 0; i < colliding_links.size(); ++i){
      setLinkColor(0.5,0.0,0.0,1.0,ocs_model_->getLinkIndex(colliding_links[i]));
      //std::cout << "Coll Link: " << colliding_links[i] << "\n";
    }

    robot_state_vis_pub_.publish(display_state_msg_);

    current_ghost_joint_states_pub_.publish(display_state_msg_.state.joint_state);

    //If we got real joint_states, publish diff visualization
    if (false && real_joint_states_.get()){
      std::vector<std::string> differing_links;
      ocs_model_->getDifferingLinks(*real_joint_states_, differing_links);

      for (size_t i = 0; i < differing_links.size(); ++i){
        setLinkColor(0.5,0.0,0.0,0.0,ocs_model_->getLinkIndex(differing_links[i]));
      }

      //publish partly translucent diff state here
      robot_state_diff_real_vis_pub_.publish(display_state_msg_);
    }

    /*
    if (marker_array_pub_.getNumSubscribers() > 0){

      visualization_msgs::MarkerArray markers;
      Eigen::MatrixXcd eigen_values;
      Eigen::MatrixXcd eigen_vectors;
      ocs_model_->getManipulationMetrics("l_arm_group", eigen_values, eigen_vectors);

      geometry_msgs::PoseStamped tmp;
      tmp.header.stamp = ros::Time::now();
      tmp.header.frame_id = "/world";
      ocs_model_->getLinkPose("l_hand",tmp.pose);

      flor_visualization_utils::drawEllipsoid(tmp, eigen_values, eigen_vectors, markers);

      marker_array_pub_.publish(markers);
    }
    */
}

void MoveItOcsModelRos::plannerConfigurationCb(const vigir_planning_msgs::PlannerConfiguration::ConstPtr& msg)
{
    ROS_INFO("Received planner settings");

    std::vector<moveit_msgs::JointConstraint> joint_constraints;

    /*
    joint_constraints.resize(msg->joint_position_constraints.size());

    for (size_t i = 0; i < msg->joint_position_constraints.size(); ++i){
      vigir_planning_msgs::toMoveitConstraint(msg->joint_position_constraints[i], joint_constraints[i]);

      joint_constraints[i].weight = 0.5;
      joint_constraints[i].joint_name = ocs_model_->getModel().getJointOfVariable(msg->joint_position_constraints[i].joint_index)->getName();
    }*/

    if (!joint_constraint_utils::toMoveitConstraint(msg->joint_position_constraints, ocs_model_->getModel(), joint_constraints)){
      ROS_ERROR("Error in joint constraint conversions, not using constraints!");
    }


    ocs_model_->setJointPositionConstraints(joint_constraints);

    collision_avoidance_active_ = !msg->disable_collision_avoidance;

    this->onModelUpdated();
}

void MoveItOcsModelRos::incomingPlanToPoseRequestCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Received plan to pose request, sending group %s", msg->data.c_str());

    const std::string& group_name = msg->data;

    std::string link_name;

    //TODO: It would be nice to not hard code the group checks
    if (group_name == "l_arm_group" || group_name == "l_arm_with_torso_group"){
      link_name = l_hand_frame_;
    }else{
      link_name = r_hand_frame_;
    }

    geometry_msgs::PoseStamped target_pose;
    target_pose.header.stamp = ros::Time::now();
    target_pose.header.frame_id = "/world";

    if (ocs_model_->getLinkPose(link_name,target_pose.pose))
    {

      flor_planning_msgs::PlanRequest plan_request;

      plan_request.pose = target_pose;
      plan_request.use_environment_obstacle_avoidance.data = true;
      plan_request.planning_group.data = group_name;

      pose_plan_request_pub_.publish(plan_request);
    }else{
      ROS_ERROR("Couldn't send plan request due to wrong link name %s in message", link_name.c_str());
    }

}

void MoveItOcsModelRos::incomingPlanToJointRequestCallback(const std_msgs::String::ConstPtr& msg)
{
  flor_planning_msgs::PlanToJointTargetRequest request;

  if ( use_drake_ik_ )
  {
    request.position.clear();
    request.planning_group = msg->data;

    // get ghost robot state for planning group
    const robot_state::RobotState *current_robot_state = ocs_model_->getState().get();

    //request.planning_group = "whole_body_group";
    if ( current_robot_state->getJointModelGroup(msg->data) == NULL) {
      ROS_ERROR("Request for unknown planning group: %s - Aborting...", msg->data.c_str());
      return;
    }

    const robot_state::JointModelGroup *current_model_group = current_robot_state->getJointModelGroup(msg->data);
    std::vector<std::string> current_joint_names = current_model_group->getJointModelNames();
    for ( int i = 0; i < current_joint_names.size(); i++) {
      std::string current_joint_name = current_joint_names[i];
      double current_position = current_robot_state->getVariablePosition(current_joint_name);
      request.position.push_back(current_position);
    }

    request.planner_id = "drake";
  }else{
    ROS_INFO("Received plan to joint config request, sending group %s", msg->data.c_str());

    const std::string& group = msg->data;

    if (!ocs_model_->getGroupJointPositions(group, request.position)){
      ROS_WARN("Received plan to joint state request for group %s but cannot determine joint states for it", msg->data.c_str());
      return;
    }

    request.planning_group = group;
    request.planner_id = ""; // use default planner
  }

  joint_plan_request_pub_.publish(request);
}

void MoveItOcsModelRos::ghostStateCallback(const std_msgs::Bool::ConstPtr& msg)
{
    use_drake_ik_ = msg->data;
}

void MoveItOcsModelRos::updateRobotStateColors()
{
    if (!collision_avoidance_active_){
      this->setLinkColors(0.5, 0.5, 0.0, 1.0);
    }else{
      this->setLinkColors(0.0, 0.5, 0.0, 1.0);
    }

}

void MoveItOcsModelRos::setLinkColor(double r, double g, double b, double a, size_t index)
{
    display_state_msg_.highlight_links[index].color.r = r;
    display_state_msg_.highlight_links[index].color.g = g;
    display_state_msg_.highlight_links[index].color.b = b;
    display_state_msg_.highlight_links[index].color.a = a;
}

void MoveItOcsModelRos::setLinkColors(double r, double g, double b, double a)
{
    for (size_t i = 0; i < display_state_msg_.highlight_links.size(); ++i){
      setLinkColor(r,g,b,a,i);
    }
}

void MoveItOcsModelRos::setPoseWithWholeBodyIK(std::vector<geometry_msgs::PoseStamped> target_poses, std::vector<std_msgs::String> target_link_names, const std::string& group_name)
{
    if ( target_link_names.size() != target_poses.size() ) {
        ROS_WARN("Different sizes for target_link_names and target_poses => Aborting");
        return;
    }

    // build request message
    const robot_state::RobotState *current_robot_state = ocs_model_->getState().get();
    moveit_msgs::RobotState robot_state_msg;
    moveit::core::robotStateToRobotStateMsg(*current_robot_state, robot_state_msg);

    const robot_state::JointModelGroup* joint_model_group = current_robot_state->getJointModelGroup(group_name);
    if ( joint_model_group == NULL ) {
        ROS_WARN("Requesting unknown joint model group: %s", group_name.c_str());
        return;
    }


    const std::vector<std::string> joint_model_names = joint_model_group->getJointModelNames();

    vigir_planning_msgs::RequestWholeBodyIK::Request request_msg;
    request_msg.ik_request.robot_state = robot_state_msg;
    request_msg.ik_request.target_poses = target_poses;
    for ( int i = 0; i < target_link_names.size(); i++) {
        request_msg.ik_request.target_link_names.push_back(target_link_names[i].data);
    }
    request_msg.ik_request.free_joint_names = joint_model_names;


    // call service and process response
    vigir_planning_msgs::RequestWholeBodyIK::Response response_msg;
    bool result = whole_body_ik_client_.call(request_msg, response_msg);
    if ( result )
    {
        // update ghost joint states with new results
        ocs_model_->setJointStates(response_msg.ik_result.result_state.joint_state);

        geometry_msgs::PoseStamped rootTransformMsg;
        rootTransformMsg.pose.orientation = response_msg.ik_result.result_state.multi_dof_joint_state.transforms[0].rotation;
        rootTransformMsg.pose.position.x = response_msg.ik_result.result_state.multi_dof_joint_state.transforms[0].translation.x;
        rootTransformMsg.pose.position.y = response_msg.ik_result.result_state.multi_dof_joint_state.transforms[0].translation.y;
        rootTransformMsg.pose.position.z = response_msg.ik_result.result_state.multi_dof_joint_state.transforms[0].translation.z;
        ocs_model_->setRootTransform(rootTransformMsg);

        this->onModelUpdated();
    }
    else
    {
        ROS_WARN("Unable to resolve whole body IK request - Doing nothing...");
    }
}

