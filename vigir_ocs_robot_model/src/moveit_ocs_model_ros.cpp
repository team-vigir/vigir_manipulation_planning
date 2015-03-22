
#include <vigir_ocs_robot_model/moveit_ocs_model_ros.h>

#include <vigir_planning_msgs/constraints_conversion.h>

MoveItOcsModelRos::MoveItOcsModelRos()
{
    ocs_model_.reset(new MoveItOcsModel());

    ros::NodeHandle nh("");

    collision_avoidance_active_ = true;

    //flor_visualization_utils::test();

    robot_state_vis_pub_ = nh.advertise<moveit_msgs::DisplayRobotState>("/flor/ghost/robot_state_vis",1, true);
    marker_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/flor/ghost/marker",1, true);
    current_ghost_joint_states_pub_ = nh.advertise<sensor_msgs::JointState>("/flor/ghost/get_joint_states",1, true);

    // Publishing endeffector and ghost poses
    left_hand_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/flor/ghost/pose/left_hand",1);
    right_hand_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/flor/ghost/pose/right_hand",1);
    pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/flor/ghost/pose/robot",1);

    // Planning requests
    pose_plan_request_pub_ = nh.advertise<flor_planning_msgs::PlanRequest>("/flor/planning/upper_body/plan_request",1);
    joint_plan_request_pub_ = nh.advertise<flor_planning_msgs::PlanToJointTargetRequest>("/flor/planning/upper_body/plan_joint_request",1);
    incoming_plan_to_pose_request_sub_ = nh.subscribe("/flor/ocs/planning/plan_to_pose_state", 5, &MoveItOcsModelRos::incomingPlanToPoseRequestCallback, this);
    incoming_plan_to_joint_request_sub_ = nh.subscribe("/flor/ocs/planning/plan_to_joint_state", 5, &MoveItOcsModelRos::incomingPlanToJointRequestCallback, this);


    pose_sub_ = nh.subscribe("/flor/ghost/set_appendage_poses", 1, &MoveItOcsModelRos::targetConfigCallback, this);
    root_pose_sub_ = nh.subscribe("/flor/ghost/set_root_pose", 1, &MoveItOcsModelRos::rootPoseCallback, this);
    incoming_joint_states_sub_ = nh.subscribe("/flor/ghost/set_joint_states", 5, &MoveItOcsModelRos::incomingJointStatesCallback, this);

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

}

void MoveItOcsModelRos::targetConfigCallback (const flor_planning_msgs::TargetConfigIkRequest::ConstPtr& msg)
{
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
    this->onModelUpdated();
}

void MoveItOcsModelRos::incomingJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    ocs_model_->setJointStates(*msg);
    this->onModelUpdated();
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
    tmp.header.frame_id = "/world";

    ocs_model_->getLinkPose("l_hand",tmp.pose);
    left_hand_pose_pub_.publish(tmp);

    ocs_model_->getLinkPose("r_hand",tmp.pose);
    right_hand_pose_pub_.publish(tmp);

    ocs_model_->getLinkPose("pelvis",tmp.pose);
    pose_pub_.publish(tmp);
}

// To be called when model changed
void MoveItOcsModelRos::onModelUpdated()
{
    robot_state::robotStateToRobotStateMsg(*ocs_model_->getState(), display_state_msg_.state);
    this->updateRobotStateColors();

    std::vector<std::string> colliding_links;
    ocs_model_->getCollidingLinks(colliding_links);
    for (size_t i = 0; i < colliding_links.size(); ++i){
      setLinkColor(0.5,0.0,0.0,1.0,ocs_model_->getLinkIndex(colliding_links[i]));
      //std::cout << "Coll Link: " << colliding_links[i] << "\n";
    }

    robot_state_vis_pub_.publish(display_state_msg_);

    sensor_msgs::JointState joint_state_ghost;
    ocs_model_->getJointStates(joint_state_ghost);
    current_ghost_joint_states_pub_.publish(joint_state_ghost);

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
}

void MoveItOcsModelRos::plannerConfigurationCb(const vigir_planning_msgs::PlannerConfiguration::ConstPtr& msg)
{
    ROS_INFO("Received planner settings");

    std::vector<moveit_msgs::JointConstraint> joint_constraints;

    joint_constraints.resize(msg->joint_position_constraints.size());

    for (size_t i = 0; i < msg->joint_position_constraints.size(); ++i){
      vigir_planning_msgs::toMoveitConstraint(msg->joint_position_constraints[i], joint_constraints[i]);

      joint_constraints[i].weight = 0.5;
      joint_constraints[i].joint_name = ocs_model_->getModel().getJointOfVariable(msg->joint_position_constraints[i].joint_index)->getName();
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

    if (group_name == "l_arm_group" || group_name == "l_arm_with_torso_group"){
      link_name = "l_hand";
    }else{
      link_name = "r_hand";
    }

    geometry_msgs::PoseStamped target_pose;
    target_pose.header.stamp = ros::Time::now();
    target_pose.header.frame_id = "/world";

    ocs_model_->getLinkPose(link_name,target_pose.pose);

    flor_planning_msgs::PlanRequest plan_request;

    plan_request.pose = target_pose;
    plan_request.use_environment_obstacle_avoidance.data = true;
    plan_request.planning_group.data = group_name;

    pose_plan_request_pub_.publish(plan_request);

}

void MoveItOcsModelRos::incomingPlanToJointRequestCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Received plan to joint config request, sending group %s", msg->data.c_str());

    const std::string& group = msg->data;

    flor_planning_msgs::PlanToJointTargetRequest request;

    if (!ocs_model_->getGroupJointPositions(group, request.position)){
      ROS_WARN("Received plan to joint state request for group %s but cannot determine joint states for it", msg->data.c_str());
      return;
    }

    request.planning_group = group;

    joint_plan_request_pub_.publish(request);
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

