#include <vigir_move_group/helper/drake_planning_helper.h>

#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/DisplayTrajectory.h>

#include <vigir_planning_msgs/RequestWholeBodyCartesianTrajectory.h>
#include <vigir_planning_msgs/RequestWholeBodyTrajectory.h>


#include <vigir_moveit_utils/planning_scene_utils.h>
#include <vigir_moveit_utils/constrained_motion_utils.h>

#include <nav_msgs/Path.h>

#include <tf/tf.h>
#include <eigen_conversions/eigen_msg.h>

#include <vigir_move_group/manipulation_action_capability.h>


namespace move_group {

DrakePlanningHelper::DrakePlanningHelper(MoveGroupManipulationAction *manipulation_action_capability)
    : manipulation_action_capability_(manipulation_action_capability)
{
    context_ = manipulation_action_capability_->context_;

    ros::NodeHandle nh;
    drake_trajectory_srv_client_ = nh.serviceClient<vigir_planning_msgs::RequestWholeBodyTrajectory>("drake_planner/request_whole_body_trajectory");
    drake_cartesian_trajectory_srv_client_ = nh.serviceClient<vigir_planning_msgs::RequestWholeBodyCartesianTrajectory>("drake_planner/request_whole_body_cartesian_trajectory");

    //Get hand parameters from server
    left_wrist_link_  = "l_hand";
    right_wrist_link_ = "r_hand";

    if(!nh.getParam("/left_wrist_link", left_wrist_link_))
        ROS_WARN("No left wrist link defined, using l_hand as default");

    if(!nh.getParam("/right_wrist_link", right_wrist_link_))
        ROS_WARN("No right wrist link defined, using r_hand as default");
}

DrakePlanningHelper::~DrakePlanningHelper() {

}

void DrakePlanningHelper::planAction(const vigir_planning_msgs::MoveGoalConstPtr& goal, vigir_planning_msgs::MoveResult &action_res) {
    struct timeval start_time;
    gettimeofday(&start_time, NULL);

    // execute planning
    plan_execution::ExecutableMotionPlan plan;
    DrakePlanningHelper::plan(goal, plan);

    // convert result
    action_res.error_code.val = plan.error_code_.val;

    if ( plan.plan_components_.size() >= 1 ) {
        manipulation_action_capability_->convertToMsg(plan.plan_components_, action_res.trajectory_start, action_res.planned_trajectory);
    }
    else {
        plan.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;

    }

    // set planning time
    struct timeval end_time, diff;
    gettimeofday(&end_time, NULL);
    timersub(&end_time, &start_time, &diff);
    action_res.planning_time = (double)diff.tv_sec + (double)diff.tv_usec/1000000.0;
    action_res.extended_planning_result.plan_completion_fraction = 1.0; // TODO: add correct fraction
}

void DrakePlanningHelper::planCartesianMotionAction(const vigir_planning_msgs::MoveGoalConstPtr& goal, vigir_planning_msgs::MoveResult &action_res) {
    struct timeval start_time;
    gettimeofday(&start_time, NULL);

    // execute planning
    plan_execution::ExecutableMotionPlan plan;
    planCartesianMotion(goal, plan);

    // convert result
    action_res.error_code.val = plan.error_code_.val;

    if ( plan.plan_components_.size() >= 1 ) {
        manipulation_action_capability_->convertToMsg(plan.plan_components_, action_res.trajectory_start, action_res.planned_trajectory);
    }
    else {
        plan.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;

    }

    // set planning time
    struct timeval end_time, diff;
    gettimeofday(&end_time, NULL);
    timersub(&end_time, &start_time, &diff);
    action_res.planning_time = (double)diff.tv_sec + (double)diff.tv_usec/1000000.0;
    action_res.extended_planning_result.plan_completion_fraction = 1.0; // TODO: add correct fraction
}

void DrakePlanningHelper::planCircularMotionAction(const vigir_planning_msgs::MoveGoalConstPtr& goal, vigir_planning_msgs::MoveResult &action_res) {
    struct timeval start_time;
    gettimeofday(&start_time, NULL);

    // execute planning
    plan_execution::ExecutableMotionPlan plan;
    planCircularMotion(goal, plan);

    // convert result
    action_res.error_code.val = plan.error_code_.val;

    if ( plan.plan_components_.size() >= 1 ) {
        manipulation_action_capability_->convertToMsg(plan.plan_components_, action_res.trajectory_start, action_res.planned_trajectory);
    }
    else {
        plan.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;

    }

    // set planning time
    struct timeval end_time, diff;
    gettimeofday(&end_time, NULL);
    timersub(&end_time, &start_time, &diff);
    action_res.planning_time = (double)diff.tv_sec + (double)diff.tv_usec/1000000.0;
    action_res.extended_planning_result.plan_completion_fraction = 1.0; // TODO: add correct fraction
}

bool DrakePlanningHelper::plan(const vigir_planning_msgs::MoveGoalConstPtr& goal, plan_execution::ExecutableMotionPlan &plan) {
    ROS_INFO("Planning request received for MoveGroup action. Forwarding to Drake.");
    manipulation_action_capability_->setMoveState(move_group::PLANNING);

    bool solved = false;
    planning_interface::MotionPlanResponse res;

    // get current robot state and model
    const robot_model::RobotModelConstPtr& robot_model = context_->planning_pipeline_->getRobotModel();
    const planning_scene::PlanningScenePtr& planning_scene = context_->planning_scene_monitor_->getPlanningScene();
    robot_state::RobotState current_robot_state = planning_scene->getCurrentState();
    moveit_msgs::RobotState current_state_msg;
    robot_state::robotStateToRobotStateMsg(current_robot_state, current_state_msg);

    vigir_planning_msgs::RequestWholeBodyTrajectory::Response drake_response_msg;

    try
    {
        //Everything OK in the beginning, this will be changed below if we encounter problems
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

        // build request message
        vigir_planning_msgs::RequestWholeBodyTrajectory::Request drake_request_msg;
        drake_request_msg.trajectory_request.current_state = current_state_msg;

        if ( goal->extended_planning_options.target_pose_times.size() > 0 )
            drake_request_msg.trajectory_request.duration = goal->extended_planning_options.target_pose_times[0];

        drake_request_msg.trajectory_request.trajectory_sample_rate = goal->extended_planning_options.trajectory_sample_rate;
        drake_request_msg.trajectory_request.check_self_collisions = goal->extended_planning_options.check_self_collisions;
        drake_request_msg.trajectory_request.motion_plan_request = goal->request;

        // call service and process response
        solved = drake_trajectory_srv_client_.call(drake_request_msg, drake_response_msg);

        if ( solved ) {
          res.trajectory_ = robot_trajectory::RobotTrajectoryPtr(new robot_trajectory::RobotTrajectory(robot_model, goal->request.group_name));
          res.trajectory_->setRobotTrajectoryMsg(current_robot_state, drake_response_msg.trajectory_result.result_trajectory);
        }
        else {
            res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        }
    }
    catch(std::runtime_error &ex)
    {
      ROS_ERROR("Planning pipeline threw an exception: %s", ex.what());
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    }
    catch(...)
    {
      ROS_ERROR("Planning pipeline threw an exception");
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    }

    // show trajectory preview and convert it to result message format
    if ( solved )
    {
      plan.plan_components_.resize(1);
      plan.plan_components_[0].trajectory_ = res.trajectory_;
      plan.plan_components_[0].description_ = "plan";

      manipulation_action_capability_->planned_traj_vis_->publishTrajectoryEndeffectorVis(*res.trajectory_);

      // display preview in rviz
      if (manipulation_action_capability_->trajectory_result_display_pub_.getNumSubscribers() > 0){
        moveit_msgs::DisplayTrajectory result_trajectory_display_msg;
        result_trajectory_display_msg.trajectory.push_back( drake_response_msg.trajectory_result.result_trajectory );
        result_trajectory_display_msg.trajectory_start = current_state_msg;
        result_trajectory_display_msg.model_id = robot_model->getName();
        manipulation_action_capability_->trajectory_result_display_pub_.publish(result_trajectory_display_msg);
      }
    }
    plan.error_code_ = res.error_code_;
    return solved;
}

bool DrakePlanningHelper::planCartesianMotion(const vigir_planning_msgs::MoveGoalConstPtr& goal, plan_execution::ExecutableMotionPlan &plan) {
    ROS_INFO("Planning request received for MoveGroup action. Forwarding to Drake.");
    manipulation_action_capability_->setMoveState(move_group::PLANNING);

    bool solved = false;
    planning_interface::MotionPlanResponse res;
    vigir_planning_msgs::RequestWholeBodyCartesianTrajectory::Response drake_response_msg;

    // get current robot state and model
    const robot_model::RobotModelConstPtr& robot_model = context_->planning_pipeline_->getRobotModel();
    const planning_scene::PlanningScenePtr& planning_scene = context_->planning_scene_monitor_->getPlanningScene();
    robot_state::RobotState current_robot_state = planning_scene->getCurrentState();
    moveit_msgs::RobotState current_state_msg;
    robot_state::robotStateToRobotStateMsg(current_robot_state, current_state_msg);
    const moveit::core::JointModelGroup *joint_model_group = current_robot_state.getJointModelGroup(goal->request.group_name);

    // if global world position is not set, try to get robot orientation from tf transform (THOR!)
    bool has_world_virtual_joint = ( std::find( current_state_msg.multi_dof_joint_state.joint_names.begin(), current_state_msg.multi_dof_joint_state.joint_names.end(), "world_virtual_joint" ) != current_state_msg.multi_dof_joint_state.joint_names.end() );
    if ( has_world_virtual_joint == false )
    {
        try{
          ROS_INFO("No world virtual joint given - using tf-transform");
          tf::StampedTransform pelvis_tf;
          transform_listener_.lookupTransform("/world", "/pelvis", ros::Time(0), pelvis_tf);

          geometry_msgs::Transform pelvis_pose_msg;
          tf::transformTFToMsg(pelvis_tf, pelvis_pose_msg);

          current_state_msg.multi_dof_joint_state.joint_names.push_back("world_virtual_joint");
          current_state_msg.multi_dof_joint_state.transforms.push_back(pelvis_pose_msg);
        }
        catch (tf::TransformException &ex) {
          ROS_WARN("%s",ex.what());
        }
    }

    try
    {
        //Everything OK in the beginning, this will be changed below if we encounter problems
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

        std::vector<std::string> target_link_names = goal->extended_planning_options.target_link_names;
        if ( target_link_names.empty()) {
            std::string eef_link_name;
            if( ! planning_scene_utils::get_eef_link(goal->request.group_name, eef_link_name)) {
              ROS_ERROR("Cannot get endeffector link name, circular planning not possible!");
              res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
              return false;
            }
            target_link_names.assign(goal->extended_planning_options.target_poses.size(), eef_link_name);
        }

        // handle reference points
        std::vector<geometry_msgs::Pose> world_target_poses = goal->extended_planning_options.target_poses;

        std::vector<geometry_msgs::Point> reference_points;
        if ( goal->extended_planning_options.reference_point.orientation.w != 0 ||
             goal->extended_planning_options.reference_point.orientation.x != 0 ||
             goal->extended_planning_options.reference_point.orientation.y != 0 ||
             goal->extended_planning_options.reference_point.orientation.z != 0 ) {

            geometry_msgs::PoseStamped reference_point_transformed;
            reference_point_transformed.pose = goal->extended_planning_options.reference_point;
            reference_point_transformed.header.frame_id = goal->extended_planning_options.reference_point_frame;

            // if we have a different reference frame, transform to end-effector frame
            if ( !goal->extended_planning_options.reference_point_frame.empty() ) {
                 transform_listener_.transformPose(target_link_names[0], reference_point_transformed, reference_point_transformed);
            }

            // update end-effector orientation of target poses
            tf::Transform base_orientation;
            tf::poseMsgToTF(reference_point_transformed.pose, base_orientation);
            base_orientation.setOrigin(tf::Vector3(0,0,0));

            for ( int i = 0; i < world_target_poses.size(); i++ ) {
                tf::Transform current_orientation;
                tf::poseMsgToTF(world_target_poses[i], current_orientation);
                current_orientation.setOrigin(tf::Vector3(0,0,0));

                tf::Transform result_orientation = current_orientation * base_orientation;
                tf::Quaternion result_quat = result_orientation.getRotation();
                world_target_poses[i].orientation.x = result_quat.getX();
                world_target_poses[i].orientation.y = result_quat.getY();
                world_target_poses[i].orientation.z = result_quat.getZ();
                world_target_poses[i].orientation.w = result_quat.getW();
            }

            // TODO: ExtendedPlanningOptions should have one of those per target pose to allow multiple end-effectors
            reference_points.assign(goal->extended_planning_options.target_poses.size(), reference_point_transformed.pose.position);
        }

        // transform waypoints to world frame if necessary
        if ( goal->extended_planning_options.target_frame != "/world" && goal->extended_planning_options.target_frame != "world" && goal->extended_planning_options.target_frame != "") {
            for ( int i = 0; i < world_target_poses.size(); i++ ) {
                geometry_msgs::PoseStamped current_pose_stamped, world_pose_stamped;
                current_pose_stamped.pose = world_target_poses[i];
                current_pose_stamped.header.frame_id = goal->extended_planning_options.target_frame;
                try {
                    transform_listener_.transformPose("world", current_pose_stamped, world_pose_stamped);
                    world_target_poses[i] = world_pose_stamped.pose;
                }
                catch(...) {
                    ROS_ERROR("[manipulation_action_capability] Error transforming target pose to world frame => Aborting!");
                    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
                    return false;
                }
            }
        }


        // build request message
        vigir_planning_msgs::RequestWholeBodyCartesianTrajectory::Request drake_request_msg;
        drake_request_msg.trajectory_request.current_state = current_state_msg;
        drake_request_msg.trajectory_request.waypoints = world_target_poses;
        drake_request_msg.trajectory_request.waypoint_times = goal->extended_planning_options.target_pose_times;
        drake_request_msg.trajectory_request.pos_on_eef = reference_points;
        drake_request_msg.trajectory_request.target_link_names = target_link_names;
        drake_request_msg.trajectory_request.target_link_axis = goal->extended_planning_options.target_link_axis;
        drake_request_msg.trajectory_request.free_joint_names = joint_model_group->getJointModelNames();
        drake_request_msg.trajectory_request.target_orientation_type = goal->extended_planning_options.target_orientation_type;
        drake_request_msg.trajectory_request.trajectory_sample_rate = goal->extended_planning_options.trajectory_sample_rate;
        drake_request_msg.trajectory_request.check_self_collisions = goal->extended_planning_options.check_self_collisions;
        drake_request_msg.trajectory_request.execute_incomplete_cartesian_plans = goal->extended_planning_options.execute_incomplete_cartesian_plans;
        drake_request_msg.trajectory_request.free_motion = (goal->extended_planning_options.target_motion_type == vigir_planning_msgs::ExtendedPlanningOptions::TYPE_FREE_MOTION);

        // call service and process response
        solved = drake_cartesian_trajectory_srv_client_.call(drake_request_msg, drake_response_msg);

        if ( solved ) {
          // if the robot model has no world joint, remove info from result trajectory
          if ( has_world_virtual_joint == false ) {
              drake_response_msg.trajectory_result.result_trajectory.multi_dof_joint_trajectory.joint_names.clear();
              drake_response_msg.trajectory_result.result_trajectory.multi_dof_joint_trajectory.points.clear();
          }

          res.trajectory_ = robot_trajectory::RobotTrajectoryPtr(new robot_trajectory::RobotTrajectory(robot_model, goal->request.group_name));
          res.trajectory_->setRobotTrajectoryMsg(current_robot_state, drake_response_msg.trajectory_result.result_trajectory);
        }
        else {
            res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        }
    }
    catch(std::runtime_error &ex)
    {
      ROS_ERROR("Planning pipeline threw an exception: %s", ex.what());
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    }
    catch(...)
    {
      ROS_ERROR("Planning pipeline threw an exception");
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    }
    if (solved)
    {
      plan.plan_components_.resize(1);
      plan.plan_components_[0].trajectory_ = res.trajectory_;
      plan.plan_components_[0].description_ = "plan";

      manipulation_action_capability_->planned_traj_vis_->publishTrajectoryEndeffectorVis(*res.trajectory_);

      // display preview in rviz
      if (manipulation_action_capability_->trajectory_result_display_pub_.getNumSubscribers() > 0){
        moveit_msgs::DisplayTrajectory result_trajectory_display_msg;
        result_trajectory_display_msg.trajectory.push_back( drake_response_msg.trajectory_result.result_trajectory );
        result_trajectory_display_msg.trajectory_start = current_state_msg;
        result_trajectory_display_msg.model_id = robot_model->getName();
        manipulation_action_capability_->trajectory_result_display_pub_.publish(result_trajectory_display_msg);
      }
    }
    plan.error_code_ = res.error_code_;
    return solved;
}

bool DrakePlanningHelper::planCircularMotion(const vigir_planning_msgs::MoveGoalConstPtr& goal, plan_execution::ExecutableMotionPlan &plan) {
    ROS_INFO("Received circular cartesian motion request! Forwarding to Drake");

    try
    {
        //Everything OK in the beginning, this will be changed below if we encounter problems
        plan.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

        if (goal->extended_planning_options.target_poses.size() != 1){
            ROS_ERROR("There has to be exactly one target pose for circular motion requests!");
            plan.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
            return false;
        }

        Eigen::Affine3d eef_start_pose;
        std::string eef_link_name;
        if ( goal->extended_planning_options.target_link_names.size() >= 1) { // get transform from link name
            eef_link_name = goal->extended_planning_options.target_link_names[0];

            if( !planning_scene_utils::getEndeffectorTransformOfLink(eef_link_name, context_->planning_scene_monitor_,
                                                                     eef_start_pose) ) {
                ROS_ERROR("Cannot get endeffector transform, cartesian planning not possible!");
                plan.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
                return false;
            }
        }
        else if ( !planning_scene_utils::getEndeffectorTransformOfLink(goal->request.group_name.substr(0,1) == "r" ? right_wrist_link_ : left_wrist_link_,
                                                                       context_->planning_scene_monitor_,
                                                                       eef_start_pose)) { // infer link name from group
            ROS_ERROR("Cannot get endeffector transform, cartesian planning not possible!");
            plan.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
            return false;
        }

        if( eef_link_name.empty() && !planning_scene_utils::get_eef_link(goal->request.group_name, eef_link_name)) {
            ROS_ERROR("Cannot get endeffector link name, circular planning not possible!");
            plan.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
            return false;
        }

        geometry_msgs::PoseStamped rotation_pose;
        rotation_pose.pose = goal->extended_planning_options.target_poses[0];
        rotation_pose.header.frame_id = goal->extended_planning_options.target_frame;

        //Can easily transform goal pose to arbitrary target frame
        if (manipulation_action_capability_->performTransform(rotation_pose, context_->planning_scene_monitor_->getRobotModel()->getModelFrame()))
        {
        Eigen::Affine3d rotation_center;
        tf::poseMsgToEigen(rotation_pose.pose, rotation_center);

        // Transform eef-pose according to reference_point
        if ( goal->extended_planning_options.reference_point.orientation.x != 0.0 ||
             goal->extended_planning_options.reference_point.orientation.y != 0.0 ||
             goal->extended_planning_options.reference_point.orientation.z != 0.0 ||
             goal->extended_planning_options.reference_point.orientation.w != 0.0 ) {

            tf::Transform wrist_T_referencePoint, targetFrame_T_referencePoint;
            geometry_msgs::Pose tmp_pose;
            tf::poseMsgToTF(goal->extended_planning_options.reference_point, wrist_T_referencePoint);
            tf::poseEigenToMsg(eef_start_pose, tmp_pose);
            tf::poseMsgToTF(tmp_pose, targetFrame_T_referencePoint);
            tf::Transform targetFrame_T_wrist = targetFrame_T_referencePoint * wrist_T_referencePoint.inverse();
            tf::poseTFToMsg(targetFrame_T_wrist,tmp_pose);
            tf::poseMsgToEigen(tmp_pose, eef_start_pose);
        }

        std::vector <geometry_msgs::Pose> pose_vec;
        constrained_motion_utils::getCircularArcPoses(rotation_center,
                                                      eef_start_pose,
                                                      pose_vec,
                                                      0.02,
                                                      goal->extended_planning_options.rotation_angle,
                                                      goal->extended_planning_options.keep_endeffector_orientation,
                                                      goal->extended_planning_options.pitch);

        if ( manipulation_action_capability_->circular_target_path_pub_.getNumSubscribers() > 0 ) {
            nav_msgs::Path path_msg;
            path_msg.header.frame_id = context_->planning_scene_monitor_->getRobotModel()->getModelFrame();
            path_msg.header.stamp = ros::Time::now();

            for ( int i = 0; i < pose_vec.size(); i++ ) {
                geometry_msgs::PoseStamped current_pose;
                current_pose.header.frame_id = context_->planning_scene_monitor_->getRobotModel()->getModelFrame();
                current_pose.header.stamp = ros::Time::now();
                current_pose.pose = pose_vec[i];

                path_msg.poses.push_back(current_pose);
            }

            manipulation_action_capability_->circular_target_path_pub_.publish(path_msg);
        }

        // make a copy of goal, so I can modify it
        vigir_planning_msgs::MoveGoalPtr new_goal( new vigir_planning_msgs::MoveGoal( *goal ) );

        new_goal->extended_planning_options.target_poses = pose_vec;
        new_goal->extended_planning_options.target_link_names.assign(pose_vec.size(), eef_link_name);
        if ( new_goal->extended_planning_options.target_link_axis.empty() == false ) {
            new_goal->extended_planning_options.target_link_axis.assign(pose_vec.size(), goal->extended_planning_options.target_link_axis[0]);
        }

        // call default cartesian motion handler
        return planCartesianMotion(new_goal, plan);
        }
        else {
            ROS_ERROR("Invalid target frame %s requested for drake circular planning!", goal->extended_planning_options.target_frame.c_str());
            plan.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        }
    }
    catch(std::runtime_error &ex)
    {
        ROS_ERROR("Planning pipeline threw an exception: %s", ex.what());
        plan.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        return false;
    }
    catch(...)
    {
        ROS_ERROR("Planning pipeline threw an exception");
        plan.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        return false;
    }
}

}
