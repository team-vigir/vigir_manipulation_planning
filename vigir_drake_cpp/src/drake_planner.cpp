#include "vigir_drake_cpp/drake_planner.h"
#include <vigir_planning_msgs/RequestDrakeIK.h>
#include <vigir_planning_msgs/ResultDrakeIK.h>
#include <vigir_planning_msgs/RequestDrakeTrajectory.h>
#include <vigir_planning_msgs/ResultDrakeTrajectory.h>

#include "RigidBodyIK.h"
#include "RigidBodyManipulator.h"
#include "RigidBodyConstraint.h"
#include "IKoptions.h"
#include <iostream>
#include <cstdlib>

#include <vigir_drake_cpp/planner_modules/position_ik_planner_module.h>
#include <vigir_drake_cpp/planner_modules/cartesian_trajectory_planner_module.h>
#include <vigir_drake_cpp/planner_modules/trajectory_planner_module.h>

using namespace std;
using namespace Eigen;

namespace vigir_drake_cpp {

DrakePlanner::DrakePlanner() 
{
    robot_model_ = new RigidBodyManipulator();
    //robot_model_->use_new_kinsol = false;

    std::string urdf_string = "";
    bool success = node_handle_.getParam("/robot_description", urdf_string);
    if ( success == false ) {
        ROS_ERROR("DrakePlanner: Unable to get URDF string from /robot_description. Aborting...");
        exit(-1);
    }

    success = robot_model_->addRobotFromURDFString(urdf_string);
    if ( success == false ) {
        ROS_ERROR("DrakePlanner: Unable to add robot from URDF string. Aborting...");
        exit(-1);
    }

    ROS_INFO("DrakePlanner: Robot has %d bodies: ", robot_model_->num_positions);

    whole_body_ik_service_ = node_handle_.advertiseService("drake_planner/request_whole_body_ik", &DrakePlanner::handleWholeBodyIKRequest, this);
    whole_body_trajectory_service_ = node_handle_.advertiseService("drake_planner/request_whole_body_trajectory", &DrakePlanner::handleWholeBodyTrajectoryRequest, this);
    whole_body_cartesian_trajectory_service_ = node_handle_.advertiseService("drake_planner/request_whole_body_cartesian_trajectory", &DrakePlanner::handleWholeBodyCartesianTrajectoryRequest, this);

    position_ik_planner_ = new PositionIKPlannerModule(robot_model_);
    trajectory_planner_ = new TrajectoryPlannerModule(robot_model_);
    cartesian_trajectory_planner_ = new CartesianTrajectoryPlannerModule(robot_model_);

    ROS_INFO("DrakePlanner: Initialization successful.");
}

DrakePlanner::~DrakePlanner() 
{
    if ( robot_model_ ) {
        delete robot_model_;
        robot_model_ = nullptr;
    }

    if ( position_ik_planner_ ) {
        delete position_ik_planner_;
        position_ik_planner_ = nullptr;
    }

    if ( cartesian_trajectory_planner_ ) {
        delete cartesian_trajectory_planner_;
        cartesian_trajectory_planner_ = nullptr;
    }

    if ( trajectory_planner_ ) {
        delete trajectory_planner_;
        trajectory_planner_ = nullptr;
    }
}

bool DrakePlanner::handleWholeBodyIKRequest(vigir_planning_msgs::RequestWholeBodyIK::Request &request, vigir_planning_msgs::RequestWholeBodyIK::Response &response)
{
    ROS_INFO("Received IK request");

    if ( request.ik_request.target_link_names.size() != request.ik_request.target_poses.size() ) {
        ROS_WARN("number of target poses and names does not match => aborting");
        return false;
    }

    bool success = position_ik_planner_->plan(request.ik_request, response.ik_result);
    ROS_INFO("Done with calculation");
    return success;
}

bool DrakePlanner::handleWholeBodyTrajectoryRequest(vigir_planning_msgs::RequestWholeBodyTrajectory::Request &request, vigir_planning_msgs::RequestWholeBodyTrajectory::Response &response)
{
    ROS_INFO("Received IK trajectory request");
    bool success = trajectory_planner_->plan(request.trajectory_request, response.trajectory_result);
    ROS_INFO("Done with calculation");
    return success;
}

bool DrakePlanner::handleWholeBodyCartesianTrajectoryRequest(vigir_planning_msgs::RequestWholeBodyCartesianTrajectory::Request &request, vigir_planning_msgs::RequestWholeBodyCartesianTrajectory::Response &response)
{
    ROS_INFO("Received IK cartesian trajectory request");
    bool success = cartesian_trajectory_planner_->plan(request.trajectory_request, response.trajectory_result);
    ROS_INFO("Done with calculation");
    return success;
}

}