#include "vigir_drake_cpp/planner_modules/planner_module.h"

#include <moveit_msgs/RobotState.h>
#include <vigir_planning_msgs/RequestDrakeIK.h>

#include <tf/tf.h>

#include <RigidBodyManipulator.h>

using namespace Eigen;

namespace vigir_drake_cpp {
  
PlannerModule::PlannerModule(RigidBodyManipulator *robot_model)
{
    robot_model_ = robot_model;
}

PlannerModule::~PlannerModule()
{
}

RigidBodyManipulator *PlannerModule::getRobotModel() {
    return robot_model_;
}

MatrixXd PlannerModule::drakeQs2MessageQs(MatrixXd &model_qs, moveit_msgs::RobotState &request_state) {
    std::vector<std::string> message_joint_names = request_state.joint_state.name;
    int num_message_qs = message_joint_names.size();
    int num_states = model_qs.cols();

    MatrixXd result_state(num_message_qs, num_states);

    for ( int i = 0; i < num_message_qs; i++ ) {
        int body_idx = robot_model_->findJointId(message_joint_names[i], -1);
        int model_q_idx = robot_model_->bodies[body_idx]->position_num_start;

        result_state.row(i) = model_qs.row(model_q_idx);
    }

    return result_state;
}

VectorXd PlannerModule::messageQs2DrakeQs(VectorXd &q0, moveit_msgs::RobotState &robot_state, bool &received_world_transform) {
    VectorXd result_qs = q0;

    // add single joint values
    std::vector<std::string> message_joint_names = robot_state.joint_state.name;
    std::vector<double> message_qs = robot_state.joint_state.position;

    for ( int i = 0; i < message_joint_names.size(); i++ ) {
        int body_idx = robot_model_->findJointId(message_joint_names[i], -1);
        if (body_idx == -1 )
            continue;

        int model_q_idx = robot_model_->bodies[body_idx]->position_num_start;
        result_qs[model_q_idx] = message_qs[i];
    }

    // add world position if possible
    received_world_transform = false;
    for ( int i = 0; i < robot_state.multi_dof_joint_state.joint_names.size(); i++) {
        if ( robot_state.multi_dof_joint_state.joint_names[i].find("world_virtual_joint") == std::string::npos )
            continue;

        geometry_msgs::Transform world_transform = robot_state.multi_dof_joint_state.transforms[i];
        result_qs[0] = world_transform.translation.x;
        result_qs[1] = world_transform.translation.y;
        result_qs[2] = world_transform.translation.z;

        tf::Quaternion rotation_quat(world_transform.rotation.x, world_transform.rotation.y, world_transform.rotation.z, world_transform.rotation.w);
        tf::Matrix3x3(rotation_quat).getRPY(result_qs[3], result_qs[4], result_qs[5]);

        received_world_transform = true;
    }

    if ( !received_world_transform ) {
        ROS_WARN("Did not receive unique world joint position...");
    }

    return result_qs;
}

}