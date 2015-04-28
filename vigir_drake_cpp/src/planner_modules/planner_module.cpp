#include "vigir_drake_cpp/planner_modules/planner_module.h"

#include <moveit_msgs/RobotState.h>
#include <vigir_planning_msgs/RequestDrakeIK.h>

#include <sstream>

#include <tf/tf.h>

#include <RigidBodyManipulator.h>

using namespace Eigen;

namespace vigir_drake_cpp {
  
PlannerModule::PlannerModule(RigidBodyManipulator *robot_model)
{
    robot_model_ = robot_model;

    q_nom = Eigen::VectorXd(robot_model->num_positions);
    q_nom << -0.010963246226300001, 0.00053433817811300002, 0.87918889522599997, 3.1409064491353811, 3.1674250364257595, 3.1415213301084979, -4.8416051868116483e-05, 0.0015894935932010412, -0.00018978302250616252, -0.25412441522026064, -0.0098131755366921425, 0.075760267674922943, -0.46318930387496948, 0.94040477275848389, -0.45157152414321899, -0.075519748032093048, -1.3442893893127441, 1.9690395179184732, 0.47574423129844667, 0.0095752710476517677, -0.00085321138612926006, 0.022117273882031441, 0.292929194229126, 0.0098817348480224609, -0.075710050761699677, -0.46247932314872742, 0.93897950649261475, -0.45088508725166321, 0.076813079416751862, 1.3628664578170777, 1.8534582696990967, -0.48796679165649415, 0.0096058044582605362, 0.00084977783262729645, -0.025142166763544083, -0.021544275805354118, 0;
}

PlannerModule::~PlannerModule()
{
}

RigidBodyManipulator *PlannerModule::getRobotModel() {
    return robot_model_;
}

Eigen::VectorXd &PlannerModule::getNominalQ() {
    return q_nom;
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
        tf::Matrix3x3(rotation_quat).getRPY(result_qs[3], result_qs[4], result_qs[5], 2);

        received_world_transform = true;
    }

    if ( !received_world_transform ) {
        ROS_WARN("Did not receive unique world joint position...");
    }

    return result_qs;
}

void PlannerModule::printSortedQs(MatrixXd &qs) {
    std::stringstream name_stream;
    std::stringstream position_stream;

    for ( int i = 0; i < this->getRobotModel()->bodies.size(); i++ ) {
        int pos = this->getRobotModel()->bodies[i]->position_num_start;
        if ( pos >= 0 && pos < this->getRobotModel()->num_positions) {
            name_stream << this->getRobotModel()->bodies[i]->jointname << ", ";
            position_stream << qs.row(pos) << ", ";
        }
        //else
        //    std::cout << this->getRobotModel()->bodies[i]->jointname << "    no matching q for position_num = " << pos <<std::endl;
    }

    std::cout << name_stream.str() << std::endl;
    std::cout << position_stream.str() << std::endl;

    std::cout << "translation = [" << qs(0) << ", " << qs(1) << ", " << qs(2) << "]" << std::endl;
    std::cout << "orientation = [" << qs(3) << ", " << qs(4) << ", " << qs(5) << "]" << std::endl;
}

}
