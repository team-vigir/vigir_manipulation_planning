#ifndef PLANNER_HELPER_H
#define PLANNER_HELPER_H

#include <moveit_msgs/RobotState.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>

class RigidBodyManipulator;

namespace Eigen {
    typedef Matrix<double, 7, 1> Vector7d;
}

namespace vigir_drake_cpp {
  
class PlannerModule
{

public:
    PlannerModule(RigidBodyManipulator *robot_model);
    ~PlannerModule();

protected:
    RigidBodyManipulator *getRobotModel();
    Eigen::VectorXd messageQs2DrakeQs(Eigen::VectorXd &q0, moveit_msgs::RobotState &robot_state, bool &received_world_transform);
    Eigen::MatrixXd drakeQs2MessageQs(Eigen::MatrixXd &model_qs, moveit_msgs::RobotState &request_state);

private:
    RigidBodyManipulator *robot_model_;
};

}

#endif // PLANNER_HELPER_H
