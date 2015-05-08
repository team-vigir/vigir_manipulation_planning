#ifndef TRAJECTORY_PLANNER_MODULE_H
#define TRAJECTORY_PLANNER_MODULE_H

#include "planner_module.h"

#include <vigir_planning_msgs/RequestDrakeTrajectory.h>
#include <vigir_planning_msgs/ResultDrakeTrajectory.h>

#include <vector>
#include <unsupported/Eigen/Splines>

class RigidBodyManipulator;
class RigidBodyConstraint;
class IKoptions;

namespace vigir_drake_cpp {

typedef Eigen::Spline<double,NUM_POSITIONS+1> TrajectorySpline;
  
class TrajectoryPlannerModule : public PlannerModule
{
public:
    TrajectoryPlannerModule(RigidBodyManipulator *robot_model );
    ~TrajectoryPlannerModule();

    bool plan(vigir_planning_msgs::RequestDrakeTrajectory &request_message, vigir_planning_msgs::ResultDrakeTrajectory &result_message);

protected:
    std::vector<RigidBodyConstraint*> buildIKConstraints(vigir_planning_msgs::RequestDrakeTrajectory &request_message, Eigen::VectorXd &q0);
    IKoptions *buildIKOptions(double duration);
    vigir_planning_msgs::ResultDrakeTrajectory buildTrajectoryResultMsg(Eigen::MatrixXd &q, Eigen::MatrixXd &qd, Eigen::MatrixXd &qdd, Eigen::VectorXd &t, std::vector<std::string> &joint_names, bool send_world_transform);
    void interpolateTrajectory(Eigen::MatrixXd &input_q, Eigen::VectorXd &input_t, Eigen::VectorXd &interpolated_t, Eigen::MatrixXd &interpolated_q, Eigen::MatrixXd &interpolated_qd, Eigen::MatrixXd &interpolated_qdd);

private:
    const int NUM_TIME_STEPS = 3;
};

}

#endif // TRAJECTORY_PLANNER_MODULE_H
