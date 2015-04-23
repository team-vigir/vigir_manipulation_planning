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
    vigir_planning_msgs::ResultDrakeTrajectory buildTrajectoryResultMsg(Eigen::MatrixXd &q_sol, Eigen::MatrixXd &qd_sol, Eigen::MatrixXd &qdd_sol, std::vector<double> t_vec,std::vector<std::string> &joint_names, bool send_world_transform);
    void interpolateTrajectory(Eigen::MatrixXd &input_q, Eigen::VectorXd &input_t, Eigen::VectorXd &knot_t);

private:
    const int NUM_TIME_STEPS = 3;
};

}

#endif // TRAJECTORY_PLANNER_MODULE_H
