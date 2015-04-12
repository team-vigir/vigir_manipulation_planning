#ifndef POSITION_IK_PLANNER_MODULE_H
#define POSITION_IK_PLANNER_MODULE_H

#include "planner_module.h"

#include "vigir_planning_msgs/RequestDrakeIK.h"
#include "vigir_planning_msgs/ResultDrakeIK.h"

#include <vector>

class RigidBodyManipulator;
class RigidBodyConstraint;
class IKoptions;

namespace vigir_drake_cpp {
  
class PositionIKPlannerModule : public PlannerModule
{
public:
    PositionIKPlannerModule(RigidBodyManipulator *robot_model);
    ~PositionIKPlannerModule();

    bool plan(vigir_planning_msgs::RequestDrakeIK &request_message, vigir_planning_msgs::ResultDrakeIK &result_message);

private:
    std::vector<RigidBodyConstraint*> buildIKConstraints(vigir_planning_msgs::RequestDrakeIK &request_message, Eigen::VectorXd &q0);
    IKoptions *buildIKOptions(vigir_planning_msgs::RequestDrakeIK &request_message);
};

}

#endif // POSITION_IK_PLANNER_MODULE_H
