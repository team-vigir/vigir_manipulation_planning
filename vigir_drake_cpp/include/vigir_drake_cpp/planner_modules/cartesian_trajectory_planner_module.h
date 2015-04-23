#ifndef CARTESIAN_TRAJECTORY_PLANNER_MODULE_H
#define CARTESIAN_TRAJECTORY_PLANNER_MODULE_H

#include "trajectory_planner_module.h"

#include <vigir_planning_msgs/RequestDrakeCartesianTrajectory.h>
#include <vigir_planning_msgs/ResultDrakeTrajectory.h>

#include <vector>

namespace vigir_drake_cpp {
  
class CartesianTrajectoryPlannerModule : protected TrajectoryPlannerModule {
    struct Waypoint {
        std::vector<std::string> target_link_names;
        std::vector<geometry_msgs::Pose> poses;
        std::vector<geometry_msgs::Point> target_link_axis;
        std::vector<bool> keep_line_and_orientation;
        double waypoint_time;
    };

public:
    CartesianTrajectoryPlannerModule(RigidBodyManipulator *robot_model );
    ~CartesianTrajectoryPlannerModule();

    bool plan(vigir_planning_msgs::RequestDrakeCartesianTrajectory &request_message, vigir_planning_msgs::ResultDrakeTrajectory &result_message);
    
protected:
    std::vector<RigidBodyConstraint*> buildIKConstraints(vigir_planning_msgs::RequestDrakeCartesianTrajectory &request_message, Waypoint *start_waypoint, Waypoint *target_waypoint, Eigen::VectorXd &q0);    
    std::vector<CartesianTrajectoryPlannerModule::Waypoint*> extractOrderedWaypoints(vigir_planning_msgs::RequestDrakeCartesianTrajectory &request_message);

private:
    const int NUM_TIME_STEPS = 3;
};

}

#endif // CARTESIAN_TRAJECTORY_PLANNER_MODULE_H
