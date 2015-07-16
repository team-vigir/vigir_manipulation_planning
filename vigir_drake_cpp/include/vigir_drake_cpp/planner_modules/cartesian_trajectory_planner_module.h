/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, Team ViGIR ( TORC Robotics LLC, TU Darmstadt, Virginia Tech, Oregon State University, Cornell University, and Leibniz University Hanover )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
//@TODO_ADD_AUTHOR_INFO
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
    std::vector<double> estimateWaypointTimes(Eigen::VectorXd &q0, std::vector<std::string> target_link_names, std::vector<geometry_msgs::Pose> target_waypoints);

private:
    const int NUM_TIME_STEPS = 3;
};

}

#endif // CARTESIAN_TRAJECTORY_PLANNER_MODULE_H
