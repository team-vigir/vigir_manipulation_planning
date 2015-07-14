/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, TORC Robotics, LLC ( Team ViGIR )
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
@TODO_ADD_AUTHOR_INFO
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
