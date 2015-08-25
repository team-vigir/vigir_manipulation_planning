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
#ifndef PLANNER_HELPER_H
#define PLANNER_HELPER_H

#include <moveit_msgs/RobotState.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>

// ugly hack, but otherwise no way to instantiate Spline template class
// sets number of robot joints
#define NUM_POSITIONS   37

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
    Eigen::VectorXd &getNominalQ();

    Eigen::VectorXd messageQs2DrakeQs(Eigen::VectorXd &q0, moveit_msgs::RobotState &robot_state, bool &received_world_transform);
    Eigen::MatrixXd drakeQs2MessageQs(Eigen::MatrixXd &model_qs, moveit_msgs::RobotState &request_state);
    void printSortedQs(Eigen::MatrixXd &qs);
    
private:
    RigidBodyManipulator *robot_model_;
    Eigen::VectorXd q_nom;
};

}

#endif // PLANNER_HELPER_H
