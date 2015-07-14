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
    IKoptions *buildIKOptions();
};

}

#endif // POSITION_IK_PLANNER_MODULE_H
