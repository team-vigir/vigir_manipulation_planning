//=================================================================================================
// Copyright (c) 2014, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef VIGIR_CONTINUOUS_PLAN_EXECUTION_
#define VIGIR_CONTINUOUS_PLAN_EXECUTION_

#include <moveit/move_group/move_group_context.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
//#include <moveit_msgs/GetMotionPlan.h>
//#include <moveit_msgs/GetCartesianPath.h>
//#include <moveit_msgs/MoveItErrorCodes.h>

//#include <moveit/trajectory_processing/iterative_time_parameterization.h>

//#include <std_msgs/Empty.h>

namespace plan_execution
{

  class ContinuousPlanExecution
  {
  public:

    ContinuousPlanExecution(const move_group::MoveGroupContextPtr context);

    virtual void initialize();
    
    void startExecution();
    void stopExecution();

  private:    
    void continuousReplanningThread();

    boost::scoped_ptr<boost::thread> continuous_replanning_thread_;
    boost::mutex continuous_replanning_mutex_;
    bool stop_continuous_replanning_;

    move_group::MoveGroupContextPtr context_;

  };

}

#endif
