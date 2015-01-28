//=================================================================================================
// Copyright (c) 2013, Stefan Kohlbrecher, TU Darmstadt
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

#ifndef VIGIR_MOVEIT_TRAJECTORY_UTILS_H__
#define VIGIR_MOVEIT_TRAJECTORY_UTILS_H__



#include <moveit/robot_trajectory/robot_trajectory.h>
#include <limits.h>


namespace trajectory_utils{

  /*
  // Doesn´t work for some reason
  static void removeZeroDurationJointTrajectoryPoints(robot_trajectory::RobotTrajectory& rt)
  {
    size_t size = size;

    robot_trajectory::RobotTrajectory rt_filtered = rt;
    rt_filtered.clear();

    for (int i = 0; i < size; ++i){

      double duration = rt.getWayPointDurationFromPrevious(i);

      if ((i != 0) && (duration != 0.0) ){
        rt_filtered.addSuffixWayPoint(rt.getWayPoint(i), duration);
      }
    }

    rt = rt_filtered;
  }
  */

  static void removeDuplicateStates(const std::vector<robot_state::RobotStatePtr>& in, std::vector<robot_state::RobotStatePtr>& out )
  {
    size_t size = in.size();


    if (size < 1){
      out = in;
      return;
    }else{
      out.push_back(in[0]);
    }

    for (int i = 1; i < size; ++i){
      if (in[i-1]->distance(*in[i]) > 0.02){
        out.push_back(in[i]);
      }
    }
  }

  static bool mergeTrajectories(robot_trajectory::RobotTrajectory& trajectory,
                                const robot_trajectory::RobotTrajectory& to_be_merged,
                                const ros::Time& trajectory_start_exec_time,
                                const ros::Time& target_time)
  {
    //Time into old trajectory
    ros::Duration time_into_trajectory = target_time - trajectory_start_exec_time;

    int before, after;
    double blend;
    trajectory.findWayPointIndicesForDurationAfterStart(time_into_trajectory.toSec(), before, after, blend);


    double dist_min = std::numeric_limits<double>::max();
    int min_index = -1;

    for (int i = 0; i < to_be_merged.getWayPointCount(); ++i){
      //traj_tmp.addSuffixWayPoint(mp_res->trajectory_->getWayPoint(i), durations[i]);
      double dist = trajectory.getWayPoint(after).distance(to_be_merged.getWayPoint(i));

      if (dist < dist_min){
        dist_min = dist;
        min_index = i;
      }
    }

    robot_trajectory::RobotTrajectory traj_tmp(trajectory.getRobotModel(), trajectory.getGroupName());

    const std::deque<double>& trajectory_durations = trajectory.getWayPointDurations();

    //Add original trajectory stitch waypoint
    traj_tmp.addSuffixWayPoint(trajectory.getWayPoint(after), (trajectory_durations[after] * (1.0 - blend)) -0.001);

    //Add to be merged waypoints
    const std::deque<double>& to_be_merged_durations = trajectory.getWayPointDurations();

    for (int i = min_index; i < to_be_merged.getWayPointCount(); ++i){
     traj_tmp.addSuffixWayPoint(to_be_merged.getWayPoint(i), to_be_merged_durations[i]);
    }

    traj_tmp.setWayPointDurationFromPrevious(1, 0.1);


    trajectory = traj_tmp;
    return true;

  }

}

#endif
