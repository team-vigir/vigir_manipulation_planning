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
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <ros/ros.h>


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

static void removeDuplicateStates(const std::vector<robot_state::RobotStatePtr>& in, std::vector<robot_state::RobotStatePtr>& out, double threshold = 0.02)
{
  size_t size = in.size();


  if (size < 1){
    out = in;
    return;
  }else{
    out.push_back(in[0]);
  }

  for (int i = 1; i < size; ++i){
    if (out.back()->distance(*in[i]) > threshold){
      out.push_back(in[i]);
    }
  }
}

static void cut_trajectory(robot_trajectory::RobotTrajectory& trajectory, const ros::Duration& cut_duration)
{
  int before, after;
  double blend;
  trajectory.findWayPointIndicesForDurationAfterStart(cut_duration.toSec(), before, after, blend);

  robot_trajectory::RobotTrajectory new_traj(trajectory.getRobotModel(), trajectory.getGroupName());

  const std::deque<double>& trajectory_durations = trajectory.getWayPointDurations();

  for (int i = after; i < trajectory.getWayPointCount(); ++i){
    new_traj.addSuffixWayPoint(trajectory.getWayPoint(i), trajectory_durations[i]);
  }

  //new_traj.setWayPointDurationFromPrevious(0, (trajectory_durations[after] * (1.0 - blend)) -0.001);
  new_traj.setWayPointDurationFromPrevious(0, 0.1);


  trajectory = new_traj;


}

class TrajectoryVisualization
{
public:
  TrajectoryVisualization(ros::NodeHandle pnh, const std::string topic_name = "eef_trajectory", double r = 1.0, double g = 0.0, double b = 0.0, double a = 1.0)
  {
    marker_array_pub_ = pnh.advertise<visualization_msgs::MarkerArray>(topic_name, 5);
    marker_.pose.orientation.w = 1.0;
    marker_.scale.x = 0.01;

    marker_.color.r = r;
    marker_.color.g = g;
    marker_.color.b = b;
    marker_.color.a = a;
  }

  void publishTrajectoryEndeffectorVis(const robot_trajectory::RobotTrajectory& trajectory, bool increase_marker_id = false)
  {
    if (marker_array_pub_.getNumSubscribers() > 0){
      const robot_model::RobotModelConstPtr& model = trajectory.getRobotModel();
      marker_.header.frame_id = model->getModelFrame();
      marker_.header.stamp = ros::Time::now();

      marker_.ns = "eef_trajectory";
      marker_.id = 0;
      marker_.type = visualization_msgs::Marker::LINE_STRIP;
      marker_.action = visualization_msgs::Marker::ADD;

      marker_.points.resize(trajectory.getWayPointCount());

      const robot_model::JointModelGroup* group = trajectory.getGroup();

      for (int i = 0; i < trajectory.getWayPointCount(); ++i){
        const robot_state::RobotState& state = trajectory.getWayPoint(i);

        //@TODO: Properly get tips for group
        const Eigen::Affine3d& transform = state.getGlobalLinkTransform("l_hand");

        marker_.points[i].x = transform.translation().x();
        marker_.points[i].y = transform.translation().y();
        marker_.points[i].z = transform.translation().z();
      }
      marker_array_.markers.push_back(marker_);
      marker_array_pub_.publish (marker_array_);
    }

  }





protected:
  ros::Publisher marker_array_pub_;
  visualization_msgs::Marker marker_;
  visualization_msgs::MarkerArray marker_array_;
};

class TrajectoryMerger
{

public:
  bool mergeTrajectories(const robot_trajectory::RobotTrajectory& trajectory,
                         const robot_trajectory::RobotTrajectory& to_be_merged,
                         const ros::Time& previous_start_time,
                         const ros::Time& next_start_time,
                         const ros::Time& merge_time,
                         const double velocity_factor,
                         robot_trajectory::RobotTrajectory& merged_traj)
  {
    //Time into old trajectory
    ros::Duration time_into_trajectory = merge_time - previous_start_time;

    ROS_INFO("Diff: %f seconds", time_into_trajectory.toSec());

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

    const std::deque<double>& trajectory_durations = trajectory.getWayPointDurations();

    //Add original trajectory stitch waypoints
    for (int i = 0; i < after; ++i){
      merged_traj.addSuffixWayPoint(trajectory.getWayPoint(i), 0.0);
    }
    ROS_INFO("Added %d original waypoints", after);


    //Add to be merged waypoints
    const std::deque<double>& to_be_merged_durations = trajectory.getWayPointDurations();

    for (int i = min_index; i < to_be_merged.getWayPointCount(); ++i){
      merged_traj.addSuffixWayPoint(to_be_merged.getWayPoint(i), 0.0);
    }
    ROS_INFO("Added %d changed waypoints", static_cast<int>(to_be_merged.getWayPointCount())- min_index);

    time_parametrization_.computeTimeStamps(merged_traj, velocity_factor);

    cut_trajectory(merged_traj, (next_start_time-previous_start_time));

    return true;
  }

private:
  trajectory_processing::IterativeParabolicTimeParameterization time_parametrization_;
};

}

#endif
