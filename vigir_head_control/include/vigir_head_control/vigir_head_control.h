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
//@TODO_ADD_AUTHOR_INFO
#ifndef VIGIR_HEAD_CONTROL_NODE_H
#define VIGIR_HEAD_CONTROL_NODE_H

#include <ros/ros.h>
#include <vigir_planning_msgs/HeadControlCommand.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2_msgs/TFMessage.h>

namespace head_control{

    class HeadControl {

    public:
      HeadControl();
      void updateHeadPosition();
    protected:
      void HeadControlCb(const vigir_planning_msgs::HeadControlCommand &command);
      void setHeadJointPosition(const double pan, const double tilt);

      std::vector<double> computeJointsForTracking(const std::string &target_frame_id);
    private:
      unsigned char tracking_mode;
      tf::Vector3 old_target_frame_origin;
      double track_frame_threshold;

      ros::Publisher joint_trajectory_pub;
      ros::Subscriber head_control_sub;
      std::string tracking_frame;
      tf::TransformListener tf;

      std::vector<double> head_cmd; // pan, tilt
      std::vector<std::string> all_frames;
    };
}

//namespace head_tracking_mode{
//  const unsigned char NONE = 0;
//  const unsigned char LEFT_HAND_TRACKING = 1;
//  const unsigned char RIGHT_HAND_TRACKING = 2;
//  const unsigned char FRAME_TRACKING = 3;
//  const unsigned char USE_PROVIDED_JOINTS = 4;
//  const unsigned char LOOK_STRAIGHT = 5;

//}
#endif // VIGIR_HEAD_CONTROL_NODE_H
