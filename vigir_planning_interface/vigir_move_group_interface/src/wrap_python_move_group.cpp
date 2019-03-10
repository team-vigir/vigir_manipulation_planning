/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#include <moveit/vigir_move_group_interface/move_group.h>
#include <moveit/py_bindings_tools/roscpp_initializer.h>
#include <moveit/py_bindings_tools/py_conversions.h>
#include <moveit/py_bindings_tools/serialize_msg.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

#include <boost/python.hpp>
#include <boost/shared_ptr.hpp>
#include <Python.h>

/** @cond IGNORE */

namespace bp = boost::python;

namespace moveit
{
namespace planning_interface
{

class VigirMoveGroupWrapper : protected py_bindings_tools::ROScppInitializer,
                         public VigirMoveGroup
{
public:

  // ROSInitializer is constructed first, and ensures ros::init() was called, if needed
  VigirMoveGroupWrapper(const std::string &group_name, const std::string &robot_description) :
    py_bindings_tools::ROScppInitializer(),
    VigirMoveGroup(Options(group_name, robot_description), boost::shared_ptr<tf::Transformer>(), ros::Duration(5, 0))
  {
  }

  bool setJointValueTargetPerJointPythonList(const std::string &joint, bp::list &values)
  {
    return setJointValueTarget(joint, py_bindings_tools::doubleFromList(values));
  }

  bool setJointValueTargetPythonList(bp::list &values)
  {
    return setJointValueTarget(py_bindings_tools::doubleFromList(values));
  }

  bool setJointValueTargetPythonDict(bp::dict &values)
  {
    bp::list k = values.keys();
    int l = bp::len(k);
    std::map<std::string, double> v;
    for (int i = 0; i < l ; ++i)
      v[bp::extract<std::string>(k[i])] = bp::extract<double>(values[k[i]]);
    return setJointValueTarget(v);
  }

  bool setJointValueTargetFromPosePython(const std::string &pose_str, const std::string &eef, bool approx)
  {
    geometry_msgs::Pose pose_msg;
    py_bindings_tools::deserializeMsg(pose_str, pose_msg);
    return approx ? setApproximateJointValueTarget(pose_msg, eef) : setJointValueTarget(pose_msg, eef);
  }

  bool setJointValueTargetFromPoseStampedPython(const std::string &pose_str, const std::string &eef, bool approx)
  {
    geometry_msgs::PoseStamped pose_msg;
    py_bindings_tools::deserializeMsg(pose_str, pose_msg);
    return approx ? setApproximateJointValueTarget(pose_msg, eef) : setJointValueTarget(pose_msg, eef);
  }

  bool setJointValueTargetFromJointStatePython(const std::string &js_str)
  {
    sensor_msgs::JointState js_msg;
    py_bindings_tools::deserializeMsg(js_str, js_msg);
    return setJointValueTarget(js_msg);
  }

  void rememberJointValuesFromPythonList(const std::string &string, bp::list &values)
  {
    rememberJointValues(string, py_bindings_tools::doubleFromList(values));
  }

  const char* getPlanningFrameCStr() const
  {
    return getPlanningFrame().c_str();
  }

  bp::list getActiveJointsList() const
  {
    return py_bindings_tools::listFromString(getActiveJoints());
  }

  bp::list getJointsList() const
  {
    return py_bindings_tools::listFromString(getJoints());
  }

  bp::list getCurrentJointValuesList()
  {
    return py_bindings_tools::listFromDouble(getCurrentJointValues());
  }

  bp::list getRandomJointValuesList()
  {
    return py_bindings_tools::listFromDouble(getRandomJointValues());
  }

  bp::dict getRememberedJointValuesPython() const
  {
    const std::map<std::string, std::vector<double> > &rv = getRememberedJointValues();
    bp::dict d;
    for (std::map<std::string, std::vector<double> >::const_iterator it = rv.begin() ; it != rv.end() ; ++it)
      d[it->first] = py_bindings_tools::listFromDouble(it->second);
    return d;
  }

  bp::list convertPoseToList(const geometry_msgs::Pose &pose) const
  {
    std::vector<double> v(7);
    v[0] = pose.position.x;
    v[1] = pose.position.y;
    v[2] = pose.position.z;
    v[3] = pose.orientation.x;
    v[4] = pose.orientation.y;
    v[5] = pose.orientation.z;
    v[6] = pose.orientation.w;
    return moveit::py_bindings_tools::listFromDouble(v);
  }

  bp::list convertTransformToList(const geometry_msgs::Transform &tr) const
  {
    std::vector<double> v(7);
    v[0] = tr.translation.x;
    v[1] = tr.translation.y;
    v[2] = tr.translation.z;
    v[3] = tr.rotation.x;
    v[4] = tr.rotation.y;
    v[5] = tr.rotation.z;
    v[6] = tr.rotation.w;
    return py_bindings_tools::listFromDouble(v);
  }

  void convertListToTransform(const bp::list &l, geometry_msgs::Transform &tr) const
  {
    std::vector<double> v = py_bindings_tools::doubleFromList(l);
    tr.translation.x =  v[0];
    tr.translation.y = v[1];
    tr.translation.z = v[2];
    tr.rotation.x = v[3];
    tr.rotation.y = v[4];
    tr.rotation.z = v[5];
    tr.rotation.w = v[6];
  }

  void convertListToPose(const bp::list &l, geometry_msgs::Pose &p) const
  {
    std::vector<double> v = py_bindings_tools::doubleFromList(l);
    p.position.x =  v[0];
    p.position.y = v[1];
    p.position.z = v[2];
    p.orientation.x = v[3];
    p.orientation.y = v[4];
    p.orientation.z = v[5];
    p.orientation.w = v[6];
  }

  bp::list getCurrentRPYPython(const std::string &end_effector_link = "")
  {
    return py_bindings_tools::listFromDouble(getCurrentRPY(end_effector_link));
  }

  bp::list getCurrentPosePython(const std::string &end_effector_link = "")
  {
    geometry_msgs::PoseStamped pose = getCurrentPose(end_effector_link);
    return convertPoseToList(pose.pose);
  }

  bp::list getRandomPosePython(const std::string &end_effector_link = "")
  {
    geometry_msgs::PoseStamped pose = getRandomPose(end_effector_link);
    return convertPoseToList(pose.pose);
  }

  bp::list getKnownConstraintsList() const
  {
    return py_bindings_tools::listFromString(getKnownConstraints());
  }

  bool placePose(const std::string &object_name, const bp::list &pose)
  {
    geometry_msgs::PoseStamped msg;
    convertListToPose(pose, msg.pose);
    msg.header.frame_id = getPoseReferenceFrame();
    msg.header.stamp = ros::Time::now();
    return place(object_name, msg);
  }

  bool placeLocation(const std::string &object_name, const std::string &location_str)
  {
    std::vector<moveit_msgs::PlaceLocation> locations(1);
    py_bindings_tools::deserializeMsg(location_str, locations[0]);
    return place(object_name, locations);
  }

  bool placeAnywhere(const std::string &object_name)
  {
    return place(object_name);
  }

  void convertListToArrayOfPoses(const bp::list &poses, std::vector<geometry_msgs::Pose> &msg)
  {
    int l = bp::len(poses);
    for (int i = 0; i < l ; ++i)
    {
      const bp::list &pose = bp::extract<bp::list>(poses[i]);
      std::vector<double> v = py_bindings_tools::doubleFromList(pose);
      if (v.size() == 6 || v.size() == 7)
      {
        Eigen::Isometry3d p;
        if (v.size() == 6)
        {
          Eigen::Quaterniond q;
          tf::quaternionTFToEigen(tf::createQuaternionFromRPY(v[3], v[4], v[5]), q);
          p = Eigen::Isometry3d(q);
        }
        else
          p = Eigen::Isometry3d(Eigen::Quaterniond(v[6], v[3], v[4], v[5]));
        p.translation() = Eigen::Vector3d(v[0], v[1], v[2]);
        geometry_msgs::Pose pm;
        tf::poseEigenToMsg(p, pm);
        msg.push_back(pm);
      }
      else
        ROS_WARN("Incorrect number of values for a pose: %u", (unsigned int)v.size());
    }
  }

  void setStartStatePython(const std::string &msg_str)
  {
    moveit_msgs::RobotState msg;
    py_bindings_tools::deserializeMsg(msg_str, msg);
    setStartState(msg);
  }

  bool setPoseTargetsPython(bp::list &poses, const std::string &end_effector_link = "")
  {
    std::vector<geometry_msgs::Pose> msg;
    convertListToArrayOfPoses(poses, msg);
    return setPoseTargets(msg, end_effector_link);
  }

  bool setPoseTargetPython(bp::list &pose, const std::string &end_effector_link = "")
  {
    std::vector<double> v = py_bindings_tools::doubleFromList(pose);
    geometry_msgs::Pose msg;
    if (v.size() == 6)
      tf::quaternionTFToMsg(tf::createQuaternionFromRPY(v[3], v[4], v[5]), msg.orientation);
    else
      if (v.size() == 7)
      {
        msg.orientation.x = v[3];
        msg.orientation.y = v[4];
        msg.orientation.z = v[5];
        msg.orientation.w = v[6];
      }
      else
      {
        ROS_ERROR("Pose description expected to consist of either 6 or 7 values");
        return false;
      }
    msg.position.x = v[0];
    msg.position.y = v[1];
    msg.position.z = v[2];
    return setPoseTarget(msg, end_effector_link);
  }

  const char* getEndEffectorLinkCStr() const
  {
    return getEndEffectorLink().c_str();
  }

  const char* getPoseReferenceFrameCStr() const
  {
    return getPoseReferenceFrame().c_str();
  }

  const char* getNameCStr() const
  {
    return getName().c_str();
  }

  bool movePython()
  {
    return move();
  }

  bool asyncMovePython()
  {
    return asyncMove();
  }

  bool attachObjectPython(const std::string &object_name, const std::string &link_name, const bp::list &touch_links)
  {
    return attachObject(object_name, link_name, py_bindings_tools::stringFromList(touch_links));
  }

  bool executePython(const std::string &plan_str)
  {
    VigirMoveGroup::Plan plan;
    py_bindings_tools::deserializeMsg(plan_str, plan.trajectory_);
    return execute(plan);
  }

  std::string getPlanPython()
  {
    VigirMoveGroup::Plan plan;
    VigirMoveGroup::plan(plan);
    return py_bindings_tools::serializeMsg(plan.trajectory_);
  }

  bp::tuple computeCartesianPathPython(const bp::list &waypoints, double eef_step, double jump_threshold, bool avoid_collisions)
  {
    std::vector<geometry_msgs::Pose> poses;
    convertListToArrayOfPoses(waypoints, poses);
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = computeCartesianPath(poses, eef_step, jump_threshold, trajectory, avoid_collisions);
    return bp::make_tuple(py_bindings_tools::serializeMsg(trajectory), fraction);
  }

  int pickGrasp(const std::string &object, const std::string &grasp_str)
  {
    moveit_msgs::Grasp grasp;
    py_bindings_tools::deserializeMsg(grasp_str, grasp);
    return pick(object, grasp).val;
  }

  int pickGrasps(const std::string &object, const bp::list &grasp_list)
  {
    int l = bp::len(grasp_list);
    std::vector<moveit_msgs::Grasp> grasps(l);
    for (int i = 0; i < l ; ++i)
      py_bindings_tools::deserializeMsg(bp::extract<std::string>(grasp_list[i]), grasps[i]);
    return pick(object, grasps).val;
  }

  void setPathConstraintsFromMsg(const std::string &constraints_str)
  {
      moveit_msgs::Constraints constraints_msg;
      py_bindings_tools::deserializeMsg(constraints_str,constraints_msg);
      setPathConstraints(constraints_msg);
  }

  std::string getPathConstraintsPython()
  {
     moveit_msgs::Constraints constraints_msg(getPathConstraints());
     std::string constraints_str = py_bindings_tools::serializeMsg(constraints_msg);
     return constraints_str;
  }

};

static void wrap_move_group_interface()
{
  bp::class_<VigirMoveGroupWrapper> VigirMoveGroupClass("VigirMoveGroup", bp::init<std::string, std::string>());

  VigirMoveGroupClass.def("async_move", &VigirMoveGroupWrapper::asyncMovePython);
  VigirMoveGroupClass.def("move", &VigirMoveGroupWrapper::movePython);
  VigirMoveGroupClass.def("execute", &VigirMoveGroupWrapper::executePython);
  moveit::planning_interface::MoveItErrorCode (VigirMoveGroupWrapper::*pick_1)(const std::string&) = &VigirMoveGroupWrapper::pick;
  VigirMoveGroupClass.def("pick", pick_1);
  VigirMoveGroupClass.def("pick", &VigirMoveGroupWrapper::pickGrasp);
  VigirMoveGroupClass.def("pick", &VigirMoveGroupWrapper::pickGrasps);
  VigirMoveGroupClass.def("place", &VigirMoveGroupWrapper::placePose);
  VigirMoveGroupClass.def("place", &VigirMoveGroupWrapper::placeLocation);
  VigirMoveGroupClass.def("place", &VigirMoveGroupWrapper::placeAnywhere);
  VigirMoveGroupClass.def("stop", &VigirMoveGroupWrapper::stop);

  VigirMoveGroupClass.def("get_name", &VigirMoveGroupWrapper::getNameCStr);
  VigirMoveGroupClass.def("get_planning_frame", &VigirMoveGroupWrapper::getPlanningFrameCStr);

  VigirMoveGroupClass.def("get_active_joints", &VigirMoveGroupWrapper::getActiveJointsList);
  VigirMoveGroupClass.def("get_joints", &VigirMoveGroupWrapper::getJointsList);
  VigirMoveGroupClass.def("get_variable_count", &VigirMoveGroupWrapper::getVariableCount);
  VigirMoveGroupClass.def("allow_looking", &VigirMoveGroupWrapper::allowLooking);
  VigirMoveGroupClass.def("allow_replanning", &VigirMoveGroupWrapper::allowReplanning);

  VigirMoveGroupClass.def("set_pose_reference_frame", &VigirMoveGroupWrapper::setPoseReferenceFrame);

  VigirMoveGroupClass.def("set_pose_reference_frame", &VigirMoveGroupWrapper::setPoseReferenceFrame);
  VigirMoveGroupClass.def("set_end_effector_link", &VigirMoveGroupWrapper::setEndEffectorLink);
  VigirMoveGroupClass.def("get_end_effector_link", &VigirMoveGroupWrapper::getEndEffectorLinkCStr);
  VigirMoveGroupClass.def("get_pose_reference_frame", &VigirMoveGroupWrapper::getPoseReferenceFrameCStr);

  VigirMoveGroupClass.def("set_pose_target", &VigirMoveGroupWrapper::setPoseTargetPython);

  VigirMoveGroupClass.def("set_pose_targets", &VigirMoveGroupWrapper::setPoseTargetsPython);

  VigirMoveGroupClass.def("set_position_target", &VigirMoveGroupWrapper::setPositionTarget);
  VigirMoveGroupClass.def("set_rpy_target", &VigirMoveGroupWrapper::setRPYTarget);
  VigirMoveGroupClass.def("set_orientation_target", &VigirMoveGroupWrapper::setOrientationTarget);

  VigirMoveGroupClass.def("get_current_pose", &VigirMoveGroupWrapper::getCurrentPosePython);
  VigirMoveGroupClass.def("get_current_rpy", &VigirMoveGroupWrapper::getCurrentRPYPython);

  VigirMoveGroupClass.def("get_random_pose", &VigirMoveGroupWrapper::getRandomPosePython);

  VigirMoveGroupClass.def("clear_pose_target", &VigirMoveGroupWrapper::clearPoseTarget);
  VigirMoveGroupClass.def("clear_pose_targets", &VigirMoveGroupWrapper::clearPoseTargets);

  VigirMoveGroupClass.def("set_joint_value_target", &VigirMoveGroupWrapper::setJointValueTargetPythonList);
  VigirMoveGroupClass.def("set_joint_value_target", &VigirMoveGroupWrapper::setJointValueTargetPythonDict);

  VigirMoveGroupClass.def("set_joint_value_target", &VigirMoveGroupWrapper::setJointValueTargetPerJointPythonList);
  bool (VigirMoveGroupWrapper::*setJointValueTarget_4)(const std::string&, double) = &VigirMoveGroupWrapper::setJointValueTarget;
  VigirMoveGroupClass.def("set_joint_value_target", setJointValueTarget_4);

  VigirMoveGroupClass.def("set_joint_value_target_from_pose", &VigirMoveGroupWrapper::setJointValueTargetFromPosePython);
  VigirMoveGroupClass.def("set_joint_value_target_from_pose_stamped", &VigirMoveGroupWrapper::setJointValueTargetFromPoseStampedPython);
  VigirMoveGroupClass.def("set_joint_value_target_from_joint_state_message", &VigirMoveGroupWrapper::setJointValueTargetFromJointStatePython);

  VigirMoveGroupClass.def("set_named_target", &VigirMoveGroupWrapper::setNamedTarget);
  VigirMoveGroupClass.def("set_random_target", &VigirMoveGroupWrapper::setRandomTarget);

  void (VigirMoveGroupWrapper::*rememberJointValues_2)(const std::string&) = &VigirMoveGroupWrapper::rememberJointValues;
  VigirMoveGroupClass.def("remember_joint_values", rememberJointValues_2);

  VigirMoveGroupClass.def("remember_joint_values",  &VigirMoveGroupWrapper::rememberJointValuesFromPythonList);

  VigirMoveGroupClass.def("start_state_monitor",  &VigirMoveGroupWrapper::startStateMonitor);
  VigirMoveGroupClass.def("get_current_joint_values",  &VigirMoveGroupWrapper::getCurrentJointValuesList);
  VigirMoveGroupClass.def("get_random_joint_values",  &VigirMoveGroupWrapper::getRandomJointValuesList);
  VigirMoveGroupClass.def("get_remembered_joint_values",  &VigirMoveGroupWrapper::getRememberedJointValuesPython);

  VigirMoveGroupClass.def("forget_joint_values", &VigirMoveGroupWrapper::forgetJointValues);

  VigirMoveGroupClass.def("get_goal_joint_tolerance", &VigirMoveGroupWrapper::getGoalJointTolerance);
  VigirMoveGroupClass.def("get_goal_position_tolerance", &VigirMoveGroupWrapper::getGoalPositionTolerance);
  VigirMoveGroupClass.def("get_goal_orientation_tolerance", &VigirMoveGroupWrapper::getGoalOrientationTolerance);

  VigirMoveGroupClass.def("set_goal_joint_tolerance", &VigirMoveGroupWrapper::setGoalJointTolerance);
  VigirMoveGroupClass.def("set_goal_position_tolerance", &VigirMoveGroupWrapper::setGoalPositionTolerance);
  VigirMoveGroupClass.def("set_goal_orientation_tolerance", &VigirMoveGroupWrapper::setGoalOrientationTolerance);
  VigirMoveGroupClass.def("set_goal_tolerance", &VigirMoveGroupWrapper::setGoalTolerance);

  VigirMoveGroupClass.def("set_start_state_to_current_state", &VigirMoveGroupWrapper::setStartStateToCurrentState);
  VigirMoveGroupClass.def("set_start_state", &VigirMoveGroupWrapper::setStartStatePython);

  bool (VigirMoveGroupWrapper::*setPathConstraints_1)(const std::string&) = &VigirMoveGroupWrapper::setPathConstraints;
  VigirMoveGroupClass.def("set_path_constraints", setPathConstraints_1);
  VigirMoveGroupClass.def("set_path_constraints_from_msg", &VigirMoveGroupWrapper::setPathConstraintsFromMsg);
  VigirMoveGroupClass.def("get_path_constraints", &VigirMoveGroupWrapper::getPathConstraintsPython);
  VigirMoveGroupClass.def("clear_path_constraints", &VigirMoveGroupWrapper::clearPathConstraints);
  VigirMoveGroupClass.def("get_known_constraints", &VigirMoveGroupWrapper::getKnownConstraintsList);
  VigirMoveGroupClass.def("set_constraints_database", &VigirMoveGroupWrapper::setConstraintsDatabase);
  VigirMoveGroupClass.def("set_workspace", &VigirMoveGroupWrapper::setWorkspace);
  VigirMoveGroupClass.def("set_planning_time", &VigirMoveGroupWrapper::setPlanningTime);
  VigirMoveGroupClass.def("get_planning_time", &VigirMoveGroupWrapper::getPlanningTime);
  VigirMoveGroupClass.def("set_planner_id", &VigirMoveGroupWrapper::setPlannerId);
  VigirMoveGroupClass.def("compute_plan", &VigirMoveGroupWrapper::getPlanPython);
  VigirMoveGroupClass.def("compute_cartesian_path", &VigirMoveGroupWrapper::computeCartesianPathPython);
  VigirMoveGroupClass.def("set_support_surface_name", &VigirMoveGroupWrapper::setSupportSurfaceName);
  VigirMoveGroupClass.def("attach_object", &VigirMoveGroupWrapper::attachObjectPython);
  VigirMoveGroupClass.def("detach_object", &VigirMoveGroupWrapper::detachObject);
}

}
}

BOOST_PYTHON_MODULE(_moveit_move_group_interface)
{
  using namespace moveit::planning_interface;
  wrap_move_group_interface();
}

/** @endcond */
