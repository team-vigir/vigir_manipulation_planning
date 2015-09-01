#ifndef VIGIR_MOVE_GROUP_DRAKE_PLANNING_ADAPTER_
#define VIGIR_MOVE_GROUP_DRAKE_PLANNING_ADAPTER_

#include <vigir_planning_msgs/MoveGoal.h>
#include <vigir_planning_msgs/MoveResult.h>
#include <moveit/move_group/move_group_capability.h>

#include <ros/ros.h>
#include <tf/tf.h>

namespace move_group {

class MoveGroupManipulationAction;

class DrakePlanningHelper {
public:
    DrakePlanningHelper(MoveGroupManipulationAction *manipulation_action_capability);
    ~DrakePlanningHelper();

    void actionPlan(const vigir_planning_msgs::MoveGoalConstPtr& goal, vigir_planning_msgs::MoveResult &action_res);
    void actionPlanCartesianMotion(const vigir_planning_msgs::MoveGoalConstPtr& goal, vigir_planning_msgs::MoveResult &action_res);
    void actionPlanCircularMotion(const vigir_planning_msgs::MoveGoalConstPtr& goal, vigir_planning_msgs::MoveResult &action_res);

    bool plan(const vigir_planning_msgs::MoveGoalConstPtr& goal, plan_execution::ExecutableMotionPlan &plan);
    bool planCartesianMotion(const vigir_planning_msgs::MoveGoalConstPtr& goal, plan_execution::ExecutableMotionPlan &plan);
    bool planCircularMotion(const vigir_planning_msgs::MoveGoalConstPtr& goal, plan_execution::ExecutableMotionPlan &plan);

private:
    tf::TransformListener transform_listener_;

    ros::ServiceClient drake_trajectory_srv_client_;
    ros::ServiceClient drake_cartesian_trajectory_srv_client_;

    MoveGroupManipulationAction *manipulation_action_capability_;
    MoveGroupContextPtr context_;
};

}

#endif
