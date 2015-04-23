#include "vigir_drake_cpp/planner_modules/trajectory_planner_module.h"

#include "RigidBodyManipulator.h"
#include "RigidBodyConstraint.h"
#include "RigidBodyIK.h"
#include "IKoptions.h"
#include "splineGeneration.h"

#include <unsupported/Eigen/Splines>

#include <algorithm>

#include <tf/tf.h>
#include <ros/ros.h>


namespace vigir_drake_cpp {
  
TrajectoryPlannerModule::TrajectoryPlannerModule(RigidBodyManipulator *robot_model ) : PlannerModule(robot_model)
{
}

TrajectoryPlannerModule::~TrajectoryPlannerModule()
{
}

bool TrajectoryPlannerModule::plan(vigir_planning_msgs::RequestDrakeTrajectory &request_message, vigir_planning_msgs::ResultDrakeTrajectory &result_message)
{
    // get trajectory duration infos and selected time steps
    double duration = request_message.duration;

    // check trajectory points between start and end point
    std::vector<double> t_vec;
    for ( double t = 0.0; t < duration; t+=duration/NUM_TIME_STEPS) {
        t_vec.push_back(t);
    }
    /*if ( t_vec[ t_vec.size()-1 ] < duration ) {
        t_vec.push_back(duration);
    }*/

    // stay at q0 for nominal trajectory
    bool received_world_transform = false;
    VectorXd q0 = VectorXd::Zero(this->getRobotModel()->num_positions);
    q0 = messageQs2DrakeQs(q0, request_message.current_state, received_world_transform);

    VectorXd qdot_0 = VectorXd::Zero(this->getRobotModel()->num_velocities);
    MatrixXd q_seed = q0.replicate(1, t_vec.size());
    MatrixXd q_nom = q_seed;

    // build list of constraints from message    
    std::vector<RigidBodyConstraint*> constraints = buildIKConstraints(request_message, q0);
    IKoptions *ik_options = buildIKOptions(duration);

    // run inverse kinematics
    int nq = this->getRobotModel()->num_positions;
    MatrixXd q_sol(nq,t_vec.size());
    MatrixXd qdot_sol(nq,t_vec.size());
    MatrixXd qddot_sol(nq,t_vec.size());


    int info;
    std::vector<std::string> infeasible_constraints;
    inverseKinTraj(this->getRobotModel(),t_vec.size(),t_vec.data(),qdot_0,q_seed,q_nom,constraints.size(),constraints.data(),q_sol,qdot_sol,qddot_sol,info,infeasible_constraints,*ik_options);

    bool success = true;
    if(info>10) { // something went wrong
        std::string constraint_string = "";
        for (auto const& constraint : infeasible_constraints) { constraint_string += (constraint+" | "); }

        ROS_WARN("SNOPT info is %d, IK mex fails to solve the problem", info);
        ROS_INFO("Infeasible constraints: %s", constraint_string.c_str());

        success = false;
    }

    if ( success ) {
        // generate spline from result matrices
        double time_step = 1.0 / request_message.trajectory_sample_rate;
        int num_steps = (duration / time_step) + 0.5;
        Eigen::VectorXd response_t(num_steps+1);

        for ( int i = 0; i <= num_steps; i++) {
            response_t(i) = i*time_step;
        }

        Eigen::VectorXd input_t = Map<VectorXd>(t_vec.data(), t_vec.size());
        interpolateTrajectory(q_sol, input_t, response_t);

        std::vector<std::string> joint_names;
        for ( int i = 0; i < request_message.motion_plan_request.goal_constraints[0].joint_constraints.size(); i++ ) {
            joint_names.push_back( request_message.motion_plan_request.goal_constraints[0].joint_constraints[i].joint_name);
        }
        result_message = buildTrajectoryResultMsg(q_sol, qdot_sol, qddot_sol, t_vec, joint_names, received_world_transform);
    }
    else {
        // return an invalid message
        result_message.is_valid = false;
    }

    delete ik_options;
    return success;
}

std::vector<RigidBodyConstraint*> TrajectoryPlannerModule::buildIKConstraints(vigir_planning_msgs::RequestDrakeTrajectory &request_message, VectorXd &q0) {
    std::vector<RigidBodyConstraint*> constraints;

    // fixed foot placement
    int l_foot_id = this->getRobotModel()->findLinkId("l_foot");
    int r_foot_id = this->getRobotModel()->findLinkId("r_foot");

    //Vector4d foot_pts;
    //foot_pts << 0.0,0.0,0.0,1.0;
    Vector4d hom_foot_pts;
    hom_foot_pts << 0.0,0.0,0.0,1.0;
    Vector3d foot_pts;
    foot_pts << 0.0,0.0,0.0;

    //this->getRobotModel()->use_new_kinsol = false;
    VectorXd v = VectorXd::Zero(this->getRobotModel()->num_velocities);
    this->getRobotModel()->doKinematicsNew(q0, v);

    //this->getRobotModel()->forwardKin(l_foot_id, foot_pts, 2, l_foot_pos);
    //this->getRobotModel()->forwardKin(r_foot_id, foot_pts, 2, r_foot_pos);

    Vector7d l_foot_pos = this->getRobotModel()->forwardKinNew(foot_pts, l_foot_id, 0, 2, 0).value();
    Vector7d r_foot_pos = this->getRobotModel()->forwardKinNew(foot_pts, r_foot_id, 0, 2, 0).value();

    constraints.push_back( new WorldPositionConstraint(this->getRobotModel(), l_foot_id, hom_foot_pts, l_foot_pos.block<3,1>(0,0), l_foot_pos.block<3,1>(0,0)) );
    constraints.push_back( new WorldPositionConstraint(this->getRobotModel(), r_foot_id, hom_foot_pts, r_foot_pos.block<3,1>(0,0), r_foot_pos.block<3,1>(0,0)) );
    constraints.push_back( new WorldQuatConstraint(this->getRobotModel(), l_foot_id, l_foot_pos.block<4,1>(3,0), 0.0));
    constraints.push_back( new WorldQuatConstraint(this->getRobotModel(), r_foot_id, l_foot_pos.block<4,1>(3,0), 0.0));

    // add quasi static constraint
    MatrixXd l_foot_contact_pts = getRobotModel()->bodies[l_foot_id]->contact_pts;
    MatrixXd r_foot_contact_pts = getRobotModel()->bodies[r_foot_id]->contact_pts;

    QuasiStaticConstraint *quasi_static_constraint = new QuasiStaticConstraint(this->getRobotModel());
    quasi_static_constraint->addContact(1, &l_foot_id, &l_foot_contact_pts);
    quasi_static_constraint->addContact(1, &r_foot_id, &r_foot_contact_pts);
    quasi_static_constraint->setActive(true);
    quasi_static_constraint->setShrinkFactor(0.9);
    constraints.push_back( quasi_static_constraint );

    // handle end effector positions and orientations
    Vector2d tspan;
    tspan << request_message.duration, request_message.duration;
    PostureConstraint *posture_constraint = new PostureConstraint(this->getRobotModel(), tspan);

    std::vector<moveit_msgs::JointConstraint> current_constraint = request_message.motion_plan_request.goal_constraints[0].joint_constraints;
    for ( int i = 0; i < current_constraint.size(); i++ ) {
        int current_body_idx = getRobotModel()->findJointId(current_constraint[i].joint_name, -1);
        int current_joint_idx = getRobotModel()->bodies[current_body_idx]->position_num_start;

        Eigen::VectorXd current_joint_min(1), current_joint_max(1);
        current_joint_min << current_constraint[i].position - current_constraint[i].tolerance_below;
        current_joint_max << current_constraint[i].position + current_constraint[i].tolerance_above;

        posture_constraint->setJointLimits(1, &current_joint_idx, current_joint_min, current_joint_max);
    }

    constraints.push_back(posture_constraint);
    return constraints;
}


IKoptions *TrajectoryPlannerModule::buildIKOptions(double duration) {
    RigidBodyManipulator *current_robot = this->getRobotModel();
    IKoptions *ik_options = new IKoptions( current_robot );

    Eigen::MatrixXd Q = MatrixXd::Identity(current_robot->num_positions, current_robot->num_positions);
    ik_options->setQ(Q);
    ik_options->setQv(duration * Q);
    ik_options->setQa(duration * duration * Q);
    ik_options->setMajorIterationsLimit(10000);
    ik_options->setIterationsLimit(500000);
    ik_options->setSuperbasicsLimit(1000);
    ik_options->setMajorOptimalityTolerance(2e-4);
    ik_options->setDebug(true);

    return ik_options;
}

vigir_planning_msgs::ResultDrakeTrajectory TrajectoryPlannerModule::buildTrajectoryResultMsg(Eigen::MatrixXd &q_sol, Eigen::MatrixXd &qd_sol, Eigen::MatrixXd &qdd_sol, std::vector<double> t_vec, std::vector<std::string> &joint_names, bool send_world_transform) {
    vigir_planning_msgs::ResultDrakeTrajectory result_trajectory_msg;
    moveit_msgs::RobotTrajectory result_trajectory;


    result_trajectory.joint_trajectory.points.resize(t_vec.size());

    int num_joints = joint_names.size();

    MatrixXd q_sol_selected(num_joints, t_vec.size());
    MatrixXd qd_sol_selected(num_joints, t_vec.size());
    MatrixXd qdd_sol_selected(num_joints, t_vec.size());
    for ( int i = 0; i < num_joints; i++ ) {
        std::string joint_name = joint_names[i];
        result_trajectory.joint_trajectory.joint_names.push_back(joint_name);

        int body_idx = getRobotModel()->findJointId(joint_name, -1);
        int joint_idx = getRobotModel()->bodies[body_idx]->position_num_start;

        q_sol_selected.row(i) = q_sol.row(joint_idx);
        qd_sol_selected.row(i) = qd_sol.row(joint_idx);
        qdd_sol_selected.row(i) = qdd_sol.row(joint_idx);
    }

    for ( int i = 0; i < t_vec.size(); i++ ) {
        VectorXd q_values = q_sol_selected.col(i);
        VectorXd qd_values = qd_sol_selected.col(i);
        VectorXd qdd_values = qdd_sol_selected.col(i);
        std::vector<double> result_q_vec(q_values.data(), q_values.data() + q_values.rows() * q_values.cols());
        std::vector<double> result_qd_vec(qd_values.data(), qd_values.data() + qd_values.rows() * qd_values.cols());
        std::vector<double> result_qdd_vec(qdd_values.data(), qdd_values.data() + qdd_values.rows() * qdd_values.cols());
        result_trajectory.joint_trajectory.points[i].positions = result_q_vec;
        result_trajectory.joint_trajectory.points[i].velocities = result_qd_vec;
        result_trajectory.joint_trajectory.points[i].accelerations = result_qdd_vec;
    }

    if ( send_world_transform ) {
        result_trajectory.multi_dof_joint_trajectory.joint_names.push_back("world_virtual_joint");

        for ( int i = 0; i < t_vec.size(); i++ ) {
            trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point;
            trajectory_point.time_from_start = ros::Duration(t_vec[i]);

            // set translation / rotation
            trajectory_point.transforms.resize(1);
            trajectory_point.transforms[0].translation.x = q_sol(0,i);
            trajectory_point.transforms[0].translation.y = q_sol(1,i);
            trajectory_point.transforms[0].translation.z = q_sol(2,i);

            tf::Quaternion rotation_quat;
            rotation_quat.setRPY(q_sol(3, i), q_sol(4, i), q_sol(5, i));
            trajectory_point.transforms[0].rotation.x = rotation_quat.x();
            trajectory_point.transforms[0].rotation.y = rotation_quat.y();
            trajectory_point.transforms[0].rotation.z = rotation_quat.z();
            trajectory_point.transforms[0].rotation.w = rotation_quat.w();

            // set velocities
            trajectory_point.velocities.resize(1);
            trajectory_point.velocities[0].linear.x = qd_sol(0, i);
            trajectory_point.velocities[0].linear.y = qd_sol(1, i);
            trajectory_point.velocities[0].linear.z = qd_sol(2, i);
            trajectory_point.velocities[0].angular.x = qd_sol(3, i);
            trajectory_point.velocities[0].angular.y = qd_sol(4, i);
            trajectory_point.velocities[0].angular.z = qd_sol(5, i);

            // set accelerations
            trajectory_point.accelerations.resize(1);
            trajectory_point.accelerations[0].linear.x = qdd_sol(0, i);
            trajectory_point.accelerations[0].linear.y = qdd_sol(1, i);
            trajectory_point.accelerations[0].linear.z = qdd_sol(2, i);
            trajectory_point.accelerations[0].angular.x = qdd_sol(3, i);
            trajectory_point.accelerations[0].angular.y = qdd_sol(4, i);
            trajectory_point.accelerations[0].angular.z = qdd_sol(5, i);

            result_trajectory.multi_dof_joint_trajectory.points.push_back(trajectory_point);
        }
    }

    result_trajectory_msg.result_trajectory = result_trajectory;
    result_trajectory_msg.is_valid = true;

   return result_trajectory_msg;
}

void TrajectoryPlannerModule::interpolateTrajectory(Eigen::MatrixXd &input_q, Eigen::VectorXd &input_t, Eigen::VectorXd &knot_t)
{
    assert(input_q.cols() == input_t.rows());
    Eigen::MatrixXd data_points(input_q.rows()+1, input_q.cols());
    data_points.row(0) = input_t.transpose();
    data_points.block(1,0, input_q.rows(), input_q.cols()) = input_q;

    const TrajectorySpline q_spline = SplineFitting<TrajectorySpline>::Interpolate(data_points, input_t.rows()-1);


    for ( int i = 0; i < knot_t.rows(); i++ ) {
      double spline_pos = (knot_t(i) - input_t.minCoeff())/(input_t.maxCoeff() - input_t.minCoeff());
      const MatrixXd values = q_spline.derivatives(spline_pos, 2); // compute position and derivatives at time points

      std::cout << "(" << knot_t(i) << ", " << values.transpose() << ")" << std::endl << std::endl;
    }
}

}
