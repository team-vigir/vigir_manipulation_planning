function [ trajectory, info_mex, infeasible_constraints ] = calcIKCartesianFeetTrajectory( robot_model, start_time, q0, request, do_self_collision_checks, world_mat, q_nominal )
%CALCIKCARTESIANTRAJECTORY Summary of this function goes here
%   Detailed explanation goes here
            
                % stay at q_nominal (at position and orientation of q0 for nominal trajectory
                q_nom_pos = q0;
                %q_nom_pos = q_nominal;
                q_nom_pos(1:6) = q0(1:6);
                
                q_lin = interp1([start_time, request.waypoint_times(end)], [q_nom_pos, q_nom_pos]', [start_time, request.waypoint_times(end)])';
                q_nom_traj = PPTrajectory(foh([start_time, request.waypoint_times(end)],q_lin));
                q_seed_traj = q_nom_traj;

                % build list of constraints from message
                activeConstraints = buildIKCartesianFeetTrajectoryConstraints(robot_model, request.target_link_name, request.waypoints, request.waypoint_times, request.free_joint_names, q0, do_self_collision_checks, world_mat, start_time);

                % build IK options (add additional constraint checks)
                ikoptions = initIKCartesianTrajectoryOptions(robot_model);
                ikoptions = ikoptions.setAdditionaltSamples( start_time:0.5:request.waypoint_times(end) );

                % run inverse kinematics (mex)
                [trajectory,info_mex,infeasible_constraints] = inverseKinTraj(robot_model, [start_time, request.waypoint_times], q_seed_traj, q_nom_traj, activeConstraints{:},ikoptions);
end

