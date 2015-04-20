function [ trajectory, success ] = calcIKTrajectory( visualizer, robot_model, q0, request )
    % get trajectory duration infos and selected time steps
    duration = 5; %request.duration;
    
    % check trajectory every 0.5s       
    if ( duration <= 0 ) % set default duration, if it is not set correctly
        duration = 5;
    end
    t = 0:0.5:duration;
    %t = 0:1.25:duration;
    %if ( t(end) < duration )
    %    t(end+1) = duration;
    %end

    % stay at q0 for nominal trajectory
    q_lin = interp1([0, duration], [q0, q0]', t)';
    q_nom_traj = PPTrajectory(foh(t,q_lin));
    q_seed_traj = q_nom_traj;

    % build list of constraints from message
    activeConstraints = buildIKTrajectoryConstraints(robot_model, request.motion_plan_request.goal_constraints.joint_constraints, q0, t);

    % run inverse kinematics (mex)
    ikoptions = initIKTrajectoryOptions(robot_model);
    [trajectory,info_mex,infeasible_constraints] = inverseKinTraj(robot_model, t, q_seed_traj, q_nom_traj, activeConstraints{:},ikoptions);

    % visualize result
    visualizer.playback(trajectory,struct('slider',true));

    % TODO: handle failed trajectories... allow all through for now
    if(info_mex>10) % something went wrong
       ros.log('WARN', 'SNOPT calculation failed');
                   
       ros.log('INFO', 'Infeasible constraints:');                     
       str = sprintf('%s |  ', infeasible_constraints{:});
       ros.log('INFO', str);                    
       
        success = false;
    else
        success = true;        
    end
    
    % visualize result
    visualizer.playback(trajectory,struct('slider',true));

end

