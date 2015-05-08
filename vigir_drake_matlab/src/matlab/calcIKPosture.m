function [ posture, success ] = calcIKPosture( visualizer, robot_model, q0, request )
    nq = robot_model.getNumPositions();

    q_seed = q0;
    q_nom(1:6) = q0(1:6);

    % show start pose    
    visualizer.draw(cputime, q0);

    % build list of constraints from message
    activeConstraints = buildIKConstraints(robot_model, request, q0);

    % run inverse kinematics (mex)
    ikoptions = initIKOptions(robot_model);
    [posture,info_mex,infeasible_constraints] = inverseKin(robot_model,q_seed,q_nom,activeConstraints{:},ikoptions);

    % visualize result
    visualizer.draw(cputime, posture);

    if(info_mex>10) % something went wrong
        str = sprintf('SNOPT info is %d, IK mex fails to solve the problem',info_mex);
        ros.log('WARN', str);
        ros.log('INFO', 'Infeasible Constraints:');

        str = sprintf('%s |  ', infeasible_constraints{:});
        ros.log('INFO', str);
        
        success = false;        
    else
        success = true;
    end
end

