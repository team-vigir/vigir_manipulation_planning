function ikoptions = initIKCartesianTrajectoryOptions( robot_model, duration, free_joint_names )
    nq = robot_model.getNumPositions();
    ikoptions = IKoptions(robot_model);
    cost = Point(robot_model.getStateFrame,1);
    cost.base_x = 10;
    cost.base_y = 10;
    cost.base_z = 10;
    cost.base_roll = 10;
    cost.base_pitch = 10;
    cost.base_yaw = 10;    
    cost = double(cost);
    
    for current_joint_name = {robot_model.body.jointname}
        if ( ~any(strcmpi(free_joint_names, current_joint_name)) && ~isempty(current_joint_name{:}))
            % this is not a free joint, lock it
            body_idx = robot_model.findJointId(current_joint_name);            
            fix_joint_idx = robot_model.body(body_idx).position_num;
            cost(fix_joint_idx) = cost(fix_joint_idx) * 100;
        end            
    end
    
    Q = diag(cost(1:nq));
    ikoptions = ikoptions.setQ(Q);
    ikoptions = ikoptions.setQv(0.01 * duration * Q);
    ikoptions = ikoptions.setQa(0.001 * duration * duration * Q);
    ikoptions = ikoptions.setMajorIterationsLimit(10000);
    ikoptions = ikoptions.setIterationsLimit(500000);
    ikoptions = ikoptions.setSuperbasicsLimit(1000);
    ikoptions = ikoptions.setMajorOptimalityTolerance(2e-4);
    ikoptions = ikoptions.setDebug(true);
    ikoptions = ikoptions.setMex(true);
    
    additional_sample_step = duration / 5;
    additional_samples = additional_sample_step:additional_sample_step:duration-additional_sample_step;
    %ikoptions = ikoptions.setAdditionaltSamples(additional_samples);
end