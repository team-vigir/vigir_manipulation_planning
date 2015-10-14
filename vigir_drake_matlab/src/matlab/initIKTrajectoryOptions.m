function ikoptions = initIKTrajectoryOptions( robot_model, duration )
    nq = robot_model.getNumPositions();
    ikoptions = IKoptions(robot_model);
    cost = Point(robot_model.getStateFrame,1);
    %cost.base_x = 100;
    %cost.base_y = 100;
    %cost.base_z = 100;
    %cost.base_roll = 100;
    %cost.base_pitch = 100;
    %cost.base_yaw = 100;
    cost = double(cost);
    Q = diag(cost(1:nq));
    ikoptions = ikoptions.setQ(Q);
    ikoptions = ikoptions.setQv( duration * ikoptions.Qv);
    ikoptions = ikoptions.setQa( duration * duration * ikoptions.Qa);
    ikoptions = ikoptions.setMajorIterationsLimit(10000);
    ikoptions = ikoptions.setIterationsLimit(500000);
    ikoptions = ikoptions.setSuperbasicsLimit(1000);
    ikoptions = ikoptions.setMajorOptimalityTolerance(2e-4);
    ikoptions = ikoptions.setDebug(true);
    ikoptions = ikoptions.setMex(true);
    
    additional_sample_step = duration / 10;
    additional_samples = additional_sample_step:additional_sample_step:duration-additional_sample_step;
    ikoptions = ikoptions.setAdditionaltSamples(additional_samples);
end