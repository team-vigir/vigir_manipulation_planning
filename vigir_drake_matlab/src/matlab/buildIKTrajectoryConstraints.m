function activeConstraints = buildIKTrajectoryConstraints(robot_model, joint_constraints, q0, t)
    % build constraints from message
    activeConstraints = {};

    % fixed foot placement
    l_foot = robot_model.findLinkId('l_foot');
    r_foot = robot_model.findLinkId('r_foot');
    r_foot_pts = [0;0;0];
    l_foot_pts = [0;0;0];

    % get current foot position and fix it
    fw_kin_options.use_mex = true;
    fw_kin_options.compute_gradients = false;
    kinsol0 = doKinematics(robot_model,q0,[],fw_kin_options);
    r_foot_pos = forwardKin(robot_model,kinsol0,r_foot,r_foot_pts,2);
    l_foot_pos = forwardKin(robot_model,kinsol0,l_foot,l_foot_pts,2);

    l_foot_position_constr    = WorldPositionConstraint(robot_model, l_foot, l_foot_pts, l_foot_pos(1:3), l_foot_pos(1:3));
    r_foot_position_constr    = WorldPositionConstraint(robot_model, r_foot, r_foot_pts, r_foot_pos(1:3), r_foot_pos(1:3));
    l_foot_orientation_constr = WorldQuatConstraint(robot_model, l_foot, l_foot_pos(4:7), 0.01);
    r_foot_orientation_constr = WorldQuatConstraint(robot_model, r_foot, r_foot_pos(4:7), 0.01);
    %activeConstraints{end+1} = l_foot_position_constr;
    %activeConstraints{end+1} = r_foot_position_constr;
    %activeConstraints{end+1} = l_foot_orientation_constr;
    %activeConstraints{end+1} = r_foot_orientation_constr;

    % add quasi static constraint
    r_foot_contact_pts = robot_model.getBody(r_foot).getTerrainContactPoints();
    l_foot_contact_pts = robot_model.getBody(l_foot).getTerrainContactPoints();
    quasi_static_constr = QuasiStaticConstraint(robot_model);
    quasi_static_constr = quasi_static_constr.addContact(r_foot,r_foot_contact_pts);
    quasi_static_constr = quasi_static_constr.addContact(l_foot,l_foot_contact_pts);
    quasi_static_constr = quasi_static_constr.setActive(true);
    quasi_static_constr = quasi_static_constr.setShrinkFactor(0.9);
    activeConstraints{end+1} = quasi_static_constr;

    % constrain final posture
    posture_constr = PostureConstraint(robot_model, [t(end) t(end)]);
    
    for i = 1:length(joint_constraints)
        current_joint = joint_constraints(i);
        current_joint_body = robot_model.findJointId(current_joint.joint_name);
        current_joint_idx = robot_model.getBody(current_joint_body).position_num;
        current_joint_min = current_joint.position - current_joint.tolerance_below;
        current_joint_max = current_joint.position + current_joint.tolerance_above;
        
        posture_constr = posture_constr.setJointLimits(current_joint_idx, current_joint_min, current_joint_max);
    end

    activeConstraints{end+1} = posture_constr;
end
