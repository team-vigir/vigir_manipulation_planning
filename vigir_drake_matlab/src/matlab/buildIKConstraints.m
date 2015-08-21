function activeConstraints = buildIKConstraints(robot_model, request, q0)
    % build constraints from message
    activeConstraints = {};

    % do not allow collisions
    %min_dist_constr = AllBodiesClosestDistanceConstraint(robot_model,0,inf);
    %activeConstraints{end+1} = min_dist_constr;
    
    % fixed foot placement
    l_foot = robot_model.findLinkId('l_foot');
    r_foot = robot_model.findLinkId('r_foot');
    r_foot_pts = [0;0;0];
    l_foot_pts = [0;0;0];

    % get current foot position and fix it
    kinsol0 = doKinematics(robot_model,q0,false,true);
    r_foot_pos = forwardKin(robot_model,kinsol0,r_foot,r_foot_pts,2);
    l_foot_pos = forwardKin(robot_model,kinsol0,l_foot,l_foot_pts,2);

    l_foot_position_constr    = WorldPositionConstraint(robot_model, l_foot, l_foot_pts, l_foot_pos(1:3), l_foot_pos(1:3));
    r_foot_position_constr    = WorldPositionConstraint(robot_model, r_foot, r_foot_pts, r_foot_pos(1:3), r_foot_pos(1:3));
    l_foot_orientation_constr = WorldQuatConstraint(robot_model, l_foot, l_foot_pos(4:7), 0);
    r_foot_orientation_constr = WorldQuatConstraint(robot_model, r_foot, r_foot_pos(4:7), 0);
    activeConstraints{end+1} = l_foot_position_constr;
    activeConstraints{end+1} = r_foot_position_constr;
    activeConstraints{end+1} = l_foot_orientation_constr;
    activeConstraints{end+1} = r_foot_orientation_constr;

    % add quasi static constraint
    r_foot_contact_pts = robot_model.getBody(r_foot).getTerrainContactPoints();
    l_foot_contact_pts = robot_model.getBody(l_foot).getTerrainContactPoints();
    quasi_static_constr = QuasiStaticConstraint(robot_model);
    quasi_static_constr = quasi_static_constr.addContact(r_foot,r_foot_contact_pts);
    quasi_static_constr = quasi_static_constr.addContact(l_foot,l_foot_contact_pts);
    quasi_static_constr = quasi_static_constr.setActive(true);
    quasi_static_constr = quasi_static_constr.setShrinkFactor(0.9);
    activeConstraints{end+1} = quasi_static_constr;

    % handle end effector positions and orientations
    for i = 1:length(request.target_poses)
        % get endeffector body ids and points
        eef_body_id = robot_model.findLinkId(request.target_link_names{i});
        eef_pts = [0;0;0];

        % goal position constraint
        goal_position = request.target_poses(i).pose.position;
        goal_position_vec = [goal_position.x; goal_position.y; goal_position.z];

        eef_position_constr = WorldPositionConstraint(robot_model, eef_body_id, eef_pts, goal_position_vec, goal_position_vec);
        activeConstraints{end+1} = eef_position_constr;

        % goal orientation constraint
        goal_orientation = request.target_poses(i).pose.orientation;
        goal_orientation_quat = [goal_orientation.w; goal_orientation.x; goal_orientation.y; goal_orientation.z];
        if ( ~all(goal_orientation_quat==0) )
            eef_orientation_constr = WorldQuatConstraint(robot_model, eef_body_id, goal_orientation_quat, 0);
			activeConstraints{end+1} = eef_orientation_constr;
        end       
    end
    
    % handle joint group
    posture_constr = PostureConstraint(robot_model);    
    for current_joint_name = {robot_model.body.jointname}
        if ( ~any(strcmpi(request.free_joint_names, current_joint_name)) && ~isempty(current_joint_name{:}) && ~strcmpi(current_joint_name, 'base') )
            % this is not a free joint, lock it
            body_idx = robot_model.findJointId(current_joint_name);            
            fix_joint_idx = robot_model.body(body_idx).position_num;
            joint_limit_min = robot_model.joint_limit_min(fix_joint_idx);
            joint_limit_max = robot_model.joint_limit_max(fix_joint_idx);
            target_joint_value = q0(fix_joint_idx);
            
            if ( target_joint_value < joint_limit_min )                
                ros.log('WARN', ['joint ' robot_model.body(body_idx).jointname ': target joint value < min joint limit (' num2str(target_joint_value) ' < ' num2str(joint_limit_min) ')' ]);
            elseif ( target_joint_value > joint_limit_max )
                ros.log('WARN', ['joint ' robot_model.body(body_idx).jointname ': target joint value > max joint limit (' num2str(target_joint_value) ' > ' num2str(joint_limit_max) ')' ]);
            else 
                posture_constr = posture_constr.setJointLimits(fix_joint_idx, target_joint_value, target_joint_value);
            end           
        end
            
    end

    activeConstraints{end+1} = posture_constr; 
end