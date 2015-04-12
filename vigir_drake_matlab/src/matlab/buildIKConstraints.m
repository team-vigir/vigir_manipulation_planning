function activeConstraints = buildIKConstraints(robot_model, target_poses, target_link_names, q0)
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
    for i = 1:length(target_poses)
        % get endeffector body ids and points
        eef_body_id = robot_model.findLinkId(target_link_names{i});
        eef_pts = [0;0;0];

        % goal position constraint
        goal_position = target_poses(i).pose.position;
        goal_position_vec = [goal_position.x; goal_position.y; goal_position.z];

        eef_position_constr = WorldPositionConstraint(robot_model, eef_body_id, eef_pts, goal_position_vec, goal_position_vec);
        activeConstraints{end+1} = eef_position_constr;

        % goal orientation constraint
        goal_orientation = target_poses(i).pose.orientation;
        goal_orientation_quat = [goal_orientation.w; goal_orientation.x; goal_orientation.y; goal_orientation.z];
        if ( all(goal_orientation_quat==0) )
            goal_orientation_quat = [1;0;0;0];
        end
        
        eef_orientation_constr = WorldQuatConstraint(robot_model, eef_body_id, goal_orientation_quat, 0);
        activeConstraints{end+1} = eef_orientation_constr;
    end
%
%             % fix joint positions of non-move-group joints
%             fix_joint_idx = [];
%             for i = 1:robot_model.getNumBodies()
%                 joint_name = robot_model.body(i).jointname;
%                 if ( isempty(joint_name) || strcmp(joint_name, 'base') )
%                     continue;
%                 end
%
%                 matches = strfind(message.free_joint_names, joint_name);
%                 if ( all(cellfun('isempty', matches)) == true ) % joint not in "free" list
%                     % fix joint position
%                     joint_idx = robot_model.findJointId(joint_name);
%                     fix_joint_idx = [fix_joint_idx joint_idx];
%                 end
%             end
%
%             move_group_constr = PostureConstraint(robot_model);
%
%             % force joint constraints to joint limits (necessary due to
%             % rounding errors)
%             min_violation_idx = find ( q0 - robot_model.joint_limit_min < 0 );
%             max_violation_idx = find ( robot_model.joint_limit_max - q0 < 0 );
%             q0(min_violation_idx) = robot_model.joint_limit_min(min_violation_idx);
%             q0(max_violation_idx) = robot_model.joint_limit_max(max_violation_idx);
%
%             move_group_constr = move_group_constr.setJointLimits(fix_joint_idx', q0(fix_joint_idx), q0(fix_joint_idx));
%             activeConstraints{end+1} = move_group_constr;
end