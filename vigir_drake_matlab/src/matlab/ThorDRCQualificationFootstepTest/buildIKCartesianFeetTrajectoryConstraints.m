function activeConstraints = buildIKCartesianFeetTrajectoryConstraints(robot_model, target_link_name, waypoints, waypoint_times, free_joint_names, q0, do_self_collision_checks, world_mat, start_time)
    % build constraints from message
    activeConstraints = {};
    
    end_time = waypoint_times(end);
    
    % avoid self-collisions
    %no_self_collision_constr = AllBodiesClosestDistanceConstraint(robot_model, 0.01, inf);
    %no_self_collision_constr = MinDistanceConstraint(robot_model, 0.001);
    %if ( do_self_collision_checks )
    %    activeConstraints{end+1} = no_self_collision_constr;
    %end
    
    % fix start posture
    start_posture_constr = PostureConstraint(robot_model, [start_time, start_time]);
    start_posture_constr = start_posture_constr.setJointLimits((1:length(q0))', q0-0.001, q0+0.001);
    activeConstraints{end+1} = start_posture_constr;
    
    % keep torso more or less upright
    torso = robot_model.findLinkId('utorso');
    torso_quat = [1, 0, 0, 0]';
    torso_mat = quat2rotmat(torso_quat);
    torso_mat = world_mat(1:3, 1:3) * torso_mat;
    torso_quat = rotmat2quat(torso_mat);    
    torso_upright_constr = WorldQuatConstraint(robot_model, torso, torso_quat, 0.15, [start_time, end_time]);
    activeConstraints{end+1} = torso_upright_constr;
    
    % fixed foot placement
    l_foot = robot_model.findLinkId('l_foot');
    r_foot = robot_model.findLinkId('r_foot');
    r_foot_pts = [0;0;0];
    l_foot_pts = [0;0;0];

    % get current foot position and fix it
    kinsol0 = doKinematics(robot_model,q0,false,true);
     
    % add quasi static constraint
    r_foot_contact_pts = robot_model.getBody(r_foot).getTerrainContactPoints();
    l_foot_contact_pts = robot_model.getBody(l_foot).getTerrainContactPoints();
          
    quasi_static_constr = QuasiStaticConstraint(robot_model, [start_time, end_time]);
    if ( strcmp(target_link_name, 'l_foot' ) == 0 )
        l_foot_pos = forwardKin(robot_model,kinsol0,l_foot,l_foot_pts,2);
        l_foot_position_constr    = WorldPositionConstraint(robot_model, l_foot, l_foot_pts, l_foot_pos(1:3)-0.001, l_foot_pos(1:3)+0.001, [start_time, end_time]);
        l_foot_orientation_constr = WorldQuatConstraint(robot_model, l_foot, l_foot_pos(4:7), 0.01, [start_time, end_time]);
    
        activeConstraints{end+1} = l_foot_position_constr;
        activeConstraints{end+1} = l_foot_orientation_constr;
        quasi_static_constr = quasi_static_constr.addContact(l_foot,l_foot_contact_pts);
    end
    
    if ( strcmp(target_link_name, 'r_foot' ) == 0 )
        r_foot_pos = forwardKin(robot_model,kinsol0,r_foot,r_foot_pts,2);
        r_foot_position_constr    = WorldPositionConstraint(robot_model, r_foot, r_foot_pts, r_foot_pos(1:3)-0.001, r_foot_pos(1:3)+0.001, [start_time, end_time]);
        r_foot_orientation_constr = WorldQuatConstraint(robot_model, r_foot, r_foot_pos(4:7), 0.01, [start_time, end_time]);
           
        activeConstraints{end+1} = r_foot_position_constr;
        activeConstraints{end+1} = r_foot_orientation_constr;
        quasi_static_constr = quasi_static_constr.addContact(r_foot,r_foot_contact_pts);
    end
      
    quasi_static_constr = quasi_static_constr.setActive(true);
    quasi_static_constr = quasi_static_constr.setShrinkFactor(0.4);
    activeConstraints{end+1} = quasi_static_constr;

    % get endeffector body ids and points
    if ( ~isempty(target_link_name) )
        eef_body_id = robot_model.findLinkId(target_link_name);
        eef_pts = [0;0;0];

        eef_pos = forwardKin(robot_model,kinsol0,eef_body_id,eef_pts,2);
        current_position = eef_pos(1:3);
        current_orientation = eef_pos(4:7);


        % waypoints are set relative to the current position
        for i = 1:length(waypoints)
            waypoints(i).position.x = waypoints(i).position.x + current_position(1);
            waypoints(i).position.y = waypoints(i).position.y + current_position(2);
            waypoints(i).position.z = waypoints(i).position.z + current_position(3);  
            waypoints(i).orientation.w = current_orientation(1);
            waypoints(i).orientation.x = current_orientation(2);
            waypoints(i).orientation.y = current_orientation(3);
            waypoints(i).orientation.z = current_orientation(4);
        end

        % add waypoint constraints
        for i = 1:length(waypoints)
            % goal position constraint
            goal_position = waypoints(i).position;
            goal_position_vec = [goal_position.x; goal_position.y; goal_position.z];

            eef_position_constr = WorldPositionConstraint(robot_model, eef_body_id, eef_pts, goal_position_vec, goal_position_vec, [waypoint_times(i), waypoint_times(i)]);
            activeConstraints{end+1} = eef_position_constr;

            % goal orientation constraint
            goal_orientation = waypoints(i).orientation;
            goal_orientation_quat = [goal_orientation.w; goal_orientation.x; goal_orientation.y; goal_orientation.z];

            eef_orientation_constr = WorldQuatConstraint(robot_model, eef_body_id, goal_orientation_quat, 0, [waypoint_times(i), waypoint_times(i)]);
            activeConstraints{end+1} = eef_orientation_constr;

            % goal axis orientation constraint
            %direction = quat2axis(goal_orientation_quat);
            %eef_axis_constr = WorldGazeDirConstraint(robot_model, eef_body_id, [0; 1; 0], direction(1:3), 0.05, [waypoint_times(1), waypoint_times(end)]);
            %activeConstraints{end+1} = eef_axis_constr;
        end
    end

end
