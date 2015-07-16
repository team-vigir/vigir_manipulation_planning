function activeConstraints = buildIKCartesianTrajectoryConstraints(robot_model, request, start_waypoint, target_waypoint, q0)
    ORIENTATION_FULL = 0;
    ORIENTATION_AXIS_ONLY = 1;
    ORIENTATION_IGNORE = 2;

    % build constraints from message
    activeConstraints = {};

    % avoid self-collisions
    if ( request.check_self_collisions )
        %no_self_collision_constr = AllBodiesClosestDistanceConstraint(robot_model, 0.01, inf);
        no_self_collision_constr = MinDistanceConstraint(robot_model, 0.001);

        activeConstraints{end+1} = no_self_collision_constr;
    end

    % keep torso more or less upright
    torso_body_idx = robot_model.findLinkId('utorso');
    torso_upright_constr = WorldGazeDirConstraint(robot_model, torso_body_idx, [0; 0; 1], [0;0;1], 0.1, [start_waypoint.waypoint_time, target_waypoint.waypoint_time]);
    %activeConstraints{end+1} = torso_upright_constr;

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
    quasi_static_constr = QuasiStaticConstraint(robot_model);
    quasi_static_constr = quasi_static_constr.addContact(r_foot,r_foot_contact_pts);
    quasi_static_constr = quasi_static_constr.addContact(l_foot,l_foot_contact_pts);

    if ( strcmp(target_waypoint.target_link_names, 'l_foot' ) == 0 )
        l_foot_pos = forwardKin(robot_model,kinsol0,l_foot,l_foot_pts,2);
        l_foot_position_constr    = WorldPositionConstraint(robot_model, l_foot, l_foot_pts, l_foot_pos(1:3)-0.001, l_foot_pos(1:3)+0.001);
        l_foot_orientation_constr = WorldQuatConstraint(robot_model, l_foot, l_foot_pos(4:7), 0.01);

        activeConstraints{end+1} = l_foot_position_constr;
        activeConstraints{end+1} = l_foot_orientation_constr;
        quasi_static_constr = quasi_static_constr.addContact(l_foot,l_foot_contact_pts);
    end

    if ( strcmp(target_waypoint.target_link_names, 'r_foot' ) == 0 )
        r_foot_pos = forwardKin(robot_model,kinsol0,r_foot,r_foot_pts,2);
        r_foot_position_constr    = WorldPositionConstraint(robot_model, r_foot, r_foot_pts, r_foot_pos(1:3)-0.001, r_foot_pos(1:3)+0.001);
        r_foot_orientation_constr = WorldQuatConstraint(robot_model, r_foot, r_foot_pos(4:7), 0.01);

        activeConstraints{end+1} = r_foot_position_constr;
        activeConstraints{end+1} = r_foot_orientation_constr;
        quasi_static_constr = quasi_static_constr.addContact(r_foot,r_foot_contact_pts);
    end

    quasi_static_constr = quasi_static_constr.setActive(true);
    quasi_static_constr = quasi_static_constr.setShrinkFactor(0.9);
    activeConstraints{end+1} = quasi_static_constr;

    % add waypoint constraints
    for i = 1:length(target_waypoint.target_link_names)
        current_waypoint_idx = find(strcmp(start_waypoint.target_link_names, target_waypoint.target_link_names{i}));

        target_link_name = target_waypoint.target_link_names{i};
        
        % get endeffector body ids and points
        eef_body_id = robot_model.findLinkId(target_link_name);
        eef_pts = [0;0;0];
               
        % goal position constraint
        goal_position = target_waypoint.waypoints(i).position;
        goal_position_vec = [goal_position.x; goal_position.y; goal_position.z];

        eef_position_constr = WorldPositionConstraint(robot_model, eef_body_id, eef_pts, goal_position_vec, goal_position_vec, [target_waypoint.waypoint_time, target_waypoint.waypoint_time]);
        activeConstraints{end+1} = eef_position_constr;

        % goal orientation constraint
        orientation = target_waypoint.waypoints(i).orientation;
        orientation_quat = [orientation.w; orientation.x; orientation.y; orientation.z];
        link_axis = [target_waypoint.target_link_axis(i).x; target_waypoint.target_link_axis(i).y; target_waypoint.target_link_axis(i).z];
            
        if ( request.free_motion  )            
            duration = [target_waypoint.waypoint_time, target_waypoint.waypoint_time];
        else            
            duration = [start_waypoint.waypoint_time, target_waypoint.waypoint_time];
        end

        if ( request.target_orientation_type == ORIENTATION_FULL )            
            eef_orientation_constr = WorldQuatConstraint(robot_model, eef_body_id, orientation_quat, 0, duration);
            activeConstraints{end+1} = eef_orientation_constr;
        elseif ( request.target_orientation_type == ORIENTATION_AXIS_ONLY && ~all(link_axis == 0) ) % goal axis orientation constraint
            %dir = quat2axis(orientation_quat);
            %eef_dir_constr = WorldGazeDirConstraint(robot_model,eef_body_id, link_axis, dir(1:3),0,duration);
            eef_axis_constr = WorldGazeOrientConstraint(robot_model, eef_body_id, link_axis, orientation_quat, 0.00, pi, duration);
            activeConstraints{end+1} = eef_axis_constr;
        end

        % setup line distance constraints between waypoints
        if ( request.free_motion == false )
            line_start = start_waypoint.waypoints(current_waypoint_idx).position;
            line_start_vec = [line_start.x; line_start.y; line_start.z];
            time_start = start_waypoint.waypoint_time;

            line_end = target_waypoint.waypoints(i).position;
            line_end_vec = [line_end.x; line_end.y; line_end.z];
            time_end = target_waypoint.waypoint_time;

            line = [line_start_vec line_end_vec];
            tspan = [time_start time_end];

            line_dist_constr = Point2LineSegDistConstraint(robot_model,eef_body_id,eef_pts,1,line,0.0,0.02,tspan);
            activeConstraints{end+1} = line_dist_constr;
        end
    end
    
    % handle joint group
    posture_constr = PostureConstraint(robot_model);    
    for current_joint_name = {robot_model.body.jointname}
        if ( ~any(strcmpi(request.free_joint_names, current_joint_name)) && ~isempty(current_joint_name{:}) && ~strcmpi(current_joint_name, 'base') )
            % this is not a free joint, lock it
            body_idx = robot_model.findJointId(current_joint_name);            
            fix_joint_idx = robot_model.body(body_idx).position_num;
            posture_constr = posture_constr.setJointLimits(fix_joint_idx, q0(fix_joint_idx), q0(fix_joint_idx));
        end
            
    end

    activeConstraints{end+1} = posture_constr;   
end
