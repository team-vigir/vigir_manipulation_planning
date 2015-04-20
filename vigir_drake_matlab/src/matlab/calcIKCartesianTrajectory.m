function [ trajectory, success ] = calcIKCartesianTrajectory( visualizer, robot_model, q0, request )
    %CALCIKCARTESIANTRAJECTORY Summary of this function goes here
    %   Detailed explanation goes here

    visualizer.draw(cputime, q0);

    interpolated_waypoints = extractOrderedWaypoints(request, robot_model, q0);
    
    nq = robot_model.getNumPositions();
    num_steps = length(interpolated_waypoints);
    success = true;
    current_waypoint.waypoint_time = 0;
    current_waypoint.target_link_names = interpolated_waypoints(1).target_link_names;
    current_waypoint.waypoints = struct('position', {}, 'orientation', {});
    current_waypoint.keep_line_and_orientation = zeros(length(current_waypoint.target_link_names), 1);
    % set waypoint for time = 0
    
    trajectory = [];
    for i = 1:num_steps
        kinsol0 = doKinematics(robot_model,q0,false,true);
        
        current_waypoint.waypoints = struct('position', {}, 'orientation', {});
        for j = 1:length(current_waypoint.target_link_names)
            eef_id = robot_model.findLinkId(current_waypoint.target_link_names{j});
            eef_pts = [0;0;0];
            
            waypoint_vec = forwardKin(robot_model,kinsol0,eef_id,eef_pts,2);
            waypoint.position.x = waypoint_vec(1);
            waypoint.position.y = waypoint_vec(2);
            waypoint.position.z = waypoint_vec(3);
            waypoint.orientation.w = waypoint_vec(4);
            waypoint.orientation.x = waypoint_vec(5);
            waypoint.orientation.y = waypoint_vec(6);
            waypoint.orientation.z = waypoint_vec(7);
            
            current_waypoint.waypoints(end+1) = waypoint;
        end
        
        

        % stay at q0 for nominal trajectory
        q_lin = interp1([current_waypoint.waypoint_time interpolated_waypoints(i).waypoint_time], [q0, q0]', [current_waypoint.waypoint_time interpolated_waypoints(i).waypoint_time])';
        q_nom_traj = PPTrajectory(foh([current_waypoint.waypoint_time interpolated_waypoints(i).waypoint_time],q_lin));
        q_seed_traj = q_nom_traj;
        
        % build IK options (add additional constraint checks)
        duration = interpolated_waypoints(i).waypoint_time - current_waypoint.waypoint_time;
        ikoptions = initIKCartesianTrajectoryOptions(robot_model, duration);
        %ikoptions = ikoptions.setAdditionaltSamples( request.waypoint_times(1):1.0:request.waypoint_times(end) );


        % build list of constraints from message
        activeConstraints = buildIKCartesianTrajectoryConstraints(robot_model, request, current_waypoint,  interpolated_waypoints(i), q0);

        % run inverse kinematics (mex)
        [current_traj,info_mex,infeasible_constraints] = inverseKinTraj(robot_model, [current_waypoint.waypoint_time interpolated_waypoints(i).waypoint_time], q_seed_traj, q_nom_traj, activeConstraints{:},ikoptions);

        if ( isempty(trajectory) )
            trajectory = current_traj;
        else
            trajectory = trajectory.append(current_traj);
        end

        if(info_mex>10) % something went wrong
            ros.log('WARN', 'SNOPT calculation failed');

            str = sprintf('Current step: %d / %d', i, num_steps);
            ros.log('INFO', str);

            ros.log('INFO', 'Infeasible constraints:');
            str = sprintf('%s |  ', infeasible_constraints{:});
            ros.log('INFO', str);

            success = false;
            %break;
        end

        q0 = current_traj.eval(interpolated_waypoints(i).waypoint_time);
        q0 = q0(1:nq);

        current_waypoint.waypoint_time = interpolated_waypoints(i).waypoint_time;
        current_waypoint.target_link_names = interpolated_waypoints(i).target_link_names;
        current_waypoint.keep_line_and_orientation = interpolated_waypoints(i).keep_line_and_orientation;
    end

    % visualize result
    visualizer.playback(trajectory,struct('slider',true));
    end

function interpolated_waypoints = extractOrderedWaypoints(request, robot_model, q0)
    % sort request by waypoint times
    [request.waypoint_times, sorted_idx] = sort(request.waypoint_times);
    request.waypoints = request.waypoints(sorted_idx);
    request.target_link_names = request.target_link_names(sorted_idx);

    % add element for each time point
    [unique_times, ~, target_idx] = unique(request.waypoint_times);
    interpolated_waypoints(length(unique_times)) = struct();
    [interpolated_waypoints.waypoint_time] = deal([]);
    [interpolated_waypoints.waypoints] = deal(struct('position', {}, 'orientation', {}));
    [interpolated_waypoints.target_link_names] = deal({});
    [interpolated_waypoints.keep_line_and_orientation] = deal([]);

    for i = 1 : length(request.waypoint_times)
        interpolated_waypoints(target_idx(i)).waypoint_time = request.waypoint_times(i);
        interpolated_waypoints(target_idx(i)).waypoints(end+1) = request.waypoints(i);
        interpolated_waypoints(target_idx(i)).target_link_names(end+1) = request.target_link_names(i);
        interpolated_waypoints(target_idx(i)).keep_line_and_orientation(end+1) = true;
    end

    % get starting pose
    kinsol0 = doKinematics(robot_model,q0,false,true);

    % interpolate waypoints linearly
    request_link_names = unique(request.target_link_names);
    for i = 1 : length(request_link_names)
        current_link_name = request_link_names{i};
        link_set = cellfun( @(x) ismember(current_link_name, x), {interpolated_waypoints.target_link_names}, 'UniformOutput', false);
        link_set = cell2mat(link_set);
        link_set_idx = find(link_set);

        if ( link_set_idx(1) > 1 ) % interpolate to first target point
            eef_id = robot_model.findLinkId(current_link_name);
            eef_pts = [0;0;0];
            starting_pose = forwardKin(robot_model,kinsol0,eef_id,eef_pts,2);

            interpolation_idx = 1:link_set_idx(1)-1;

            % get target pose
            waypoint_idx = strcmp(current_link_name, interpolated_waypoints(link_set_idx(1)).target_link_names);
            target_waypoint = interpolated_waypoints(link_set_idx(1)).waypoints(waypoint_idx);
            for j = 1 : length(interpolation_idx)
                current_idx = interpolation_idx(j);

                start_pos_vec = starting_pose(1:3)';
                target_pos_vec = [target_waypoint.position.x, target_waypoint.position.y, target_waypoint.position.z];

                start_orientation_quat = starting_pose(4:7)';
                target_orientation_quat = [target_waypoint.orientation.w, target_waypoint.orientation.x, target_waypoint.orientation.y, target_waypoint.orientation.z];

                start_time = 0;
                target_time = interpolated_waypoints(link_set_idx(1)).waypoint_time;

                current_time = interpolated_waypoints(current_idx).waypoint_time;
                normalized_current_time = (current_time - start_time) / (target_time - start_time);

                % linear interpolation of position and slerp interpolation of
                % orientation
                current_pos_vec = interp1([start_time, target_time], [start_pos_vec; target_pos_vec], current_time);
                current_orientation_quat = slerp(start_orientation_quat, target_orientation_quat, normalized_current_time, 0.01);

                % put values back into waypoint structure
                interpolated_waypoints(current_idx).waypoints(end+1).position.x = current_pos_vec(1);
                interpolated_waypoints(current_idx).waypoints(end).position.y = current_pos_vec(2);
                interpolated_waypoints(current_idx).waypoints(end).position.z = current_pos_vec(3);
                interpolated_waypoints(current_idx).waypoints(end).orientation.w = current_orientation_quat(1);
                interpolated_waypoints(current_idx).waypoints(end).orientation.x = current_orientation_quat(2);
                interpolated_waypoints(current_idx).waypoints(end).orientation.y = current_orientation_quat(3);
                interpolated_waypoints(current_idx).waypoints(end).orientation.z = current_orientation_quat(4);
                interpolated_waypoints(current_idx).target_link_names{end+1} = current_link_name;
                interpolated_waypoints(current_idx).keep_line_and_orientation(end+1) = false;
            end

            % book-keeping for later
            link_set(interpolation_idx) = true;
            link_set_idx = find(link_set);
        end

        if ( link_set_idx(end) < length( link_set ) ) % fix the final position
            interpolation_idx = link_set_idx(end)+1 : length( link_set );
            interpolated_waypoints(interpolation_idx).target_link_names{end+1} = current_link_name;

            % get target pose
            waypoint_idx = strcmp(current_link_name, interpolated_waypoints(link_set_idx(end)).target_link_names);
            interpolated_waypoints(interpolation_idx).waypoints(end+1) = interpolated_waypoints(link_set_idx(end)).waypoints(waypoint_idx);
            interpolated_waypoints(interpolation_idx).keep_line_and_orientation(end+1) = true;
            
            % book-keeping for later
            link_set(interpolation_idx) = true;
            link_set_idx = find(link_set);
        end

        % interpolate remaining values in between
        link_missing = ~link_set;
        link_missing_idx = find(link_missing);
        for j = 1:length(link_missing_idx)
            % find indices of neighors where current link name is set
            current_idx = link_missing_idx(j);
            start_idx = link_missing_idx(j)-1;

            end_idx = link_missing_idx(j)+1;
            while( any( end_idx == link_missing_idx ) )
                end_idx = end_idx + 1;
            end

            start_waypoint_idx = strcmp(current_link_name, interpolated_waypoints(start_idx).target_link_names);
            start_waypoint = interpolated_waypoints(start_idx).waypoints(start_waypoint_idx);

            end_waypoint_idx = strcmp(current_link_name, interpolated_waypoints(end_idx).target_link_names);
            end_waypoint = interpolated_waypoints(end_idx).waypoints(end_waypoint_idx);

            start_pos_vec = [start_waypoint.position.x, start_waypoint.position.y, start_waypoint.position.z];
            end_pos_vec = [end_waypoint.position.x, end_waypoint.position.y, end_waypoint.position.z];

            start_orientation_quat = [start_waypoint.orientation.w, start_waypoint.orientation.x, start_waypoint.orientation.y, start_waypoint.orientation.z];
            end_orientation_quat = [end_waypoint.orientation.w, end_waypoint.orientation.x, end_waypoint.orientation.y, end_waypoint.orientation.z];

            start_time = interpolated_waypoints(start_idx).waypoint_time;
            end_time = interpolated_waypoints(end_idx).waypoint_time;

            current_time = interpolated_waypoints(current_idx).waypoint_time;
            normalized_current_time = (current_time - start_time) / (end_time - start_time);

            % linear interpolation of position and slerp interpolation of
            % orientation
            current_pos_vec = interp1([start_time, end_time], [start_pos_vec; end_pos_vec], current_time);
            current_orientation_quat = slerp(start_orientation_quat, end_orientation_quat, normalized_current_time, 0.01);

            % put values back into waypoint structure
            interpolated_waypoints(current_idx).waypoints(end+1).position.x = current_pos_vec(1);
            interpolated_waypoints(current_idx).waypoints(end).position.y = current_pos_vec(2);
            interpolated_waypoints(current_idx).waypoints(end).position.z = current_pos_vec(3);
            interpolated_waypoints(current_idx).waypoints(end).orientation.w = current_orientation_quat(1);
            interpolated_waypoints(current_idx).waypoints(end).orientation.x = current_orientation_quat(2);
            interpolated_waypoints(current_idx).waypoints(end).orientation.y = current_orientation_quat(3);
            interpolated_waypoints(current_idx).waypoints(end).orientation.z = current_orientation_quat(4);
            interpolated_waypoints(current_idx).target_link_names{end+1} = current_link_name;
            interpolated_waypoints(current_idx).keep_line_and_orientation(end+1) = true;
        end
    end


end

