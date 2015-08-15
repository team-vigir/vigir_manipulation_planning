function [ trajectory, success, request ] = calcIKCartesianTrajectory( visualizer, robot_model, q0, request )
    %CALCIKCARTESIANTRAJECTORY Summary of this function goes here
    %   Detailed explanation goes here
    
    if ( isempty(request.waypoints) )
        success = true;
        request.waypoint_times = 1;   
        
        qqdot = [q0; zeros(size(q0, 1), 1)];
        
        q_lin = interp1([0 1], [qqdot, qqdot]', [0 1])';
        trajectory = PPTrajectory(foh([0 1],q_lin));
       
        return;
    end
    
    if ( ~isempty(visualizer) )
        visualizer.draw(cputime, q0);
    end
    
    if ( isempty(request.target_link_axis) )
        request.target_link_axis = deal(struct('x', {}, 'y', {}, 'z', {}));
        for i = 1:length(request.waypoints)
            request.target_link_axis(end+1).x = 0;
            request.target_link_axis(end).y = 0;
            request.target_link_axis(end).z = 1;
        end
    end

    if ( isempty(request.waypoint_times) )
        request.waypoint_times = estimateWaypointTimes(robot_model, q0, request.target_link_names, request.waypoints);
    end
     
    interpolated_waypoints = extractOrderedWaypoints(request, robot_model, q0);
    
    nq = robot_model.getNumPositions();
    num_steps = length(interpolated_waypoints);
    success = true;
    start_waypoint.waypoint_time = 0;
    start_waypoint.target_link_names = interpolated_waypoints(1).target_link_names;
    
    kinsol0 = doKinematics(robot_model,q0,false,true);
    start_waypoint.waypoints = deal(struct('position', {}, 'orientation', {}));
    for i = 1:length(start_waypoint.target_link_names)
        current_link_name = start_waypoint.target_link_names{i};
        eef_id = robot_model.findLinkId(current_link_name);
        eef_pts = [0;0;0];
        starting_pose = forwardKin(robot_model,kinsol0,eef_id,eef_pts,2);        
        start_waypoint.waypoints(end+1).position.x = starting_pose(1);
        start_waypoint.waypoints(end).position.y = starting_pose(2);
        start_waypoint.waypoints(end).position.z = starting_pose(3);
        start_waypoint.waypoints(end).orientation.w = starting_pose(4);
        start_waypoint.waypoints(end).orientation.x = starting_pose(5);
        start_waypoint.waypoints(end).orientation.y = starting_pose(6);
        start_waypoint.waypoints(end).orientation.z = starting_pose(7);
    end

    % set waypoint for time = 0

    trajectory = [];
    last_trajectory_step = [];
    for i = 1:num_steps
      if (i > 1)
        start_waypoint = target_waypoint;
      end
        target_waypoint = interpolated_waypoints(i);
        

        % stay at q0 for nominal trajectory
        q_lin = interp1([start_waypoint.waypoint_time target_waypoint.waypoint_time], [q0, q0]', [start_waypoint.waypoint_time target_waypoint.waypoint_time])';
        q_nom_traj = PPTrajectory(foh([start_waypoint.waypoint_time target_waypoint.waypoint_time],q_lin));
        q_seed_traj = q_nom_traj;
        
        % build IK options (add additional constraint checks)
        duration = target_waypoint.waypoint_time - start_waypoint.waypoint_time;
        ikoptions = initIKCartesianTrajectoryOptions(robot_model, duration);
        %ikoptions = ikoptions.setAdditionaltSamples( request.waypoint_times(1):1.0:request.waypoint_times(end) );


        % build list of constraints from message
        activeConstraints = buildIKCartesianTrajectoryConstraints(robot_model, request, start_waypoint,  target_waypoint, q0);

        % run inverse kinematics (mex)
        [current_traj,info_mex,infeasible_constraints] = inverseKinTraj(robot_model, [start_waypoint.waypoint_time target_waypoint.waypoint_time], q_seed_traj, q_nom_traj, activeConstraints{:},ikoptions);

        if(info_mex>10) % something went wrong
            ros.log('WARN', 'SNOPT calculation failed');

            str = sprintf('Current step: %d / %d', i, num_steps);
            ros.log('INFO', str);

            ros.log('INFO', 'Infeasible constraints:');
            str = sprintf('%s |  ', infeasible_constraints{:});
            ros.log('INFO', str);
            
            if ( i == 1 || request.execute_incomplete_cartesian_plans == false ) % fail if incomplete paths are not allowed or the first partial step was impossible
                success = false;     
                break;
            else % accept previous path, but abort at this point
                break;
            end                    
        end

        if ( isempty(trajectory) )
            trajectory = current_traj;
        else
            trajectory = trajectory.append(current_traj);
        end
        
        last_trajectory_step = current_traj;
        
        q0 = current_traj.eval(interpolated_waypoints(i).waypoint_time);
        q0 = q0(1:nq);
    end

    % visualize result
    if ( ~isempty(visualizer) && ~isempty(trajectory))
        visualizer.playback(trajectory,struct('slider',true));
    elseif ( ~isempty(visualizer) && ~isempty(last_trajectory_step) )
        visualizer.playback(last_trajectory_step);
    end
    
    % get eef frame axis
    l_hand = robot_model.findLinkId('l_hand');
%     kinsol0 = doKinematics(robot_model,q0_save,false,true);
%         world_pos_x = forwardKin(robot_model,kinsol0,l_hand,[0,0,0;1,0,0]',0);
%         world_pos_y = forwardKin(robot_model,kinsol0,l_hand,[0,0,0;0,1,0]',0);
%         world_pos_z = forwardKin(robot_model,kinsol0,l_hand,[0,0,0;0,0,1]',0);
%         axis_world_x = world_pos_x(:,2)-world_pos_x(:,1);
%         axis_world_y = world_pos_y(:,2)-world_pos_y(:,1);
%         axis_world_z = world_pos_z(:,2)-world_pos_z(:,1);
%         disp(['start-x-axis:  ' num2str(axis_world_x')]);
%         disp(['start-y-axis:  ' num2str(axis_world_y')]);
%         disp(['start-z-axis:  ' num2str(axis_world_z')]);
%         disp(['start-[0,0,0]: ' num2str(world_pos_x(:,1)')]);
        
        kinsol0 = doKinematics(robot_model,q0,false,true);
        world_pos_x = forwardKin(robot_model,kinsol0,l_hand,[0,0,0;1,0,0]',0);
        world_pos_y = forwardKin(robot_model,kinsol0,l_hand,[0,0,0;0,1,0]',0);
        world_pos_z = forwardKin(robot_model,kinsol0,l_hand,[0,0,0;0,0,1]',0);
        axis_world_x = world_pos_x(:,2)-world_pos_x(:,1);
        axis_world_y = world_pos_y(:,2)-world_pos_y(:,1);
        axis_world_z = world_pos_z(:,2)-world_pos_z(:,1);
        disp(['final-x-axis:  ' num2str(axis_world_x')]);
        disp(['final-y-axis:  ' num2str(axis_world_y')]);
        disp(['final-z-axis:  ' num2str(axis_world_z')]);
        disp(['final-[0,0,0]: ' num2str(world_pos_x(:,1)')]);
end

function interpolated_waypoints = extractOrderedWaypoints(request, robot_model, q0)
    % sort request by waypoint times
    [request.waypoint_times, sorted_idx] = sort(request.waypoint_times);
    request.waypoints = request.waypoints(sorted_idx);
    request.target_link_names = request.target_link_names(sorted_idx);
    request.target_link_axis = request.target_link_axis(sorted_idx);

    % add element for each time point
    [unique_times, ~, target_idx] = unique(request.waypoint_times);
    interpolated_waypoints(length(unique_times)) = struct();
    [interpolated_waypoints.waypoint_time] = deal([]);
    [interpolated_waypoints.waypoints] = deal(struct('position', {}, 'orientation', {}));
    [interpolated_waypoints.target_link_names] = deal({});
    [interpolated_waypoints.target_link_axis] = deal(struct('x', {}, 'y', {}, 'z', {}));
    [interpolated_waypoints.keep_line_and_orientation] = deal([]);

    for i = 1 : length(request.waypoint_times)
        interpolated_waypoints(target_idx(i)).waypoint_time = request.waypoint_times(i);
        interpolated_waypoints(target_idx(i)).waypoints(end+1) = request.waypoints(i);
        interpolated_waypoints(target_idx(i)).target_link_names(end+1) = request.target_link_names(i);
        interpolated_waypoints(target_idx(i)).target_link_axis(end+1) = request.target_link_axis(i);
        interpolated_waypoints(target_idx(i)).keep_line_and_orientation(end+1) = true;
    end

%    % get starting pose
%    kinsol0 = doKinematics(robot_model,q0,false,true);
%
%     % interpolate waypoints linearly
%     request_link_names = unique(request.target_link_names);
%     for i = 1 : length(request_link_names)
%         current_link_name = request_link_names{i};        
%         link_set = cellfun( @(x) ismember(current_link_name, x), {interpolated_waypoints.target_link_names}, 'UniformOutput', false);
%         link_set = cell2mat(link_set);
%         link_set_idx = find(link_set);
% 
%         if ( link_set_idx(1) > 1 ) % interpolate to first target point
%             eef_id = robot_model.findLinkId(current_link_name);
%             eef_pts = [0;0;0];
%             starting_pose = forwardKin(robot_model,kinsol0,eef_id,eef_pts,2);
% 
%             interpolation_idx = 1:link_set_idx(1)-1;
% 
%             % get target pose
%             waypoint_idx = strcmp(current_link_name, interpolated_waypoints(link_set_idx(1)).target_link_names);
%             target_waypoint = interpolated_waypoints(link_set_idx(1)).waypoints(waypoint_idx);
%             for j = 1 : length(interpolation_idx)
%                 current_idx = interpolation_idx(j);
% 
%                 start_pos_vec = starting_pose(1:3)';
%                 target_pos_vec = [target_waypoint.position.x, target_waypoint.position.y, target_waypoint.position.z];
% 
%                 start_orientation_quat = starting_pose(4:7)';
%                 target_orientation_quat = [target_waypoint.orientation.w, target_waypoint.orientation.x, target_waypoint.orientation.y, target_waypoint.orientation.z];
% 
%                 start_time = 0;
%                 target_time = interpolated_waypoints(link_set_idx(1)).waypoint_time;
% 
%                 current_time = interpolated_waypoints(current_idx).waypoint_time;
%                 normalized_current_time = (current_time - start_time) / (target_time - start_time);
% 
%                 % linear interpolation of position and slerp interpolation of
%                 % orientation
%                 current_pos_vec = interp1([start_time, target_time], [start_pos_vec; target_pos_vec], current_time);
%                 current_orientation_quat = slerp(start_orientation_quat, target_orientation_quat, normalized_current_time, 0.01);
% 
%                 % put values back into waypoint structure
%                 interpolated_waypoints(current_idx).waypoints(end+1).position.x = current_pos_vec(1);
%                 interpolated_waypoints(current_idx).waypoints(end).position.y = current_pos_vec(2);
%                 interpolated_waypoints(current_idx).waypoints(end).position.z = current_pos_vec(3);
%                 interpolated_waypoints(current_idx).waypoints(end).orientation.w = current_orientation_quat(1);
%                 interpolated_waypoints(current_idx).waypoints(end).orientation.x = current_orientation_quat(2);
%                 interpolated_waypoints(current_idx).waypoints(end).orientation.y = current_orientation_quat(3);
%                 interpolated_waypoints(current_idx).waypoints(end).orientation.z = current_orientation_quat(4);
%                 interpolated_waypoints(current_idx).target_link_names{end+1} = current_link_name;
%                 interpolated_waypoints(current_idx).keep_line_and_orientation(end+1) = false;
%             end
% 
%             % book-keeping for later
%             link_set(interpolation_idx) = true;
%             link_set_idx = find(link_set);
%         end
% 
%         if ( link_set_idx(end) < length( link_set ) ) % fix the final position
%             interpolation_idx = link_set_idx(end)+1 : length( link_set );
%             interpolated_waypoints(interpolation_idx).target_link_names{end+1} = current_link_name;
% 
%             % get target pose
%             waypoint_idx = strcmp(current_link_name, interpolated_waypoints(link_set_idx(end)).target_link_names);
%             interpolated_waypoints(interpolation_idx).waypoints(end+1) = interpolated_waypoints(link_set_idx(end)).waypoints(waypoint_idx);
%             interpolated_waypoints(interpolation_idx).keep_line_and_orientation(end+1) = true;
%             
%             % book-keeping for later
%             link_set(interpolation_idx) = true;
%             link_set_idx = find(link_set);
%         end
% 
%         % interpolate remaining values in between
%         link_missing = ~link_set;
%         link_missing_idx = find(link_missing);
%         for j = 1:length(link_missing_idx)
%             % find indices of neighors where current link name is set
%             current_idx = link_missing_idx(j);
%             start_idx = link_missing_idx(j)-1;
% 
%             end_idx = link_missing_idx(j)+1;
%             while( any( end_idx == link_missing_idx ) )
%                 end_idx = end_idx + 1;
%             end
% 
%             start_waypoint_idx = strcmp(current_link_name, interpolated_waypoints(start_idx).target_link_names);
%             start_waypoint = interpolated_waypoints(start_idx).waypoints(start_waypoint_idx);
% 
%             end_waypoint_idx = strcmp(current_link_name, interpolated_waypoints(end_idx).target_link_names);
%             end_waypoint = interpolated_waypoints(end_idx).waypoints(end_waypoint_idx);
% 
%             start_pos_vec = [start_waypoint.position.x, start_waypoint.position.y, start_waypoint.position.z];
%             end_pos_vec = [end_waypoint.position.x, end_waypoint.position.y, end_waypoint.position.z];
% 
%             start_orientation_quat = [start_waypoint.orientation.w, start_waypoint.orientation.x, start_waypoint.orientation.y, start_waypoint.orientation.z];
%             end_orientation_quat = [end_waypoint.orientation.w, end_waypoint.orientation.x, end_waypoint.orientation.y, end_waypoint.orientation.z];
% 
%             start_time = interpolated_waypoints(start_idx).waypoint_time;
%             end_time = interpolated_waypoints(end_idx).waypoint_time;
% 
%             current_time = interpolated_waypoints(current_idx).waypoint_time;
%             normalized_current_time = (current_time - start_time) / (end_time - start_time);
% 
%             % linear interpolation of position and slerp interpolation of
%             % orientation
%             current_pos_vec = interp1([start_time, end_time], [start_pos_vec; end_pos_vec], current_time);
%             current_orientation_quat = slerp(start_orientation_quat, end_orientation_quat, normalized_current_time, 0.01);
% 
%             % put values back into waypoint structure
%             interpolated_waypoints(current_idx).waypoints(end+1).position.x = current_pos_vec(1);
%             interpolated_waypoints(current_idx).waypoints(end).position.y = current_pos_vec(2);
%             interpolated_waypoints(current_idx).waypoints(end).position.z = current_pos_vec(3);
%             interpolated_waypoints(current_idx).waypoints(end).orientation.w = current_orientation_quat(1);
%             interpolated_waypoints(current_idx).waypoints(end).orientation.x = current_orientation_quat(2);
%             interpolated_waypoints(current_idx).waypoints(end).orientation.y = current_orientation_quat(3);
%             interpolated_waypoints(current_idx).waypoints(end).orientation.z = current_orientation_quat(4);
%             interpolated_waypoints(current_idx).target_link_names{end+1} = current_link_name;
%             interpolated_waypoints(current_idx).keep_line_and_orientation(end+1) = true;
%         end
%     end
end

function waypoint_times = estimateWaypointTimes(robot_model, q0, target_link_names, target_poses)
    scale_factor = 8; % TODO: Where to get that from? 
    minimum_trajectory_time = 0.5;

    kinsol = doKinematics(robot_model,q0,false,true);    
    start_link_names = unique(target_link_names);
    num_poses_per_time_step = length(start_link_names);
    
    for i = 1:num_poses_per_time_step;
        current_link_name = target_link_names{i};
        body_idx = robot_model.findLinkId(current_link_name);
        
        start_poses = forwardKin(robot_model,kinsol,body_idx,[0;0;0],0);
        current_start_pose.position.x = start_poses(1);
        current_start_pose.position.y = start_poses(2);
        current_start_pose.position.z = start_poses(3);
        current_start_pose.orientation = struct();
        target_poses = [current_start_pose, target_poses];
    end
    
    target_link_names = {start_link_names{:}, target_link_names{:}};
      
    num_waypoint_times = length(target_link_names);
    num_time_steps = num_waypoint_times / num_poses_per_time_step;
    
    waypoint_times = zeros(1, num_waypoint_times);
    
    previous_waypoint_time = 0;
    for i = 0:num_time_steps-2;
        % calculate distance for each end-effector
        distance = 0;
        for j = 1:num_poses_per_time_step
            start_pose_idx = i*num_poses_per_time_step + j;
            target_pose_idx = (i+1)*num_poses_per_time_step+1:(i+1)*num_poses_per_time_step+num_poses_per_time_step;
            
            link_name = target_link_names(start_pose_idx);
            target_pose_idx = (i+1)*num_poses_per_time_step+find(strcmpi(link_name, target_link_names(target_pose_idx)));
            
            
            start_pose = [target_poses(start_pose_idx).position.x, target_poses(start_pose_idx).position.y, target_poses(start_pose_idx).position.z];
            target_pose = [target_poses(target_pose_idx).position.x, target_poses(target_pose_idx).position.y, target_poses(target_pose_idx).position.z];
            
            link_distance = norm(target_pose - start_pose);
            if ( link_distance > distance )
                distance = link_distance;
            end
        end
        
        current_waypoint_time = previous_waypoint_time + distance * scale_factor;
        for j = 1:num_poses_per_time_step
            waypoint_times( (i+1)*num_poses_per_time_step + j) = current_waypoint_time;
        end
        
        previous_waypoint_time = current_waypoint_time;
    end
    
    waypoint_times = waypoint_times(num_poses_per_time_step+1:end);
    
    % ensure that trajectory takes at least some minimum time
    if ( waypoint_times(end) < minimum_trajectory_time )
        scale = minimum_trajectory_time / waypoint_times(end);
        waypoint_times = waypoint_times * scale;
    end
end

