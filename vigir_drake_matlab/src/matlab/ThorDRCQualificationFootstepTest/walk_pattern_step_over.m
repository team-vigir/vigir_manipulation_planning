function [ num_steps, waypoint_times_step, waypoints_step, target_link_name_step, t ] = walk_pattern_step_over(world_mat)

num_steps = 6;
time_per_waypoint = 2;

current_t = 0;
current_step = 1;


% first step
waypoint_times_step{current_step} = [current_t+time_per_waypoint current_t+2*time_per_waypoint];
waypoint.position = setRelativeTargetPosition(0, 0, 0.25, world_mat);
waypoints_step{current_step} = waypoint;
waypoint.position = setRelativeTargetPosition(0, 0.3, 0.21, world_mat);
waypoints_step{current_step} = [waypoints_step{current_step} waypoint];
target_link_name_step{current_step} = 'r_foot';
current_t = current_t + 2*time_per_waypoint;
current_step = current_step + 1;


% second step
waypoint_times_step{current_step} = [current_t+time_per_waypoint current_t+2*time_per_waypoint];
waypoint.position = setRelativeTargetPosition(0, 0, 0.25, world_mat);
waypoints_step{current_step} = waypoint;
waypoint.position = setRelativeTargetPosition(0, 0.3, 0.21, world_mat);
waypoints_step{current_step} = [waypoints_step{current_step} waypoint];
target_link_name_step{current_step} = 'l_foot';
current_t = current_t + 2*time_per_waypoint;
current_step = current_step + 1;
 
% back to both feet
waypoint_times_step{current_step} = [current_t+1, current_t+2];
waypoints_step{current_step} = [];
target_link_name_step{current_step} = '';
current_t = current_t + 2;
current_step = current_step + 1;

% first step
waypoint_times_step{current_step} = [current_t+time_per_waypoint current_t+2*time_per_waypoint current_t+3*time_per_waypoint];
waypoint.position = setRelativeTargetPosition(0, 0.0, 0.05, world_mat);
waypoints_step{current_step} = waypoint;
waypoint.position = setRelativeTargetPosition(0, 0.25, 0.05, world_mat);
waypoints_step{current_step} = [waypoints_step{current_step} waypoint];
waypoint.position = setRelativeTargetPosition(0, 0.3, -0.21, world_mat);
waypoints_step{current_step} = [waypoints_step{current_step} waypoint];
target_link_name_step{current_step} = 'r_foot';
current_t = current_t + 3*time_per_waypoint;
current_step = current_step + 1;


% second step
waypoint_times_step{current_step} = [current_t+time_per_waypoint current_t+2*time_per_waypoint current_t+3*time_per_waypoint];
waypoint.position = setRelativeTargetPosition(0, 0.0, 0.05, world_mat);
waypoints_step{current_step} = waypoint;
waypoint.position = setRelativeTargetPosition(0, 0.25, 0.05, world_mat);
waypoints_step{current_step} = [waypoints_step{current_step} waypoint];
waypoint.position = setRelativeTargetPosition(0, 0.3, -0.21, world_mat);
waypoints_step{current_step} = [waypoints_step{current_step} waypoint];
target_link_name_step{current_step} = 'l_foot';
current_t = current_t + 3*time_per_waypoint;
current_step = current_step + 1;

% back to both feet
waypoint_times_step{current_step} = [current_t+1, current_t+2];
waypoints_step{current_step} = [];
target_link_name_step{current_step} = '';
current_t = current_t + 2;
current_step = current_step + 1;

t = 0:0.25:current_t;

end

