pose_end = get_current_robot_pose(node_object.robot_model);

pose_traj_start = last_trajectory.eval( last_trajectory.tspan(1) );
pose_traj_start = pose_traj_start(1:node_object.robot_model.num_positions);

pose_traj_end = last_trajectory.eval( last_trajectory.tspan(2));    
pose_traj_end = pose_traj_end(1:node_object.robot_model.num_positions);

disp(' ');

disp('Compare pose_start <-> pose_traj_start:');
name_joint_values(pose_start-pose_traj_start, node_object.robot_model, true, 0.001);
disp(' ');

disp('Compare pose_traj_start <-> pose_traj_end:');
name_joint_values(pose_traj_start-pose_traj_end, node_object.robot_model, true, 0.001);
disp(' ');

disp('Compare pose_traj_end <-> pose_end:');
name_joint_values(pose_traj_end-pose_end, node_object.robot_model, true, 0.001);
disp(' ');

disp('Compare pose_start <-> pose_end:');
name_joint_values(pose_start-pose_end, node_object.robot_model, true, 0.001);
disp(' ');
