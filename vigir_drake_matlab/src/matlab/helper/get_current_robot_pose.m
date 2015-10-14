function current_robot_pose = get_current_robot_pose( robot_model )
%GET_CURRENT_ROBOT_POSE Summary of this function goes here
%   Detailed explanation goes here

    addpath('/usr/local/MATLAB/R2014a/ros/indigo/matlab');
    sensor_msgs;
    
    ros.init();
       
    joint_state_sub = ros.Subscriber('/thor_mang/joint_states','sensor_msgs/JointState', 1);
    imu_sub = ros.Subscriber('/thor_mang/pelvis_imu', 'sensor_msgs/Imu', 1);
    
    [joint_state_msg, ~, ~] = joint_state_sub.poll(10);
    [imu_msg, ~, ~] = imu_sub.poll(10);
        
    orientation_quat = [imu_msg.orientation.w;imu_msg.orientation.x;imu_msg.orientation.y;imu_msg.orientation.z];
    orientation_rpy = quat2rpy(orientation_quat);
    current_robot_pose(4:6) = orientation_rpy;
        
    current_robot_pose = handle_new_joint_state(joint_state_msg, robot_model, current_robot_pose)';
end

function current_robot_pose = handle_new_joint_state(joint_state_msg, robot_model, old_robot_pose)
    message_joint_names = joint_state_msg.name;
    message_qs = joint_state_msg.position;
    
    current_robot_pose = old_robot_pose;

    for i = 1:length(message_qs)
        current_name = message_joint_names{i};
        current_q = message_qs(i);

        body_idx = robot_model.findJointId(current_name);
        model_q_idx = robot_model.getBody(body_idx).position_num;
        current_robot_pose(model_q_idx) = current_q;
    end
end
