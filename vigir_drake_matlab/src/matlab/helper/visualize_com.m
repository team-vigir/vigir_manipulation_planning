function visualize_com()
    addpath('/usr/local/MATLAB/R2014a/ros/indigo/matlab');
    sensor_msgs;
    
    ros.init();
      
    if(~checkDependency('snopt'))
        ros.log('ERROR', 'Need package "snopt" to run...');
        error('Need package "snopt" to run...');
    end

    % add vigir paths to ros package path (needed to load robot
    % model correctly)
    vigir_paths = getenv('ROS_VIGIR_PACKAGE_PATH');
    rosmatlab_paths = getenv('ROS_PACKAGE_PATH');
    if ( length([rosmatlab_paths ':' vigir_paths]) <= 32766 )
        setenv('ROS_PACKAGE_PATH', [rosmatlab_paths ':' vigir_paths]);
    else
        warning('Unable to set ROS_PACKAGE_PATH');
    end

    robot_model = RigidBodyManipulator();

    options.floating = 'rpy';
    urdf_string = ros.param.get('/robot_description');

    w = warning('off','Drake:RigidBodyManipulator:ReplacedCylinder');
    warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
    robot_model = addRobotFromURDFString(robot_model,urdf_string,[],[],options);
    warning(w);

    % load nominal pose from param server
    pose_str = ros.param.get('/drake_nominal_pose');
    if ( isempty(pose_str) )
        ros.log('ERROR', 'Please specify /drake_nominal_pose ROS parameter...');
        error('Please specify /drake_nominal_pose ROS parameter...');
    end
    robot_nominal_pose = sscanf(pose_str, '%f, ');
    if ( length(robot_nominal_pose) ~= robot_model.num_positions)
        ros.log('WARN', 'Nominal pose does not match number of robot joints. Setting to 0');
        robot_nominal_pose = zeros(robot_model.num_positions, 1);
    end
    
    current_robot_pose = robot_nominal_pose;

    % construct visualizer
    robot_visualizer = robot_model.constructVisualizer();

    
    joint_state_sub = ros.Subscriber('/thor_mang/joint_states','sensor_msgs/JointState', 1);
    imu_sub = ros.Subscriber('/thor_mang/pelvis_imu', 'sensor_msgs/Imu', 1);

    % update visualization from current pose
    figure;
    hold on

    while(1)
        [joint_state_msg, ~, ~] = joint_state_sub.poll(10);
        [imu_msg, ~, ~] = imu_sub.poll(10);
        
        orientation_quat = [imu_msg.orientation.w;imu_msg.orientation.x;imu_msg.orientation.y;imu_msg.orientation.z];
        orientation_rpy = quat2rpy(orientation_quat);
        current_robot_pose(4:6) = orientation_rpy;
        
        current_robot_pose = handle_new_joint_state(joint_state_msg, robot_model, current_robot_pose);
        
        robot_visualizer.draw(cputime, current_robot_pose);

        % calculate foot contact points in world
        kinsol0 = doKinematics(robot_model,current_robot_pose,false,true);

        l_foot = robot_model.findLinkId('l_foot');
        r_foot = robot_model.findLinkId('r_foot');
        r_foot_contact_pts = robot_model.getBody(r_foot).getTerrainContactPoints();
        l_foot_contact_pts = robot_model.getBody(l_foot).getTerrainContactPoints();

        r_foot_contact_pts = forwardKin(robot_model,kinsol0,r_foot,r_foot_contact_pts,0);
        l_foot_contact_pts = forwardKin(robot_model,kinsol0,l_foot,l_foot_contact_pts,0);
        
        all_contact_pts = [l_foot_contact_pts, r_foot_contact_pts];
        hull = convhull( all_contact_pts(1, :), all_contact_pts(2, :));
        hull_pts = all_contact_pts(hull);
        
        com = robot_model.getCOM(current_robot_pose);
        opt_com = mean(all_contact_pts);
        

        clf
        hold on
        scatter3(l_foot_contact_pts(1,:), l_foot_contact_pts(2, :), l_foot_contact_pts(3, :), 'MarkerFaceColor', [0.0, 0.0, 0.0]);
        scatter3(r_foot_contact_pts(1,:), r_foot_contact_pts(2, :), r_foot_contact_pts(3, :), 'MarkerFaceColor', [1.0, 0.0, 0.0]);
        scatter3(com(1), com(2), com(3), 'MarkerFaceColor', [0.0, 1.0, 0.0]);
        scatter3(com(1), com(2), 0, 'MarkerFaceColor',[0.0, 0.0, 1.0]); % CoM projected to the floor
        plot3(hull_pts(1,:), hull_pts(2,:), zeros(1, length(hull_pts(1,:)))); % convex hull of contact pts
        scatter3(opt_com(1), opt_com(2), opt_com(3), 'MarkerFaceColor',[1.0, 1.0, 1.0]);
        legend('left foot', 'right foot', 'CoM', 'CoM projection', 'convex hull', 'optimal CoM');
        axis([-0.5, 0.5, -0.5, 0.5]);
        hold off
        
        pause(0.1);
    end

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
