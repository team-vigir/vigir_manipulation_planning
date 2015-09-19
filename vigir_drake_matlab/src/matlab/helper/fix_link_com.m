function fix_link_com(link_name)   
    if ( nargin == 0 )
        link_name = 'utorso';
        warning(['No link name specified, using ' link_name]);
    end

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

    % get current robot position and visualize
    joint_state_sub = ros.Subscriber('/thor_mang/joint_states','sensor_msgs/JointState', 1);
    imu_sub = ros.Subscriber('/thor_mang/pelvis_imu', 'sensor_msgs/Imu', 1);

    [joint_state_msg, ~, ~] = joint_state_sub.poll(10);
    [imu_msg, ~, ~] = imu_sub.poll(10);

    orientation_quat = [imu_msg.orientation.w;imu_msg.orientation.x;imu_msg.orientation.y;imu_msg.orientation.z];
    orientation_rpy = quat2rpy(orientation_quat);
    current_robot_pose(4:6) = orientation_rpy;

    current_robot_pose = handle_new_joint_state(joint_state_msg, robot_model, current_robot_pose);
    
    robot_visualizer.draw(cputime, current_robot_pose);
    
    % display current state
    figure    
    plot_robot_com(robot_model, current_robot_pose, 'current CoM state');
    
    % get current chest values
    body_idx = robot_model.findLinkId(link_name);
    body = robot_model.getBody(body_idx);
    
    body_com = body.com;
    body_mass = body.mass;
    body_inertia = body.inertia;
    
    % change chest com to fit    
    figure
    title('new CoM state');
    
    while ( true )
        com_delta = input('Enter CoM delta (or nothing to quit): ');
        if ( isempty(com_delta) )
            break;
        end
        
        old_body_com = body_com;
        if ( size(com_delta, 1) == 1 )
            body_com(1) = body_com(1) + com_delta;
        elseif ( all( size(com_delta) == [3, 1] ))
            body_com = body_com + com_delta;
        else
            disp('Invalid delta value: enter "dx" or "[dx; dy; dz]"');
            continue;
        end            

        body = body.setInertial(body_mass, body_com, body_inertia);
        robot_model = robot_model.setBody(body_idx, body);
        robot_model = robot_model.compile();

        % display changes        
        plot_robot_com(robot_model, current_robot_pose, 'new CoM state');

        % display final com
        disp(['new body CoM: ' num2str(body_com') '  (old was: ' num2str(old_body_com') ')']);                    
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
