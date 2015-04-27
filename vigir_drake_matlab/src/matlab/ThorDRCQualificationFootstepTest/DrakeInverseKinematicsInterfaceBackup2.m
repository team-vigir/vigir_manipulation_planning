classdef DrakeInverseKinematicsInterface
    %DRAKEINVERSEKINEMATICSINTERFACE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = private, GetAccess = public)
        ik_request_subscriber
        ik_result_publisher
        
        trajectory_request_subscriber
        trajectory_result_publisher
        
        cartesian_trajectory_request_subscriber
        cartesian_trajectory_result_publisher
        
        robot_model
        robot_visualizer
        
        currrent_state
        
        do_self_collision_checks
    end
    
    methods
        function obj = DrakeInverseKinematicsInterface
            obj.do_self_collision_checks = false;
            
            % load message packages
            moveit_msgs;
            vigir_planning_msgs;  
            
            % init rosmatlab
            ros.init();
            ros.log('INFO', 'Initializing DrakeInverseKinematicsInterface...');
            
            if ( obj.do_self_collision_checks == false )
                ros.log('WARN', 'Beware: Self-collision checks are currently disabled!');
            end
            
            if(~checkDependency('snopt'))
                ros.log('ERROR', 'Need package "snopt" to run...');
                error('Need package "snopt" to run...');
            end
            
            % add vigir paths to ros package path (needed to load robot
            % model correctly)
            vigir_paths = getenv('ROS_VIGIR_PACKAGE_PATH');
            rosmatlab_paths = getenv('ROS_PACKAGE_PATH');
            setenv('ROS_PACKAGE_PATH', [rosmatlab_paths ':' vigir_paths]);
            
            % load robot from rosparam /robot_description
            ros.log('INFO', 'Loading robot model...');            
            obj.robot_model = RigidBodyManipulator();
            
            terrain = RigidBodyHeightMapTerrain([10, 1.11, 1.1, 0.9, 0.89, -10], [-10; 10], [0, 0, 0.2, 0.20, 0, 0; 0, 0, 0.2, 0.2, 0, 0], eye(4));
            
            obj.robot_model = obj.robot_model.setTerrain(terrain);            
            
            options.floating = 'rpy';      
            %options.floating = false;
            urdf_string = ros.param.get('/robot_description');
            srdf_string = ros.param.get('/robot_description_semantic');
            
            w = warning('off','Drake:RigidBodyManipulator:ReplacedCylinder');   
            warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
            %warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
            obj.robot_model = addRobotFromURDFString(obj.robot_model,urdf_string,[],[],options);
            warning(w);
            
            % setup collision groups from srdf
            if ( obj.do_self_collision_checks )
                obj.robot_model = loadCollisionFilterGroupsFromSRDFString(obj.robot_model,srdf_string);
            end
            
            % construct visualizer
            obj.robot_visualizer = obj.robot_model.constructVisualizer();
            
            % init IK publishers / subscribers
            obj.ik_result_publisher = ros.Publisher('/drake_planner/ik_result', 'vigir_planning_msgs/ResultDrakeIK', 1, false);
            obj.ik_request_subscriber = ros.Subscriber('/drake_planner/request_drake_ik','vigir_planning_msgs/RequestDrakeIK', 1);
            obj.ik_request_subscriber.addlistener('Callback', @obj.handleIKRequest);
            obj.ik_request_subscriber.start();
            
            % init IK trajectory publishers / subscribers
            obj.trajectory_result_publisher = ros.Publisher('/drake_planner/trajectory_result', 'vigir_planning_msgs/ResultDrakeTrajectory', 1, false);
            obj.trajectory_request_subscriber = ros.Subscriber('/drake_planner/request_drake_trajectory','vigir_planning_msgs/RequestDrakeTrajectory', 1);
            obj.trajectory_request_subscriber.addlistener('Callback', @obj.handleIKTrajectoryRequest);
            obj.trajectory_request_subscriber.start();
            
            % init IK cartesian trajectory publishers / subscribers
            obj.cartesian_trajectory_result_publisher = ros.Publisher('/drake_planner/cartesian_trajectory_result', 'vigir_planning_msgs/ResultDrakeCartesianTrajectory', 1, false);
            obj.cartesian_trajectory_request_subscriber = ros.Subscriber('/drake_planner/request_drake_cartesian_trajectory','vigir_planning_msgs/RequestDrakeCartesianTrajectory', 1);
            obj.cartesian_trajectory_request_subscriber.addlistener('Callback', @obj.handleIKCartesianTrajectoryRequest);
            obj.cartesian_trajectory_request_subscriber.start();
            
            
            obj.currrent_state = [];
            ros.log('INFO', 'DrakeKinematicsInterface: Initialization complete!');            
        end
        
        function handleIKRequest(obj, ~, event)
            ros.log('INFO', 'Received IK request...');
            tic

            % get current qs from message
            nq = obj.robot_model.getNumPositions();
            q0 = zeros(nq, 1);
            q0 = obj.messageQ2ModelQ(q0, event.message.robot_state);
            obj.robot_visualizer.draw(cputime, q0);
            
            
            % get floating base position from message
            world_virtual_joints = strfind(event.message.robot_state.multi_dof_joint_state.joint_names, 'world_virtual_joint');
            world_virtual_joints_idx = find(~cellfun(@isempty,world_virtual_joints));
            if ( length(world_virtual_joints_idx) == 1 )
                joint_state = event.message.robot_state.multi_dof_joint_state(world_virtual_joints_idx);
                translation_vec = [joint_state.transforms.translation.x; joint_state.transforms.translation.y; joint_state.transforms.translation.z];
                q0(1:3) = translation_vec;                
                
                rotation_quat = [joint_state.transforms.rotation.w, joint_state.transforms.rotation.x, joint_state.transforms.rotation.y, joint_state.transforms.rotation.z];
                rotation_rpy = quat2rpy(rotation_quat);
                q0(4:6) = rotation_rpy;
            else
                ros.log('WARN', 'Did not receive unique world joint position...');                
            end
            
            q_seed = q0;
            q_nom = [q0(1:6); zeros(nq-6,1)];
            
            % show start pose            
            obj.robot_visualizer.draw(cputime, q0);
               
            % build list of constraints from message
            activeConstraints = buildIKConstraints(obj.robot_model, event.message.target_poses, event.message.target_link_names, q0);
                        
            % run inverse kinematics (mex)
            ikoptions = initIKOptions(obj.robot_model);            
            [qik,info_mex,infeasible_constraints] = inverseKin(obj.robot_model,q_seed,q_nom,activeConstraints{:},ikoptions);
            
            % visualize result
            obj.robot_visualizer.draw(cputime, qik);
            
            if(info_mex>10) % something went wrong
                str = sprintf('SNOPT info is %d, IK mex fails to solve the problem',info_mex);
                ros.log('WARN', str);
                ros.log('INFO', 'Infeasible Constraints:');
                
                str = sprintf('%s |  ', infeasible_constraints{:});
                ros.log('INFO', str);
                
                % return starting pose in an invalid message
                result_message = vigir_planning_msgs.ResultDrakeIK;
                result_message.result_state = event.message.robot_state;
                
                result_message.is_valid = 0;
            else
                % build result message form q values
                result_message = vigir_planning_msgs.ResultDrakeIK;
                result_message.result_state = event.message.robot_state;
                
                % map message robotModel qs to message order
                result_qs = obj.modelQ2MessageQ(qik, event.message.robot_state);
                result_message.result_state.joint_state.position = result_qs;
                
                % extract world position and orientation                
                result_translation = qik(1:3);
                result_rotation = rpy2quat(qik(4:6));

                result_message.result_state.multi_dof_joint_state(world_virtual_joints_idx).transforms.translation.x = result_translation(1);
                result_message.result_state.multi_dof_joint_state(world_virtual_joints_idx).transforms.translation.y = result_translation(2);
                result_message.result_state.multi_dof_joint_state(world_virtual_joints_idx).transforms.translation.z = result_translation(3);
                result_message.result_state.multi_dof_joint_state(world_virtual_joints_idx).transforms.rotation.w = result_rotation(1);
                result_message.result_state.multi_dof_joint_state(world_virtual_joints_idx).transforms.rotation.x = result_rotation(2);
                result_message.result_state.multi_dof_joint_state(world_virtual_joints_idx).transforms.rotation.y = result_rotation(3);
                result_message.result_state.multi_dof_joint_state(world_virtual_joints_idx).transforms.rotation.z = result_rotation(4);
                
                result_message.is_valid = 1;
            end
            
            % publish result message
            used_time = toc;
            str = sprintf('Sending response (calculation took %f seconds', used_time);
            ros.log('INFO', str);
            obj.ik_result_publisher.publish(result_message);
        end
        
        function handleIKTrajectoryRequest(obj, ~, event)
            ros.log('INFO', 'Received IK trajectory request');
            tic
            
            % start qs from request message current state and potential
            % updates (motion_plan_request.start_state)
            nq = obj.robot_model.getNumPositions();
            q0 = zeros(nq, 1);
            q0 = obj.messageQ2ModelQ(q0, event.message.current_state);
            q0 = obj.messageQ2ModelQ(q0, event.message.motion_plan_request.start_state);
            
            % get floating base position from message
            world_virtual_joints = strfind(event.message.current_state.multi_dof_joint_state.joint_names, 'world_virtual_joint');
            world_virtual_joints_idx = find(~cellfun(@isempty,world_virtual_joints));
            if ( length(world_virtual_joints_idx) == 1 )
                joint_state = event.message.current_state.multi_dof_joint_state(world_virtual_joints_idx);
                translation_vec = [joint_state.transforms.translation.x; joint_state.transforms.translation.y; joint_state.transforms.translation.z];
                q0(1:3) = translation_vec;                
                
                rotation_quat = [joint_state.transforms.rotation.w, joint_state.transforms.rotation.x, joint_state.transforms.rotation.y, joint_state.transforms.rotation.z];
                rotation_rpy = quat2rpy(rotation_quat);
                q0(4:6) = rotation_rpy;
            else
                ros.log('WARN', 'Did not receive unique world joint position...');                
            end
            
            % get trajectory duration infos and selected time steps
            duration = event.message.duration;
            num_time_steps = event.message.num_time_steps;
            t = linspace(0,duration,num_time_steps);
            
            % stay at q0 for nominal trajectory
            q_lin = interp1([0, duration], [q0, q0]', t)';
            q_nom_traj = PPTrajectory(foh(t,q_lin));
            q_seed_traj = q_nom_traj;
            
            % build list of constraints from message
            activeConstraints = buildIKTrajectoryConstraints(obj.robot_model, event.message.motion_plan_request.goal_constraints.joint_constraints, q0, t);
            
            % run inverse kinematics (mex)
            ikoptions = initIKTrajectoryOptions(obj.robot_model);
            [trajectory,info_mex,infeasible_constraints] = inverseKinTraj(obj.robot_model, t, q_seed_traj, q_nom_traj, activeConstraints{:},ikoptions);
            
            % visualize result
            obj.robot_visualizer.playback(trajectory,struct('slider',true));
            
            % TODO: handle failed trajectories... allow all through for now
            if(info_mex>10) % something went wrong
                 warning('SNOPT info is %d, IK mex fails to solve the problem',info_mex);
                 disp('Infeasible constraints:');
                 disp(infeasible_constraints);
                 
                 % return invalid message
                 result_message = vigir_planning_msgs.ResultDrakeTrajectory;
                 result_message.is_valid = 0;
             else
                % get q and qdot values from trajectory
                qqdot_values = trajectory.eval(t);
                
                message_qs = obj.modelQ2MessageQ(qqdot_values(1:nq, :), event.message.current_state);
                message_qds = obj.modelQ2MessageQ(qqdot_values(nq+1:2*nq, :), event.message.current_state);
                
                % build result message from trajectory
                result_message = vigir_planning_msgs.ResultDrakeTrajectory;
                result_message.result_trajectory = moveit_msgs.RobotTrajectory;
                result_message.result_trajectory.joint_trajectory = trajectory_msgs.JointTrajectory;
                
                result_message.result_trajectory.joint_trajectory.joint_names = event.message.current_state.joint_state.name;
                trajectory_points = repmat(trajectory_msgs.JointTrajectoryPoint, length(t),1);
                for i = 1:num_time_steps
                    trajectory_points(i,1).positions = message_qs(i,:);
                    trajectory_points(i,1).velocities = message_qds(i,:);
                    trajectory_points(i,1).time_from_start = t(i);
                end
                result_message.result_trajectory.joint_trajectory.points = trajectory_points;

                % add world position and velocities
                result_message.result_trajectory.multi_dof_joint_trajectory = trajectory_msgs.MultiDOFJointTrajectory;
                result_message.result_trajectory.multi_dof_joint_trajectory.joint_names = 'world_virtual_joint';
                multi_dof_trajectory_points = repmat(trajectory_msgs.MultiDOFJointTrajectoryPoint, length(t),1);
                for i = 1:num_time_steps
                    % extract world position and orientation transforms                
                    result_translation = qqdot_values(1:3, i);
                    result_rotation = rpy2quat( qqdot_values(4:6, i));

                    multi_dof_trajectory_points(i, 1).transforms(1).translation.x = result_translation(1);
                    multi_dof_trajectory_points(i, 1).transforms(1).translation.y = result_translation(2);
                    multi_dof_trajectory_points(i, 1).transforms(1).translation.z = result_translation(3);
                    multi_dof_trajectory_points(i, 1).transforms(1).rotation.w = result_rotation(1);
                    multi_dof_trajectory_points(i, 1).transforms(1).rotation.x = result_rotation(2);
                    multi_dof_trajectory_points(i, 1).transforms(1).rotation.y = result_rotation(3);
                    multi_dof_trajectory_points(i, 1).transforms(1).rotation.z = result_rotation(4);
                    
                    % extract world position and orientation velocities
                    result_linear_vel = qqdot_values(nq+1:nq+3, i);
                    result_angular_vel = qqdot_values(nq+4:nq+6, i);
                    
                    multi_dof_trajectory_points(i, 1).velocities(1).linear.x = result_linear_vel(1);
                    multi_dof_trajectory_points(i, 1).velocities(1).linear.y = result_linear_vel(2);
                    multi_dof_trajectory_points(i, 1).velocities(1).linear.z = result_linear_vel(3);                    
                    multi_dof_trajectory_points(i, 1).velocities(1).angular.x = result_angular_vel(1);
                    multi_dof_trajectory_points(i, 1).velocities(1).angular.y = result_angular_vel(2);
                    multi_dof_trajectory_points(i, 1).velocities(1).angular.z = result_angular_vel(3);
                    
                    % extract timing
                    multi_dof_trajectory_points(i,1).time_from_start = t(i);
                end
                result_message.result_trajectory.multi_dof_joint_trajectory.points = multi_dof_trajectory_points;
                
                result_message.is_valid = 1;
            end
            
            % publish result message
            used_time = toc;
            str = sprintf('Sending response (calculation took %f seconds', used_time);
            ros.log('INFO', str);
            obj.trajectory_result_publisher.publish(result_message);
            
        end
        
        function handleIKCartesianTrajectoryRequest(obj, ~, event)
            ros.log('INFO', 'Received IK cartesian trajectory request');
            tic
                                   
            % start qs from request message current state and potential
            % updates (motion_plan_request.start_state)
            nq = obj.robot_model.getNumPositions();
            q0 = zeros(nq, 1);
            q0 = obj.messageQ2ModelQ(q0, event.message.current_state);
           
            % get floating base position from message
            world_virtual_joints = strfind(event.message.current_state.multi_dof_joint_state.joint_names, 'world_virtual_joint');
            world_virtual_joints_idx = find(~cellfun(@isempty,world_virtual_joints));
            if ( length(world_virtual_joints_idx) == 1 )
                joint_state = event.message.current_state.multi_dof_joint_state(world_virtual_joints_idx);
                translation_vec = [joint_state.transforms.translation.x; joint_state.transforms.translation.y; joint_state.transforms.translation.z];
                q0(1:3) = translation_vec;                
                
                rotation_quat = [joint_state.transforms.rotation.w, joint_state.transforms.rotation.x, joint_state.transforms.rotation.y, joint_state.transforms.rotation.z];
                rotation_rpy = quat2rpy(rotation_quat);
                q0(4:6) = rotation_rpy;
            else
                ros.log('WARN', 'Did not receive unique world joint position...');                
            end
            
            obj.robot_visualizer = obj.robot_model.constructVisualizer();
            obj.robot_visualizer.draw(cputime, q0);
            
            % calculate trajectory
            %[trajectory, info_mex, infeasible_constraints] = calcIKCartesianTrajectory(obj.robot_model, q0, event.message, obj.do_self_collision_checks);
            %[trajectory, info_mex, infeasible_constraints] = calcIKCartesianTrajectoryParts(obj.robot_model, q0, event.message, obj.do_self_collision_checks);
                    
 %       current_position = l_foot_pos(1:3);
 %   current_orientation = l_foot_pos(4:7);
                            
            num_steps = 2;
            t = 0:1:25; % time points at which the final trajectory will be sampled
            
            %world_quat = [1, 0, 0, 0];
            world_quat = [0.99692, 0.0003, 0.0784, -1.98149*10^(-5)];
            world_quat = world_quat/norm(world_quat);
            
            world_mat = eye(4);
            
            world_mat(1:3, 1:3) = quat2rotmat(world_quat');            
            world_rpy = quat2rpy(world_quat);
            q0(4:6) = world_rpy;

            % shift weight 
            %waypoint_times_step{1} = [5 5.1];
            %waypoint.position = obj.setRelativeTargetPosition(0, 0, 0, world_mat);
            %waypoints_step{1} = waypoint;
            %waypoint.position = obj.setRelativeTargetPosition(0, 0, 0, world_mat);
            %waypoints_step{1} = [waypoints_step{1} waypoint];
            %target_link_name_step{1} = 'pelvis';
            
            % first step
            waypoint_times_step{1} = [10 15];
            waypoint.position = obj.setRelativeTargetPosition(0, 0, 0.30, world_mat);
            waypoints_step{1} = waypoint;
            waypoint.position = obj.setRelativeTargetPosition(0, 0.3, 0.21, world_mat);
            waypoints_step{1} = [waypoints_step{1} waypoint];
            target_link_name_step{1} = 'r_foot';
           
            % second step
            waypoint_times_step{2} = [20 25];
            waypoint.position = obj.setRelativeTargetPosition(0, 0, 0.30, world_mat);
            waypoints_step{2} = waypoint;
            waypoint.position = obj.setRelativeTargetPosition(0, 0.3, 0.21, world_mat);
            waypoints_step{2} = [waypoints_step{2} waypoint];
            target_link_name_step{2} = 'l_foot';  
            
            % third step
            waypoint_times_step{3} = [25 30];
            waypoint.position = obj.setRelativeTargetPosition(0, 0, 0.30, world_mat);
            waypoints_step{3} = waypoint;
            waypoint.position = obj.setRelativeTargetPosition(0, 0.6, 0, world_mat);
            waypoints_step{3} = [waypoints_step{3} waypoint];
            target_link_name_step{3} = 'r_foot';
            
            % fourth step
            waypoint_times_step{4} = [35 40];
            waypoint.position = obj.setRelativeTargetPosition(0, 0, 0.30, world_mat);
            waypoints_step{4} = waypoint;
            waypoint.position = obj.setRelativeTargetPosition(0, 0.6, 0, world_mat);
            waypoints_step{4} = [waypoints_step{4} waypoint];
            target_link_name_step{4} = 'l_foot';

            result_trajectory = [];
            start_time = 0;
            for step = 1:num_steps
                event.message.waypoint_times = waypoint_times_step{step};
                event.message.waypoints = waypoints_step{step};
                event.message.target_link_name = target_link_name_step{step};
                
                if ( step > 1 ) % keep current q0
                    start_time = waypoint_times_step{step-1}(end);
                    q0 = qqdot_values(1:nq, end);
                end


                [trajectory, info_mex, infeasible_constraints] = calcIKCartesianFeetTrajectory(obj.robot_model, start_time, q0, event.message, obj.do_self_collision_checks);
                if ( isempty(result_trajectory) )
                    result_trajectory = trajectory;
                else
                    result_trajectory = result_trajectory.append(trajectory);
                end   
                
                qqdot_values = trajectory.eval(waypoint_times_step{step}(end));
            end
            
             % visualize result
             obj.robot_visualizer.playback(result_trajectory,struct('slider',true));
             
             % TODO: handle failed trajectories... allow all through for now
             if(info_mex>10) % something went wrong
                 warning('SNOPT info is %d, IK mex fails to solve the problem',info_mex);
                 disp('Infeasible constraints:');
                 disp(infeasible_constraints);
                 
                 % return invalid message
                 result_message.result_trajectory = vigir_planning_msgs.ResultDrakeTrajectory;
                 result_message.is_valid = 0;
             else
                 num_time_steps = length(t);
                 
                 % get q and qdot values from trajectory
                 qqdot_values = result_trajectory.eval(t);
                 
                 message_qs = obj.modelQ2MessageQ(qqdot_values(1:nq, :), event.message.current_state);
                 message_qds = obj.modelQ2MessageQ(qqdot_values(nq+1:2*nq, :), event.message.current_state);
                 
                 % build result message from trajectory
                 result_message = vigir_planning_msgs.ResultDrakeCartesianTrajectory;
                 result_message.result_trajectory = moveit_msgs.RobotTrajectory;
                 result_message.result_trajectory.joint_trajectory = trajectory_msgs.JointTrajectory;
                 
                 result_message.result_trajectory.joint_trajectory.joint_names = event.message.current_state.joint_state.name(1:30);
                 trajectory_points = repmat(trajectory_msgs.JointTrajectoryPoint, length(t),1);
                 for i = 1:num_time_steps
                     trajectory_points(i,1).positions = message_qs(i,1:30);
                     trajectory_points(i,1).velocities = message_qds(i,1:30);
                     trajectory_points(i,1).time_from_start = t(i);
                 end
                 result_message.result_trajectory.joint_trajectory.points = trajectory_points;
                 result_message.is_valid = 1;
             end
             
             % publish result message
             used_time = toc;
             str = sprintf('Sending response (calculation took %f seconds', used_time);
             ros.log('INFO', str);
             %obj.cartesian_trajectory_result_publisher.publish(result_message);
            
        end
        
    end
    
    methods(Access = private)
        % take the robot state from a request message and extract the joint
        % positions in the order they are used by the robot model here;
        % q0 sets the default position if the joint is not mentioned
        function model_qs = messageQ2ModelQ(obj, q0, robot_state)
            message_joint_names = robot_state.joint_state.name;
            message_qs = robot_state.joint_state.position;
                        
            model_qs = q0;
            for i = 1:length(message_qs)
                current_name = message_joint_names{i};
                current_q = message_qs(i);
                
                body_idx = obj.robot_model.findJointId(current_name);
                model_q_idx = obj.robot_model.getBody(body_idx).position_num;
                model_qs(model_q_idx) = current_q;
            end
        end
        
        % take qs in the order of the currently loaded model and map them to
        % the order used for communicating with ROS (taken from the robot
        % state of the request message
        function message_qs = modelQ2MessageQ(obj, model_qs, request_state)
            message_joint_names = request_state.joint_state.name;            
            num_message_qs = length(message_joint_names);
            num_states = size(model_qs, 2);
            message_qs = zeros(num_states,num_message_qs);
            for i = 1:num_message_qs
                current_name = message_joint_names{i};
                
                body_idx = obj.robot_model.findJointId(current_name);
                model_q_idx = obj.robot_model.getBody(body_idx).position_num;
                message_qs(:,i) = model_qs(model_q_idx,:)';
            end
        end
        
        function position = setRelativeTargetPosition(obj, diff_left, diff_front, diff_height, mat) 
            pos_vec = [diff_front, diff_left, diff_height, 1]';
            res_vec = mat*pos_vec;
            position.x = res_vec(1);
            position.y = res_vec(2);
            position.z = res_vec(3);
        end
    end

end








