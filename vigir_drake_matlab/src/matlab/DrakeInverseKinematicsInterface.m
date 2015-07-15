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
    end
    
    methods
        function obj = DrakeInverseKinematicsInterface
            addpath('/usr/local/MATLAB/R2014a/ros/indigo/matlab');
            % load message packages
            moveit_msgs;
            vigir_planning_msgs;
            
            % init rosmatlab
            ros.init();
            ros.log('INFO', 'Initializing DrakeInverseKinematicsInterface...');
            
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
            
            options.floating = 'rpy';
            %options.floating = false;
            urdf_string = ros.param.get('/robot_description');
            %srdf_string = ros.param.get('/robot_description_semantic');
            
            w = warning('off','Drake:RigidBodyManipulator:ReplacedCylinder');
            warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
            %warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
            obj.robot_model = addRobotFromURDFString(obj.robot_model,urdf_string,[],[],options);
            warning(w);
            
            % setup collision groups from srdf
            %obj.robot_model = loadCollisionFilterGroupsFromSRDFString(obj.robot_model,srdf_string);
            
            % construct visualizer
            if ( strcmpi(getenv('SHOW_DRAKE_VISUALIZATION'), 'TRUE' ) )
                obj.robot_visualizer = obj.robot_model.constructVisualizer();
            else
                obj.robot_visualizer = [];
            end
            
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
            obj.cartesian_trajectory_result_publisher = ros.Publisher('/drake_planner/cartesian_trajectory_result', 'vigir_planning_msgs/ResultDrakeTrajectory', 1, false);
            obj.cartesian_trajectory_request_subscriber = ros.Subscriber('/drake_planner/request_drake_cartesian_trajectory','vigir_planning_msgs/RequestDrakeCartesianTrajectory', 1);
            obj.cartesian_trajectory_request_subscriber.addlistener('Callback', @obj.handleIKCartesianTrajectoryRequest);
            obj.cartesian_trajectory_request_subscriber.start();
            
            ros.log('INFO', 'DrakeKinematicsInterface: Initialization complete!');
        end
        
        function handleIKRequest(obj, ~, event)
            ros.log('INFO', 'Received IK request...');
            tic
            
            % get current qs from message
            nq = obj.robot_model.getNumPositions();
            q0 = zeros(nq, 1);
            q0 = obj.messageQ2ModelQ(q0, event.message.robot_state);
            
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
            
            %disp('q0 = ');
            %obj.printSortedQs(q0);
            
            [ posture, success ] = calcIKPosture( obj.robot_visualizer, obj.robot_model, q0, event.message );
            
            if(success) % all is well
                %disp('q_sol = ');
                %obj.printSortedQs(posture);

                % build result message form q values
                result_message = vigir_planning_msgs.ResultDrakeIK;
                result_message.result_state = event.message.robot_state;
                
                % map message robotModel qs to message order
                result_qs = obj.modelQ2MessageQ(posture, event.message.robot_state);
                result_message.result_state.joint_state.position = result_qs;
                
                % extract world position and orientation
                result_translation = posture(1:3);
                result_rotation = rpy2quat(posture(4:6));
                
                result_message.result_state.multi_dof_joint_state(world_virtual_joints_idx).transforms.translation.x = result_translation(1);
                result_message.result_state.multi_dof_joint_state(world_virtual_joints_idx).transforms.translation.y = result_translation(2);
                result_message.result_state.multi_dof_joint_state(world_virtual_joints_idx).transforms.translation.z = result_translation(3);
                result_message.result_state.multi_dof_joint_state(world_virtual_joints_idx).transforms.rotation.w = result_rotation(1);
                result_message.result_state.multi_dof_joint_state(world_virtual_joints_idx).transforms.rotation.x = result_rotation(2);
                result_message.result_state.multi_dof_joint_state(world_virtual_joints_idx).transforms.rotation.y = result_rotation(3);
                result_message.result_state.multi_dof_joint_state(world_virtual_joints_idx).transforms.rotation.z = result_rotation(4);
                
                result_message.is_valid = 1;
            else
                % return starting pose in an invalid message
                result_message = vigir_planning_msgs.ResultDrakeIK;
                result_message.result_state = event.message.robot_state;
                
                result_message.is_valid = 0;
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
                send_world_joint = true;
            else
                ros.log('WARN', 'Did not receive unique world joint position...');
                send_world_joint = false;
            end
            
            if ( event.message.duration <= 0 ) % set default duration, if it is not set correctly
                event.message.duration = 5;
            end
            
            [trajectory, success] = calcIKTrajectory(obj.robot_visualizer, obj.robot_model, q0, event.message);
            
            if(success) % all is right
                % calculate time points for trajectory evaluation
                if ( event.message.trajectory_sample_rate == 0.0 )
                    event.message.trajectory_sample_rate = 4.0;
                end
                
                time_steps = 1/event.message.trajectory_sample_rate;                    
                t = 0:time_steps:event.message.duration;
                if ( t(end) < event.message.duration )
                    t(end+1) = event.message.duration;
                end
                
                result_message = obj.buildTrajectoryResultMessage(trajectory, t, send_world_joint, {event.message.motion_plan_request.goal_constraints.joint_constraints.joint_name});
            else
                % return invalid message
                result_message = vigir_planning_msgs.ResultDrakeTrajectory;
                result_message.is_valid = 0;
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
            
            request = event.message;
            
            % start qs from request message current state and potential
            % updates (motion_plan_request.start_state)
            nq = obj.robot_model.getNumPositions();
            q0 = zeros(nq, 1);
            q0 = obj.messageQ2ModelQ(q0, event.message.current_state);
                        
            % get floating base position from message
            world_virtual_joints = strfind(event.message.current_state.multi_dof_joint_state.joint_names, 'world_virtual_joint');
            world_virtual_joints_idx = find(~cellfun(@isempty,world_virtual_joints));
            if ( length(world_virtual_joints_idx) == 1 )                
                transform = event.message.current_state.multi_dof_joint_state.transforms(world_virtual_joints_idx);
                translation_vec = [transform.translation.x; transform.translation.y; transform.translation.z];
                q0(1:3) = translation_vec;
                
                rotation_quat = [transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z];
                rotation_rpy = quat2rpy(rotation_quat);
                q0(4:6) = rotation_rpy;
                
                send_world_joint = true;
            else
                ros.log('WARN', 'Did not receive unique world joint position...');
                send_world_joint = false;
            end
            
            %disp( 'q0 = ' );
            %obj.printSortedQs(q0);
            
            % calculate trajectory            
            [trajectory, success] = calcIKCartesianTrajectory(obj.robot_visualizer, obj.robot_model, q0, event.message);
            
            if(success) % if everything is okay, send trajectory message
                % calculate time points for trajectory evaluation
                if ( event.message.trajectory_sample_rate == 0.0 )
                    event.message.trajectory_sample_rate = 4.0;
                end
                
                time_steps = 1/event.message.trajectory_sample_rate;
                t = 0:time_steps:request.waypoint_times(end);
                
                result_message = obj.buildTrajectoryResultMessage(trajectory, t, send_world_joint, event.message.free_joint_names);
                
            else
                % return invalid message
                result_message = vigir_planning_msgs.ResultDrakeTrajectory;
                result_message.is_valid = 0;
                
            end
            
            % publish result message
            used_time = toc;
                        
            str = sprintf('Sending response (calculation took %f seconds', used_time);
            ros.log('INFO', str);
            obj.cartesian_trajectory_result_publisher.publish(result_message);
                        
            
            
        end
       
        function visualizer = getVisualizer(obj)
            visualizer = obj.robot_visualizer;
        end
        
        function robot_model = getRobotModel(obj)
            robot_model = obj.robot_model;
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
        
        function result_message = buildTrajectoryResultMessage(obj, trajectory, t, send_world_joint, joint_names)
            % get q and qdot values from trajectory
            qqdot_values = trajectory.eval(t);
            
            nq = obj.robot_model.getNumPositions();
            qs = qqdot_values(1:nq, :);
            qds = qqdot_values(nq+1:2*nq, :);
            
            %disp('Result Qs: ');
            %obj.printSortedQs(qs);
            
            %disp('Result QDs: ');
            %obj.printSortedQs(qds);
            
            % build result message from trajectory
            result_message = vigir_planning_msgs.ResultDrakeTrajectory;
            result_message.result_trajectory = moveit_msgs.RobotTrajectory;
            result_message.result_trajectory.joint_trajectory = trajectory_msgs.JointTrajectory;
            
            result_message.result_trajectory.joint_trajectory.joint_names = joint_names;
            
            [found_bodies, found_idx] = ismember({obj.robot_model.body.jointname}, joint_names);
            result_message.result_trajectory.joint_trajectory.joint_names = joint_names(found_idx(found_bodies));
            joint_idx = cell2mat({obj.robot_model.body(found_bodies).position_num});
            trajectory_points = repmat(trajectory_msgs.JointTrajectoryPoint, length(t),1);
            for i = 1:length(t)
                
                trajectory_points(i,1).positions = qs(joint_idx,i)';
                trajectory_points(i,1).velocities = qds(joint_idx,i)';
                trajectory_points(i,1).time_from_start = t(i);
            end
            result_message.result_trajectory.joint_trajectory.points = trajectory_points;
            
            % add world position and velocities
            if ( send_world_joint )
                result_message.result_trajectory.multi_dof_joint_trajectory = trajectory_msgs.MultiDOFJointTrajectory;
                result_message.result_trajectory.multi_dof_joint_trajectory.joint_names = 'world_virtual_joint';
                multi_dof_trajectory_points = repmat(trajectory_msgs.MultiDOFJointTrajectoryPoint, length(t),1);
                for i = 1:length(t)
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
            end
            
            result_message.is_valid = 1;
        end
        
            
        function printSortedQs(obj, qs)
            for i = 1:length(obj.robot_model.body)
                pos = obj.robot_model.body(i).position_num;
                                
                if ( any(pos < 1) || any(pos > length(qs)) )
                    disp( [ obj.robot_model.body(i).jointname '   no matching q for position_num = ' num2str(pos) ] );
                else
                    if ( length(pos) > 1 )
                        disp( obj.robot_model.body(i).jointname )
                        disp( num2str( qs(pos, :) ));
                        disp( '================');
                    else
                        disp( [ obj.robot_model.body(i).jointname '   ' num2str( qs(pos, :) )] );
                    end
                end
            end
        end
    end
end








