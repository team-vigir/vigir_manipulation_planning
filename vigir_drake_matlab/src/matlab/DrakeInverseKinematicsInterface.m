classdef DrakeInverseKinematicsInterface < handle
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
        
        robot_nominal_pose
        
        % gravity compensation stuff
        do_gravity_compensation
        gravity_compensation_factor
        
        % pose correction stuff
        do_current_pose_error_compensation
        controller_state_subscribers
        
        % for Cartesian trajectories: do final global optimization
        do_cartesian_global_optimization
    end
    
    methods
        function obj = DrakeInverseKinematicsInterface
            addpath('/usr/local/MATLAB/R2014a/ros/indigo/matlab');
            % load message packages
            moveit_msgs;
            control_msgs;
            vigir_planning_msgs;
            
            % init rosmatlab
            ros.init();
            ros.log('INFO', '[DrakeInverseKinematicsInterface] Initializing...');
            
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
            
            % load robot from rosparam /robot_description
            ros.log('INFO', '[DrakeInverseKinematicsInterface] Loading robot model...');
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
            
            % load nominal pose from param server
            pose_str = ros.param.get('/drake_nominal_pose');
            obj.robot_nominal_pose = sscanf(pose_str, '%f, ');
            if ( length(obj.robot_nominal_pose) ~= obj.robot_model.num_positions)
                ros.log('WARN', '[DrakeInverseKinematicsInterface] Nominal pose does not match number of robot joints. Setting to 0');
                obj.robot_nominal_pose = zeros(obj.robot_model.num_positions, 1);
            end
            
            % construct visualizer
            show_visualization = ros.param.get('/drake_show_visualization');
            if ( show_visualization )
                ros.log('INFO', '[DrakeInverseKinematicsInterface] Initializing visualizer');
                obj.robot_visualizer = obj.robot_model.constructVisualizer();
                obj.robot_visualizer.draw(cputime, obj.robot_nominal_pose);
            else
                obj.robot_visualizer = [];
            end        
            
            obj.do_gravity_compensation = ros.param.get('/drake_do_gravity_compensation');
            if ( isempty(obj.do_gravity_compensation ))
                obj.do_gravity_compensation = false;
            end
            
            obj.gravity_compensation_factor = ros.param.get('/drake_gravity_compensation_factor');
            if ( isempty(obj.gravity_compensation_factor ))
                obj.gravity_compensation_factor = 0.0032;
            end
            
            obj.do_current_pose_error_compensation = ros.param.get('/drake_do_current_pose_error_compensation');
            if ( isempty(obj.gravity_compensation_factor ))
                obj.do_current_pose_error_compensation = false;
            end
            
            controller_topics = ros.param.get('/drake_controller_state_topics');
            obj.controller_state_subscribers = {};
            for i = 1:length(controller_topics)
                obj.controller_state_subscribers{end+1} = ros.Subscriber(controller_topics{i}, 'control_msgs/JointTrajectoryControllerState', 1);
            end
            
            obj.do_cartesian_global_optimization = ros.param.get('/drake_do_cartesian_global_optimization');
            if ( isempty( obj.do_cartesian_global_optimization )
                obj.do_cartesian_global_optimization = false;
            end
            
            
            obj.reset_ros_interface();
                
            ros.log('INFO', '[DrakeInverseKinematicsInterface] Initialization complete!');
        end
             
        function handleIKRequest(obj, ~, event)
            ros.log('INFO', '[DrakeInverseKinematicsInterface] Received IK request...');
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
                ros.log('WARN', '[DrakeInverseKinematicsInterface] Did not receive unique world joint position...');
            end
            
            [ posture, success ] = calcIKPosture( obj.robot_visualizer, obj.robot_model, obj.robot_nominal_pose, q0, event.message );
            
            if(success) % all is well
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
            ros.log('INFO', sprintf('[DrakeInverseKinematicsInterface] Sending response (calculation took %f seconds', used_time));
            obj.ik_result_publisher.publish(result_message);
        end
        
        function handleIKTrajectoryRequest(obj, ~, event)
            ros.log('INFO', '[DrakeInverseKinematicsInterface] Received IK trajectory request');
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
                ros.log('WARN', '[DrakeInverseKinematicsInterface] Did not receive unique world joint position...');
                send_world_joint = false;
            end
            
            if ( event.message.duration <= 0 ) % set default duration, if it is not set correctly
                event.message.duration = 10;
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
            ros.log('INFO', sprintf('[DrakeInverseKinematicsInterface] Sending response (calculation took %f seconds', used_time));
            obj.trajectory_result_publisher.publish(result_message);
            
        end
        
        function handleIKCartesianTrajectoryRequest(obj, ~, event)
            ros.log('INFO', '[DrakeInverseKinematicsInterface] Received IK cartesian trajectory request');
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
                transform = event.message.current_state.multi_dof_joint_state.transforms(world_virtual_joints_idx);
                translation_vec = [transform.translation.x; transform.translation.y; transform.translation.z];
                q0(1:3) = translation_vec;
                
                rotation_quat = [transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z];
                rotation_rpy = quat2rpy(rotation_quat);
                q0(4:6) = rotation_rpy;
                
                send_world_joint = true;
            else
                ros.log('WARN', '[DrakeInverseKinematicsInterface] Did not receive unique world joint position...');
                send_world_joint = false;
            end
            
            % calculate trajectory            
            [trajectory, success, request] = calcIKCartesianTrajectory(obj.robot_visualizer, obj.robot_model, q0, event.message, obj.do_cartesian_global_optimization);
            
            if(success) % if everything is okay, send trajectory message
                % calculate time points for trajectory evaluation
                if ( request.trajectory_sample_rate == 0.0 )
                    request.trajectory_sample_rate = 4.0;
                end
                
                time_steps = 1/request.trajectory_sample_rate;
                t = 0:time_steps:request.waypoint_times(end);
                
                if ( t(end) < request.waypoint_times(end) )
                    t = [t, request.waypoint_times(end)];
                end
                
                result_message = obj.buildTrajectoryResultMessage(trajectory, t, send_world_joint, event.message.free_joint_names);
                
            else
                % return invalid message
                result_message = vigir_planning_msgs.ResultDrakeTrajectory;
                result_message.is_valid = 0;
                
            end
            
            % publish result message
            used_time = toc;            
            ros.log('INFO', sprintf('[DrakeInverseKinematicsInterface] Sending response (calculation took %f seconds', used_time));
            obj.cartesian_trajectory_result_publisher.publish(result_message);
        end
       
        function visualizer = getVisualizer(obj)
            visualizer = obj.robot_visualizer;
        end
        
        function robot_model = getRobotModel(obj)
            robot_model = obj.robot_model;
        end     
        
        function update_robot_model(obj, new_robot_model)
            obj.robot_model = new_robot_model;                        
            
            % update visualizer with new model (if necessary)
            if ( ~isempty(obj.robot_visualizer) )
                obj.robot_visualizer = obj.robot_model.constructVisualizer();   
                obj.robot_visualizer.draw(cputime, obj.robot_nominal_pose);
            end    
        end     
        
        function obj = set_do_gravity_compensation(obj, do_gravity_compensation, gravity_compensation_factor)
            obj.do_gravity_compensation = do_gravity_compensation;
            
            if ( nargin > 2 )
                obj.gravity_compensation_factor = gravity_compensation_factor;
            end
        end
        
         function obj = set_do_current_pose_error_compensation(obj, do_current_pose_error_compensation)
            obj.do_current_pose_error_compensation = do_current_pose_error_compensation;
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
            % store last trajectory in global variable for easy access
            global last_trajectory;
            last_trajectory = trajectory;
            
            % get q and qdot values from trajectory
            qqdot_values = trajectory.eval(t);
            
            nq = obj.robot_model.getNumPositions();
            qs = qqdot_values(1:nq, :);
            qds = qqdot_values(nq+1:2*nq, :);
            
            if ( obj.do_gravity_compensation )
                qs = obj.correct_gravity_influence(qs);
            end
            
            if ( obj.do_current_pose_error_compensation )
                qs = obj.correct_current_pose_error(qs);
            end

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
        
        function corrected_qs = correct_gravity_influence(obj, qs)
            corrected_qs = zeros(size(qs));
            Gs = zeros(size(qs));
            
            for i = 1:size(qs, 2)
                current_qs = qs(:, i);
                [~, G] = obj.manipulatorDynamics(current_qs, zeros(obj.getNumVelocities,1));
                corrected_qs(:, i) = current_qs + obj.gravity_compensation_factor * G;
                Gs(:, i) = G;
            end
        end
        
        function corrected_qs = correct_current_pose_error(obj, qs)
            % get current robot pose error
            current_errors = [];
            error_joint_names = {};
            for i = 1:length(obj.controller_state_subscribers)
                [joint_state_msg, ~, ~] = obj.controller_state_subscribers{i}.poll(10);
                
                error_joint_names = {error_joint_names{:}, joint_state_msg.joint_names{:}};
                current_errors = [current_errors, joint_state_msg.error.positions];
            end
            
            % get error joint indexes
            error_joint_idx = zeros( size(current_errors) );
            for i = 1:length(error_joint_names)
                error_joint_idx(i) = obj.robot_model.findPositionIndices(error_joint_names{i});
            end
            
            % update q values
            influence = 1.0;
            influence_step = 1.0/size(qs, 2);
            corrected_qs = qs;
            for i = 1:size(qs, 2)
                corrected_qs(error_joint_idx,i) = qs(error_joint_idx,i) + current_errors'*influence;                
                influence = influence - influence_step;
            end
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
        
        function reset_ros_interface(obj)                        
            % (re-)init IK publishers / subscribers
            if ( ~isempty(obj.ik_result_publisher) )
               obj.ik_result_publisher.delete();               
               obj.ik_result_publisher = [];
            end
            obj.ik_result_publisher = ros.Publisher('/drake_planner/ik_result', 'vigir_planning_msgs/ResultDrakeIK', 1, false);
            
            if ( ~isempty(obj.ik_request_subscriber) )
                obj.ik_request_subscriber.stop();
                obj.ik_request_subscriber.delete();
                obj.ik_request_subscriber = [];
            end
            obj.ik_request_subscriber = ros.Subscriber('/drake_planner/request_drake_ik','vigir_planning_msgs/RequestDrakeIK', 1);
            obj.ik_request_subscriber.addlistener('Callback', @obj.handleIKRequest);            
            obj.ik_request_subscriber.start();
            
            
            % (re-)init IK trajectory publishers / subscribers
            if ( ~isempty(obj.trajectory_result_publisher) )
               obj.trajectory_result_publisher.delete();
               obj.trajectory_result_publisher = [];
            end
            obj.trajectory_result_publisher = ros.Publisher('/drake_planner/trajectory_result', 'vigir_planning_msgs/ResultDrakeTrajectory', 1, false);
            
            if ( ~isempty(obj.trajectory_request_subscriber) )
                obj.trajectory_request_subscriber.stop();
                obj.trajectory_request_subscriber.delete();
                obj.trajectory_request_subscriber = [];
            end
            obj.trajectory_request_subscriber = ros.Subscriber('/drake_planner/request_drake_trajectory','vigir_planning_msgs/RequestDrakeTrajectory', 1);
            obj.trajectory_request_subscriber.addlistener('Callback', @obj.handleIKTrajectoryRequest);
            obj.trajectory_request_subscriber.start();
            
            % (re-)init IK cartesian trajectory publishers / subscribers
            if ( ~isempty(obj.cartesian_trajectory_result_publisher) )
               obj.cartesian_trajectory_result_publisher.delete();
               obj.cartesian_trajectory_result_publisher = [];
            end
            obj.cartesian_trajectory_result_publisher = ros.Publisher('/drake_planner/cartesian_trajectory_result', 'vigir_planning_msgs/ResultDrakeTrajectory', 1, false);
            
            if ( ~isempty(obj.cartesian_trajectory_request_subscriber) )
                obj.cartesian_trajectory_request_subscriber.stop();
                obj.cartesian_trajectory_request_subscriber.delete();
                obj.cartesian_trajectory_request_subscriber = [];
            end
            obj.cartesian_trajectory_request_subscriber = ros.Subscriber('/drake_planner/request_drake_cartesian_trajectory','vigir_planning_msgs/RequestDrakeCartesianTrajectory', 1);
            obj.cartesian_trajectory_request_subscriber.addlistener('Callback', @obj.handleIKCartesianTrajectoryRequest);
            obj.cartesian_trajectory_request_subscriber.start();
        end
    end
end
