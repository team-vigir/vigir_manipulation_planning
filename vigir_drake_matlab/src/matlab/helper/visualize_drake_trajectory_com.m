function visualize_drake_trajectory_com(trajectory, robot_model)
    sample_rate = 4.0;
    

    addpath('/usr/local/MATLAB/R2014a/ros/indigo/matlab');
        
    if(~checkDependency('snopt'))
        ros.log('ERROR', 'Need package "snopt" to run...');
        error('Need package "snopt" to run...');
    end
    
    % construct visualizer
    robot_visualizer = robot_model.constructVisualizer();

    %
    t = trajectory.tspan(1):1/sample_rate:trajectory.tspan(2);
    if ( t(end) < trajectory.tspan(2) )
        t(end+1) = trajectory.tspan(2);
    end
    
    qqdot_values = trajectory.eval(t);
    nq = robot_model.getNumPositions();
    qs = qqdot_values(1:nq, :);
    
    % update visualization from current pose
    com_fig = figure;
    hold on

    for i = 1:length(t)
        current_q = qs(:,i);
                
        robot_visualizer.draw(cputime, current_q);

        % calculate foot contact points in world
        figure(com_fig);
        clf
        plot_robot_com(robot_model, current_q, sprintf('Current trajectory CoM (time = %f)', t(i)));
        drawnow
    end
end
