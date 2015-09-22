function hull_distance = plot_robot_com( robot_model, robot_pose, figure_title )
    kinsol0 = doKinematics(robot_model,robot_pose,false,true);

    l_foot = robot_model.findLinkId('l_foot');
    r_foot = robot_model.findLinkId('r_foot');
    r_foot_contact_pts = robot_model.getBody(r_foot).getTerrainContactPoints();
    l_foot_contact_pts = robot_model.getBody(l_foot).getTerrainContactPoints();

    r_foot_contact_pts = forwardKin(robot_model,kinsol0,r_foot,r_foot_contact_pts,0);
    l_foot_contact_pts = forwardKin(robot_model,kinsol0,l_foot,l_foot_contact_pts,0);

    all_contact_pts = [l_foot_contact_pts, r_foot_contact_pts];
    hull = convhull( all_contact_pts(1, :), all_contact_pts(2, :));
    hull_pts = all_contact_pts(:, hull);

    com = robot_model.getCOM(robot_pose);
    
    % display results
    hold on
    title(figure_title)
    scatter3(l_foot_contact_pts(1,:), l_foot_contact_pts(2, :), l_foot_contact_pts(3, :), 'MarkerFaceColor', [0.0, 0.0, 0.0]);
    scatter3(r_foot_contact_pts(1,:), r_foot_contact_pts(2, :), r_foot_contact_pts(3, :), 'MarkerFaceColor', [1.0, 0.0, 0.0]);
    scatter3(com(1), com(2), com(3), 'MarkerFaceColor', [0.0, 1.0, 0.0]);    
    plot3(hull_pts(1,:), hull_pts(2,:), zeros(1, length(hull_pts(1,:)))); % convex hull of contact pts
    legend('left foot', 'right foot', 'CoM', 'convex hull');
    axis([-0.5, 0.5, -0.5, 0.5]);
    hold off
    
    % calculate distance CoM to hull
    com_dist = p_poly_dist(com(1), com(2), hull_pts(1,:), hull_pts(2,:));    
    disp(['CoM dist to hull: ' num2str(com_dist)]);
    
    hull_distance = com_dist;
end

