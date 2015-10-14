addpath('helper');

node_object = DrakeInverseKinematicsInterface();

show_com_widget = ros.param.get('/drake_show_com_widget');
if ( show_com_widget )
    disp('Starting FixLinkCoM widget');
    fix_link_com_ui(node_object);
end

% global variable to easily access last planned trajectory
global last_trajectory;
robot_model = node_object.robot_model;