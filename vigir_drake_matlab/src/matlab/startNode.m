addpath('helper');

nodeObject = DrakeInverseKinematicsInterface();

show_com_widget = ros.param.get('/drake_show_com_widget');
if ( show_com_widget )
    disp('Starting FixLinkCoM widget');
    fix_link_com_ui(nodeObject);
end
