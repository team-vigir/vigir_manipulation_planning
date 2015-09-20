if ( exist('nodeObject', 'var') )
    error('There already exists a DrakeInverseKinematicsInterface node. There can only be one!');
end

addpath('helper');

nodeObject = DrakeInverseKinematicsInterface();
fix_link_com_ui(@nodeObject.update_robot_model);