if ( exist('nodeObject', 'var') )
    error('There already exists a DrakeInverseKinematicsInterface node. There can only be one!');
end

nodeObject = DrakeInverseKinematicsInterface();