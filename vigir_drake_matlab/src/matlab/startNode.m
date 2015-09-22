addpath('helper');

nodeObject = DrakeInverseKinematicsInterface();

if ( strcmpi(getenv('SHOW_DRAKE_VISUALIZATION'), 'TRUE' ) )
    fix_link_com_ui(nodeObject);
end
