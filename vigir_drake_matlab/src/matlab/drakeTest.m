addpath('/usr/local/MATLAB/R2014a/ros/indigo/matlab');
% load message packages
moveit_msgs;
vigir_planning_msgs;

ros.init();
ros.log('INFO', 'Initializing ROS...');

if(~checkDependency('snopt'))
  ros.log('ERROR', 'Need package "snopt" to run...');
  error('Need package "snopt" to run...');
end

ros.log('INFO', 'Initializing Drake...');
robot_model = RigidBodyManipulator();

cd /home/vigir/vigir_repo/drake-distro/drake/systems/plants/test;

ros.log('INFO', 'Testing Drake IK trajectory');
testIKtraj

ros.log('INFO', 'Test successful');