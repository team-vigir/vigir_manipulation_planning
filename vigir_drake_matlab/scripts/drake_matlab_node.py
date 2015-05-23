#!/usr/bin/python
import os;
import rospkg;
import sys;

# check command line arguments
with_gui = False
if len(sys.argv) > 1: 
  if sys.argv[1] == '--with-gui':
    print('Starting with UI...');
    with_gui = True

# get rosmatlab path
rosmatlab_path = os.environ['MATLAB_ROOT'] + "/ros/indigo"

# save current ros paths in a separate environment variable
os.environ['ROS_VIGIR_PACKAGE_PATH'] = os.environ['ROS_PACKAGE_PATH']

package_path = rospkg.RosPack().get_path('vigir_drake_matlab')
matlab_codepath = package_path + "/src/matlab"
matlab_command =  "startNode"

os.chdir(matlab_codepath);

if ( with_gui == True ):
  os.environ['SHOW_DRAKE_VISUALIZATION'] = 'TRUE'
  os.system(rosmatlab_path + "/env.sh matlab -desktop")
else:
  os.environ['SHOW_DRAKE_VISUALIZATION'] = 'FALSE'
  os.system(rosmatlab_path + "/env.sh matlab -nodesktop -r \"run('startNode.m')\"")


