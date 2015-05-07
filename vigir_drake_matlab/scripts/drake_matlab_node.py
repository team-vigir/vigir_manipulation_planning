#!/usr/bin/python
import os;
import rospkg;

# get rosmatlab path
rosmatlab_path = os.environ['MATLAB_ROOT'] + "/ros/indigo";

# save current ros paths in a separate environment variable
os.environ['ROS_VIGIR_PACKAGE_PATH'] = os.environ['ROS_PACKAGE_PATH'];

package_path = rospkg.RosPack().get_path('vigir_drake_matlab');
matlab_codepath = package_path + "/src/matlab";
matlab_command =  "startNode";

os.chdir(matlab_codepath);
#os.system(rosmatlab_path + "/env.sh matlab -nodesktop -r " + matlab_command);
os.system(rosmatlab_path + "/env.sh matlab -desktop");
#os.system(rosmatlab_path + "/env.sh matlab -nodesktop");

