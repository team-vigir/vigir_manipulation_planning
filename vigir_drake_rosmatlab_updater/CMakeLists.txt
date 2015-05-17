cmake_minimum_required(VERSION 2.8.3)
project(vigir_drake_rosmatlab_updater)

if(EXISTS $ENV{VIGIR_ROOT_DIR}/rosmatlab)
   message("Found rosmatlab package, updating messages")
   execute_process(COMMAND $ENV{VIGIR_ROOT_DIR}/rosinstall/install_scripts/update_rosmatlab_msgs.sh)
endif()
