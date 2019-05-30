export GAZEBO_RESOURCE_PATH=/usr/share/mavlink_sitl_gazebo:${GAZEBO_RESOURCE_PATH}
export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/mavlink_sitl_gazebo/plugins:${GAZEBO_PLUGIN_PATH}
export GAZEBO_MODEL_PATH=/usr/share/mavlink_sitl_gazebo/models:${GAZEBO_MODEL_PATH}
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/lib/x86_64-linux-gnu/mavlink_sitl_gazebo/plugins
export GAZEBO_MODEL_DATABASE_URI=""
