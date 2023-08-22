#!/bin/bash
#set -e

ROS_VERSION=${ROS_VERSION:-noetic}

# if [ -f $HOME/.catkin_ws ]; then
#     echo "Loading catkin ws from $HOME/.catkin_ws"
#     source $HOME/.catkin_ws
# fi

# setup ros environment
if [ "${CATKIN_WS}" == "" ]; then
    echo "Loading default environment: /opt/ros/$ROS_VERSION/setup.bash"
    source /opt/ros/$ROS_VERSION/setup.bash
else
    echo "Loading environment from workspace: $CATKIN_WS"
    source $CATKIN_WS/devel/setup.bash
fi

exec "$@"
