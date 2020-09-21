#!/usr/bin/env zsh

SIMULATION_ROOT_DIR=$(builtin cd "`dirname "$0"`" > /dev/null && pwd)
. "$SIMULATION_ROOT_DIR/devel/setup.sh"

export SIMULATION_ROOT_DIR=${SIMULATION_ROOT_DIR}

export GAZEBO_MODEL_PATH=${SIMULATION_ROOT_DIR}/src/gazebo/models:${GAZEBO_MODEL_PATH}
export GAZEBO_PLUGIN_PATH=${SIMULATION_ROOT_DIR}/devel/lib:${GAZEBO_PLUGIN_PATH}
export LD_LIBRARY_PATH=${SIMULATION_ROOT_DIR}/devel/lib:${LD_LIBRARY_PATH}
source /opt/ros/kinetic/setup.zsh
