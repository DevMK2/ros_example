#!/usr/bin/env bash

SIMULATION_ROOT_DIR=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)
. "$SIMULATION_ROOT_DIR/devel/setup.sh"

export SIMULATION_ROOT_DIR=${SIMULATION_ROOT_DIR}

export GAZEBO_MODEL_PATH=${SIMULATION_ROOT_DIR}/src/gazebo/models:${GAZEBO_MODEL_PATH}
export GAZEBO_PLUGIN_PATH=${SIMULATION_ROOT_DIR}/devel/lib:${GAZEBO_PLUGIN_PATH}
export LD_LIBRARY_PATH=${SIMULATION_ROOT_DIR}/devel/lib:${LD_LIBRARY_PATH}

# optional
source /opt/ros/kinetic/setup.bash
