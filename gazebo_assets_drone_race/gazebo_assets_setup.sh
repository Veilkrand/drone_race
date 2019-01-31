#!/bin/sh

# Source Gazebo model path to include this package models

#export "GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH:+${GAZEBO_MODEL_PATH}:}$(rospack find gazebo_assets_drone_race)/models"

export GAZEBO_MODEL_PATH=$(rospack find gazebo_assets_drone_race)/models:$GAZEBO_MODEL_PATH
export GAZEBO_RESOURCE_PATH=$(rospack find gazebo_assets_drone_race)/worlds:$GAZEBO_MODEL_PATH
