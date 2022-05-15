#!/usr/bin/env bash

# Check if we've got a robot spawned yet
while (rosservice call /gazebo/get_model_state '{model_name: tahoma}' | grep 'success: False'); do sleep 1; done

spawn(){
  local name="$1"
  local uniqueid="$name$2"
  shift
  shift

  local x="$1"
  local y="$2"
  local z="$3"
  local yaw="$4"
  rosrun gazebo_ros spawn_model -database "$name" -sdf -model "$uniqueid" -x "$x" -y "$y" -z "$z" -Y "$yaw"
}

set_pose(){
  # Won't set yaw becasue quaternions...
  # Anything fancier than this and we'll have to spawn via Python a la
  # https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_ros/scripts/spawn_model
  local name="$1"
  local uniqueid="$name$2"
  shift
  shift

  local x="$1"
  local y="$2"
  local z="$3"
  local yaw="$4"
  rosservice call /gazebo/set_model_state "model_state:
  model_name: \"$uniqueid\"
  pose:
    position:
      x: $x
      y: $y
      z: $z
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  reference_frame: ''
"
}

spawn_reset(){
  spawn "$@"
  set_pose "$@"
}

# Look through http://models.gazebosim.org/ for the default objects
spawn_reset coke_can 0       0.69 -0.18 1.18 1.57

spawn_reset "wood_cube_5cm" 0 0.72 -0.30 1.66 0.30

spawn_reset "wood_cube_10cm" 0 0.71 0.30 0.89 0.30
