#! /bin/bash
set -e

if ! [ -z "${VIRTUAL_ENV}" ] ; then
  echo "Do not run from a virtualenv"
  exit 1
fi

OPENMOWER_ROS_DIR=${OPENMOWER_ROS_DIR:-~/OpenMower/ROS}

source /opt/ros/noetic/setup.bash
source ${OPENMOWER_ROS_DIR}/devel/setup.bash

rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py . mower_msgs

# overwrite arduino code with STM32 code
cp extra/* ros_lib/
rm ros_lib/ArduinoHardware.h
rm ros_lib/ArduinoTcpHardware.h
