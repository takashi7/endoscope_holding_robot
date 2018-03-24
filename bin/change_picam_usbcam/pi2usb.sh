#!/bin/bash

cd ~/SLAM/ORB_SLAM2/Examples/ROS/ORB_SLAM2/src
cp ros_mono_usb.cc ros_mono.cc
cd ~/SLAM/ORB_SLAM2
./build_ros.sh
