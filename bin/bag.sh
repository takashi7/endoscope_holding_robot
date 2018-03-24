#!/bin/bash

cd ~/bagfiles
rosbag record -O subset /camera/twist/compare
