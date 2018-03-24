#!/bin/bash

gnome-terminal --geometry=80x24+0+0 -e "sh -c 'roscore;exec bash'"

sleep 1s

gnome-terminal --geometry=80x24+0+500 -e "sh -c 'roslaunch ~/SLAM/ORB_SLAM2/launch/camera.launch;exec bash'"
gnome-terminal --geometry=80x24+1000+0 -e "sh -c 'cd ~/SLAM/ORB_SLAM2 && rosrun ORB_SLAM2 Mono Vocabulary/ORBvoc.txt Examples/Monocular/ELP1080p.yaml;exec bash'"

sleep 8s

gnome-terminal  --geometry=80x24+1000+500 -e "sh -c 'roslaunch asparagus aspara.launch;exec bash'"
gnome-terminal  --geometry=80x24+1000+500 -e "sh -c 'rosrun asparagus master;exec bash'"


