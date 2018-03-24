#!/bin/bash

gnome-terminal --geometry=80x24+0+0 -e "sh -c 'roscore;exec bash'"
gnome-terminal --geometry=80x24+0+500 -e "sh -c 'ssh pi@192.168.10.187;exec bash'"

sleep 20s

gnome-terminal --geometry=80x24+1000+0 -e "sh -c 'cd ~/SLAM/ORB_SLAM2 && rosrun ORB_SLAM2 Mono Vocabulary/ORBvoc.txt Examples/Monocular/camerav2_320x240.yaml;exec bash'"

sleep 8s

gnome-terminal  --geometry=80x24+1000+500 -e "sh -c 'roslaunch asparagus aspara.launch;exec bash'"
gnome-terminal  --geometry=80x24+1000+500 -e "sh -c 'rosrun asparagus master;exec bash'"

#sleep 5s

#gnome-terminal --geometry=80x24+1000+500 -e "sh -c 'rqt;exec bash'"
