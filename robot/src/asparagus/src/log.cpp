#include <ros/ros.h>
#include "geometry_msgs/Twist.h"

#include<iostream>
#include<fstream>

#include <stdio.h>
#include <time.h>


using namespace std;

ofstream ofs_camera("camera.csv");
ofstream ofs_robot("robot.csv");

struct timeval tv;

void camera_Callback(const geometry_msgs::Twist& msg)
{
  	//ROS_INFO("camera");
  	gettimeofday(&tv, NULL);	
	ofs_camera << tv.tv_usec  << "," << msg.linear.z << endl;
}

void robot_Callback(const geometry_msgs::Twist& msg)
{
	//ROS_INFO("robot");
	gettimeofday(&tv, NULL);	
	ofs_robot << tv.tv_usec << "," << msg.linear.z << endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "log");
	ros::NodeHandle node;
	ros::Subscriber camera = node.subscribe("camera/twist/compare", 10, camera_Callback);
	ros::Subscriber robot = node.subscribe("asparagus/twist", 10, robot_Callback);
	ros::spin();
  return 0;
}

