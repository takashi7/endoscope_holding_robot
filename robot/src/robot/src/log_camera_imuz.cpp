#include <ros/ros.h>
#include "geometry_msgs/Twist.h"

#include<iostream>
#include<fstream>

#include <stdio.h>
#include <chrono>

#include <time.h>

using namespace std;

ofstream ofs_camera("camera.csv");
ofstream ofs_imuz("imuz.csv");


//std::chrono::system_clock::time_point  start_camera, end_camera;
//std::chrono::system_clock::time_point  start_imuz, end_imuz;
struct timespec startTime_camera, endTime_camera;
struct timespec startTime_imuz, endTime_imuz;

int count_camera = 0;
int count_imuz = 0;


void camera_Callback(const geometry_msgs::Twist& msg)
{
  	//ROS_INFO("camera");
  	if(count_camera == 0){
  		ofs_camera << 0 << "," << msg.angular.x << "," << msg.angular.y << "," << msg.angular.z << "," << msg.linear.x << "," << msg.linear.y << "," << msg.linear.z << endl;
		//start_camera = std::chrono::system_clock::now();
		clock_gettime(CLOCK_REALTIME, &startTime_camera);
		++count_camera;
	}else{
		//end_camera = std::chrono::system_clock::now();
		//float msec = std::chrono::duration_cast<std::chrono::milliseconds>(end_camera - start_camera).count();
		clock_gettime(CLOCK_REALTIME, &endTime_camera);
		double msec;
		if (endTime_camera.tv_nsec < startTime_camera.tv_nsec) 
   			msec = 1e-6 * (endTime_camera.tv_nsec + 1000000000 - startTime_camera.tv_nsec);
  		else
   			msec = 1e-6 * (endTime_camera.tv_nsec - startTime_camera.tv_nsec);
		ofs_camera << msec << "," << msg.angular.x << "," << msg.angular.y << "," << msg.angular.z << "," << msg.linear.x << "," << msg.linear.y << "," << msg.linear.z << endl;
		//start_camera = std::chrono::system_clock::now();
		clock_gettime(CLOCK_REALTIME, &startTime_camera);
	}	
}

void imuz_Callback(const geometry_msgs::Twist& msg)
{
	//ROS_INFO("imuz");
	if(count_imuz == 0){
  		ofs_imuz << 0 << "," << msg.angular.x << "," << msg.angular.y << "," << msg.angular.z << endl;
		//start_imuz = std::chrono::system_clock::now();
		clock_gettime(CLOCK_REALTIME, &startTime_imuz);
		++count_imuz;
	}else{
		//end_imuz = std::chrono::system_clock::now();
		//float msec = std::chrono::duration_cast<std::chrono::milliseconds>(end_imuz - start_imuz).count();
		clock_gettime(CLOCK_REALTIME, &endTime_imuz);
		double msec;
		if (endTime_imuz.tv_nsec < startTime_imuz.tv_nsec) 
   			msec = 1e-6 * (endTime_imuz.tv_nsec + 1000000000 - startTime_imuz.tv_nsec);
  		else
   			msec = 1e-6 * (endTime_imuz.tv_nsec - startTime_imuz.tv_nsec);
		ofs_imuz << msec << "," << msg.angular.x << "," << msg.angular.y << "," << msg.angular.z << endl;
		//start_imuz = std::chrono::system_clock::now();
		clock_gettime(CLOCK_REALTIME, &startTime_imuz);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "log_camera_imuz");
	ros::NodeHandle node;
	ros::Subscriber camera = node.subscribe("camera/twist/compare", 10, camera_Callback);
	ros::Subscriber imuz = node.subscribe("imuz/deg/angular", 10, imuz_Callback);
	ros::spin();
	return 0;
}

