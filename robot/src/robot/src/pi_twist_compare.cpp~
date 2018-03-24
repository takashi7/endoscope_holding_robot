#include <ros/ros.h>
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "robot_twist");
  ros::NodeHandle node;
  tf::TransformBroadcaster br;
  tf::TransformListener listener;
  ros::Publisher caemera_pose = node.advertise<geometry_msgs::Twist>("camera/twist/compare", 10);    
  
  ros::Rate rate(30.0);
  while (node.ok()){
    //geometry_msgs::Twist tempTwist;
    geometry_msgs::Twist objectTwist;
    tf::StampedTransform transform;
    tf::Matrix3x3 rotation;
    try{
      //listener.lookupTwist("/camera_pose", "/world", ros::Time(0), ros::Duration(0.1), tempTwist);
      //transform = tempTwist;
      //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "world_velocity")); 
      //listener.lookupTransform("/pre_camera_pose", "/camera_pose", ros::Time(0), transform);  
      listener.lookupTransform("/pre_camera_pose", "/camera_pose", ros::Time(0), transform);   
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(0.01).sleep();
      continue;
    }
    float loop_hz = 30; 
    //float dt = 1/30;
    float coeff_angular = 100;
    float coeff_linear = 500;
    rotation = transform.getBasis();    
    rotation.getRPY(objectTwist.angular.x, objectTwist.angular.y, objectTwist.angular.z);
    objectTwist.angular.x = objectTwist.angular.x * loop_hz * coeff_angular;
    objectTwist.angular.y = objectTwist.angular.y * loop_hz * coeff_angular;
    objectTwist.angular.z = objectTwist.angular.z * loop_hz * coeff_angular;
    
    objectTwist.linear.x = float(transform.getOrigin().x()) * loop_hz * coeff_linear;
    objectTwist.linear.y = float(transform.getOrigin().y()) * loop_hz * coeff_linear;
    objectTwist.linear.z = float(transform.getOrigin().z()) * loop_hz * coeff_linear;

    
    caemera_pose.publish(objectTwist);

    rate.sleep();
  }
  return 0;
};
