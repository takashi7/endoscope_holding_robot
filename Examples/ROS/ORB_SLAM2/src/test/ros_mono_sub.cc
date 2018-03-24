#include <ros/ros.h>
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_listener.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "Mono_lis");

  ros::NodeHandle node;

  tf::TransformListener listener;
  
  ros::Publisher caemera_pose = node.advertise<geometry_msgs::Twist>("camera_pose", 10);  
  
  double roll, pitch, yaw;
  
  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/world", "/camera_pose", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
   
    geometry_msgs::Twist cam_msg;
    cam_msg.angular.x = transform.getOrigin().x();
    cam_msg.angular.y = transform.getOrigin().y();
    cam_msg.angular.z = transform.getOrigin().z();
    
    /*
    cam_msg.linear = transform.getRotation()
    btQuaternion btq(cam_msg.linear.x,cam_msg.linear.y,cam_msg.linear.z,cam_msg.linear.w);
 　  tf::Matrix3x3(btq).getRPY(roll, pitch, yaw);
    cam_msg.linear.x = roll;
    cam_msg.linear.y = pitch;
    cam_msg.linear.z = yaw;
    */
    
    caemera_pose.publish(cam_msg);

    rate.sleep();
  
  
  //ros::spin();
  }
  return 0;
};
