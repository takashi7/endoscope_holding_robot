/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"


// add
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>

//#include <image_transport/image_transport.h> // my pi

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    //void GrabImage(const sensor_msgs::CompressedImageConstPtr& msg); // my pi
    
    tf::Transform cvMatToTF ( cv::Mat Tcw );

    ORB_SLAM2::System* mpSLAM;
    
    tf::TransformBroadcaster br;
};

tf::Transform pre_transform;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    //ros::Subscriber sub = nodeHandler.subscribe("/raspicam_node/image/compressed", 1, &ImageGrabber::GrabImage,&igb); // my pi
    //image_transport::Subscriber sub; // my pi
    // add
    ros::Publisher pub_vision = nodeHandler.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",100);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}


////void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
//void ImageGrabber::GrabImage(const sensor_msgs::CompressedImageConstPtr& msg) 
//// my pi
//{
//    //cv::Mat msg = cv::imdecode(cv::Mat(msg_pi->data),1); // my pi
//    // Copy the ros image message to cv::Mat.
//    cv_bridge::CvImageConstPtr cv_ptr;
//    try
//    {
//        //cv_ptr = cv_bridge::toCvShare(msg);
//        cv_ptr = cv_bridge::toCvCopy(msg); //my pi
//    }
//    catch (cv_bridge::Exception& e)
//    {
//        ROS_ERROR("cv_bridge exception: %s", e.what());
//        return;
//    }

//    // add all
//    cv::Mat pose = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
//    ros::Time t = msg->header.stamp;
//    
//    if (pose.empty())
//         return;
//    
//    tf::Transform transform = cvMatToTF(pose);
//    //static tf::Transform pre_transform = pre_transform;
//    pre_transform = pre_transform;
//    
//    br.sendTransform(tf::StampedTransform(pre_transform, t, "world", "pre_camera_pose"));
//    br.sendTransform(tf::StampedTransform(transform, t, "world", "camera_pose"));    
//    
//    pre_transform = transform;
//}




void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // add all
    cv::Mat pose = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    ros::Time t = msg->header.stamp;
    
    if (pose.empty())
         return;
    
    tf::Transform transform = cvMatToTF(pose);
    //static tf::Transform pre_transform = pre_transform;
    pre_transform = pre_transform;
    
    br.sendTransform(tf::StampedTransform(pre_transform, t, "world", "pre_camera_pose"));
    br.sendTransform(tf::StampedTransform(transform, t, "world", "camera_pose"));    
    
    pre_transform = transform;
}


tf::Transform ImageGrabber::cvMatToTF ( cv::Mat Tcw )
{
     tf::Transform cam_to_first_keyframe_transform;
     // invert since Tcw (transform from world to camera)
     cv::Mat pose = Tcw.inv();

     //Extract transformation from the raw orb SLAM pose
     tf::Vector3 origin;
     //tf::Quaternion transform_quat;
     tf::Matrix3x3 transform_matrix;

     origin.setValue ( pose.at<float> ( 0, 3 ), pose.at<float> ( 1, 3 ), pose.at<float> ( 2, 3 ) );

     transform_matrix.setValue ( pose.at<float> ( 0, 0 ), pose.at<float> ( 0, 1 ), pose.at<float> ( 0, 2 ),
                                 pose.at<float> ( 1, 0 ), pose.at<float> ( 1, 1 ), pose.at<float> ( 1, 2 ),
                                 pose.at<float> ( 2, 0 ), pose.at<float> ( 2, 1 ), pose.at<float> ( 2, 2 ) );

   
     //transform_matrix.getRotation(transform_quat);
     cam_to_first_keyframe_transform.setOrigin ( origin );
     cam_to_first_keyframe_transform.setBasis ( transform_matrix );

     return cam_to_first_keyframe_transform;
}


