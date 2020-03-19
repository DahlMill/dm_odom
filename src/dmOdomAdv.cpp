/*
 * @Author: DahlMill
 * @Date: 2020-03-09 21:27:02
 * @LastEditTime: 2020-03-09 21:28:44
 * @LastEditors: DahlMill
 * @Description: 
 * @FilePath: /catkin_ws/src/dm_odom/src/dmOdomAdv.cpp
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/Odometry.h>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("readView", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &odom)
{
    printf("odom %.3lf %.3lf\n", odom->pose.pose.position.x, odom->pose.pose.position.y);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "showpath");

    ros::NodeHandle ph;

    ros::Subscriber odomSub = ph.subscribe<nav_msgs::Odometry>("/odom", 10, odomCallback);

    ros::NodeHandle nh;
    cv::namedWindow("readView");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);

    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        ros::spinOnce(); // check for incoming messages
        loop_rate.sleep();
    }

    cv::destroyWindow("readView");

    return 0;
}