/*
 * @Author: DahlMill
 * @Date: 2020-03-09 20:59:09
 * @LastEditTime: 2020-04-13 09:13:21
 * @LastEditors: Please set LastEditors
 * @Description: 
 * @FilePath: /catkin_ws/src/dm_odom/src/dmOdom.cpp
 */

// R
//           -1 -1.22465e-16            0
//  1.22465e-16           -1            0
//            0            0            1

// T
//     1   0   0
//     0   1   0
//     0   0   1

#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;

#define IMG_PATH "/home/dm/CodeBase/SLAM_DATA/img/"
#define TXT_PATH "/home/dm/CodeBase/SLAM_DATA/img/imglog.txt"

//字符串分割函数
std::vector<std::string> split(std::string str, std::string pattern)
{
    std::string::size_type pos;
    std::vector<std::string> result;
    str += pattern; //扩展字符串以方便操作
    int size = str.size();

    for (int i = 0; i < size; i++)
    {
        pos = str.find(pattern, i);
        if (pos < size)
        {
            std::string s = str.substr(i, pos - i);
            result.push_back(s);
            i = pos + pattern.size() - 1;
        }
    }
    return result;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dm_odometry_publisher");
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    // tf::TransformBroadcaster odom_broadcaster;

    // ros::NodeHandle nh;
    image_transport::ImageTransport it(n);
    image_transport::Publisher pub = it.advertise("camera/image", 1);

    ifstream inFile;
    inFile.open(TXT_PATH, ios::in);

    int imgCount = 0;

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    double vx = 0;  //1;
    double vy = 0;  //-1;
    double vth = 0; //1;

    // ros::Time current_time, last_time;
    // current_time = ros::Time::now();
    // last_time = ros::Time::now();

    ros::Time chassisTimeStamp;
    ros::Time slamTimeStamp;

    ros::Rate r(100);
    // cv::namedWindow("writeView");
    // cv::startWindowThread();

    int iCountBuf = 0;

    // geometry_msgs::Quaternion quat =
    //         tf::createQuaternionMsgFromYaw(180);

    // cout << quat << endl;

    // return 0;

    while (!inFile.eof() && n.ok())
    {
        string strLine;
        getline(inFile, strLine);
        // 打印行读取结果
        // cout << "line " << strLine << endl;

        if (strLine == "")
        {
            cout << "get line null" << endl;
            break;
        }

        std::vector<std::string> vStr = split(strLine, " ");
        slamTimeStamp.sec = atoi(vStr[0].c_str());
        slamTimeStamp.nsec = atoi(vStr[1].c_str()) * 1000;
        imgCount = atoi(vStr[2].c_str());

        chassisTimeStamp.sec = atoi(vStr[3].c_str());
        chassisTimeStamp.nsec = atoi(vStr[4].c_str()) * 1000;
        x = atoi(vStr[5].c_str());
        y = atoi(vStr[6].c_str());
        th = atoi(vStr[7].c_str()) / 10.0;
        th = (2.0 * M_PI) * (th / 360.0);

        // 打印转换结果
        cout << slamTimeStamp.sec << " " << slamTimeStamp.nsec << " " << imgCount << " " << chassisTimeStamp.sec << " " << chassisTimeStamp.nsec << " " << x << " " << y << " " << th << endl;

        ros::spinOnce(); // check for incoming messages
        // current_time = ros::Time::now();

        //compute odometry in a typical way given the velocities of the robot
        // double dt = (current_time - last_time).toSec();
        // double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        // double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        // double delta_th = vth * dt;

        // x += delta_x;
        // y += delta_y;
        // th += delta_th;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat =
            tf::createQuaternionMsgFromYaw(th);
        //first, we'll publish the transform over tf
        // geometry_msgs::TransformStamped odom_trans;
        // odom_trans.header.stamp = current_time;
        // odom_trans.header.frame_id = "odom";
        // odom_trans.child_frame_id = "base_link";
        // odom_trans.transform.translation.x = x;
        // odom_trans.transform.translation.y = y;
        // odom_trans.transform.translation.z = 0.0;
        // odom_trans.transform.rotation = odom_quat;

        //send the transform
        // odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        // odom.header.stamp = current_time;

        odom.header.stamp = chassisTimeStamp;

        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        // ROS_INFO("x [%lf] y [%lf] th [%lf] dt [%lf]", x, y, th, dt);

        //publish the message
        odom_pub.publish(odom);
        // last_time = current_time;

        if(iCountBuf == imgCount)
            continue;

        ostringstream strImgPath;
        strImgPath << IMG_PATH << "img" << imgCount << ".jpg";
        // CV_LOAD_IMAGE_COLOR
        cv::Mat image = cv::imread(strImgPath.str(), CV_LOAD_IMAGE_COLOR);
        // cv::imshow("writeView", image);
        // cv::waitKey(1);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        msg->header.stamp = slamTimeStamp;
        pub.publish(msg);
        iCountBuf = imgCount;
        r.sleep();
    }

    // cv::destroyWindow("writeView");
}
