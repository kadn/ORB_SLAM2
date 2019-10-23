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

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/core/core.hpp>

#include "../../../include/System.h"
#include "../../../include/Converter.h"

using namespace std;

bool is_publish_tf = false;

class ImageGrabber
{
  public:
    ImageGrabber(ORB_SLAM2::System *pSLAM) : mpSLAM(pSLAM), nh("~")
    {
        odomPub = nh.advertise<nav_msgs::Odometry>("/odom_rgbd", 1);
    }

    void GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD);

    ORB_SLAM2::System *mpSLAM;

    ros::NodeHandle nh;
    ros::Publisher odomPub;

    tf::TransformBroadcaster odom_tf;
    tf::Transform transform;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if (argc != 3)
    {
        cerr << endl
             << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::RGBD);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;
    nh.param("/publish_tf", is_publish_tf, false);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth_registered/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec());
    //因为近似将摄像头竖直着放置，所以在这里先不考虑与地面对齐的情况
    cv::Mat Twc(4, 4, CV_32F);

    if (!Tcw.empty())
    {
        //得到当前相机位置到 camera0的变换矩阵
        cv::Mat Rwc(3, 3, CV_32F);
        cv::Mat twc(3, 1, CV_32F);
        Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
        twc = -Rwc * Tcw.rowRange(0, 3).col(3);

        Twc.at<float>(0, 0) = Rwc.at<float>(0, 0);
        Twc.at<float>(0, 1) = Rwc.at<float>(0, 1);
        Twc.at<float>(0, 2) = Rwc.at<float>(0, 2);

        Twc.at<float>(1, 0) = Rwc.at<float>(1, 0);
        Twc.at<float>(1, 1) = Rwc.at<float>(1, 1);
        Twc.at<float>(1, 2) = Rwc.at<float>(1, 2);

        Twc.at<float>(2, 0) = Rwc.at<float>(2, 0);
        Twc.at<float>(2, 1) = Rwc.at<float>(2, 1);
        Twc.at<float>(2, 2) = Rwc.at<float>(2, 2);

        Twc.at<float>(0, 3) = twc.at<float>(0, 0);
        Twc.at<float>(1, 3) = twc.at<float>(1, 0);
        Twc.at<float>(2, 3) = twc.at<float>(2, 0);

        Twc.at<float>(3, 0) = 0.0;
        Twc.at<float>(3, 1) = 0.0;
        Twc.at<float>(3, 2) = 0.0;
        Twc.at<float>(3, 3) = 1.0;
        // std::cout << "Tcw :" << endl
        //           << Tcw << endl;
        std::cout << "Twc :" << endl
                  << Twc << endl;

        //Toc : odom->camera0, eigenTcc : camera0->camera, 即Twc,  Tcb : camera->base_link
        Eigen::Matrix<double,4,4> To_c0;
        Eigen::Matrix<double,4,4> Tc_b;
        Eigen::Matrix<double,4,4> Tc0_c;
        To_c0 << 0 , 0, 1, 0.15,
              -1,  0, 0, 0,
               0, -1, 0, 0,
               0,  0, 0, 1;

        Tc_b << 0,  -1,  0,  0,
               0,   0, -1,  0 , 
               1,   0,  0,  -0.15,
               0,   0,  0,  1;
        for(int i=0;i<4;i++)
            for(int j=0; j<4; j++)
                Tc0_c(i,j)=Twc.at<float>(i,j);

        Eigen::Matrix<double,4,4> T=To_c0 * Tc0_c * Tc_b;
        cv::Mat Tob= ORB_SLAM2::Converter::toCvMat(T);

        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = Tob.at<float>(0, 3);
        odom.pose.pose.position.y = Tob.at<float>(1, 3);
        odom.pose.pose.position.z = Tob.at<float>(2, 3);

        std::vector<float> q = ORB_SLAM2::Converter::toQuaternion(Tob);
        odom.pose.pose.orientation.x = q[0];
        odom.pose.pose.orientation.y = q[1];
        odom.pose.pose.orientation.z = q[2];
        odom.pose.pose.orientation.w = q[3];
        odomPub.publish(odom);

        if (is_publish_tf)
        {
            transform.setOrigin(tf::Vector3(odom.pose.pose.position.x,
                                            odom.pose.pose.position.y,
                                            odom.pose.pose.position.z));
            tf::Quaternion q_tf(q[0], q[1], q[2], q[3]);
            transform.setRotation(q_tf);
            odom_tf.sendTransform(tf::StampedTransform(transform,
                                                       ros::Time::now(),
                                                       odom.header.frame_id,
                                                       odom.child_frame_id));
        }
    }
}
