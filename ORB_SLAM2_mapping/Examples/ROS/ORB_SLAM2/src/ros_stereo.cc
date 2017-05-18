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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include  "geometry_msgs/PoseWithCovariance.h"
#include  "geometry_msgs/PoseWithCovarianceStamped.h"

#include "ros_mapping.h"

#include "tf/message_filter.h"
#include "tf/transform_listener.h"
#include "tf/tfMessage.h"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
//#include <sensor_msgs/point_field_conversion.h>
#include <time.h>

//cmxnew:
#include"tf/LinearMath/Quaternion.h"
#include"sensor_msgs/Imu.h"
//#include "bullet/LinearMath/btTransform.h"

#define TF_EULER_DEFAULT_ZYX

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

#include "tf/transform_datatypes.h"
#include  <tf/transform_broadcaster.h>
#include "tf/LinearMath/Transform.h"
//cmx:
//#include "System.h"
#include "../../../include/Tracking.h"

//cmxlaser:
//#include "Frame.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"

using namespace ORB_SLAM2;
using namespace std;

double PI = 3.14159265359;

class ImageGrabber
{
public:
     ros::Publisher pub_pose_world_;
     ros::Publisher pub_pose_worldimu_;
     ros::Publisher pub_pose_;
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}
    ImageGrabber(ORB_SLAM2::System* pSLAM,ros::NodeHandle &nh):mpSLAM(pSLAM){
    pub_pose_world_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_world",1);
    pub_pose_worldimu_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_worldimu",1);
    pub_pose_=nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose",1);
}
    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    void attitudeCallback(const sensor_msgs::ImuConstPtr& msg);

    //ORB_SLAM2::System* mpSLAM;
    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
};

bool isattitudecall;
bool isTrackInid;
boost::mutex mAttitudelock;
sensor_msgs::Imu attitude_data_org;
cv::Mat attitude_data_;

//cmxnew:
void ImageGrabber::attitudeCallback(const sensor_msgs::ImuConstPtr& msg)
{
    isattitudecall=true;
    mAttitudelock.lock();
    //timestamp????????
    attitude_data_org.header.stamp=ros::Time::now();
    attitude_data_org.orientation=msg->orientation;

    //attitude_data_org.angular_velocity=msg->angular_velocity;
    tf::Quaternion orientationimu;
    orientationimu.setValue(attitude_data_org.orientation.x, attitude_data_org.orientation.y, attitude_data_org.orientation.z, attitude_data_org.orientation.w);
    tf::Matrix3x3 Rotationimu(orientationimu);

    tfScalar roll1, pitch1, yaw1;
     Rotationimu.getEulerYPR(yaw1,pitch1,roll1);

    cv::Mat Rroll, Rpitch, Rei, Ric,  Rec, Rce;
    float roll[3][3]={ {1.0,0,0},{0, cos(roll1), -sin(roll1) },{ 0, sin(roll1), cos(roll1)  } };
    float pitch[3][3]={{cos(pitch1), 0 , sin(pitch1)} ,{ 0, 1.0, 0}, { -sin(pitch1), 0, cos(pitch1) } };
    Rroll=cv::Mat(3,3,CV_32F,roll);
    Rpitch=cv::Mat(3,3,CV_32F,pitch);
    Rei=Rpitch*Rroll;

    float ic[3][3]={{1,0,0},{0,0,1 },{ 0,-1,0} };
    Ric=cv::Mat(3,3,CV_32F, ic);
    Rec=Rei*Ric;
    Rce=Rec.t();
    attitude_data_=Rce.clone();

     mAttitudelock.unlock();

}


/// Creat ros-related mapping
My_mapping::ros_mapping* ros_mapper;
std::thread* mptMapping;
tf::TransformBroadcaster* tfb_;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    ros::NodeHandle nh;
    //cmxlaser:
    const std::string&max_size_param_name="max_scans";

    ImageGrabber igb(&SLAM,nh);

    //ImageGrabber igb(&SLAM);
    isattitudecall=false;
    ros::Subscriber sub_attitude_=nh.subscribe<sensor_msgs::Imu>("/imu/data",1,boost::bind(&ImageGrabber::attitudeCallback,&igb,_1));
    isTrackInid=false;

    /// setup ros mapper, a new thread
    ros_mapper = new My_mapping::ros_mapping(max_size_param_name,&SLAM);
    mptMapping = new thread(&My_mapping::ros_mapping::run,ros_mapper);

    tfb_ = new tf::TransformBroadcaster();


    stringstream ss(argv[3]);
	ss >> boolalpha >> igb.do_rectify;

    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

   // ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "camera/right/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
      //cmx:

          cv::Mat pose = mpSLAM->TrackRGBD(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());

         //cmxnew:
                isTrackInid=mpSLAM->mbTrackInit;

                 if (pose.empty())
                   return;

           //  camera in the world frame:rwc,twc.
                  cv::Mat Rotationwc=pose.rowRange(0,3).colRange(0,3).t();
                  cv::Mat Translationwc=-Rotationwc*pose.rowRange(0,3).col(3);

          //rcw:.
                 cv::Mat Rotationcw=pose.rowRange(0,3).colRange(0,3);

                 tf::Matrix3x3 Rotation_world( Rotationwc.at<float>(0,0),   Rotationwc.at<float>(0,1),   Rotationwc.at<float>(0,2),
                                              Rotationwc.at<float>(1,0),   Rotationwc.at<float>(1,1),  Rotationwc.at<float>(1,2),
                                                 Rotationwc.at<float>(2,0), Rotationwc.at<float>(2,1), Rotationwc.at<float>(2,2));
                 tf::Vector3 Translation_world( Translationwc.at<float>(0),Translationwc.at<float>(1),Translationwc.at<float>(2) );
        //cmxnew:
                 //world frame: x:right  y: forward z:up    Rw2c=Rw2w1*Rw1c
                // const tf::Matrix3x3 rotation90degX( 1, 0, 0, 0, 0, 1, 0, -1, 0);
               //  tf::Matrix3x3 globalRotation_rh =rotation90degX*Rotation_world;
               //  const tf::Vector3 globalTranslation_rh = rotation90degX*Translation_world;

                  tf::Quaternion q_world;
                  //globalRotation_rh.getRotation(q_world);
                  Rotation_world.getRotation(q_world);

                  static tf::TransformBroadcaster br;
                 //tf::Transform transform = tf::Transform( globalRotation_rh, globalTranslation_rh);
                 tf::Transform transform = tf::Transform( Rotation_world, Translation_world);
                 br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_link", "camera_pose"));

                  geometry_msgs::PoseWithCovarianceStampedPtr msg_pose(new geometry_msgs::PoseWithCovarianceStamped);

                  msg_pose->pose.pose.orientation.x=q_world.x();
                  msg_pose->pose.pose.orientation.y=q_world.y();
                  msg_pose->pose.pose.orientation.z=q_world.z();
                  msg_pose->pose.pose.orientation.w=q_world.w();
                  /*
                  msg_pose->pose.pose.position.x=globalTranslation_rh.x();
                  msg_pose->pose.pose.position.y=globalTranslation_rh.y();
                  msg_pose->pose.pose.position.z=globalTranslation_rh.z();
                 */
                  msg_pose->pose.pose.position.x=Translation_world.x();
                  msg_pose->pose.pose.position.y=Translation_world.y();
                  msg_pose->pose.pose.position.z=Translation_world.z();
                  cv::Mat covar=mpSLAM->GetCovMat();
                  cv::Mat CovRot_pre=cv::Mat::eye(6,6,CV_32F);

                 (Rotationcw.rowRange(0,3).colRange(0,3)).copyTo(CovRot_pre.rowRange(0,3).colRange(0,3));
                 (Rotationcw.rowRange(0,3).colRange(0,3)).copyTo(CovRot_pre.rowRange(3,6).colRange(3,6));

                  cv::Mat covar_world_pre=CovRot_pre.t()*covar*CovRot_pre;

              //
                  cv::Mat CovRot=cv::Mat::eye(6,6,CV_32F);
                  //Rw1w2
                  cv::Mat Rotationww=cv::Mat::eye(3,3,CV_32F);
                  Rotationww.at<float>(0,0)=1.0;
                  Rotationww.at<float>(0,1)=0.0;
                  Rotationww.at<float>(0,2)=0.0;
                  Rotationww.at<float>(1,0)=0.0;
                  Rotationww.at<float>(1,1)=0.0;
                  Rotationww.at<float>(1,2)=-1.0;
                  Rotationww.at<float>(2,0)=0.0;
                  Rotationww.at<float>(2,1)=1.0;
                  Rotationww.at<float>(2,2)=0.0;

                  (Rotationww.rowRange(0,3).colRange(0,3)).copyTo(CovRot.rowRange(0,3).colRange(0,3));
                  (Rotationww.rowRange(0,3).colRange(0,3)).copyTo(CovRot.rowRange(3,6).colRange(3,6));

                  cv::Mat covar_world=CovRot.t()*covar_world_pre*CovRot;

                      for (unsigned int i = 0; i < msg_pose->pose.covariance.size(); i++)
                  {
                      msg_pose->pose.covariance[i] = sqrt(fabs(covar_world.at<float>((i % 6),(i / 6))));
                   }

                  //msg_pose->header = cv_ptrRGB->header;
                      msg_pose->header.stamp =cv_ptrLeft->header.stamp ;
                      msg_pose->header.seq = cv_ptrLeft->header.seq;
                      msg_pose->header.frame_id = "/world";
                  pub_pose_world_.publish(msg_pose);


    }
    else
    {
        mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());

        //cmx：
                cv::Mat pose = mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());

         //cmxnew:
                isTrackInid=mpSLAM->mbTrackInit;

                 if (pose.empty())
                   return;

           //  camera in the world frame:rwc,twc.
                  cv::Mat Rotationwc=pose.rowRange(0,3).colRange(0,3).t();
                  cv::Mat Translationwc=-Rotationwc*pose.rowRange(0,3).col(3);

          //rcw:.
                 cv::Mat Rotationcw=pose.rowRange(0,3).colRange(0,3);

                 tf::Matrix3x3 Rotation_world( Rotationwc.at<float>(0,0),   Rotationwc.at<float>(0,1),   Rotationwc.at<float>(0,2),
                                              Rotationwc.at<float>(1,0),   Rotationwc.at<float>(1,1),  Rotationwc.at<float>(1,2),
                                                 Rotationwc.at<float>(2,0), Rotationwc.at<float>(2,1), Rotationwc.at<float>(2,2));
                 tf::Vector3 Translation_world( Translationwc.at<float>(0),Translationwc.at<float>(1),Translationwc.at<float>(2) );
        //cmxnew:
                 //world frame: x:right  y: forward z:up    Rw2c=Rw2w1*Rw1c
                // const tf::Matrix3x3 rotation90degX( 1, 0, 0, 0, 0, 1, 0, -1, 0);
               //  tf::Matrix3x3 globalRotation_rh =rotation90degX*Rotation_world;
               //  const tf::Vector3 globalTranslation_rh = rotation90degX*Translation_world;

                  tf::Quaternion q_world;
                  //globalRotation_rh.getRotation(q_world);
                  Rotation_world.getRotation(q_world);

                  static tf::TransformBroadcaster br;
                 //tf::Transform transform = tf::Transform( globalRotation_rh, globalTranslation_rh);
                 tf::Transform transform = tf::Transform( Rotation_world, Translation_world);
                 br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_link", "camera_pose"));

                  geometry_msgs::PoseWithCovarianceStampedPtr msg_pose(new geometry_msgs::PoseWithCovarianceStamped);

                  msg_pose->pose.pose.orientation.x=q_world.x();
                  msg_pose->pose.pose.orientation.y=q_world.y();
                  msg_pose->pose.pose.orientation.z=q_world.z();
                  msg_pose->pose.pose.orientation.w=q_world.w();
                  /*
                  msg_pose->pose.pose.position.x=globalTranslation_rh.x();
                  msg_pose->pose.pose.position.y=globalTranslation_rh.y();
                  msg_pose->pose.pose.position.z=globalTranslation_rh.z();
                 */
                  msg_pose->pose.pose.position.x=Translation_world.x();
                  msg_pose->pose.pose.position.y=Translation_world.y();
                  msg_pose->pose.pose.position.z=Translation_world.z();
                  cv::Mat covar=mpSLAM->GetCovMat();
                  cv::Mat CovRot_pre=cv::Mat::eye(6,6,CV_32F);

                 (Rotationcw.rowRange(0,3).colRange(0,3)).copyTo(CovRot_pre.rowRange(0,3).colRange(0,3));
                 (Rotationcw.rowRange(0,3).colRange(0,3)).copyTo(CovRot_pre.rowRange(3,6).colRange(3,6));

                  cv::Mat covar_world_pre=CovRot_pre.t()*covar*CovRot_pre;

              //
                  cv::Mat CovRot=cv::Mat::eye(6,6,CV_32F);
                  //Rw1w2
                  cv::Mat Rotationww=cv::Mat::eye(3,3,CV_32F);
                  Rotationww.at<float>(0,0)=1.0;
                  Rotationww.at<float>(0,1)=0.0;
                  Rotationww.at<float>(0,2)=0.0;
                  Rotationww.at<float>(1,0)=0.0;
                  Rotationww.at<float>(1,1)=0.0;
                  Rotationww.at<float>(1,2)=-1.0;
                  Rotationww.at<float>(2,0)=0.0;
                  Rotationww.at<float>(2,1)=1.0;
                  Rotationww.at<float>(2,2)=0.0;

                  (Rotationww.rowRange(0,3).colRange(0,3)).copyTo(CovRot.rowRange(0,3).colRange(0,3));
                  (Rotationww.rowRange(0,3).colRange(0,3)).copyTo(CovRot.rowRange(3,6).colRange(3,6));

                  cv::Mat covar_world=CovRot.t()*covar_world_pre*CovRot;

                      for (unsigned int i = 0; i < msg_pose->pose.covariance.size(); i++)
                  {
                      msg_pose->pose.covariance[i] = sqrt(fabs(covar_world.at<float>((i % 6),(i / 6))));
                   }

                  //msg_pose->header = cv_ptrRGB->header;
                      msg_pose->header.stamp =cv_ptrLeft->header.stamp ;
                      msg_pose->header.seq = cv_ptrLeft->header.seq;
                      msg_pose->header.frame_id = "/world";
                  pub_pose_world_.publish(msg_pose);


    }

}


