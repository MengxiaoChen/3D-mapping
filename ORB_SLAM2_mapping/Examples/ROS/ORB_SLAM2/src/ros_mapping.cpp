#include <sensor_msgs/PointCloud2.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
#include "../../../include/laser_geometry.h"
//????
//#include "sensor_msgs/LaserScan.h"
//#include "tf/message_filter.h"
//#include "tf/transform_listener.h"
//#include "tf/tfMessage.h"
//#include "laser_geometry.h"
//#include <sensor_msgs/PointCloud2.h>
//#include <sensor_msgs/PointCloud.h>
//#include <sensor_msgs/point_field_conversion.h>
//#include <time.h>

#include "ros_mapping.h"

using namespace laser_geometry;
using namespace std;

namespace My_mapping {

ros_mapping::ros_mapping(const std::string&max_size_param_name,ORB_SLAM2::System* SLAM) :private_ns_("~")
{
    mySLAM=SLAM;

    m_cal_time=0;
    m_count=0;
    // **** Initialize TransformListener ****
    double tf_cache_time_secs ;
    private_ns_.param("tf_cache_time_secs", tf_cache_time_secs, 10.0) ;
    if (tf_cache_time_secs < 0)
      ROS_ERROR("Parameter tf_cache_time_secs<0 (%f)", tf_cache_time_secs) ;

    tf_ = new tf::TransformListener(n_, ros::Duration(tf_cache_time_secs));
    ROS_INFO("TF Cache Time: %f Seconds", tf_cache_time_secs) ;

// ***** Set fixed_frame *****
private_ns_.param("fixed_frame", fixed_frame_, std::string("ERROR_NO_NAME"));
ROS_INFO("Fixed Frame: %s", fixed_frame_.c_str()) ;
if (fixed_frame_ == "ERROR_NO_NAME")
 {
    ROS_ERROR("Need to set parameter fixed_frame") ;
  }
// ***** Set max_scans *****
const int default_max_scans = 400 ;
int tmp_max_scans ;
private_ns_.param(max_size_param_name, tmp_max_scans, default_max_scans);
if (tmp_max_scans < 0)
{
  ROS_ERROR("Parameter max_scans<0 (%i)", tmp_max_scans) ;
  tmp_max_scans = default_max_scans ;
}
max_scans_ = tmp_max_scans ;
ROS_INFO("Max Scans in History: %u", max_scans_) ;
total_pts_ = 0 ;    // We're always going to start with no points in our history

// ***** Set downsample_factor *****
int tmp_downsample_factor ;
private_ns_.param("downsample_factor", tmp_downsample_factor, 1);
if (tmp_downsample_factor < 1)
{
  ROS_ERROR("Parameter downsample_factor<1: %i", tmp_downsample_factor) ;
  tmp_downsample_factor = 1 ;
}
downsample_factor_ = tmp_downsample_factor ;
if (downsample_factor_ != 1)
  ROS_WARN("Downsample set to [%u]. Note that this is an unreleased/unstable feature", downsample_factor_);

//subscribe
tf_filter_ = NULL;
start("scan");

//assembleScans(scan_hist_);
ROS_INFO("AFTER ASSEMBLESCANS");
pub_cur_cloud_ = private_ns_.advertise<sensor_msgs::PointCloud>("cur_cloud",1);
pub_pointcloud_unit_ = private_ns_.advertise<sensor_msgs::PointCloud>("pointcloud_unit",1);
pub_pointclouds_=private_ns_.advertise<sensor_msgs::PointCloud>("pointclouds",1);
pub_pointclouds_out_=private_ns_.advertise<sensor_msgs::PointCloud2>("pointcloudsout",1);
}

void ros_mapping::start(const std::string& in_topic_name)
{
  ROS_DEBUG("Called start(string). Starting to listen on message_filter::Subscriber the input stream");
    //tf_filter_ = NULL;
  if (tf_filter_!=NULL)
    ROS_ERROR("assembler::start() was called twice!. This is bad, and could leak memory") ;
  else
  {

    scan_sub_.subscribe(n_, in_topic_name, 10);

    tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(scan_sub_, *tf_, fixed_frame_, 10);
    tf_filter_->registerCallback( boost::bind(&ros_mapping::msgCallback, this, _1) );
   //ROS_INFO("SUB SCAN");
  }

}

void ros_mapping::msgCallback(const boost::shared_ptr<const sensor_msgs::LaserScan>& scan_ptr)
{
  ROS_DEBUG("starting msgCallback");
  const sensor_msgs::LaserScan scan = *scan_ptr ;

  sensor_msgs::PointCloud cur_cloud ;

  //ROS_INFO("start msgcallback");
  try
  {
   ConvertToCloud(fixed_frame_, scan, cur_cloud) ;   // Convert scan into a point cloud
  }
  catch(tf::TransformException& ex)
  {
    ROS_WARN("Transform Exception %s", ex.what()) ;
    return ;
  }
  //
 // pub_cur_cloud_.publish(cur_cloud);

  // Add the current scan (now of type PointCloud) into our history of scans
  scan_hist_mutex_.lock() ;
  if (scan_hist_.size() == max_scans_)                           // Is our deque full?
  {
    total_pts_ -= scan_hist_.front().points.size () ;            // We're removing an elem, so this reduces our total point count
    scan_hist_.pop_front() ;                                     // The front of the deque has the oldest elem, so we can get rid of it
  }
  scan_hist_.push_back(cur_cloud) ;                              // Add the newest scan to the back of the deque
  total_pts_ += cur_cloud.points.size () ;                       // Add the new scan to the running total of points
  //printf("Scans: %4u  Points: %10u\n", scan_hist_.size(), total_pts_) ;
  scan_hist_mutex_.unlock() ;
  ROS_DEBUG("done with msgCallback");
}

void ros_mapping::ConvertToCloud(const string& fixed_frame_id, const sensor_msgs::LaserScan& scan_in, sensor_msgs::PointCloud& cloud_out)
{
  laser_geometry::LaserProjection projector_;
  // convert laser scan to point cloud
    projector_.projectLaser(scan_in, cloud_out);

    if (cloud_out.header.frame_id != fixed_frame_id)
    {
      tf_->transformPointCloud(fixed_frame_id, cloud_out, cloud_out);
    }
  return;
}

 void ros_mapping::assembleScans( const double&start_time,const double&end_time)
{
  scan_hist_mutex_.lock();

  // Determine where in our history  we actually are
  unsigned int i = 0;

//std::cout<<"scan_hist_.size() ="<<scan_hist_.size() <<std::endl;
  // Find the beginning of the request. Probably should be a search
  while ( i < scan_hist_.size() &&                                                    // Don't go past end of deque
          scan_hist_[i].header.stamp < ros::Time(start_time ))                                    // Keep stepping until we've exceeded the start time
  {
     // std::cout<<"scan_hist_[i].header.stamp"<<scan_hist_[i].header.stamp<<std::endl;
    i++ ;
  }

 //unsigned int start_index = i ;
   unsigned int start_index = i;
  //std::cout<<"start_index"<<start_index<<std::endl;
  unsigned int req_pts = 0 ;      // Keep a total of the points in the current request

  // Find the end of the request
  while ( i < scan_hist_.size() &&                                                    // Don't go past end of deque
          scan_hist_[i].header.stamp <ros::Time(end_time))                                      // Don't go past the end-time of the request
  {
    req_pts += (scan_hist_[i].points.size ()+downsample_factor_-1)/downsample_factor_ ;
    i += downsample_factor_ ;
  }
  unsigned int past_end_index = i ;

  // std::cout<<"pastend_index"<<past_end_index<<std::endl;

  //typedef  ::sensor_msgs::PointCloud_<std::allocator<void>>  _cloud_type;
 //_cloud_type cmx_cloud;

 sensor_msgs::PointCloud pointcloud_unit ;

  if (start_index == past_end_index)
  {
    pointcloud_unit.header.frame_id = fixed_frame_ ;
    pointcloud_unit.header.stamp =ros::Time(end_time) ;
    pointcloud_unit.points.resize (0) ;
    pointcloud_unit.channels.resize (0) ;
  }
  else
  {

    // Note: We are assuming that channel information is consistent across multiple scans. If not, then bad things (segfaulting) will happen
    // Allocate space for the cloud
   //?!
  pointcloud_unit.points.resize (req_pts);


  //std::cout<<"scan_hist_[start_index].channels.size ()="<<scan_hist_[start_index].channels.size ()<<std::endl;

   const unsigned int num_channels = scan_hist_[start_index].channels.size ();
  //  num_channels=0;
    //std::cout<<"num="<<num_channels<<std::endl;

   pointcloud_unit.channels.resize (num_channels) ;

    for (i = 0; i<num_channels; i++)
    {
      pointcloud_unit.channels[i].name = scan_hist_[start_index].channels[i].name ;
      pointcloud_unit.channels[i].values.resize (req_pts) ;
    }

    //resp.cloud.header.stamp = req.end ;
    pointcloud_unit.header.frame_id = fixed_frame_ ;
    unsigned int cloud_count = 0;

    for (i=start_index; i<past_end_index; i+=downsample_factor_)
    {

      // Sanity check: Each channel should be the same length as the points vector
      for (unsigned int chan_ind = 0; chan_ind < scan_hist_[i].channels.size(); chan_ind++)
      {
        if (scan_hist_[i].points.size () != scan_hist_[i].channels[chan_ind].values.size())
          ROS_FATAL("Trying to add a malformed point cloud. Cloud has %u points, but channel %u has %u elems", (int)scan_hist_[i].points.size (), chan_ind, (int)scan_hist_[i].channels[chan_ind].values.size ());
      }
     // ROS_INFO("BEFORE GET CLOUD");
      for(unsigned int j=0; j<scan_hist_[i].points.size (); j+=downsample_factor_)
      {
        pointcloud_unit.points[cloud_count].x = scan_hist_[i].points[j].x ;
        pointcloud_unit.points[cloud_count].y = scan_hist_[i].points[j].y ;
        pointcloud_unit.points[cloud_count].z = scan_hist_[i].points[j].z ;

        for (unsigned int k=0; k<num_channels; k++)
          pointcloud_unit.channels[k].values[cloud_count] = scan_hist_[i].channels[k].values[j] ;

        cloud_count++ ;
      }
      pointcloud_unit.header.stamp = scan_hist_[i].header.stamp;
    }
    //
    //ROS_INFO("PULISH CLOUD unit");
   // pub_pointcloud_unit_.publish(pointcloud_unit);
// publish final pointsclouds:

   publishpointclouds(pointcloud_unit);

}
  scan_hist_mutex_.unlock() ;

  ROS_DEBUG("Point Cloud Results: Aggregated from index %u->%u. BufferSize: %lu. Points in cloud: %u", start_index, past_end_index, scan_hist_.size(), (int)pointcloud_unit.points.size ()) ;
  return ;
}

//cmxlaser:publish pointclouds:
 void ros_mapping::publishpointclouds(const sensor_msgs::PointCloud&cloud_unit)
 {

        static sensor_msgs::PointCloud point_clouds ;

         //the num of units:
        static unsigned int num_units=0;
        //?
        static unsigned int num_points = 0 ;
        static unsigned int pointcloud_count = 0;

         unsigned int numchannels =cloud_unit.channels.size();
          // std::cout<<"num="<<numchannels<<std::endl;

         if (num_units==0)
         {

           point_clouds.header.frame_id = fixed_frame_ ;
           point_clouds.header.stamp =ros::Time::now();
           point_clouds.points.resize (0) ;
           point_clouds.channels.resize (0) ;
           //pc1:
           point_clouds.header.stamp =cloud_unit.header.stamp;
           num_points+=cloud_unit.points.size();
           point_clouds.points.resize(num_points);

          // const unsigned int numchannels =cloud_unit.channels.size();
              //std::cout<<"num="<<numchannels<<std::endl;
           point_clouds.channels.resize (numchannels);

           //copy pc1 channels:
            for (unsigned int i = 0 ; i < numchannels; i++)
               {
               point_clouds.channels[i].name = cloud_unit.channels[i].name;
               point_clouds.channels[i].values.resize(num_points);
               //std::copy(cloud_unit.channels[i].values.begin(), cloud_unit.channels[i].values.end(), point_clouds.channels[i].values.begin());
              }

           //copy pc1 points and channel values:
          //std::copy(cloud_unit.points.begin(), cloud_unit.points.end(), point_clouds.points.begin());

            for(unsigned int j=0; j<cloud_unit.points.size (); j+=downsample_factor_)
                {
                 point_clouds.points[pointcloud_count].x = cloud_unit.points[j].x ;
                 point_clouds.points[pointcloud_count].y = cloud_unit.points[j].y ;
                 point_clouds.points[pointcloud_count].z = cloud_unit.points[j].z ;

                 for (unsigned int k=0; k<numchannels; k++)
                   point_clouds.channels[k].values[pointcloud_count] = cloud_unit.channels[k].values[j] ;
                 pointcloud_count++ ;
                }
         }
        else
        {
           num_points += (cloud_unit.points.size ()+downsample_factor_-1)/downsample_factor_ ;
           point_clouds.points.resize (num_points);
           point_clouds.channels.resize (numchannels);
           for (unsigned int i = 0; i<numchannels; i++)
           {
             point_clouds.channels[i].name =cloud_unit.channels[i].name ;
             point_clouds.channels[i].values.resize (num_points) ;
           }
          point_clouds.header.frame_id = fixed_frame_ ;
          point_clouds.header.stamp =cloud_unit.header.stamp;
          //add points:
          for(unsigned int j=0; j<cloud_unit.points.size (); j+=downsample_factor_)
              {
               point_clouds.points[pointcloud_count].x = cloud_unit.points[j].x ;
               point_clouds.points[pointcloud_count].y = cloud_unit.points[j].y ;
               point_clouds.points[pointcloud_count].z = cloud_unit.points[j].z ;

               for (unsigned int k=0; k<numchannels; k++)
                 point_clouds.channels[k].values[pointcloud_count] = cloud_unit.channels[k].values[j] ;

               pointcloud_count++ ;
              }
         }
         //
          num_units++;
        //publish clouds:
         // std::cout<<"publish pointclouds final!"<<std::endl;
       //pub_pointclouds_.publish(point_clouds);

   //cmxlaser:convert pointcloud to pointcloud2:
        static sensor_msgs::PointCloud2 point_clouds_out;
        point_clouds_out.header = point_clouds.header;
        point_clouds_out.width  = point_clouds.points.size ();
        point_clouds_out.height = 1;

        // point_clouds_out.fields.resize (3 + point_clouds.channels.size ());
        point_clouds_out.fields.resize (4 + point_clouds.channels.size ());

             // Convert x/y/z to fields
         point_clouds_out.fields[0].name = "x"; point_clouds_out.fields[1].name = "y"; point_clouds_out.fields[2].name = "z";

         point_clouds_out.fields[3].name="rgb";

          int offset = 0;
             // All offsets are *4, as all field data types are float32
          for (size_t d = 0; d < point_clouds_out.fields.size (); ++d, offset += 4)
              {
            point_clouds_out.fields[d].offset = offset;
            point_clouds_out.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
            point_clouds_out.fields[d].count  = 1;
          }
             point_clouds_out.point_step = offset;
             point_clouds_out.row_step   = point_clouds_out.point_step * point_clouds_out.width;
          // Convert the remaining of the channels to fields
              for (size_t d = 0; d < point_clouds.channels.size (); ++d)
         //point_clouds_out.fields[3 + d].name = point_clouds.channels[d].name;
         point_clouds_out.fields[4 + d].name = point_clouds.channels[d].name;
         point_clouds_out.data.resize (point_clouds.points.size () * point_clouds_out.point_step);
         point_clouds_out.is_bigendian = false;  // @todo ?
         point_clouds_out.is_dense = false;
 // Copy the data points
         for (size_t cp = 0; cp < point_clouds.points.size (); ++cp)
      {
       //  std::cout<<"z="<<point_clouds.points[cp].z<<std::endl;
        memcpy (&point_clouds_out.data[cp*point_clouds_out.point_step+point_clouds_out.fields[0].offset],&point_clouds.points[cp].x,sizeof(float));
        memcpy (&point_clouds_out.data[cp*point_clouds_out.point_step+point_clouds_out.fields[1].offset],&point_clouds.points[cp].y,sizeof(float));
        memcpy (&point_clouds_out.data[cp*point_clouds_out.point_step+point_clouds_out.fields[2].offset],&point_clouds.points[cp].z,sizeof(float));

        int r=0xff;
        int g=0xff;
        int b=0xff;
        r=r*((3.1-point_clouds_out.data[cp*point_clouds_out.point_step+point_clouds_out.fields[2].offset])/3.5);
        g=g*((1.4-fabs(point_clouds_out.data[cp*point_clouds_out.point_step+point_clouds_out.fields[2].offset]-1.4))/1.4);
        b=b*((point_clouds_out.data[cp*point_clouds_out.point_step+point_clouds_out.fields[2].offset]+0.4)/3.5);

        int rgb=((int)r)<<16|((int)g)<<8|((int)b);

        float* float_rgb = reinterpret_cast<float*>(&rgb);
        memcpy(&point_clouds_out.data[cp*point_clouds_out.point_step+point_clouds_out.fields[3].offset],float_rgb,sizeof(float));

        //why z ????
        //std::cout<<"z="<<point_clouds_out.data[cp*point_clouds_out.point_step+point_clouds_out.fields[2].offset]<<std::endl;

        for (size_t d = 0;d<point_clouds.channels.size(); ++d)
         {
            if (point_clouds.channels[d].values.size()==point_clouds.points.size())
              {
                // memcpy (&point_clouds_out.data[cp*point_clouds_out.point_step+point_clouds_out.fields[3+d].offset],&point_clouds.channels[d].values[cp],sizeof(float));
             memcpy (&point_clouds_out.data[cp*point_clouds_out.point_step+point_clouds_out.fields[4+d].offset],&point_clouds.channels[d].values[cp],sizeof(float));
             }
         }
     }
    pub_pointclouds_out_.publish(point_clouds_out);

}

 void ros_mapping::run()
 {
    //???
     ros::Rate loop_rate(10);

     ORB_SLAM2::KeyFrame* lastkf;
     bool first=true;
      while(1)
      {
         // ??if(     ) break;
          //if(SLAM.CheckNewKF())
          {
             std::list< ORB_SLAM2::KeyFrame*> sysKeyFrames;
              sysKeyFrames=mySLAM->GetKeyFrame();

            //lock???
                if(sysKeyFrames.size()>3)
             {
                 // std::cout<<"syskeyframes sizes"<<sysKeyFrames.size()<<std::endl;
                  int count =0;

                  list< ORB_SLAM2::KeyFrame*>::iterator litnext = sysKeyFrames.begin();
                  litnext++;
                 for(list< ORB_SLAM2::KeyFrame*>::iterator lit = sysKeyFrames.begin(); ; )
                    {
                        if(!first && count==0)
                        {
                            ORB_SLAM2::KeyFrame* firstKF = *lit;
                            double begin_time;
                           begin_time=firstKF->mTimeStamp;
                           //std::cout<<"BEGIN_time="<<ros::Time(begin_time)<<std::endl;

                           double last_time;
                          last_time=lastkf->mTimeStamp;
                         // std::cout<<"last_time="<<ros::Time(last_time)<<std::endl;
                            assembleScans(last_time,begin_time);
                        }

                        if(first)
                            first=false;
                        if(count == sysKeyFrames.size()-2)
                            lastkf = *litnext;

                          ORB_SLAM2::KeyFrame* firstkf = *lit;

                          double Start_time;
                         Start_time=firstkf->mTimeStamp;

                          ORB_SLAM2::KeyFrame* secondkf = *litnext;
                          double End_time;
                          End_time=secondkf->mTimeStamp;
                         // std::cout<<"end_time="<<ros::Time(End_time)<<std::endl;
                          //??
                          //pcp.assembleScans(Start_time,End_time);
                          assembleScans(Start_time,End_time);
                          lit++;litnext++;count++;
                          if(litnext==sysKeyFrames.end())
                              break;
                    }

               mySLAM->Cleanlist();
             // std::cout<<"CLEAN CLEAN CLEAN"<<std::endl;
             }
          }
          ros::spinOnce();
          loop_rate.sleep();
      }
 }


}//namespace
