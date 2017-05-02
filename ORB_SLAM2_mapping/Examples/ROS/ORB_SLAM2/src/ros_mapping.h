#ifndef ROS_MAPPING_H
#define ROS_MAPPING_H
#include <opencv2/core/core.hpp>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
#include <tf/transform_broadcaster.h>
#include <nodelet/nodelet.h>
#include <iostream>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "../../../include/System.h"

#include "../../../include/KeyFrame.h"
#include "../../../include/KeyFrameDatabase.h"

#include "tf/message_filter.h"
#include "tf/transform_listener.h"
#include "tf/tfMessage.h"
#include "sensor_msgs/LaserScan.h"

//#include "../../../include/laser_geometry.h"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
//#include <sensor_msgs/point_field_conversion.h>
#include <time.h>

//using namespace ORB_SLAM2;

//using namespace laser_geometry;
using namespace std;
/*
namespace laser_geometry{
    class LaserProjection;
};
*/
namespace My_mapping
{
/*
class Viewer;
class FrameDrawer;
class Map;
class Tracking;
class LocalMapping;
class LoopClosing;
class System;
*/

class ros_mapping
{
 public:
    //?
     ros_mapping(const std::string&max_size_param_name,ORB_SLAM2::System* SLAM);

      void run();

     void start(const std::string& in_topic_name);

     void ConvertToCloud(const string& fixed_frame_id, const sensor_msgs::LaserScan& scan_in, sensor_msgs::PointCloud& cloud_out);

     void assembleScans(const double&start_time,const double&end_time);

   //cmxlaser: publish clouds:
    void publishpointclouds(const sensor_msgs::PointCloud&cloud_out);

    ros::Publisher pub_cur_cloud_;
    ros::Publisher pub_pointcloud_unit_;
    ros::Publisher pub_pointclouds_;
    ros::Publisher pub_pointclouds_out_;
    long long m_cal_time;
    long long m_count;

    ORB_SLAM2::System* mySLAM;
   //laser_geometry::LaserProjection projector_;


protected:
   tf::TransformListener* tf_ ;
   tf::MessageFilter<sensor_msgs::LaserScan>* tf_filter_;
   ros::NodeHandle private_ns_;
   ros::NodeHandle n_;

private:

    void msgCallback(const boost::shared_ptr<const sensor_msgs::LaserScan>& scan_ptr);

    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub_;
  //bool ignore_laser_skew_;
  // laser_geometry::LaserProjection projector_;
 // ros::Duration max_tolerance_;   // The longest tolerance we've needed on a scan so far
 //filters::FilterChain<sensor_msgs::LaserScan> filter_chain_;
  mutable sensor_msgs::LaserScan scan_filtered_;
  //! \brief Stores history of scans
  std::deque<sensor_msgs::PointCloud> scan_hist_ ;
  boost::mutex scan_hist_mutex_ ;
  //! \brief The max number of scans to store in the scan history
  unsigned int max_scans_ ;
  //! \brief The frame to transform data into upon receipt
  std::string fixed_frame_ ;
  //! \brief The number points currently in the scan history
  unsigned int total_pts_ ;
  //! \brief Specify how much to downsample the data. A value of 1 preserves all the data. 3 would keep 1/3 of the data.
  unsigned int downsample_factor_ ;
};

}//namespace orb_slam2
#endif // ROS_MAPPING_H
