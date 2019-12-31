/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.

*/

#ifndef _VELODYNE_RECORD_POINT_H_
#define _VELODYNE_RECORD_POINT_H_ 1

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <velodyne_pointcloud/pointcloudXYZI.h>

namespace velodyne_pointcloud
{
  class Record
  {
  public:

    Record(ros::NodeHandle node, ros::NodeHandle private_nh);
    ~Record();
  
    //  sensor_msgs::PointCloud2 cloud_msg_;

  private:
    
    void recordpoints(const sensor_msgs::PointCloud2ConstPtr& msg);
  
    void recordimu(const sensor_msgs::Imu imu_msg);

    //boost::shared_ptr<velodyne_rawdata::RawData> data_;
    rosbag::Bag bag_;

    ros::Subscriber sub1_, sub2_;

    sensor_msgs::PointCloud2 last_msg_;

    struct
    {
      std::string bag_dir;            // 
    } config_;

    // boost::shared_ptr<PointcloudXYZI> cont_ptr_;
    
    // PointcloudXYZI cloud_msg_;

  };

} // namespace velodyne_recordpoint

#endif // _VELODYNE_RECORD_POINT_H_
