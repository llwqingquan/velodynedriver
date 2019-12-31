/*
 *  Copyright (C) 2019 LLW
 *
 *  $Id$
 */

/** @file

    This class record points in type:PointCloud2.

*/

#include "velodyne_pointcloud/record.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include <string>
#include <ros/ros.h>

namespace velodyne_pointcloud
{
/** @brief Constructor. */
Record::Record(ros::NodeHandle node, ros::NodeHandle private_nh)
{
  // name of file
  // std::string str = "/home/llw/data1/bag/";
  ///// when use private_nh.param you have to set an default value
  ///// private_nh.getParam does not need an default value
  // private_nh.param("bag_dir",config_.bag_dir,std::string("/home/llw/data1/bag/"));
  if (!private_nh.getParam("bag_dir", config_.bag_dir))
  {
    ROS_ERROR("set the bag file directory!");
  }
  ros::Time Time = ros::Time::now();
  uint32_t sec = Time.sec;
  std::string str = config_.bag_dir + std::to_string(sec) + ".bag";
  // open bag file and set compression mode
  bag_.open(str, rosbag::bagmode::Write);
  bag_.setCompression(rosbag::CompressionType::LZ4);

  // subscribe to velodyne_points
  sub1_ = node.subscribe("velodyne_points", 5000, &Record::recordpoints, (Record*)this);
  // subscribe to imu_msg
  // sub2_ = node.subscribe("imu_msg", 5000, &Record::recordimu, (Record*)this);
}

/** @brief Destructor. */
Record::~Record()
{
  // close bag file
  bag_.close();
}

/** @brief Callback for record pointcloud data. */
void Record::recordpoints(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  // bag_.write("velodyne_points", ros::Time::now(), msg);
  bag_.write("velodyne_points", msg->header.stamp, msg);
  ROS_INFO_THROTTLE(1, "velodyne_points is writing!");
}

/** @brief Callback for record imu data. */
void Record::recordimu(const sensor_msgs::Imu imu_msg)
{
  // bag_.write("imu", ros::Time::now(), imu_msg);
  bag_.write("imu", imu_msg.header.stamp, imu_msg);
  ROS_INFO_THROTTLE(1, "imu_msg is writing!");
}
}  // namespace velodyne_pointcloud
