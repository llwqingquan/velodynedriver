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
    ROS_ERROR("recordBag: please set the bag file directory!");
  }
  if (!private_nh.getParam("topic_name_lidar", config_.topic_name_lidar))
  {
    ROS_ERROR("recordBag: please set lidar topic name!");
  }
  if (!private_nh.getParam("topic_name_imu", config_.topic_name_imu))
  {
    ROS_ERROR("recordBag: please set imu topic name!");
  }
  ros::Time Time = ros::Time::now();
  time_t timesec = Time.sec;
  tm* temp = localtime(&timesec);
  std::string filename = std::to_string(temp->tm_year + 1900) + "-" + std::to_string(temp->tm_mon + 1) + "-" +
                         std::to_string(temp->tm_mday) + "-" + std::to_string(temp->tm_hour) + "-" +
                         std::to_string(temp->tm_min) + "-" + std::to_string(temp->tm_sec);
  std::string str = config_.bag_dir + filename + ".bag";
  // open bag file and set compression mode
  bag_.open(str, rosbag::bagmode::Write);
  bag_.setCompression(rosbag::CompressionType::LZ4);

  // subscribe to velodyne_points
  sub1_ = node.subscribe("velodyne_points", 10000, &Record::recordpoints, (Record*)this);
  // subscribe to imu_msg
  sub2_ = node.subscribe("imu_msg", 5000, &Record::recordimu, (Record*)this);
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
  bag_.write(config_.topic_name_lidar, msg->header.stamp, msg);
  ROS_INFO_THROTTLE(2, "bag is writing: point_cloud!");
}

/** @brief Callback for record imu data. */
void Record::recordimu(const sensor_msgs::Imu imu_msg)
{
  // bag_.write("imu", ros::Time::now(), imu_msg);
  bag_.write(config_.topic_name_imu, imu_msg.header.stamp, imu_msg);
  ROS_INFO_THROTTLE(2, "bag is writing: imu_data!");
}
}  // namespace velodyne_pointcloud
