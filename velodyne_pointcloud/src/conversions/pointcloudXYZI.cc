/*
 *  Copyright (C) 2019
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class define the new pointcloud

*/

#include <velodyne_pointcloud/pointcloudXYZI.h>

namespace velodyne_pointcloud
{
// constructor

PointcloudXYZI::PointcloudXYZI(const double max_range, const double min_range, const std::string& target_frame,
                               const std::string& fixed_frame, const unsigned int scans_per_block,
                               boost::shared_ptr<tf::TransformListener> tf_ptr)
  : DataContainerBase(max_range, min_range, target_frame, fixed_frame, 0, 1, true, scans_per_block, tf_ptr, 4, 
                      "x", 1,sensor_msgs::PointField::FLOAT32, 
                      "y", 1, sensor_msgs::PointField::FLOAT32, 
                      "z", 1,sensor_msgs::PointField::FLOAT32, 
                      "intensity", 1, sensor_msgs::PointField::FLOAT32)
  , iter_x(cloud, "x")
  , iter_y(cloud, "y")
  , iter_z(cloud, "z")
  , iter_intensity(cloud, "intensity")
{
}

void PointcloudXYZI::setup(const velodyne_msgs::VelodyneScan::ConstPtr& scan_msg)
{
  DataContainerBase::setup(scan_msg);
  iter_x = sensor_msgs::PointCloud2Iterator<float>(cloud, "x");
  iter_y = sensor_msgs::PointCloud2Iterator<float>(cloud, "y");
  iter_z = sensor_msgs::PointCloud2Iterator<float>(cloud, "z");
  iter_intensity = sensor_msgs::PointCloud2Iterator<float>(cloud, "intensity");
}

void PointcloudXYZI::newLine()
{
  // ++cloud.width;
  // ++iter_x;
  // ++iter_y;
  // ++iter_z;
  // ++iter_intensity;
}

void PointcloudXYZI::addPoint(float x, float y, float z, const uint16_t ring, const uint16_t azimuth,
                              const float distance, const float intensity, const float time)
{
  if (!pointInRange(distance))
    return;

  // convert polar coordinates to Euclidean XYZ
  if (config_.transform)
    transformPoint(x, y, z);

  *iter_x = x;
  *iter_y = y;
  *iter_z = z;
  *iter_intensity = intensity;

  ++cloud.width;
  ++iter_x;
  ++iter_y;
  ++iter_z;
  ++iter_intensity;
}

// PointcloudXYZI::PointcloudXYZI(): iter_x_(cloud_msg_, "x"),
//                                   iter_y_(cloud_msg_, "y"),
//                                   iter_z_(cloud_msg_, "z"),
//                                   iter_intensity_(cloud_msg_, "intensity")
// {

//   cloud_msg_.fields.clear();
//   cloud_msg_.fields.reserve(4);

//   offset_ = addPointField(cloud_msg_, "x", 1, sensor_msgs::PointField::FLOAT32, 0);
//   offset_ = addPointField(cloud_msg_, "y", 1, sensor_msgs::PointField::FLOAT32, offset_);
//   offset_ = addPointField(cloud_msg_, "z", 1, sensor_msgs::PointField::FLOAT32, offset_);
//   offset_ = addPointField(cloud_msg_, "intensity", 1, sensor_msgs::PointField::FLOAT32, offset_);
//   cloud_msg_.point_step = offset_;
//   cloud_msg_.row_step = 1 * cloud_msg_.point_step;
//   iter_x_ = sensor_msgs::PointCloud2Iterator<float>(cloud_msg_, "x");
//   iter_y_ = sensor_msgs::PointCloud2Iterator<float>(cloud_msg_, "y");
//   iter_z_ = sensor_msgs::PointCloud2Iterator<float>(cloud_msg_, "z");
//   iter_intensity_ = sensor_msgs::PointCloud2Iterator<float>(cloud_msg_, "intensity");

// }

// void PointcloudXYZI::newLine()
// {
//   // iter_x_ = iter_x_ + 1;
//   // iter_y_ = iter_y_ + 1;
//   // iter_z_ = iter_z_ + 1;
//   // iter_intensity_ = iter_intensity_ + 1;

//   // ++cloud.height;
// }

// void PointcloudXYZI::addPoint(float x, float y, float z,float intensity)
// {
//   // *iter_x_ = x;
//   // *iter_y_ = y;
//   // *iter_z_ = z;
//   // *iter_intensity_ = intensity;

//   // ++iter_x_;
//   // ++iter_y_;
//   // ++iter_z_;
//   // ++iter_intensity_;

// }

// const sensor_msgs::PointCloud2& PointcloudXYZI::getCloudMsg(const sensor_msgs::PointCloud2ConstPtr& msg)
// {
//   // cloud.data.resize(cloud.point_step * cloud.width * cloud.height);

//   // if (!config_.target_frame.empty())
//   // {
//   //   cloud.header.frame_id = config_.target_frame;
//   // }

//   // ROS_DEBUG_STREAM("Prepared cloud width" << cloud.height * cloud.width
//   //                                         << " Velodyne points, time: " << cloud.header.stamp);

//   cloud_msg_.header = msg->header;
//   // cloud_msg_.data.resize(scan_msg->packets.size() * config_.scans_per_packet * cloud.point_step);
//   cloud_msg_.width = msg->width;
//   cloud_msg_.height = msg->height;
//   cloud_msg_.is_dense = msg->is_dense;

//   sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
//   sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
//   sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
//   sensor_msgs::PointCloud2ConstIterator<float> it_i(*msg, "intensity");
//   sensor_msgs::PointCloud2ConstIterator<float> it_t(*msg, "time");
//   sensor_msgs::PointCloud2ConstIterator<uint16_t> it_r(*msg, "ring");

//   for ( ; it_x != it_x.end(); ++it_x, ++it_y, ++it_i)
//   {
//       const float x = *it_x;  // x
//       const float y = *it_y;  // y
//       const float z = *it_z;  // z
//       const float i = *it_i;  // intensity

//       addPoint(x,y,z,i);
//   }

//   return cloud_msg_;
// }

}  // namespace velodyne_pointcloud
