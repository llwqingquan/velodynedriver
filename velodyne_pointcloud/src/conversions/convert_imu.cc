/*
 *  Copyright (C) 2019 LLW
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class conver imr file to msg file

*/
#include <ros/ros.h>
#include <string>
#include "velodyne_pointcloud/convert_imu.h"
#include <velodyne_driver/time_conversion.hpp>

namespace convert_imu
{
/** @brief Constructor. */
Imu::Imu(ros::NodeHandle node, ros::NodeHandle private_nh)
{
  if (!private_nh.getParam("imu_file", config_.file_name))
  {
    ROS_ERROR("get IMU file error!");
  }

  if (!private_nh.getParam("imu_type", config_.imu_type))
  {
    ROS_ERROR("need to set IMU type!");
  }

  if (!private_nh.getParam("imu_frame_ID", config_.frame_id))
  {
    ROS_ERROR("need to set IMU frame ID!");
  }

  if (!private_nh.getParam("imu_freq_Hz", config_.freq))
  {
    ROS_ERROR("need to set IMU data Frequency!");
  }

  if (config_.imu_type == "ADIS16465")
  {
    config_.gyro_scale = 1.0 / 2982616.178 * config_.freq;
    config_.accel_scale = 1.0 / 21474836.48 * config_.freq;
  }
  else if (config_.imu_type == "STIM300")
  {
    config_.gyro_scale = 1.0 / 2097152.0 * config_.freq;
    config_.accel_scale = 1.0 / 4194304.0 * config_.freq;
  }
  else
  {
    ROS_ERROR("unknown IMU type!");
  }

  private_nh.param("imr_header", config_.imr_header, false);

  // handle imu file
  imuFile_.open(config_.file_name, std::ios::in | std::ios::binary);
  if (imuFile_.is_open())
  {
    ROS_INFO_STREAM("Opening IMU file \"" << config_.file_name << "\"");
  }
  else
  {
    ROS_ERROR("IMU file open failed!");
  }

  imu_.header.seq = 0;
  imu_.header.frame_id = config_.frame_id;
  // creat publish topic
  imu_pub_ = node.advertise<sensor_msgs::Imu>("imu_msg", 5000);
}

/** @brief Destructor. */
Imu::~Imu()
{
  // close file
  imuFile_.close();
}

/** @brief convert imr file to msg. */
bool Imu::process()
{
  if (!imuFile_.is_open())
  {
    ROS_ERROR_THROTTLE(1, "File open failed!");
    return false;
  }
  else if (imuFile_.eof())
  {
    ROS_INFO_THROTTLE(2, "IMU file reach the end!");
    // imuFile_.close();
    return false;
  }
  else
  {
    // ROS_INFO("File open!");
  }

  if (imuFile_.tellg() == 0)
  {
    imuFile_.read(buffer_, 512);
    // imuFile_.seekg(512, std::ios::cur);
    // for(uint8_t i = 0; i<512; i++){
    //   imr_header_.str[i] = buffer_[i];
    // }
    // ROS_INFO_STREAM("IMR Header:"<<imr_header_.bit.szHeader);
    // ROS_INFO_STREAM("IMR bIsIntelOrMotorola:"<<imr_header_.bit.bIsIntelOrMotorola);
    // ROS_INFO_STREAM("IMR iUtcOrGpsTime:"<<imr_header_.bit.iUtcOrGpsTime);
    // ROS_INFO_STREAM("IMR bDeltaTheta:"<<imr_header_.bit.bDeltaTheta);
    // ROS_INFO_STREAM("IMR bDeltaVelocity:"<<imr_header_.bit.bDeltaVelocity);
    // ROS_INFO_STREAM("IMR dDataRateHz:"<<imr_header_.bit.dDataRateHz);
    // ROS_INFO_STREAM("IMR dGyroScaleFactor:"<<imr_header_.bit.dGyroScaleFactor);
    // ROS_INFO_STREAM("IMR dAccelScaleFactor:"<<imr_header_.bit.dAccelScaleFactor);
  }
  else
  {
    imuFile_.read(buffer_, 32);
    // imuFile_.seekg(32, std::ios::cur);
    for (uint8_t i = 0; i < 32; i++)
    {
      imu_buf_.str[i] = buffer_[i];
    }

    // header
    // seq
    imu_.header.seq++;

    // stamp
    // use imu time secs, ros hour
    // uint32_t hourSec  = (uint32_t)(imu_buf_.elem.Time)/3600;
    // uint32_t usec     = (uint32_t)((imu_buf_.elem.Time - (double)hourSec)*1e6);
    // uint8_t usecChar[4];
    // usecChar[0] = (uint8_t)(usec  & 0xff);
    // usecChar[1] = (uint8_t)(usec>>8 & 0xff);
    // usecChar[2] = (uint8_t)(usec>>16 & 0xff);
    // usecChar[3] = (uint8_t)(usec>>24 & 0xff);
    // imu_.header.stamp = rosTimeFromGpsTimestamp(usecChar);

    // use imu data time gpstime, secs top of a week,from rec dsdata
    imu_.header.stamp.sec = (uint32_t)(imu_buf_.elem.Time);
    imu_.header.stamp.nsec = (uint32_t)((imu_buf_.elem.Time - (double)imu_.header.stamp.sec) * 1e9);

    // use ros time
    // imu_.header.stamp = ros::Time::now();

    // gyro
    imu_.angular_velocity.x = imu_buf_.elem.gx * config_.gyro_scale;
    imu_.angular_velocity.y = imu_buf_.elem.gy * config_.gyro_scale;
    imu_.angular_velocity.z = imu_buf_.elem.gz * config_.gyro_scale;
    // accel
    imu_.linear_acceleration.x = imu_buf_.elem.ax * config_.accel_scale;
    imu_.linear_acceleration.y = imu_buf_.elem.ay * config_.accel_scale;
    imu_.linear_acceleration.z = imu_buf_.elem.az * config_.accel_scale;

    // publish imu_msg
    imu_pub_.publish(imu_);
  }
}

}  // namespace convert_imu
