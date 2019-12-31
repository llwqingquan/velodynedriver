/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2019,LLW
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.

*/

#ifndef _CONVERT_IMU_H_
#define _CONVERT_IMU_H_

#include <ros/ros.h>
#include <fstream>
#include <sensor_msgs/Imu.h>

namespace convert_imu
{
class Imu
{
public:
  Imu(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~Imu();

  bool process();
  ros::Publisher imu_pub_;

private:
  // configuration parameters
  struct
  {
    std::string file_name;  //
    std::string imu_ID;
    bool imr_header;
    double gyro_scale;
    double accel_scale;
    double freq;
  } config_;

  std::ifstream imuFile_;

  sensor_msgs::Imu imu_;

  struct imu_def
  {
    double Time;
    int32_t gx;
    int32_t gy;
    int32_t gz;
    int32_t ax;
    int32_t ay;
    int32_t az;
  };

  union imu_buf_def
  {
    char str[32];
    struct imu_def elem;
  };

  union imu_buf_def imu_buf_;

  char buffer_[512];

  // struct imr_header_def {
  //   char    szHeader[8];
  //   int8_t  bIsIntelOrMotorola;
  //   double  dVersionNumber;
  //   int32_t bDeltaTheta;
  //   int32_t bDeltaVelocity;
  //   double  dDataRateHz;
  //   double  dGyroScaleFactor;
  //   double  dAccelScaleFactor;
  //   int32_t iUtcOrGpsTime;
  //   int32_t iRcvTimeOrCorrTime;
  //   double  dTimeTagBias;
  //   char    szImuName[32];
  //   uint8_t reserved1[4];
  //   char    szProgramName[32];
  //   uint8_t tCreate[12];
  //   bool    bLeverArmValid;
  //   int32_t IXoffset;
  //   int32_t IYoffset;
  //   int32_t IZoffset;
  //   int8_t  Reserved[354];
  // };

  // union imr_head_buf_def{
  //     char str[512];
  //     struct imr_header_def bit;
  // };

  // imr_head_buf_def imr_header_;

  // ros::Subscriber velodyne_points_;

  // boost::shared_ptr<PointcloudXYZI> cont_ptr_;

  // PointcloudXYZI cloud_msg_;
};

}  // namespace convert_imu

#endif  // _CONVERT_IMR_H_
