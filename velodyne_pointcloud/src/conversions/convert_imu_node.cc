/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

    This ROS node convert imr file to msg file

*/

#include <ros/ros.h>
#include "velodyne_pointcloud/convert_imu.h"

/** Main node entry point. */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "convert_imu_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  // create record class, which subscribes to pointcloud data
  convert_imu::Imu con_imu(node, priv_nh);
  
  //wait the record bag open
  ros::Duration(2).sleep();

  ros::Rate loop_rate(5000);
  // loop until shut down or end of file
  while (ros::ok())
  {
    // poll device until end of file
    bool succ = con_imu.process();

    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
