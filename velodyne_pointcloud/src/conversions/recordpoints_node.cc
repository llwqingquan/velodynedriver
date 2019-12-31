/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

    This ROS node record PointCloud2 in bag file.

*/

#include <ros/ros.h>
#include "velodyne_pointcloud/record.h"

/** Main node entry point. */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "recordpoints_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  // create record class, which subscribes to pointcloud data
  velodyne_pointcloud::Record record(node, priv_nh);
  // handle callbacks until shut down
  ros::spin();

  return 0;
}
