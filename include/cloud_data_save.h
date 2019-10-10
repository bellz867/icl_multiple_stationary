#ifndef CLOUDDATASAVE_H
#define CLOUDDATASAVE_H

#include <iostream>
#include <vector>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <cloud_data_save.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
//
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

struct CloudDataSave
{
  float time;
  PointCloudRGB cloudTrue;
  PointCloudRGB cloudHat;
  Eigen::Vector3f pcw;
  Eigen::Vector3f pcwHat;
  std::vector<uint8_t> dkKnowns;

  CloudDataSave();
  CloudDataSave(float timeInit, PointCloudRGB cloudTrueInit, PointCloudRGB cloudHatInit, Eigen::Vector3f pcwInit, Eigen::Vector3f pcwHatInit, std::vector<uint8_t> dkKnownsInit);
};

#endif
