#ifndef KEYFRAMEPLANES_H
#define KEYFRAMEPLANES_H

// #include <iostream>
// #include <fstream>
// #include <ctime>
#include <vector>

#include <geometry_msgs/Point32.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
//
// #include <Eigen/Dense>
// #include <Eigen/Geometry>
//
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

struct KeyframePlanes
{
  int keyInd;
  float minarea,maxarea,minheight,maxheight;
  std::vector<int> planesInd;
  std::vector<PointCloudRGB> planes;
  KeyframePlanes();
  KeyframePlanes(float minareaInit, float maxareaInit, float minheightInit, float maxheightInit, int keyIndInt, int planeIndInit, PointCloudRGB& cloud);
  // bool checkPatch(std::vector<geometry_msgs::Point32>& planePointsInit);
  void addplane(int planeInd, PointCloudRGB& cloud);
  void update(int planeIndInd, PointCloudRGB& cloud);
};

#endif
