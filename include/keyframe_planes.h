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
// typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

struct KeyframePlanes
{
  int keyInd;
  float minarea,maxarea,minheight,maxheight;
  std::vector<int> planesInd;
  std::vector<std::vector<pcl::PointXYZRGB>> planesPoints;
  KeyframePlanes();
  KeyframePlanes(float minareaInit, float maxareaInit, float minheightInit, float maxheightInit, int keyIndInt, int planeIndInit, std::vector<geometry_msgs::Point32>& planePointsInit, std::vector<uint8_t>& planeColorsInit, int rows, int cols);
  bool checkPatch(std::vector<geometry_msgs::Point32>& planePointsInit, int rows, int cols);
  void addplane(int planeInd, std::vector<geometry_msgs::Point32>& planePointsInit, std::vector<uint8_t>& planeColorsInit, int rows, int cols);
  void update(int planeIndInd, std::vector<geometry_msgs::Point32>& planePointsInit, std::vector<uint8_t>& planeColorsInit, int rows, int cols);
};

#endif
