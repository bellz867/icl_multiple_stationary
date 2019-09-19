#ifndef KEYFRAMEPLANES_H
#define KEYFRAMEPLANES_H

// #include <iostream>
// #include <fstream>
// #include <ctime>
#include <vector>
#include <fstream>

#include <ros/ros.h>
#include <geometry_msgs/Point32.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <cloud_data_save.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
//
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

struct KeyframePlanes
{
  int keyInd;
  float minarea,maxarea,minheight,maxheight;
  bool allPtsKnown;
  bool saveExp;
  std::string expName;
  std::vector<uint8_t> dkKnowns;
  std::vector<uint8_t> inds;
  std::vector<int> planesInd;
  std::vector<PointCloudRGB> planes,planesTrue;
  PointCloudRGB planesDraw,planesTrueDraw;
  Eigen::Vector3f wBL,wCL,wTL,wTR,wCR,wBR;
  Eigen::Vector3f nBL,nCL,nTL,nTR,nCR,nBR;
  CloudDataSave* cloudDataSave;
  std::vector<CloudDataSave*> cloudDataSaves;
  ros::Time startTime;
  ~KeyframePlanes();
  KeyframePlanes();
  KeyframePlanes(float minareaInit, float maxareaInit, float minheightInit, float maxheightInit, int keyIndInt, int planeIndInit,
                 PointCloudRGB& cloud, Eigen::Vector3f pcw, Eigen::Vector4f qcw, Eigen::Vector3f pcwHat,Eigen::Vector4f qcwHat,
                 PointCloudRGB& cloudTrueInit, bool allPtsKnownInit, std::vector<uint8_t> indsInit, std::vector<uint8_t> dkKnownsInit,
                 bool saveExpInit, std::string expNameInit, ros::Time t);
  // bool checkPatch(std::vector<geometry_msgs::Point32>& planePointsInit);
  void addplane(int planeInd, PointCloudRGB& cloud, Eigen::Vector3f pcw, Eigen::Vector4f qcw, Eigen::Vector3f pcwHat, Eigen::Vector4f qcwHat);
  void update(int planeIndInd, PointCloudRGB& cloud, Eigen::Vector3f pcw, Eigen::Vector4f qcw, Eigen::Vector3f pcwHat, Eigen::Vector4f qcwHat, bool allPtsKnownInit, std::vector<uint8_t> indsInit, std::vector<uint8_t> dkKnownsInit, ros::Time t);
};

#endif
