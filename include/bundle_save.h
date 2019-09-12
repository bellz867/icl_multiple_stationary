#ifndef BUNDLESAVE_H
#define BUNDLESAVE_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

struct BundleSave
{
  Eigen::Vector4f qkc;
  Eigen::Vector3f pkc;
  std::vector<Eigen::Vector2f> cPts;
  std::vector<Eigen::Vector3f> piks;
  std::vector<int> is;

  BundleSave();
  BundleSave(Eigen::Vector4f qkcInit, Eigen::Vector3f pkcInit, std::vector<Eigen::Vector2f> cPtsInit, std::vector<Eigen::Vector3f> piksInit, std::vector<int> isInit);
};

#endif
