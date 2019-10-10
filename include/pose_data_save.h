#ifndef POSEDATASAVE_H
#define POSEDATASAVE_H

#include <iostream>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

struct PoseDataSave
{
  float time;
  Eigen::Vector3f pbw;
  Eigen::Vector4f qbw;
  Eigen::Vector3f pbwHat;
  Eigen::Vector4f qbwHat;
  int mode;

  PoseDataSave();
  PoseDataSave(float timeInit, Eigen::Vector3f pbwInit, Eigen::Vector4f qbwInit, Eigen::Vector3f pbwHatInit, Eigen::Vector4f qbwHatInit, int modeInit);
};

#endif
