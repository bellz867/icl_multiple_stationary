#ifndef ICLSAVE_H
#define ICLSAVE_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

struct ICLSave
{
  Eigen::Vector2f psi;
  Eigen::Vector2f psiDot;
  Eigen::Vector2f uv;
  ros::Time t;
  float dt;

  ICLSave();
  ICLSave(Eigen::Vector2f psiInit, Eigen::Vector2f psiDotInit, Eigen::Vector2f uvInit, ros::Time t, float dt);
};

#endif
