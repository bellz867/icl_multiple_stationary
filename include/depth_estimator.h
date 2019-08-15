#ifndef DEPTHESTIMATOR_H
#define DEPTHESTIMATOR_H

#define _USE_MATH_DEFINES
#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <deque>

#include <helper_functions.h>
#include <depth_estimator_ekf.h>
#include <depth_estimator_icl_ext.h>

//depth estimator
struct DepthEstimator
{
  int depthInd;
  Eigen::Vector3f mk,mc,pik,uk,uc,ptk,ptc;
  float zcHatEKF;
  float dkHatICLExt;
  float dcHatICLExt;
  float dkcHatICLExt;
  ros::Time lastt;
  ros::Time startt;
  DepthEstimatorEKF depthEstimatorEKF;
  DepthEstimatorICLExt depthEstimatorICLExt;

  DepthEstimator();
  DepthEstimator(int depthIndInit, Eigen::Vector3f mInit, ros::Time t, float zmin, float zmax, float zInit, float tau, float fx, float fy, float cx, float cy);
  Eigen::Vector3f current();
  Eigen::Vector3f predict(Eigen::Vector3f v, Eigen::Vector3f w, float dt);
  float update(Eigen::Vector3f mcMeas, Eigen::Vector3f tkc, Eigen::Matrix3f Rkc, Eigen::Vector3f v, Eigen::Vector3f w, ros::Time t, Eigen::Vector3f pkc, Eigen::Vector4f qkc);
};

#endif
