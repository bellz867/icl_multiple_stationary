#ifndef DEPTHESTIMATORICLExt_H
#define DEPTHESTIMATORICLExt_H

#define _USE_MATH_DEFINES
#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <deque>
#include <helper_functions.h>
#include <vector_derivative_estimator.h>

//depth estimator
struct DepthEstimatorICLExt
{
  std::deque<Eigen::Vector2f> psiBuff;
  std::deque<Eigen::Vector2f> psiDotBuff;
  std::deque<Eigen::Vector2f> uvBuff;
  Eigen::Vector2f uvInt;
  Eigen::Vector2f psiDotInt;
  std::deque<ros::Time> tBuff;
  std::deque<float> dtBuff;
  ros::Time tLastSave;
  Eigen::Vector3f pkcLastSave;
  float yysum,yusum;
  Eigen::Vector3f uk;
  float dkHat;
  float dcHat;
  float dkcHat;
  float zmin;
  float zmax;
  float tau;
  bool firstzk;
  bool dkKnown;
  int numSaved,numThrown;
  VectorDerivativeEstimator uDotEstimator;
  VectorDerivativeEstimator psiDotEstimator;
  float timeConverge;

  DepthEstimatorICLExt();
  void initialize(Eigen::Vector3f uInit, float zminInit, float zmaxInit, float zInit, float tauInit, ros::Time t);
  Eigen::Vector3f current();
  Eigen::Vector3f update(Eigen::Vector3f mc, Eigen::Vector3f tkc, Eigen::Matrix3f Rkc, Eigen::Vector3f v, Eigen::Vector3f w, Eigen::Vector3f pkc, ros::Time t, float dt);
  Eigen::Vector3f predict(Eigen::Vector3f v, Eigen::Vector3f w, float dt);
};

#endif
