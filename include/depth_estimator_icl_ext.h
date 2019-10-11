#ifndef DEPTHESTIMATORICLExt_H
#define DEPTHESTIMATORICLExt_H

#define _USE_MATH_DEFINES
#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <deque>
#include <helper_functions.h>
#include <vector_derivative_estimator.h>
#include <icl_save.h>

//depth estimator
struct DepthEstimatorICLExt
{
  ICLSave* buffNew;
  std::deque<ICLSave*> buffs;
  // std::deque<Eigen::Vector3f> pkcBuff;
  // std::deque<Eigen::Vector2f> psiBuff;
  // std::deque<Eigen::Vector2f> psiDotBuff;
  // std::deque<Eigen::Vector2f> uvBuff;
  // std::deque<ros::Time> tBuff;
  // std::deque<float> dtBuff;
  Eigen::Vector2f uvInt;
  Eigen::Vector2f psiDotInt;
  Eigen::Vector2f psiLast;
  ros::Time tLastSave;
  Eigen::Vector3f pkcLastSave;
  float yysum,yusum;
  Eigen::Vector2f kPtInit;
  Eigen::Vector3f uk;
  Eigen::Matrix2f W;
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
  float fx,fy,cx,cy;
  ros::Time startTime;
  Eigen::Vector3f up;
  bool firstUpdate;

  DepthEstimatorICLExt();
  void initialize(Eigen::Vector3f uInit, float zminInit, float zmaxInit, float zInit, float tauInit, ros::Time t,
                  float fxInit, float fyInit, float cxInit, float cyInit);
  Eigen::Vector3f current();
  Eigen::VectorXf update(Eigen::Vector3f mc, Eigen::Vector3f tkc, Eigen::Matrix3f Rkc, Eigen::Vector3f v, Eigen::Vector3f w, ros::Time t, float dt,  Eigen::Vector3f pkc, Eigen::Vector4f qkc);
  Eigen::Vector3f predict(Eigen::Vector3f v, Eigen::Vector3f w, float dt, Eigen::Vector3f pkc, Eigen::Vector4f qkc);
};

#endif
