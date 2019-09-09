#include <depth_estimator.h>

DepthEstimator::DepthEstimator()
{}

DepthEstimator::DepthEstimator(int depthIndInit, Eigen::Vector3f mInit, ros::Time t, float zmin, float zmax, float zInit, float tau, float fx, float fy, float cx, float cy)
{
  badCount = 0;
  depthInd = depthIndInit;
  mk = mInit;
  mc = mk;
  ptk(0) = fx*mk(0)+cx;
  ptk(1) = fx*mk(1)+cy;
  ptk(2) = 1.0;
  uk = mk/mk.norm();
  uc = uk;
  lastt = t;
  zcHatEKF = zInit;
  dcHatICLExt = zInit;
  dkHatICLExt = zInit;
  dkcHatICLExt = 0.0;
  startt = t;

  depthEstimatorEKF.initialize(mk.segment(0,2),zmin,zmax,zInit);
  depthEstimatorICLExt.initialize(uk,zmin,zmax,zInit,tau,t,fx,fy,cx,cy);
}

Eigen::Vector3f DepthEstimator::current()
{
  Eigen::Vector3f mcEKF = depthEstimatorEKF.current();
  Eigen::Vector3f mcICL = depthEstimatorICLExt.current();
  return mcICL;
}

Eigen::Vector3f DepthEstimator::predict(Eigen::Vector3f v, Eigen::Vector3f w, float dt, Eigen::Vector3f pkc, Eigen::Vector4f qkc)
{
  Eigen::Vector3f mcEKF = depthEstimatorEKF.predict(v,w,dt);
  Eigen::Vector3f mcICL = depthEstimatorICLExt.predict(v,w,dt,pkc,qkc);
  return mcICL;
}

float DepthEstimator::update(Eigen::Vector3f mcMeas, Eigen::Vector3f tkc, Eigen::Matrix3f Rkc, Eigen::Vector3f v, Eigen::Vector3f w, ros::Time t, Eigen::Vector3f pkc, Eigen::Vector4f qkc)
{
  // std::cout << "\n hi1 \n";
  float dt = (t - lastt).toSec();
  lastt = t;

  Eigen::Vector3f xHatEKF = depthEstimatorEKF.update(mcMeas.segment(0,2));
  mc(0) = xHatEKF(0);
  mc(1) = xHatEKF(1);
  zcHatEKF = 1.0/xHatEKF(2);

  // std::cout << "\n hi2 \n";

  // float mcTau = 1.0/(2.0*M_PI*120.0);
  // float kmc = dt/(mcTau + dt);
  // mc += kmc*(mcMeas - mc);
  // mc(2) = 1.0;

  if (tkc.norm() < 0.001)
  {
    mk = mcMeas;
    return dkcHatICLExt;
  }

  uc = mcMeas/mcMeas.norm();
  // Eigen::Vector3f ukc = tkc/tkc.norm();

  // std::cout << "\n hi3 \n";

  Eigen::Vector3f dkdcdkcICLExt = depthEstimatorICLExt.update(uc,tkc,Rkc,v,w,t,dt,pkc,qkc);
  dkHatICLExt = dkdcdkcICLExt(0);
  dcHatICLExt = dkdcdkcICLExt(1);
  dkcHatICLExt = dkdcdkcICLExt(2);

  return dkcHatICLExt;
}
