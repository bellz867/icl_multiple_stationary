#include <depth_estimator.h>

DepthEstimator::DepthEstimator()
{}

DepthEstimator::DepthEstimator(int depthIndInit, Eigen::Vector3f mInit, ros::Time t, float zminInit, float zmaxInit, float zInit, float tauInit)
{
  depthInd = depthIndInit;
  mk = mInit;
  mc = mk;
  uk = mk/mk.norm();
  uc = uk;
  lastt = t;
  zmin = zminInit;
  zmax = zmaxInit;
  zcHatEKF = zInit;
  dcHatICLExt = zInit;
  dkcHatICLExt = 0.0;
  tau = tauInit;
  startt = t;
  firstzk = true;
  dkKnown = false;
  uvInt = Eigen::Vector2f::Zero();

  depthEstimatorEKF.initialize(mk.segment(0,2),zmin,zmax,zInit);
  depthEstimatorICLExt.initialize(uk,zmin,zmax,zInit,tau,t);
}

Eigen::Vector3f DepthEstimator::currentPoint()
{
  return Eigen::Vector3f(depthEstimatorEKF.xHat(0),depthEstimatorEKF.xHat(1),1.0);
}

Eigen::Vector3f DepthEstimator::predict(Eigen::Vector3f v, Eigen::Vector3f w, float dt)
{
  return (depthEstimatorEKF.predict(v,w,dt));
}

float DepthEstimator::update(Eigen::Matrix3f H, Eigen::Vector3f mcMeas, Eigen::RowVector3f nkT, Eigen::Vector3f tkc, Eigen::Matrix3f Rkc, Eigen::Vector3f v, Eigen::Vector3f w, ros::Time t, Eigen::Vector3f pkc, Eigen::Vector4f qkc)
{
  // std::cout << "\n hi1 \n";
  float dt = (t - lastt).toSec();
  lastt = t;

  zcHatEKF = depthEstimatorEKF.update(mcMeas.segment(0,2));

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
  Eigen::Vector3f ukc = tkc/tkc.norm();

  // std::cout << "\n hi3 \n";

  Eigen::Vector3f dkdcdkcICLExt = depthEstimatorICLExt.update(uc,ukc,Rkc,v,w,pkc,t,dt);
  dcHatICLExt = dkdcdkcICLExt(1);
  dkcHatICLExt = dkdcdkcICLExt(2);

  return dkcHatICLExt;
}
