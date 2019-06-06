#include <depth_estimator.h>

DepthEstimator::DepthEstimator()
{}

DepthEstimator::DepthEstimator(int depthIndInit, Eigen::Vector3f mInit, ros::Time t, float zminInit, float zmaxInit, float tauInit)
{
  depthInd = depthIndInit;
  mk = mInit;
  mc = mk;
  uk = mk/mk.norm();
  uc = uk;
  lastt = t;
  zmin = zminInit;
  zmax = zmaxInit;
  zcHatEKF = zmin;
  dcHatICLExt = zmin;
  dkcHatICLExt = 0.0;
  tau = tauInit;
  startt = t;
  firstzk = true;
  dkKnown = false;
  uvInt = Eigen::Vector2f::Zero();

  depthEstimatorEKF.initialize(mk.segment(0,2),zmin,zmax);
  depthEstimatorICLExt.initialize(uk,zmin,zmax,tau,t);
}

Eigen::Vector3f DepthEstimator::predict(Eigen::Vector3f v, Eigen::Vector3f w, float dt)
{
  return (depthEstimatorEKF.predict(v,w,dt));
}

float DepthEstimator::update(Eigen::Matrix3f H, Eigen::Vector3f mcMeas, Eigen::RowVector3f nkT, Eigen::Vector3f tkc, Eigen::Matrix3f Rkc, Eigen::Vector3f v, Eigen::Vector3f w, ros::Time t, Eigen::Vector3f pkc, Eigen::Vector4f qkc)
{
  float dt = (t - lastt).toSec();
  lastt = t;

  zcHatEKF = depthEstimatorEKF.update(mcMeas.segment(0,2));

  float mcTau = 1.0/(2.0*M_PI*120.0);
  float kmc = dt/(mcTau + dt);
  mc += kmc*(mcMeas - mc);
  mc(2) = 1.0;

  if (tkc.norm() < 0.001)
  {
    mk = mc;
    return dkcHatICLExt;
  }

  uc = mc/mc.norm();
  Eigen::Vector3f ukc = tkc/tkc.norm();

  Eigen::Vector3f dkdcdkcICLExt = depthEstimatorICLExt.update(uc,ukc,Rkc,v,w,pkc,t,dt);
  dcHatICLExt = dkdcdkcICLExt(1);
  dkcHatICLExt = dkdcdkcICLExt(2);

  return dkcHatICLExt;
}
