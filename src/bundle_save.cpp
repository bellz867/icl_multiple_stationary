#include <bundle_save.h>

BundleSave::BundleSave()
{}

BundleSave::BundleSave(Eigen::Vector4f qkcInit, Eigen::Vector3f pkcInit, std::vector<Eigen::Vector2f> cPtsInit, std::vector<Eigen::Vector3f> piksInit, std::vector<int> isInit)
{
  qkc = qkcInit;
  pkc = pkcInit;
  cPts = cPtsInit;
  piks = piksInit;
  is = isInit;
}
