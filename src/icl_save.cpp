#include <icl_save.h>

ICLSave::ICLSave()
{}

ICLSave::ICLSave(Eigen::Vector2f psiInit, Eigen::Vector2f psiDotInit, Eigen::Vector2f uvInit, ros::Time tInit, float dtInit)
{
  psi = psiInit;
  psiDot = psiDotInit;
  uv = uvInit;
  t = tInit;
  dt = dtInit;
}
