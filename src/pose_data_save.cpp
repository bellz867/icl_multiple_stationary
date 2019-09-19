#include <pose_data_save.h>

PoseDataSave::PoseDataSave()
{}

PoseDataSave::PoseDataSave(float timeInit, Eigen::Vector3f pbwInit, Eigen::Vector4f qbwInit, Eigen::Vector3f pbwHatInit, Eigen::Vector4f qbwHatInit)
{
  time = timeInit;
  pbw = pbwInit;
  qbw = qbwInit;
  pbwHat = pbwHatInit;
  qbwHat = qbwHatInit;
}
