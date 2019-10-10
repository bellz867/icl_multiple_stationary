#include <cloud_data_save.h>

CloudDataSave::CloudDataSave()
{}

CloudDataSave::CloudDataSave(float timeInit, PointCloudRGB cloudTrueInit, PointCloudRGB cloudHatInit, Eigen::Vector3f pcwInit, Eigen::Vector3f pcwHatInit, std::vector<uint8_t> dkKnownsInit)
{
  time = timeInit;
  cloudTrue = cloudTrueInit;
  cloudHat = cloudHatInit;
  pcw = pcwInit;
  pcwHat = pcwHatInit;
  dkKnowns = dkKnownsInit;
}
