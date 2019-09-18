#include <cloud_data_save.h>

CloudDataSave::CloudDataSave()
{}

CloudDataSave::CloudDataSave(float timeInit, std::vector<float> dksInit, std::vector<float> dkHatsInit, std::vector<uint8_t> dkKnownsInit)
{
  time = timeInit;
  dks = dksInit;
  dkHats = dkHatsInit;
  dkKnowns = dkKnownsInit;
}
