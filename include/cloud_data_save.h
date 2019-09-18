#ifndef CLOUDDATASAVE_H
#define CLOUDDATASAVE_H

#include <iostream>
#include <vector>

struct CloudDataSave
{
  float time;
  std::vector<float> dks;
  std::vector<float> dkHats;
  std::vector<uint8_t> dkKnowns;

  CloudDataSave();
  CloudDataSave(float timeInit, std::vector<float> dksInit, std::vector<float> dkHatsInit, std::vector<uint8_t> dkKnownsInit);
};

#endif
