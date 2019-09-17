#include <keyframe_planes.h>

KeyframePlanes::KeyframePlanes()
{
}

KeyframePlanes::KeyframePlanes(float minareaInit, float maxareaInit, float minheightInit, float maxheightInit, int keyIndInit, int planeIndInit, PointCloudRGB& cloud, Eigen::Vector3f pcw, Eigen::Vector4f qcw, Eigen::Vector3f pcwHat, Eigen::Vector4f qcwHat, PointCloudRGB& cloud_true, bool allPtsKnownInit, std::vector<uint8_t> indsInit, std::vector<uint8_t> dkKnownsInit)
{
  allPtsKnown = allPtsKnownInit;
  dkKnowns = dkKnownsInit;
  inds = indsInit;
  minarea = minareaInit;
  maxarea = maxareaInit;
  minheight = minheightInit;
  maxheight = maxheightInit;

  // std::cout << "\n keyInd " << keyIndInit << " planeInd " << planeIndInit << " planesPoints init entered " << std::endl;
  keyInd = keyIndInit;
  planesInd.push_back(planeIndInit);
  planes.push_back(cloud);
  // cloudTrue
  // planesTrue.push_back(cloudTrue);

  //get the lines for the 6 walls
  // cloud_true.at(0) = blL;
  // cloud_true.at(1) = brL;
  // cloud_true.at(2) = trL;
  // cloud_true.at(3) = tlL;
  // cloud_true.at(4) = blR;
  // cloud_true.at(5) = brR;
  // cloud_true.at(6) = trR;
  // cloud_true.at(7) = tlR;

  // lines describing each wall
  wBL = Eigen::Vector3f(cloud_true.at(0).x,cloud_true.at(0).y,cloud_true.at(1).x);
  wCL = Eigen::Vector3f(cloud_true.at(0).x,cloud_true.at(0).y,cloud_true.at(3).y);
  wTL = Eigen::Vector3f(cloud_true.at(0).x,cloud_true.at(3).y,cloud_true.at(1).x);
  wTR = Eigen::Vector3f(cloud_true.at(7).x,cloud_true.at(6).y,cloud_true.at(6).x);
  wCR = Eigen::Vector3f(cloud_true.at(6).x,cloud_true.at(0).y,cloud_true.at(3).y);
  wBR = Eigen::Vector3f(cloud_true.at(7).x,cloud_true.at(5).y,cloud_true.at(6).x);

  //normals to each wall
  nBL = Eigen::Vector3f(0,-1,0);
  nCL = Eigen::Vector3f(-1,0,0);
  nTL = Eigen::Vector3f(0,1,0);
  nTR = Eigen::Vector3f(0,1,0);
  nCR = Eigen::Vector3f(1,0,0);
  nBR = Eigen::Vector3f(0,-1,0);

  // approximate the true distance for each point to each wall
  Eigen::Vector3f piw(0.0,0.0,0.0);
  Eigen::Vector3f pic(0.0,0.0,0.0);
  Eigen::Vector3f uic(0.0,0.0,0.0);
  float nu = 0;
  float dic = 0;
  float dicmin = 0;
  std::vector<float> dics(6);

  PointCloudRGB cloudTrue;
  cloudTrue.clear();
  cloudTrue.height = 1;
  cloudTrue.width = cloud.width;
  cloudTrue.is_dense = true;
  cloudTrue.resize(cloud.size());

  // std::cout << "\n cloud width " << cloud.width << " cloud true width " << cloudTrue.width;
  // std::cout << "\n cloud size " << cloud.size() << " cloud true size " << cloudTrue.size();
  PointCloudRGB::iterator itcT = cloudTrue.begin();
  pcl::PointXYZRGB ptxyz;
  int numberPts = 0;
  for (PointCloudRGB::iterator itc = cloud.begin(); itc != cloud.end(); itc++)
  {
    //subtract off approximated position to get direction
    piw << (*itc).x,(*itc).y,(*itc).z;
    pic = piw - pcwHat;
    uic = pic/pic.norm();
    bool firstnu = true;

    // std::cout << "\n piwHatx " << piw(0) << ", piwHaty " << piw(1) << ", piwHatz " << piw(2);

    // get the distance to each wall if the dot product is positive
    //BL
    nu = nBL(1)*uic(1);
    if (nu > 0)
    {
      dic = fabsf(pcw(1)-wBL(1))/nu;
      if (((pcw(0)+uic(0)*dic) <= wBL(0)) && ((pcw(0)+uic(0)*dic) >= wBL(2)))
      {
        dicmin = dic;
        firstnu = false;
      }
    }

    //CL
    nu = nCL(0)*uic(0);
    if (nu > 0)
    {
      dic = fabsf(pcw(0)-wCL(0))/nu;
      if (firstnu)
      {
        if (((pcw(1)+uic(1)*dic) <= wCL(1)) && ((pcw(1)+uic(1)*dic) >= wCL(2)))
        {
          dicmin = dic;
          firstnu = false;
        }
      }
      else
      {
        if (dic < dicmin)
        {
          if (((pcw(1)+uic(1)*dic) <= wCL(1)) && ((pcw(1)+uic(1)*dic) >= wCL(2)))
          {
            dicmin = dic;
          }
        }
      }
    }

    //TL
    nu = nTL(1)*uic(1);
    if (nu > 0)
    {
      dic = fabsf(pcw(1)-wTL(1))/nu;
      if (firstnu)
      {
        if (((pcw(0)+uic(0)*dic) <= wTL(0)) && ((pcw(0)+uic(0)*dic) >= wTL(2)))
        {
          dicmin = dic;
          firstnu = false;
        }
      }
      else
      {
        if (dic < dicmin)
        {
          if (((pcw(0)+uic(0)*dic) <= wTL(0)) && ((pcw(0)+uic(0)*dic) >= wTL(2)))
          {
            dicmin = dic;
          }
        }
      }
    }

    //TR
    nu = nTR(1)*uic(1);
    if (nu > 0)
    {
      dic = fabsf(pcw(1)-wTR(1))/nu;
      if (firstnu)
      {
        if (((pcw(0)+uic(0)*dic) <= wTR(0)) && ((pcw(0)+uic(0)*dic) >= wTR(2)))
        {
          dicmin = dic;
          firstnu = false;
        }
      }
      else
      {
        if (dic < dicmin)
        {
          if (((pcw(0)+uic(0)*dic) <= wTR(0)) && ((pcw(0)+uic(0)*dic) >= wTR(2)))
          {
            dicmin = dic;
          }
        }
      }
    }

    //CR
    nu = nCR(0)*uic(0);
    if (nu > 0)
    {
      dic = fabsf(pcw(0)-wCR(0))/nu;
      if (firstnu)
      {
        if (((pcw(1)+uic(1)*dic) <= wCR(1)) && ((pcw(1)+uic(1)*dic) >= wCR(2)))
        {
          dicmin = dic;
          firstnu = false;
        }
      }
      else
      {
        if (dic < dicmin)
        {
          if (((pcw(1)+uic(1)*dic) <= wCR(1)) && ((pcw(1)+uic(1)*dic) >= wCR(2)))
          {
            dicmin = dic;
          }
        }
      }
    }

    //BR
    nu = nBR(1)*uic(1);
    if (nu > 0)
    {
      dic = fabsf(pcw(1)-wBR(1))/nu;
      if (firstnu)
      {
        if (((pcw(0)+uic(0)*dic) <= wBR(0)) && ((pcw(0)+uic(0)*dic) >= wBR(2)))
        {
          dicmin = dic;
          firstnu = false;
        }
      }
      else
      {
        if (dic < dicmin)
        {
          if (((pcw(0)+uic(0)*dic) <= wBR(0)) && ((pcw(0)+uic(0)*dic) >= wBR(2)))
          {
            dicmin = dic;
          }
        }
      }
    }

    pic = uic*dicmin;
    piw = pcw + pic;
    ptxyz.x = piw(0);
    ptxyz.y = piw(1);
    ptxyz.z = piw(2);
    ptxyz.r = 255;
    ptxyz.g = 0.0;
    ptxyz.b = 0.0;
    // std::cout << ", piwx " << piw(0) << ", piwy " << piw(1) << ", piwz " << piw(2) << std::endl;
    if (!firstnu && (pic.norm() < 4.0))
    {
      *itcT = ptxyz;
      itcT++;
      numberPts++;
    }
  }

  cloudTrue.resize(numberPts);
  planesTrue.push_back(cloudTrue);

  // std::cout << "\n keyInd " << keyIndInit << " planeInd " << planeIndInit << " planesPoints size init " << planePointsInit.size() << std::endl;
}

void KeyframePlanes::addplane(int planeInd, PointCloudRGB& cloud, Eigen::Vector3f pcw, Eigen::Vector4f qcw, Eigen::Vector3f pcwHat, Eigen::Vector4f qcwHat)
{
  // std::cout << "\n keyInd " << keyInd << " planeInd " << planeInd << " entered add plane planePointsInit size " << planePointsInit.size() << std::endl;
  planesInd.push_back(planeInd);
  planes.push_back(cloud);

  // approximate the true distance for each point to each wall
  Eigen::Vector3f piw(0.0,0.0,0.0);
  Eigen::Vector3f pic(0.0,0.0,0.0);
  Eigen::Vector3f uic(0.0,0.0,0.0);
  float nu = 0;
  float dic = 0;
  float dicmin = 0;
  std::vector<float> dics(6);
  bool firstnu = true;
  PointCloudRGB cloudTrue;
  cloudTrue.clear();
  cloudTrue.height = 1;
  cloudTrue.width = cloud.width;
  cloudTrue.is_dense = true;
  cloudTrue.resize(cloud.size());
  PointCloudRGB::iterator itcT = cloudTrue.begin();
  pcl::PointXYZRGB ptxyz;
  for (PointCloudRGB::iterator itc = cloud.begin(); itc != cloud.end(); itc++)
  {
    //subtract off approximated position to get direction
    piw << (*itc).x,(*itc).y,(*itc).z;
    pic = piw - pcwHat;
    uic = pic/pic.norm();

    // get the distance to each wall if the dot product is positive
    //BL
    nu = nBL(1)*uic(1);
    if (nu > 0)
    {
      dic = fabsf(pcw(1)-wBL(1))/nu;
      if (((pcw(0)+uic(0)*dic) <= wBL(0)) && ((pcw(0)+uic(0)*dic) >= wBL(2)))
      {
        dicmin = dic;
        firstnu = false;
      }
    }

    //CL
    nu = nCL(0)*uic(0);
    if (nu > 0)
    {
      dic = fabsf(pcw(0)-wCL(0))/nu;
      if (firstnu)
      {
        if (((pcw(1)+uic(1)*dic) <= wCL(1)) && ((pcw(1)+uic(1)*dic) >= wCL(2)))
        {
          dicmin = dic;
          firstnu = false;
        }
      }
      else
      {
        if (dic < dicmin)
        {
          if (((pcw(1)+uic(1)*dic) <= wCL(1)) && ((pcw(1)+uic(1)*dic) >= wCL(2)))
          {
            dicmin = dic;
          }
        }
      }
    }

    //TL
    nu = nTL(1)*uic(1);
    if (nu > 0)
    {
      dic = fabsf(pcw(1)-wTL(1))/nu;
      if (firstnu)
      {
        if (((pcw(0)+uic(0)*dic) <= wTL(0)) && ((pcw(0)+uic(0)*dic) >= wTL(2)))
        {
          dicmin = dic;
          firstnu = false;
        }
      }
      else
      {
        if (dic < dicmin)
        {
          if (((pcw(0)+uic(0)*dic) <= wTL(0)) && ((pcw(0)+uic(0)*dic) >= wTL(2)))
          {
            dicmin = dic;
          }
        }
      }
    }

    //TR
    nu = nTR(1)*uic(1);
    if (nu > 0)
    {
      dic = fabsf(pcw(1)-wTR(1))/nu;
      if (firstnu)
      {
        if (((pcw(0)+uic(0)*dic) <= wTR(0)) && ((pcw(0)+uic(0)*dic) >= wTR(2)))
        {
          dicmin = dic;
          firstnu = false;
        }
      }
      else
      {
        if (dic < dicmin)
        {
          if (((pcw(0)+uic(0)*dic) <= wTR(0)) && ((pcw(0)+uic(0)*dic) >= wTR(2)))
          {
            dicmin = dic;
          }
        }
      }
    }

    //CR
    nu = nCR(0)*uic(0);
    if (nu > 0)
    {
      dic = fabsf(pcw(0)-wCR(0))/nu;
      if (firstnu)
      {
        if (((pcw(1)+uic(1)*dic) <= wCR(1)) && ((pcw(1)+uic(1)*dic) >= wCR(2)))
        {
          dicmin = dic;
          firstnu = false;
        }
      }
      else
      {
        if (dic < dicmin)
        {
          if (((pcw(1)+uic(1)*dic) <= wCR(1)) && ((pcw(1)+uic(1)*dic) >= wCR(2)))
          {
            dicmin = dic;
          }
        }
      }
    }

    //BR
    nu = nBR(1)*uic(1);
    if (nu > 0)
    {
      dic = fabsf(pcw(1)-wBR(1))/nu;
      if (firstnu)
      {
        if (((pcw(0)+uic(0)*dic) <= wBR(0)) && ((pcw(0)+uic(0)*dic) >= wBR(2)))
        {
          dicmin = dic;
          firstnu = false;
        }
      }
      else
      {
        if (dic < dicmin)
        {
          if (((pcw(0)+uic(0)*dic) <= wBR(0)) && ((pcw(0)+uic(0)*dic) >= wBR(2)))
          {
            dicmin = dic;
          }
        }
      }
    }


    pic = uic*dicmin;
    piw = pcw + pic;
    ptxyz.x = piw(0);
    ptxyz.y = piw(1);
    ptxyz.z = piw(2);
    ptxyz.r = 255;
    ptxyz.g = 0.0;
    ptxyz.b = 0.0;
    *itcT = ptxyz;
    itcT++;
  }

  planesTrue.push_back(cloudTrue);


  // // std::cout << "\n keyInd " << keyInd << " planeInd " << planeInd << " planePoints size before add plane " << planePoints.size() << std::endl;
  //
  // planesPoints.push_back(planePoints);

  // std::cout << "\n keyInd " << keyInd << " planeInd " << planeInd << " planesPoints size after add plane " << planesPoints.size() << std::endl;
}

void KeyframePlanes::update(int planeIndInd, PointCloudRGB& cloud, Eigen::Vector3f pcw, Eigen::Vector4f qcw, Eigen::Vector3f pcwHat, Eigen::Vector4f qcwHat, std::vector<uint8_t> indsInit)
{
  std::cout << "inds.size() " << inds.size() << " indsNew.size() " << indsInit.size() << std::endl;
  for (int ii = 0; ii < indsInit.size(); ii++)
  {
    bool indFound = false;
    std::vector<uint8_t>::iterator indsIt = inds.begin();
    int jj = 0;
    while (!ros::isShuttingDown() && !indFound && (indsIt != inds.end()))
    {
      std::cout << "check " << int(indsInit.at(ii)) << " ii " << ii << " old " << int(*indsIt) << " jj " << jj << std::endl;
      if(int(*indsIt) == int(indsInit.at(ii)))
      {
        planes.at(planeIndInd).points.at(jj) = cloud.points.at(ii);
        indFound = true;
      }
      indsIt++;
      jj++;
    }
    // planes.at(planeIndInd) = cloud;
  }


  // // approximate the true distance for each point to each wall
  // Eigen::Vector3f piw(0.0,0.0,0.0);
  // Eigen::Vector3f pic(0.0,0.0,0.0);
  // Eigen::Vector3f uic(0.0,0.0,0.0);
  // float nu = 0;
  // float dic = 0;
  // float dicmin = 0;
  // std::vector<float> dics(6);
  // bool firstnu = true;
  // PointCloudRGB cloudTrue;
  // cloudTrue.height = 1;
  // cloudTrue.width = cloud.width;
  // cloudTrue.is_dense = true;
  // cloudTrue.resize(cloud.size());
  // PointCloudRGB::iterator itcT = cloudTrue.begin();
  // pcl::PointXYZRGB ptxyz;
  // for (PointCloudRGB::iterator itc = cloud.begin(); itc != cloud.end(); itc++)
  // {
  //   //subtract off approximated position to get direction
  //   piw << (*itc).x,(*itc).y,(*itc).z;
  //   pic = piw - pcwHat;
  //   uic = pic/pic.norm();
  //
  //   // get the distance to each wall if the dot product is positive
  //   //BL
  //   nu = nBL(1)*uic(1);
  //   if (nu > 0)
  //   {
  //     dic = fabsf(pcw(1)-wBL(1))/nu;
  //     if (((pcw(0)+uic(0)*dic) <= wBL(0)) && ((pcw(0)+uic(0)*dic) >= wBL(2)))
  //     {
  //       dicmin = dic;
  //       firstnu = false;
  //     }
  //   }
  //
  //   //CL
  //   nu = nCL(0)*uic(0);
  //   if (nu > 0)
  //   {
  //     dic = fabsf(pcw(0)-wCL(0))/nu;
  //     if (firstnu)
  //     {
  //       if (((pcw(1)+uic(1)*dic) <= wCL(1)) && ((pcw(1)+uic(1)*dic) >= wCL(2)))
  //       {
  //         dicmin = dic;
  //         firstnu = false;
  //       }
  //     }
  //     else
  //     {
  //       if (dic < dicmin)
  //       {
  //         if (((pcw(1)+uic(1)*dic) <= wCL(1)) && ((pcw(1)+uic(1)*dic) >= wCL(2)))
  //         {
  //           dicmin = dic;
  //         }
  //       }
  //     }
  //   }
  //
  //   //TL
  //   nu = nTL(1)*uic(1);
  //   if (nu > 0)
  //   {
  //     dic = fabsf(pcw(1)-wTL(1))/nu;
  //     if (firstnu)
  //     {
  //       if (((pcw(0)+uic(0)*dic) <= wTL(0)) && ((pcw(0)+uic(0)*dic) >= wTL(2)))
  //       {
  //         dicmin = dic;
  //         firstnu = false;
  //       }
  //     }
  //     else
  //     {
  //       if (dic < dicmin)
  //       {
  //         if (((pcw(0)+uic(0)*dic) <= wTL(0)) && ((pcw(0)+uic(0)*dic) >= wTL(2)))
  //         {
  //           dicmin = dic;
  //         }
  //       }
  //     }
  //   }
  //
  //   //TR
  //   nu = nTR(1)*uic(1);
  //   if (nu > 0)
  //   {
  //     dic = fabsf(pcw(1)-wTR(1))/nu;
  //     if (firstnu)
  //     {
  //       if (((pcw(0)+uic(0)*dic) <= wTR(0)) && ((pcw(0)+uic(0)*dic) >= wTR(2)))
  //       {
  //         dicmin = dic;
  //         firstnu = false;
  //       }
  //     }
  //     else
  //     {
  //       if (dic < dicmin)
  //       {
  //         if (((pcw(0)+uic(0)*dic) <= wTR(0)) && ((pcw(0)+uic(0)*dic) >= wTR(2)))
  //         {
  //           dicmin = dic;
  //         }
  //       }
  //     }
  //   }
  //
  //   //CR
  //   nu = nCR(0)*uic(0);
  //   if (nu > 0)
  //   {
  //     dic = fabsf(pcw(0)-wCR(0))/nu;
  //     if (firstnu)
  //     {
  //       if (((pcw(1)+uic(1)*dic) <= wCR(1)) && ((pcw(1)+uic(1)*dic) >= wCR(2)))
  //       {
  //         dicmin = dic;
  //         firstnu = false;
  //       }
  //     }
  //     else
  //     {
  //       if (dic < dicmin)
  //       {
  //         if (((pcw(1)+uic(1)*dic) <= wCR(1)) && ((pcw(1)+uic(1)*dic) >= wCR(2)))
  //         {
  //           dicmin = dic;
  //         }
  //       }
  //     }
  //   }
  //
  //   //BR
  //   nu = nBR(1)*uic(1);
  //   if (nu > 0)
  //   {
  //     dic = fabsf(pcw(1)-wBR(1))/nu;
  //     if (firstnu)
  //     {
  //       if (((pcw(0)+uic(0)*dic) <= wBR(0)) && ((pcw(0)+uic(0)*dic) >= wBR(2)))
  //       {
  //         dicmin = dic;
  //         firstnu = false;
  //       }
  //     }
  //     else
  //     {
  //       if (dic < dicmin)
  //       {
  //         if (((pcw(0)+uic(0)*dic) <= wBR(0)) && ((pcw(0)+uic(0)*dic) >= wBR(2)))
  //         {
  //           dicmin = dic;
  //         }
  //       }
  //     }
  //   }
  //
  //
  //   pic = uic*dicmin;
  //   piw = pcw + pic;
  //   ptxyz.x = piw(0);
  //   ptxyz.y = piw(1);
  //   ptxyz.z = piw(2);
  //   ptxyz.r = std::min((*itc).g+100,255);
  //   ptxyz.g = (*itc).g;
  //   ptxyz.b = (*itc).g;
  //   *itcT = ptxyz;
  //   itcT++;
  // }
  //
  // planesTrue.at(planeIndInd) = cloudTrue;

  // std::cout << "\n keyInd " << keyInd << " planeInd " << planesInd.at(planeIndInd) << " planesPoints size update " << planesPoints.at(planeIndInd).size() << std::endl;
}
