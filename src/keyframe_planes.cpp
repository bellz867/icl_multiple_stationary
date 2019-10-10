#include <keyframe_planes.h>

KeyframePlanes::~KeyframePlanes()
{
  std::cout << std::endl << keyInd << " allPtsKnown " << int(allPtsKnown) << std::endl;
	if (saveExp && allPtsKnown)
	{
		std::cout << std::endl << "saving " << keyInd << std::endl;
		std::ofstream saveFile("/home/ncr/ncr_ws/src/icl_multiple_stationary/experiment/key_"+std::to_string(keyInd)+".csv");
		if (saveFile.is_open())
		{
      std::cout << "\nopen\n";

      // std::cout << "time,";
      // std::cout << "pcwx," << "pcwy," << "pcwz,";
      // std::cout << "pcwHatx," << "pcwHaty," << "pcwHatz,";
      // std::cout << std::endl;

      saveFile << "time,";
      saveFile << "pcwx," << "pcwy," << "pcwz,";
      saveFile << "pcwHatx," << "pcwHaty," << "pcwHatz,";
      //go through and check which will be saved and write header
      int numSaved = 0;
      for (int ii = 0; ii < cloudDataSaves.back()->dkKnowns.size(); ii++)
      {
        if (bool(cloudDataSaves.back()->dkKnowns.at(ii)))
        {
          // std::cout << "p" << numSaved << "wx" << ",";
          // std::cout << "p" << numSaved << "wy" << ",";
          // std::cout << "p" << numSaved << "wz" << ",";
          // std::cout << "p" << numSaved << "wHatx" << ",";
          // std::cout << "p" << numSaved << "wHaty" << ",";
          // std::cout << "p" << numSaved << "wHatz" << ",";
          // std::cout << "converged" << numSaved << ",";
          // std::cout << "\n";

          saveFile << "p" << numSaved << "wx" << ",";
          saveFile << "p" << numSaved << "wy" << ",";
          saveFile << "p" << numSaved << "wz" << ",";
          saveFile << "p" << numSaved << "wHatx" << ",";
          saveFile << "p" << numSaved << "wHaty" << ",";
          saveFile << "p" << numSaved << "wHatz" << ",";
          saveFile << "converged" << numSaved << ",";
          numSaved++;
        }
      }
			saveFile << "\n";

      std::cout << "\n wrote header \n";

			for (int ii = 0; ii < cloudDataSaves.size(); ii++)
			{
        // std::cout << cloudDataSaves.at(ii)->time << ",";
        // std::cout << cloudDataSaves.at(ii)->pcw(0) << "," << cloudDataSaves.at(ii)->pcw(1) << "," << cloudDataSaves.at(ii)->pcw(2) << ",";
        // std::cout << cloudDataSaves.at(ii)->pcwHat(0) << "," << cloudDataSaves.at(ii)->pcwHat(1) << "," << cloudDataSaves.at(ii)->pcwHat(2) << ",";
        // std::cout << std::endl;

        saveFile << cloudDataSaves.at(ii)->time << ",";
        saveFile << cloudDataSaves.at(ii)->pcw(0) << "," << cloudDataSaves.at(ii)->pcw(1) << "," << cloudDataSaves.at(ii)->pcw(2) << ",";
        saveFile << cloudDataSaves.at(ii)->pcwHat(0) << "," << cloudDataSaves.at(ii)->pcwHat(1) << "," << cloudDataSaves.at(ii)->pcwHat(2) << ",";
        std::cout << "\n dkKnown.size()" << cloudDataSaves.at(ii)->dkKnowns.size();
        std::cout << " cloudTrue.size() " << cloudDataSaves.at(ii)->cloudTrue.size();
        std::cout << " cloudHat.size() " << cloudDataSaves.at(ii)->cloudHat.size();
        std::cout << std::endl;
        for (int jj = 0; jj < cloudDataSaves.at(ii)->dkKnowns.size(); jj++)
        {

          if (bool(cloudDataSaves.back()->dkKnowns.at(jj)))
          {
            // std::cout << cloudDataSaves.at(ii)->cloudTrue.at(jj).x << ",";
            // std::cout << cloudDataSaves.at(ii)->cloudTrue.at(jj).y << ",";
            // std::cout << cloudDataSaves.at(ii)->cloudTrue.at(jj).z << ",";
            // std::cout << cloudDataSaves.at(ii)->cloudHat.at(jj).x << ",";
            // std::cout << cloudDataSaves.at(ii)->cloudHat.at(jj).y << ",";
            // std::cout << cloudDataSaves.at(ii)->cloudHat.at(jj).z << ",";
            // std::cout << int(cloudDataSaves.at(ii)->dkKnowns.at(jj)) << ",";
            // std::cout << "\n";

            saveFile << cloudDataSaves.at(ii)->cloudTrue.at(jj).x << ",";
            saveFile << cloudDataSaves.at(ii)->cloudTrue.at(jj).y << ",";
            saveFile << cloudDataSaves.at(ii)->cloudTrue.at(jj).z << ",";
            saveFile << cloudDataSaves.at(ii)->cloudHat.at(jj).x << ",";
            saveFile << cloudDataSaves.at(ii)->cloudHat.at(jj).y << ",";
            saveFile << cloudDataSaves.at(ii)->cloudHat.at(jj).z << ",";
            saveFile << int(cloudDataSaves.at(ii)->dkKnowns.at(jj)) << ",";
          }
        }
  			saveFile << "\n";

				delete cloudDataSaves.at(ii);

        std::cout << "\n wrote " << ii << std::endl;
			}
			saveFile.close();

      std::cout << "\nclose\n";
		}
		std::cout << std::endl << "saved " << keyInd << std::endl;
	}
}

KeyframePlanes::KeyframePlanes()
{
}

KeyframePlanes::KeyframePlanes(float minareaInit, float maxareaInit, float minheightInit, float maxheightInit, int keyIndInit, int planeIndInit, PointCloudRGB& cloud, Eigen::Vector3f pcw, Eigen::Vector4f qcw, Eigen::Vector3f pcwHat, Eigen::Vector4f qcwHat, PointCloudRGB& cloud_true, bool allPtsKnownInit, std::vector<uint8_t> indsInit, std::vector<uint8_t> dkKnownsInit, bool saveExpInit, std::string expNameInit, ros::Time t)
{
  allPtsKnown = allPtsKnownInit;
  dkKnowns = dkKnownsInit;
  inds = indsInit;
  minarea = minareaInit;
  maxarea = maxareaInit;
  minheight = minheightInit;
  maxheight = maxheightInit;
  saveExp = saveExpInit;
  expName = expNameInit;
  startTime = t;

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
  float dicmin = 1.5;
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
    // if (!firstnu && (pic.norm() < 4.0))
    // {
    //   *itcT = ptxyz;
    //   itcT++;
    //   numberPts++;
    // }
    *itcT = ptxyz;
    itcT++;
    numberPts++;
  }

  cloudTrue.resize(numberPts);
  planesTrue.push_back(cloudTrue);

  // std::vector<float> dks(planesTrue.at(0).size());
  // std::vector<float> dkHats(planesTrue.at(0).size());
  // float xk = 0;
  // float yk = 0;
  // float zk = 0;
  // float xkHat = 0;
  // float ykHat = 0;
  // float zkHat = 0;

  planesDraw.clear();
  planesDraw.height = 1;
  planesDraw.width = planesTrue.at(0).size();
  planesDraw.is_dense = true;
  planesDraw.resize(planesTrue.at(0).size());

  planesTrueDraw.clear();
  planesTrueDraw.height = 1;
  planesTrueDraw.width = planesTrue.at(0).size();
  planesTrueDraw.is_dense = true;
  planesTrueDraw.resize(planesTrue.at(0).size());

  //get number known
  int numKnown = 0;
  for (int ii = 0; ii < planesTrue.at(0).size(); ii++)
  {
    // xk = planesTrue.at(0).at(ii).x - pcw(0);
    // yk = planesTrue.at(0).at(ii).y - pcw(1);
    // zk = planesTrue.at(0).at(ii).z - pcw(2);
    // dks.at(ii) = sqrtf(xk*xk + yk*yk + zk*zk);
    // xkHat = planes.at(0).at(ii).x - pcwHat(0);
    // ykHat = planes.at(0).at(ii).y - pcwHat(1);
    // zkHat = planes.at(0).at(ii).z - pcwHat(2);
    // dkHats.at(ii) = sqrtf(xkHat*xkHat + ykHat*ykHat + zkHat*zkHat);

    if (bool(dkKnowns.at(ii)))
    {
      planesDraw.at(numKnown) = planes.at(0).at(ii);
      planesTrueDraw.at(numKnown) = planesTrue.at(0).at(ii);
      numKnown++;
    }
  }

  planesDraw.resize(numKnown);
  planesTrueDraw.resize(numKnown);

  float Dt = (t-startTime).toSec();

  cloudDataSave = new CloudDataSave(Dt,planesTrue.at(0),planes.at(0),pcw,pcwHat,dkKnowns);
  cloudDataSaves.push_back(cloudDataSave);

  // std::cout << "\n keyInd " << keyIndInit << " planeInd " << planeIndInit << " planesPoints size init " << planePointsInit.size() << std::endl;
}

// void KeyframePlanes::addplane(int planeInd, PointCloudRGB& cloud, Eigen::Vector3f pcw, Eigen::Vector4f qcw, Eigen::Vector3f pcwHat, Eigen::Vector4f qcwHat)
// {
//   // std::cout << "\n keyInd " << keyInd << " planeInd " << planeInd << " entered add plane planePointsInit size " << planePointsInit.size() << std::endl;
//   planesInd.push_back(planeInd);
//   planes.push_back(cloud);
//
//   // approximate the true distance for each point to each wall
//   Eigen::Vector3f piw(0.0,0.0,0.0);
//   Eigen::Vector3f pic(0.0,0.0,0.0);
//   Eigen::Vector3f uic(0.0,0.0,0.0);
//   float nu = 0;
//   float dic = 0;
//   float dicmin = 0;
//   std::vector<float> dics(6);
//   bool firstnu = true;
//   PointCloudRGB cloudTrue;
//   cloudTrue.clear();
//   cloudTrue.height = 1;
//   cloudTrue.width = cloud.width;
//   cloudTrue.is_dense = true;
//   cloudTrue.resize(cloud.size());
//   PointCloudRGB::iterator itcT = cloudTrue.begin();
//   pcl::PointXYZRGB ptxyz;
//   for (PointCloudRGB::iterator itc = cloud.begin(); itc != cloud.end(); itc++)
//   {
//     //subtract off approximated position to get direction
//     piw << (*itc).x,(*itc).y,(*itc).z;
//     pic = piw - pcwHat;
//     uic = pic/pic.norm();
//
//     // get the distance to each wall if the dot product is positive
//     //BL
//     nu = nBL(1)*uic(1);
//     if (nu > 0)
//     {
//       dic = fabsf(pcw(1)-wBL(1))/nu;
//       if (((pcw(0)+uic(0)*dic) <= wBL(0)) && ((pcw(0)+uic(0)*dic) >= wBL(2)))
//       {
//         dicmin = dic;
//         firstnu = false;
//       }
//     }
//
//     //CL
//     nu = nCL(0)*uic(0);
//     if (nu > 0)
//     {
//       dic = fabsf(pcw(0)-wCL(0))/nu;
//       if (firstnu)
//       {
//         if (((pcw(1)+uic(1)*dic) <= wCL(1)) && ((pcw(1)+uic(1)*dic) >= wCL(2)))
//         {
//           dicmin = dic;
//           firstnu = false;
//         }
//       }
//       else
//       {
//         if (dic < dicmin)
//         {
//           if (((pcw(1)+uic(1)*dic) <= wCL(1)) && ((pcw(1)+uic(1)*dic) >= wCL(2)))
//           {
//             dicmin = dic;
//           }
//         }
//       }
//     }
//
//     //TL
//     nu = nTL(1)*uic(1);
//     if (nu > 0)
//     {
//       dic = fabsf(pcw(1)-wTL(1))/nu;
//       if (firstnu)
//       {
//         if (((pcw(0)+uic(0)*dic) <= wTL(0)) && ((pcw(0)+uic(0)*dic) >= wTL(2)))
//         {
//           dicmin = dic;
//           firstnu = false;
//         }
//       }
//       else
//       {
//         if (dic < dicmin)
//         {
//           if (((pcw(0)+uic(0)*dic) <= wTL(0)) && ((pcw(0)+uic(0)*dic) >= wTL(2)))
//           {
//             dicmin = dic;
//           }
//         }
//       }
//     }
//
//     //TR
//     nu = nTR(1)*uic(1);
//     if (nu > 0)
//     {
//       dic = fabsf(pcw(1)-wTR(1))/nu;
//       if (firstnu)
//       {
//         if (((pcw(0)+uic(0)*dic) <= wTR(0)) && ((pcw(0)+uic(0)*dic) >= wTR(2)))
//         {
//           dicmin = dic;
//           firstnu = false;
//         }
//       }
//       else
//       {
//         if (dic < dicmin)
//         {
//           if (((pcw(0)+uic(0)*dic) <= wTR(0)) && ((pcw(0)+uic(0)*dic) >= wTR(2)))
//           {
//             dicmin = dic;
//           }
//         }
//       }
//     }
//
//     //CR
//     nu = nCR(0)*uic(0);
//     if (nu > 0)
//     {
//       dic = fabsf(pcw(0)-wCR(0))/nu;
//       if (firstnu)
//       {
//         if (((pcw(1)+uic(1)*dic) <= wCR(1)) && ((pcw(1)+uic(1)*dic) >= wCR(2)))
//         {
//           dicmin = dic;
//           firstnu = false;
//         }
//       }
//       else
//       {
//         if (dic < dicmin)
//         {
//           if (((pcw(1)+uic(1)*dic) <= wCR(1)) && ((pcw(1)+uic(1)*dic) >= wCR(2)))
//           {
//             dicmin = dic;
//           }
//         }
//       }
//     }
//
//     //BR
//     nu = nBR(1)*uic(1);
//     if (nu > 0)
//     {
//       dic = fabsf(pcw(1)-wBR(1))/nu;
//       if (firstnu)
//       {
//         if (((pcw(0)+uic(0)*dic) <= wBR(0)) && ((pcw(0)+uic(0)*dic) >= wBR(2)))
//         {
//           dicmin = dic;
//           firstnu = false;
//         }
//       }
//       else
//       {
//         if (dic < dicmin)
//         {
//           if (((pcw(0)+uic(0)*dic) <= wBR(0)) && ((pcw(0)+uic(0)*dic) >= wBR(2)))
//           {
//             dicmin = dic;
//           }
//         }
//       }
//     }
//
//
//     pic = uic*dicmin;
//     piw = pcw + pic;
//     ptxyz.x = piw(0);
//     ptxyz.y = piw(1);
//     ptxyz.z = piw(2);
//     ptxyz.r = 255;
//     ptxyz.g = 0.0;
//     ptxyz.b = 0.0;
//     *itcT = ptxyz;
//     itcT++;
//   }
//
//   planesTrue.push_back(cloudTrue);
//
//
//   // // std::cout << "\n keyInd " << keyInd << " planeInd " << planeInd << " planePoints size before add plane " << planePoints.size() << std::endl;
//   //
//   // planesPoints.push_back(planePoints);
//
//   // std::cout << "\n keyInd " << keyInd << " planeInd " << planeInd << " planesPoints size after add plane " << planesPoints.size() << std::endl;
// }

void KeyframePlanes::update(int planeIndInd, PointCloudRGB& cloud, Eigen::Vector3f pcw, Eigen::Vector4f qcw, Eigen::Vector3f pcwHat, Eigen::Vector4f qcwHat, bool allPtsKnownInit, std::vector<uint8_t> indsInit, std::vector<uint8_t> dkKnownsInit, ros::Time t)
{
  allPtsKnown = allPtsKnownInit;
  // Eigen::Vector3f piw(0.0,0.0,0.0);
  // Eigen::Vector3f pic(0.0,0.0,0.0);
  // Eigen::Vector3f uic(0.0,0.0,0.0);
  // float nu = 0;
  // float dic = 0;
  // float dicmin = 0;
  // std::vector<float> dics(6);
  // bool firstnu = true;
  // std::cout << "inds.size() " << inds.size() << " indsNew.size() " << indsInit.size() << std::endl;
  for (int ii = 0; ii < indsInit.size(); ii++)
  {
    bool indFound = false;
    std::vector<uint8_t>::iterator indsIt = inds.begin();
    int jj = 0;
    while (!ros::isShuttingDown() && !indFound && (indsIt != inds.end()))
    {
      // std::cout << "check " << int(indsInit.at(ii)) << " ii " << ii << " old " << int(*indsIt) << " jj " << jj << std::endl;
      if(int(*indsIt) == int(indsInit.at(ii)))
      {
        planes.at(planeIndInd).points.at(jj) = cloud.points.at(ii);
        dkKnowns.at(jj) = dkKnownsInit.at(ii);
        indFound = true;
      }
      indsIt++;
      jj++;
    }
    // planes.at(planeIndInd) = cloud;
  }

  planesDraw.clear();
  planesDraw.height = 1;
  planesDraw.width = planesTrue.at(0).size();
  planesDraw.is_dense = true;
  planesDraw.resize(planesTrue.at(0).size());

  planesTrueDraw.clear();
  planesTrueDraw.height = 1;
  planesTrueDraw.width = planesTrue.at(0).size();
  planesTrueDraw.is_dense = true;
  planesTrueDraw.resize(planesTrue.at(0).size());

  // std::vector<float> dks(planesTrue.at(0).size());
  // std::vector<float> dkHats(planesTrue.at(0).size());
  // float xk = 0;
  // float yk = 0;
  // float zk = 0;
  // float xkHat = 0;
  // float ykHat = 0;
  // float zkHat = 0;
  int numKnown = 0;
  for (int ii = 0; ii < planesTrue.at(0).size(); ii++)
  {
    // xk = planesTrue.at(0).at(ii).x - pcw(0);
    // yk = planesTrue.at(0).at(ii).y - pcw(1);
    // zk = planesTrue.at(0).at(ii).z - pcw(2);
    // dks.at(ii) = sqrtf(xk*xk + yk*yk + zk*zk);
    // xkHat = planes.at(0).at(ii).x - pcwHat(0);
    // ykHat = planes.at(0).at(ii).y - pcwHat(1);
    // zkHat = planes.at(0).at(ii).z - pcwHat(2);
    // dkHats.at(ii) = sqrtf(xkHat*xkHat + ykHat*ykHat + zkHat*zkHat);

    if (bool(dkKnowns.at(ii)))
    {
      planesDraw.at(numKnown) = planes.at(0).at(ii);
      planesTrueDraw.at(numKnown) = planesTrue.at(0).at(ii);
      numKnown++;
    }
  }

  planesDraw.resize(numKnown);
  planesTrueDraw.resize(numKnown);

  float Dt = (t-startTime).toSec();

  cloudDataSave = new CloudDataSave(Dt,planesTrue.at(0),planes.at(0),pcw,pcwHat,dkKnowns);
  cloudDataSaves.push_back(cloudDataSave);


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


// void KeyframePlanes::update(int planeIndInd, PointCloudRGB& cloud, Eigen::Vector3f pcw, Eigen::Vector4f qcw, Eigen::Vector3f pcwHat, Eigen::Vector4f qcwHat, bool allPtsKnownInit, std::vector<uint8_t> indsInit, std::vector<uint8_t> dkKnownsInit, ros::Time t)
// {
//   allPtsKnown = allPtsKnownInit;
//   // std::cout << "inds.size() " << inds.size() << " indsNew.size() " << indsInit.size() << std::endl;
//   for (int ii = 0; ii < indsInit.size(); ii++)
//   {
//     bool indFound = false;
//     std::vector<uint8_t>::iterator indsIt = inds.begin();
//     int jj = 0;
//     while (!ros::isShuttingDown() && !indFound && (indsIt != inds.end()))
//     {
//       // std::cout << "check " << int(indsInit.at(ii)) << " ii " << ii << " old " << int(*indsIt) << " jj " << jj << std::endl;
//       if(int(*indsIt) == int(indsInit.at(ii)))
//       {
//         planes.at(planeIndInd).points.at(jj) = cloud.points.at(ii);
//         dkKnowns.at(jj) = dkKnownsInit.at(ii);
//         indFound = true;
//       }
//       indsIt++;
//       jj++;
//     }
//     // planes.at(planeIndInd) = cloud;
//   }
//
//   planesDraw.clear();
//   planesDraw.height = 1;
//   planesDraw.width = planesTrue.at(0).size();
//   planesDraw.is_dense = true;
//   planesDraw.resize(planesTrue.at(0).size());
//
//   planesTrueDraw.clear();
//   planesTrueDraw.height = 1;
//   planesTrueDraw.width = planesTrue.at(0).size();
//   planesTrueDraw.is_dense = true;
//   planesTrueDraw.resize(planesTrue.at(0).size());
//
//   std::vector<float> dks(planesTrue.at(0).size());
//   std::vector<float> dkHats(planesTrue.at(0).size());
//   float xk = 0;
//   float yk = 0;
//   float zk = 0;
//   float xkHat = 0;
//   float ykHat = 0;
//   float zkHat = 0;
//   int numKnown = 0;
//   for (int ii = 0; ii < planesTrue.at(0).size(); ii++)
//   {
//     xk = planesTrue.at(0).at(ii).x - pcw(0);
//     yk = planesTrue.at(0).at(ii).y - pcw(1);
//     zk = planesTrue.at(0).at(ii).z - pcw(2);
//     dks.at(ii) = sqrtf(xk*xk + yk*yk + zk*zk);
//     xkHat = planes.at(0).at(ii).x - pcwHat(0);
//     ykHat = planes.at(0).at(ii).y - pcwHat(1);
//     zkHat = planes.at(0).at(ii).z - pcwHat(2);
//     dkHats.at(ii) = sqrtf(xkHat*xkHat + ykHat*ykHat + zkHat*zkHat);
//
//     if (bool(dkKnowns.at(ii)))
//     {
//       planesDraw.at(numKnown) = planes.at(0).at(ii);
//       planesTrueDraw.at(numKnown) = planesTrue.at(0).at(ii);
//       numKnown++;
//     }
//   }
//
//   planesDraw.resize(numKnown);
//   planesTrueDraw.resize(numKnown);
//
//   float Dt = (t-startTime).toSec();
//
//   cloudDataSave = new CloudDataSave(Dt,dks,dkHats,dkKnowns);
//   cloudDataSaves.push_back(cloudDataSave);
//
//
//   // // approximate the true distance for each point to each wall
//   // Eigen::Vector3f piw(0.0,0.0,0.0);
//   // Eigen::Vector3f pic(0.0,0.0,0.0);
//   // Eigen::Vector3f uic(0.0,0.0,0.0);
//   // float nu = 0;
//   // float dic = 0;
//   // float dicmin = 0;
//   // std::vector<float> dics(6);
//   // bool firstnu = true;
//   // PointCloudRGB cloudTrue;
//   // cloudTrue.height = 1;
//   // cloudTrue.width = cloud.width;
//   // cloudTrue.is_dense = true;
//   // cloudTrue.resize(cloud.size());
//   // PointCloudRGB::iterator itcT = cloudTrue.begin();
//   // pcl::PointXYZRGB ptxyz;
//   // for (PointCloudRGB::iterator itc = cloud.begin(); itc != cloud.end(); itc++)
//   // {
//   //   //subtract off approximated position to get direction
//   //   piw << (*itc).x,(*itc).y,(*itc).z;
//   //   pic = piw - pcwHat;
//   //   uic = pic/pic.norm();
//   //
//   //   // get the distance to each wall if the dot product is positive
//   //   //BL
//   //   nu = nBL(1)*uic(1);
//   //   if (nu > 0)
//   //   {
//   //     dic = fabsf(pcw(1)-wBL(1))/nu;
//   //     if (((pcw(0)+uic(0)*dic) <= wBL(0)) && ((pcw(0)+uic(0)*dic) >= wBL(2)))
//   //     {
//   //       dicmin = dic;
//   //       firstnu = false;
//   //     }
//   //   }
//   //
//   //   //CL
//   //   nu = nCL(0)*uic(0);
//   //   if (nu > 0)
//   //   {
//   //     dic = fabsf(pcw(0)-wCL(0))/nu;
//   //     if (firstnu)
//   //     {
//   //       if (((pcw(1)+uic(1)*dic) <= wCL(1)) && ((pcw(1)+uic(1)*dic) >= wCL(2)))
//   //       {
//   //         dicmin = dic;
//   //         firstnu = false;
//   //       }
//   //     }
//   //     else
//   //     {
//   //       if (dic < dicmin)
//   //       {
//   //         if (((pcw(1)+uic(1)*dic) <= wCL(1)) && ((pcw(1)+uic(1)*dic) >= wCL(2)))
//   //         {
//   //           dicmin = dic;
//   //         }
//   //       }
//   //     }
//   //   }
//   //
//   //   //TL
//   //   nu = nTL(1)*uic(1);
//   //   if (nu > 0)
//   //   {
//   //     dic = fabsf(pcw(1)-wTL(1))/nu;
//   //     if (firstnu)
//   //     {
//   //       if (((pcw(0)+uic(0)*dic) <= wTL(0)) && ((pcw(0)+uic(0)*dic) >= wTL(2)))
//   //       {
//   //         dicmin = dic;
//   //         firstnu = false;
//   //       }
//   //     }
//   //     else
//   //     {
//   //       if (dic < dicmin)
//   //       {
//   //         if (((pcw(0)+uic(0)*dic) <= wTL(0)) && ((pcw(0)+uic(0)*dic) >= wTL(2)))
//   //         {
//   //           dicmin = dic;
//   //         }
//   //       }
//   //     }
//   //   }
//   //
//   //   //TR
//   //   nu = nTR(1)*uic(1);
//   //   if (nu > 0)
//   //   {
//   //     dic = fabsf(pcw(1)-wTR(1))/nu;
//   //     if (firstnu)
//   //     {
//   //       if (((pcw(0)+uic(0)*dic) <= wTR(0)) && ((pcw(0)+uic(0)*dic) >= wTR(2)))
//   //       {
//   //         dicmin = dic;
//   //         firstnu = false;
//   //       }
//   //     }
//   //     else
//   //     {
//   //       if (dic < dicmin)
//   //       {
//   //         if (((pcw(0)+uic(0)*dic) <= wTR(0)) && ((pcw(0)+uic(0)*dic) >= wTR(2)))
//   //         {
//   //           dicmin = dic;
//   //         }
//   //       }
//   //     }
//   //   }
//   //
//   //   //CR
//   //   nu = nCR(0)*uic(0);
//   //   if (nu > 0)
//   //   {
//   //     dic = fabsf(pcw(0)-wCR(0))/nu;
//   //     if (firstnu)
//   //     {
//   //       if (((pcw(1)+uic(1)*dic) <= wCR(1)) && ((pcw(1)+uic(1)*dic) >= wCR(2)))
//   //       {
//   //         dicmin = dic;
//   //         firstnu = false;
//   //       }
//   //     }
//   //     else
//   //     {
//   //       if (dic < dicmin)
//   //       {
//   //         if (((pcw(1)+uic(1)*dic) <= wCR(1)) && ((pcw(1)+uic(1)*dic) >= wCR(2)))
//   //         {
//   //           dicmin = dic;
//   //         }
//   //       }
//   //     }
//   //   }
//   //
//   //   //BR
//   //   nu = nBR(1)*uic(1);
//   //   if (nu > 0)
//   //   {
//   //     dic = fabsf(pcw(1)-wBR(1))/nu;
//   //     if (firstnu)
//   //     {
//   //       if (((pcw(0)+uic(0)*dic) <= wBR(0)) && ((pcw(0)+uic(0)*dic) >= wBR(2)))
//   //       {
//   //         dicmin = dic;
//   //         firstnu = false;
//   //       }
//   //     }
//   //     else
//   //     {
//   //       if (dic < dicmin)
//   //       {
//   //         if (((pcw(0)+uic(0)*dic) <= wBR(0)) && ((pcw(0)+uic(0)*dic) >= wBR(2)))
//   //         {
//   //           dicmin = dic;
//   //         }
//   //       }
//   //     }
//   //   }
//   //
//   //
//   //   pic = uic*dicmin;
//   //   piw = pcw + pic;
//   //   ptxyz.x = piw(0);
//   //   ptxyz.y = piw(1);
//   //   ptxyz.z = piw(2);
//   //   ptxyz.r = std::min((*itc).g+100,255);
//   //   ptxyz.g = (*itc).g;
//   //   ptxyz.b = (*itc).g;
//   //   *itcT = ptxyz;
//   //   itcT++;
//   // }
//   //
//   // planesTrue.at(planeIndInd) = cloudTrue;
//
//   // std::cout << "\n keyInd " << keyInd << " planeInd " << planesInd.at(planeIndInd) << " planesPoints size update " << planesPoints.at(planeIndInd).size() << std::endl;
// }
