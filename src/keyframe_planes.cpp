#include <keyframe_planes.h>

KeyframePlanes::KeyframePlanes()
{
}

KeyframePlanes::KeyframePlanes(float minareaInit, float maxareaInit, float minheightInit, float maxheightInit, int keyIndInit, int planeIndInit, PointCloudRGB& cloud)
{
  minarea = minareaInit;
  maxarea = maxareaInit;
  minheight = minheightInit;
  maxheight = maxheightInit;

  // std::cout << "\n keyInd " << keyIndInit << " planeInd " << planeIndInit << " planesPoints init entered " << std::endl;
  keyInd = keyIndInit;
  planesInd.push_back(planeIndInit);
  planes.push_back(cloud);

  //if the patch satisfies the conditions then add the plane
  // if (checkPatch(planePointsInit, rows, cols))
  // {
  //   return;
  // }
  //
  // std::vector<pcl::PointXYZRGB> planePoints(planePointsInit.size());
  // std::vector<geometry_msgs::Point32>::iterator ppIit = planePointsInit.begin();
  // std::vector<uint8_t>::iterator pcIit = planeColorsInit.begin();
  // pcl::PointXYZRGB pt;
  // for (std::vector<pcl::PointXYZRGB>::iterator ppit = planePoints.begin(); ppit != planePoints.end(); ppit++)
  // {
  //   pt.x = (*ppIit).x;
  //   pt.y = (*ppIit).y;
  //   pt.z = (*ppIit).z;
  //   pt.r = *pcIit;
  //   pt.g = *pcIit;
  //   pt.b = *pcIit;
  //   *ppit = pt;
  //   ppIit++;
  //   pcIit++;
  // }
  // planesPoints.push_back(planePoints);

  // std::cout << "\n keyInd " << keyIndInit << " planeInd " << planeIndInit << " planesPoints size init " << planePointsInit.size() << std::endl;
}

// bool KeyframePlanes::checkPatch(std::vector<geometry_msgs::Point32>& planePointsInit, int rows, int cols)
// {
//   //check the area of the patch, if it is too small then dont add
//   int trInd = cols-1;
//   int brInd = rows*cols-1;
//   int blInd = brInd-cols+1;
//
//   Eigen::Vector3f tl(planePointsInit.at(0).x,planePointsInit.at(0).y,planePointsInit.at(0).z);
//   Eigen::Vector3f tr(planePointsInit.at(trInd).x,planePointsInit.at(trInd).y,planePointsInit.at(trInd).z);
//   Eigen::Vector3f bl(planePointsInit.at(blInd).x,planePointsInit.at(blInd).y,planePointsInit.at(blInd).z);
//   Eigen::Vector3f br(planePointsInit.at(brInd).x,planePointsInit.at(brInd).y,planePointsInit.at(brInd).z);
//   // std::cout << "\n rows " << rows << std::endl;
//   // std::cout << "\n cols " << cols << std::endl;
//   // std::cout << "\n trInd " << trInd << std::endl;
//   // std::cout << "\n blInd " << blInd << std::endl;
//   // std::cout << "\n brInd " << brInd << std::endl;
//   // std::cout << "\n tlx " << tl(0) << " tly " << tl(1) << " tlz " << tl(2) << std::endl;
//   // std::cout << "\n trx " << tr(0) << " try " << tr(1) << " trz " << tr(2) << std::endl;
//   // std::cout << "\n blx " << bl(0) << " bly " << bl(1) << " blz " << bl(2) << std::endl;
//   // std::cout << "\n brx " << br(0) << " bry " << br(1) << " brz " << br(2) << std::endl;
//
//   //check the area
//   bool badArea = false;
//   float a = (bl-br).norm();
//   float b = (tr-br).norm();
//   float c = (tl-tr).norm();
//   float d = (tl-bl).norm();
//   float p = (br-tl).norm();
//   float q = (bl-tr).norm();
//   float arg1 = 2.0*p*q;
//   float arg2 = b*b+d*d-a*a-c*c;
//   float arg = arg1*arg1 - arg2*arg2;
//   // std::cout << "\n arg " << arg << std::endl;
//   if (arg > 0.0)
//   {
//     float area = 0.25*sqrtf(arg);
//     badArea = ((area < minarea) || (area > maxarea));
//   }
//   else
//   {
//     badArea = true;
//   }
//
//   //check if the centroid is too high or low
//   Eigen::Vector3f centroid = 0.25*(tl + tr + bl + br);
//   bool badHeight = ((centroid(2) > maxheight) || (centroid(2) < minheight));
//
//   bool badPatch = badArea || badHeight;
//
//   return badPatch;
// }

void KeyframePlanes::addplane(int planeInd, PointCloudRGB& cloud)
{
  // //if the patch satisfies the conditions then add the plane
  // if (checkPatch(planePointsInit, rows, cols))
  // {
  //   return;
  // }

  // std::cout << "\n keyInd " << keyInd << " planeInd " << planeInd << " entered add plane planePointsInit size " << planePointsInit.size() << std::endl;
  planesInd.push_back(planeInd);
  planes.push_back(cloud);
  // std::vector<pcl::PointXYZRGB> planePoints;
  // pcl::PointXYZRGB pt;
  // for (int ii = 0; ii < planePointsInit.size(); ii++)
  // {
  //   pt.x = planePointsInit.at(ii).x;
  //   pt.y = planePointsInit.at(ii).y;
  //   pt.z = planePointsInit.at(ii).z;
  //   pt.r = planeColorsInit.at(ii);
  //   pt.g = planeColorsInit.at(ii);
  //   pt.b = planeColorsInit.at(ii);
  //   planePoints.push_back(pt);
  // }
  // // std::cout << "\n keyInd " << keyInd << " planeInd " << planeInd << " planePoints size before add plane " << planePoints.size() << std::endl;
  //
  // planesPoints.push_back(planePoints);

  // std::cout << "\n keyInd " << keyInd << " planeInd " << planeInd << " planesPoints size after add plane " << planesPoints.size() << std::endl;
}

void KeyframePlanes::update(int planeIndInd, PointCloudRGB& cloud)
{
  // //if the patch satisfies the conditions then add the plane
  // if (checkPatch(planePointsInit, rows, cols))
  // {
  //   planesPoints.erase(planesPoints.begin()+planeIndInd);
  //   planesInd.erase(planesInd.begin()+planeIndInd);
  //   return;
  // }

  // std::cout << "\n keyInd " << keyInd << " planeInd " << planesInd.at(planeIndInd) << " entered " << std::endl;
  // std::vector<pcl::PointXYZRGB> planePoints;
  // pcl::PointXYZRGB pt;
  // for (int ii = 0; ii < planePointsInit.size(); ii++)
  // {
  //   pt.x = planePointsInit.at(ii).x;
  //   pt.y = planePointsInit.at(ii).y;
  //   pt.z = planePointsInit.at(ii).z;
  //   pt.r = planeColorsInit.at(ii);
  //   pt.g = planeColorsInit.at(ii);
  //   pt.b = planeColorsInit.at(ii);
  //   planePoints.push_back(pt);
  // }
  // planesPoints.at(planeIndInd) = cloud;
  planes.at(planeIndInd) = cloud;

  // std::cout << "\n keyInd " << keyInd << " planeInd " << planesInd.at(planeIndInd) << " planesPoints size update " << planesPoints.at(planeIndInd).size() << std::endl;
}
