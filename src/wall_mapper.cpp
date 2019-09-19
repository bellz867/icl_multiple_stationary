#include <wall_mapper.h>

WallMapper::~WallMapper()
{
	for (int ii = 0; ii < keyframePlanes.size(); ii++)
	{
		delete keyframePlanes.at(ii);
	}
	keyframePlanes.clear();
}

WallMapper::WallMapper() : it(nh)
{
	// Get parameters
	ros::NodeHandle nhp("~");
	nhp.param<std::string>("cameraName", cameraName, "camera");
	nhp.param<int>("width", width, 100);
	nhp.param<int>("height", height, 100);
	nhp.param<float>("minarea", minarea, 0.25);
	nhp.param<float>("maxarea", maxarea, 25.0);
	nhp.param<float>("minheight", minheight, 0.25);
	nhp.param<float>("maxheight", maxheight, 25.0);
	nhp.param<bool>("saveExp", saveExp, false);
	nhp.param<std::string>("expName", expName, "");
	//
	// boost::filesystem::path dir("path");
	//
  //   if(!(boost::filesystem::exists(dir))){
  //       std::cout<<"Doesn't Exists"<<std::endl;
	//
  //       if (boost::filesystem::create_directory(dir))
  //           std::cout << "....Successfully Created !" << std::end;
  //   }

	wallSub = nh.subscribe("/wall_points",100,&WallMapper::wallCB,this);
	odomSub = nh.subscribe("/mocap/camera/odom",1,&WallMapper::odomCB,this);
	// wallPub = it.advertise("wall_image",1);
	pointCloudPub = nh.advertise<PointCloud> ("wall_map", 1);
	pointCloudTruePub = nh.advertise<PointCloud> ("wall_map_true", 1);

	center = cv::Point2i(width/2,height/2);

	camCenter = center;
	camEnd = center + cv::Point2i(0,5);

	////////
	//corners
	pcl::PointXYZRGB blL(255,0,0);
	blL.x = 6.16;
	blL.y = 1.86;
	blL.z = 0.0;
	pcl::PointXYZRGB brL(255,0,0);
	brL.x = 1.2;
	brL.y = 1.86;
	brL.z = 0.0;
	pcl::PointXYZRGB trL(255,0,0);
	trL.x = 1.2;
	trL.y = -1.86;
	trL.z = 0.0;
	pcl::PointXYZRGB tlL(255,0,0);
	tlL.x = 6.16;
	tlL.y = -1.86;
	tlL.z = 0.0;
	pcl::PointXYZRGB blR(255,0,0);
	blR.x = -1.2;
	blR.y = 1.86;
	blR.z = 0.0;
	pcl::PointXYZRGB brR(255,0,0);
	brR.x = -6.16;
	brR.y = 1.86;
	brR.z = 0.0;
	pcl::PointXYZRGB trR(255,0,0);
	trR.x = -6.16;
	trR.y = -1.86;
	trR.z = 0.0;
	pcl::PointXYZRGB tlR(255,0,0);
	tlR.x = -1.2;
	tlR.y = -1.86;
	tlR.z = 0.0;

	cloud_true.height = 1;
	cloud_true.width = 8;
	cloud_true.is_dense = true;
	cloud_true.resize(8);
	cloud_true.at(0) = blL;
	cloud_true.at(1) = brL;
	cloud_true.at(2) = trL;
	cloud_true.at(3) = tlL;
	cloud_true.at(4) = blR;
	cloud_true.at(5) = brR;
	cloud_true.at(6) = trR;
	cloud_true.at(7) = tlR;

	pcw = Eigen::Vector3f::Zero();
	qcw = Eigen::Vector4f::Zero();
	qcw(0) = 1.0;

	//draw the lines between the walls
	// for (PointCloudRGB::iterator cloudIt = cloud.begin(); cloudIt != cloud.end(); cloudIt++)
	// {
	//
	// }
}

void WallMapper::odomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
	Eigen::Vector3f pcwNew(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
	Eigen::Vector4f qcwNew(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
	qcwNew /= qcwNew.norm();

	odomMutex.lock();

	pcw = pcwNew;
	qcw = qcwNew;

	if (firstOdom)
	{
		firstOdom = false;
	}

	odomMutex.unlock();

	PointCloudRGB::Ptr map_true(new PointCloudRGB);
	pcl_conversions::toPCL(msg->header.stamp,map_true->header.stamp);

	// std::cout << "\n wall 1 1 \n";
	map_true->header.frame_id = "world";

	// std::cout << "\n wall 1 2 \n";
	map_true->height = 1;
	map_true->is_dense = true;
	map_true->points.clear();
	*map_true += cloud_true;
	map_true->width = map_true->points.size();
	pointCloudTruePub.publish(map_true);

	//the angle should be approximately the angle about the y so use the w and y components
	// get the yaw from the current estimate
	// float sinyaw = 2.0*(qcwNew(0)*qcwNew(3) + qcwNew(1)*qcwNew(2));
	// float cosyaw = 1.0 - 2.0*(qcwNew(2)*qcwNew(2) + qcwNew(3)*qcwNew(3));
	// float angle = atan2f(sinyaw,cosyaw);
	// float angle = atan2f(qcwNew(2),qcwNew(0))*2.0;
	// cv::Point2i camCenterNew = center + cv::Point2i(10*pcwNew(0),-10*pcwNew(2));
	// cv::Point2i camEndNew = camCenterNew + cv::Point2i(8*sinf(angle),-8*cosf(angle));

	// std::cout << "\n angle " << angle << std::endl;
	// std::cout << "\n cosf(angle) " << 5*cosf(angle) << std::endl;
	// std::cout << "\n sinf(angle) " << 5*sinf(angle) << std::endl;
	// std::cout << "\n camCenterNew " << camCenterNew << std::endl;

	// std::cout << "\n camCenterNew " << camEndNew << std::endl;

	// camCenter = camCenterNew;
	// camEnd = camEndNew;



	// // get all the points and draw them on the image
	// // std::cout << "\n wall 1 \n";
	// PointCloudRGB::Ptr map(new PointCloudRGB);
	// pcl_conversions::toPCL(msg->header.stamp,map->header.stamp);
	//
	// // std::cout << "\n wall 1 1 \n";
	// map->header.frame_id = "map";
	//
	// // std::cout << "\n wall 1 2 \n";
	// map->height = 1;
	// map->is_dense = true;
	// map->points.clear();
	//
	// // std::cout << "\n wall 2 \n";
	//
	// // cv::Mat image(cv::Size(width,height),CV_8UC1,cv::Scalar(0, 0, 0));
	// for (int i = 0; i < keyClouds.size(); i++)
	// {
	// 	for (int j = 0; j < keyClouds.at(i).size(); j++)
	// 	{
	// 		// if ((wallPts.at(i).at(j).y < height) && (wallPts.at(i).at(j).y >= 0) && (wallPts.at(i).at(j).x < width) && (wallPts.at(i).at(j).x >= 0))
	// 		// {
	// 		// 	image.at<uint8_t>(wallPts.at(i).at(j).y,wallPts.at(i).at(j).x) = 255;
	// 		// }
	// 		pcl::PointXYZRGB pt;
	// 		pt.x = keyClouds.at(i).at(j).x;
	// 		pt.y = keyClouds.at(i).at(j).y;
	// 		pt.z = keyClouds.at(i).at(j).z;
	// 		pt.r = keyColors.at(i).at(j);
	// 		pt.g = keyColors.at(i).at(j);
	// 		pt.b = keyColors.at(i).at(j);
	// 		map->points.push_back(pt);
	// 		//cv::circle(image, wallPts.at(i).at(j), 1, cv::Scalar(255, 255, 255), -1, 1);
	// 	}
	// }
	// map->width = map->points.size();
	//
	// // std::cout << "\n wall 3 \n";
	// pointCloudPub.publish(map);

	// cv::circle(image, camCenter, 5, cv::Scalar(150, 150, 150), 1, 1);
	// cv::circle(image, center, 5, cv::Scalar(150, 150, 150), 1, 1);
	// cv::line(image, camCenter, camEnd, cv::Scalar(150,150,150),1);
	// cv::line(image, center, center-cv::Point2i(0,8), cv::Scalar(150,150,150),1);

	// //publish equalized image
	// cv_bridge::CvImage image_msg;
	// image_msg.header = msg->header; // Same timestamp and tf frame as input image
	// image_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
	// image_msg.image = image; // Your cv::Mat
	// wallPub.publish(image_msg.toImageMsg());
}

//wall sub callback
void WallMapper::wallCB(const icl_multiple_stationary::Wall::ConstPtr& msg)
{
	if (firstOdom)
	{
		return;
	}

	ros::Time t = msg->header.stamp;
	PointCloudRGB cloud;
	cloud.clear();
	pcl::fromROSMsg(msg->cloud,cloud);
	int keyInd = int(msg->keyInd);
	int patchInd = int(msg->patchInd);
	Eigen::Vector3f pcwHat(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
	Eigen::Vector4f qcwHat(msg->pose.orientation.w,msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z);

	// std::cout << "\n keyInd " << keyInd << " patchInd " << patchInd << " keyframePlanes.size() " << keyframePlanes.size() << std::endl;

	// determine if the key index is a set yet or not
	bool keyOnWall = false;
	bool patchOnWall = false;
	int keyIndInd = 0;
	int patchIndInd = 0;
	int i = 0;
	int j = 0;
	while ((i < keyframePlanes.size()) && !keyOnWall)
	{
		j = 0;
		if (keyframePlanes.at(i)->keyInd == keyInd)
		{
			keyOnWall = true;
			keyIndInd = i;
			while ((j < keyframePlanes.at(i)->planesInd.size()) && !patchOnWall)
			{
				if (keyframePlanes.at(i)->planesInd.at(j) == patchInd)
				{
					patchOnWall = true;
					patchIndInd = j;
				}
				else
				{
					j++;
				}
			}
		}
		else
		{
			i++;
		}
	}
	// std::cout << "\n keyOnWall " << keyOnWall << " patchOnWall " << patchOnWall << std::endl;

	// if it is on the wall update the points at the index otherwise add the set
	if (keyOnWall)
	{
		wallMutex.lock();
		if(patchOnWall)
		{
			keyframePlanes.at(keyIndInd)->update(patchIndInd,cloud,pcw,qcw,pcwHat,qcwHat,bool(msg->allPtsKnown),msg->inds,msg->dkKnowns,msg->header.stamp);
		}
		// else
		// {
		// 	keyframePlanes.at(keyIndInd)->addplane(patchInd,cloud,pcw,qcw,pcwHat,qcwHat);
		// }
		wallMutex.unlock();
	}
	else
	{
		wallMutex.lock();
		KeyframePlanes* newKeyframePlanes = new KeyframePlanes(minarea,maxarea,minheight,maxheight,keyInd,patchInd,cloud,pcw,qcw,pcwHat,qcwHat,cloud_true,bool(msg->allPtsKnown),msg->inds,msg->dkKnowns,saveExp,expName,msg->header.stamp);
		keyframePlanes.push_back(newKeyframePlanes);
		wallMutex.unlock();
	}

	//plot the points
	// get all the points and draw them on the image
	// std::cout << "\n wall 1 \n";
	PointCloudRGB::Ptr map(new PointCloudRGB);
	pcl_conversions::toPCL(msg->header.stamp,map->header.stamp);

	// std::cout << "\n wall 1 1 \n";
	map->header.frame_id = "world";

	// std::cout << "\n wall 1 2 \n";
	map->height = 1;
	map->is_dense = true;
	map->points.clear();

	// std::cout << "\n wall 2 \n";

	//for each key
	for (int i = 0; i < keyframePlanes.size(); i++)
	{
		//for each plane in that key
		for (int j = 0; j < keyframePlanes.at(i)->planes.size(); j++)
		{
			if (keyframePlanes.at(i)->allPtsKnown)
			{
				// *map += keyframePlanes.at(i)->planes.at(j);
				// *map += keyframePlanes.at(i)->planesTrue.at(j);
				*map += keyframePlanes.at(i)->planesDraw;
				*map += keyframePlanes.at(i)->planesTrueDraw;
			}
			// //for each point on that plane
			// for (int k = 0; k < keyframePlanes.at(i)->planesPoints.at(j).size(); k++)
			// {
			// 	*map += keyframePlanes.at(i)->planesPoints.at(j).at(k));
			// }
		}
	}
	map->width = map->points.size();

	ROS_WARN("wall map->width %d",int(map->width));
	// std::cout << "\n wall 3 \n";
	pointCloudPub.publish(map);

	// // get all the points and draw them on the image
	// cv::Mat image(cv::Size(width,height),CV_8UC1,cv::Scalar(0, 0, 0));
	// for (int i = 0; i < wallPts.size(); i++)
	// {
	// 	for (int j = 0; j < wallPts.at(i).size(); j++)
	// 	{
	// 		if ((wallPts.at(i).at(j).y < height) && (wallPts.at(i).at(j).y >= 0) && (wallPts.at(i).at(j).x < width) && (wallPts.at(i).at(j).x >= 0))
	// 		{
	// 			image.at<uint8_t>(wallPts.at(i).at(j).y,wallPts.at(i).at(j).x) = 255;
	// 		}
	// 		//cv::circle(image, wallPts.at(i).at(j), 1, cv::Scalar(255, 255, 255), -1, 1);
	// 	}
	// }
	// cv::circle(image, camCenter, 5, cv::Scalar(150, 150, 150), 1, 1);
	// cv::line(image, camCenter, camEnd, cv::Scalar(150,150,150),1);
	//
	// //publish equalized image
	// cv_bridge::CvImage image_msg;
	// image_msg.header = msg->header; // Same timestamp and tf frame as input image
	// image_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
	// image_msg.image = image; // Your cv::Mat
	// wallPub.publish(image_msg.toImageMsg());
}
