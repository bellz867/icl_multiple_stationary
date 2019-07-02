#include <wall_mapper.h>

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

	wallSub = nh.subscribe("/wall_points",25,&WallMapper::wallCB,this);
	odomSub = nh.subscribe(cameraName+"/odom",1,&WallMapper::odomCB,this);
	// wallPub = it.advertise("wall_image",1);
	pointCloudPub = nh.advertise<PointCloud> ("wall_map", 1);

	center = cv::Point2i(width/2,height/2);

	camCenter = center;
	camEnd = center + cv::Point2i(0,5);
}

void WallMapper::odomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
	Eigen::Vector3f pcwNew(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
	Eigen::Vector4f qcwNew(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
	qcwNew /= qcwNew.norm();

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

	if (firstOdom)
	{
		firstOdom = false;
	}

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
	std::vector<geometry_msgs::Point32> wallPtsMsg = msg->wallPts;
	std::vector<uint8_t> wallColorsMsg = msg->colors;
	int keyInd = int(msg->keyInd);
	int patchInd = int(msg->patchInd);
	int rows = int(msg->rows);
	int cols = int(msg->cols);

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
		if(patchOnWall)
		{
			keyframePlanes.at(keyIndInd)->update(patchIndInd,wallPtsMsg,wallColorsMsg,rows,cols);
		}
		else
		{
			keyframePlanes.at(keyIndInd)->addplane(patchInd,wallPtsMsg,wallColorsMsg,rows,cols);
		}
	}
	else
	{
		KeyframePlanes* newKeyframePlanes = new KeyframePlanes(minarea,maxarea,minheight,maxheight,keyInd,patchInd,wallPtsMsg,wallColorsMsg,rows,cols);
		keyframePlanes.push_back(newKeyframePlanes);
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
		for (int j = 0; j < keyframePlanes.at(i)->planesPoints.size(); j++)
		{
			//for each point on that plane
			for (int k = 0; k < keyframePlanes.at(i)->planesPoints.at(j).size(); k++)
			{
				map->points.push_back(keyframePlanes.at(i)->planesPoints.at(j).at(k));
			}
		}
	}
	map->width = map->points.size();

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
