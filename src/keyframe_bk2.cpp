#include <keyframe.h>

Keyframe::Keyframe() : it(nh)
{
}

Keyframe::Keyframe(int keyIndInit, cv::Mat& camMat, std::vector<cv::Mat>& masksInit, int imageWidthInit, int imageHeightInit) : it(nh)
{
	// Get parameters
	ros::NodeHandle nhp("~");
	nhp.param<std::string>("cameraName", cameraName, "camera");
	nhp.param<int>("patchRadius", patchRadius, 50);
	nhp.param<float>("fq", fq, 1.0);
	nhp.param<float>("fp", fp, 1.0);
	nhp.param<float>("ft", ft, 1.0);
	nhp.param<float>("fn", fn, 1.0);
	nhp.param<float>("fd", fd, 1.0);
	nhp.param<float>("zmin", zmin, 1.0);
	nhp.param<float>("zmax", zmax, 1.0);
	nhp.param<float>("tau", tau, 1.0);
	nhp.param<int>("minDistance", minDistance, 9);
	nhp.param<int>("blockSize", blockSize, 3);
	nhp.param<float>("qualityLevel", qualityLevel, 0.1);
	nhp.param<int>("minFeaturesBad", minFeaturesBad, 25);
	nhp.param<int>("minFeaturesDanger", minFeaturesDanger, 50);
	nhp.param<int>("partitionRows", partitionRows, 2);
	nhp.param<int>("partitionCols", partitionCols, 2);
	nhp.param<int>("numberFeaturesToFindPerPart", numberFeaturesToFindPerPart, 1.0);
	nhp.param<bool>("saveExp", saveExp, false);
	nhp.param<std::string>("expName", expName, "exp");

	keyInd = keyIndInit;

	// nh = ros::NodeHandle("keyframe_"+std::to_string(keyInd))
	firstGray = true;
	keyframeShutdown = false;
	firstOutput = true;
	tooFewFeatures = false;
	foundKeyImage = false;
	keyHeight = 0.0;
	keyToCeiling = 0.0;
	keyframeInDanger = false;
	firstOdom = true;
	patchIndMax = 0;
	imageWidth = imageWidthInit;
	imageHeight = imageHeightInit;

	for (int ii = 0; ii < masksInit.size(); ii++)
	{
		masks.push_back(masksInit.at(ii).clone());
	}

	// tlmask = tlmaskInit.clone();
	// trmask = trmaskInit.clone();
	// blmask = blmaskInit.clone();
	// brmask = brmaskInit.clone();

	maxdt = 0.075;

	fx = camMat.at<float>(0,0);
	fy = camMat.at<float>(1,1);
	cx = camMat.at<float>(0,2);
	cy = camMat.at<float>(1,2);

	clock_t initializeKeyframeTime = clock();

	imageSub = it.subscribe(cameraName+"/image_undistort", 1000, &Keyframe::imageCB,this);
	odomSub = nh.subscribe(cameraName+"/odom", 1000, &Keyframe::odomCB,this);

	// ROS_WARN("keyframe needed subs time %3.7f",float(clock()-initializeKeyframeTime)/CLOCKS_PER_SEC);
}

void Keyframe::startup()
{
	// std::cout << "\n keyframe " << keyInd << " starting up \n";

	// clock_t initializeKeyframeTime = clock();

	// Publishers and subscribers

	// keyPub = it.advertise("/keyframe_"+std::to_string(keyInd)+"/key_image",1);
	imageOutputPub = it.advertise(cameraName+"/features",1);
	odomPub = nh.advertise<nav_msgs::Odometry>("/keyframe_"+std::to_string(keyInd)+"/odom",1);
	pointCloudPub = nh.advertise<PointCloud> ("wall_map", 1);

	std::cout << "\n keyframe " << keyInd << " started \n";

}

void Keyframe::odomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
	if (keyframeShutdown)
	{
		return;
	}
	std::lock_guard<std::mutex> odomMutexGuard(odomMutex);
	odomSync.push_back(*msg);
	// std::cout << "\n newMsg.header.stamp \n" << newMsg.header.stamp << std::endl;
	// odomSync.push_back(newMsg);
}

bool Keyframe::findFeatures(cv::Mat& gray, ros::Time t, nav_msgs::Odometry imageOdom)
{
	std::vector<cv::Point2f> kPts2f;
	clock_t initializeKeyframeTime = clock();

	// std::cout << "\n gray.size() " << gray.size() << std::endl;

	int patchInd = 0;
	try
	{
		std::lock_guard<std::mutex> patchMutexGuard(patchMutex);

		// determine step size
		int colstep = imageWidth/partitionCols;
		int rowstep = imageHeight/partitionRows;
		std::vector<cv::Point2f> ptsii;
		std::cout << "\n key \n";
		for (int ii = 2; ii < partitionRows-2; ii++)
		{
			//choose the entire partition as each patches feature seperated by min seperation
			int rowii = rowstep*ii;

			for (int jj = 2; jj < partitionCols-2; jj++)
			{
				int colii = colstep*jj;
				ptsii.push_back(cv::Point2f(colii,rowii));
				// std::cout << "\n ptx " << colii << " pty " << rowii << std::endl;
			}
		}

		newPatch = new PatchEstimator(imageWidth,imageHeight,minFeaturesDanger,minFeaturesBad,keyInd,patchInd,gray,imageOdom,ptsii,fx,fy,cx,cy,zmin,zmax,t,fq,fp,ft,fn,fd,cameraName,tau,saveExp,expName);
		patchs.push_back(newPatch);
		patchIndMax = patchInd;
		// delete newPatch;
		// activePatchs.push_back(patchInd);
		// std::cout << "\n patch " << patchInd << " after create" << std::endl;
		patchInd++;
	}
	catch (cv::Exception e)
	{
		ROS_ERROR("good features to track failed");
		return false;
	}

	if (patchs.size() > 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void Keyframe::imageCB(const sensor_msgs::Image::ConstPtr& msg)
{
	// ROS_WARN("KEYFRAME %d GRAY START",keyInd);
	if (keyframeShutdown)
	{
		return;
	}
	clock_t processTime = clock();

	// sync the times to find the closest
	ros::Time t = msg->header.stamp;
	std::vector<float> timeDiff;
	// nav_msgs::Odometry imageOdom;
	// int numMatch = odomSync.size();

	// std::cout << "\n t key " << t << std::endl;
	{
		std::lock_guard<std::mutex> odomMutexGuard(odomMutex);
		std::vector<float> timeDiff;
		int numMatch = odomSync.size();

		// std::cout << "\n t key " << t << std::endl;

		// std::cout << "\n numMatch " << numMatch << std::endl;
		if (numMatch > 0)
		{
			// std::cout << "\n recent dt \n" << (odomSync.at(numMatch-1).header.stamp - t).toSec() << std::endl;
			for (int ii = 0; ii < numMatch; ii++)
			{
				float dtii = fabsf((odomSync.at(ii).header.stamp - t).toSec());
				timeDiff.push_back(fabsf(dtii));
			}

			int minTimeInd = std::distance(timeDiff.begin(),std::min_element(timeDiff.begin(),timeDiff.end()));
			imageOdom = odomSync.at(minTimeInd);

			// std::cout << "\n odomSync size " << odomSync.size() << std::endl;
			if ((odomSync.size() > 1) && (minTimeInd < odomSync.size()-1))
			{
				for (int ii = 0; ii <= minTimeInd; ii++)
				{
					odomSync.pop_front();
				}
			}

			// std::cout << "\n odomSync size " << odomSync.size() << std::endl;
			if (firstGray)
			{
				firstGray = false;
			}
		}
		else
		{
			// ROS_ERROR("KEYFRAME %d NO NEW ODOM",keyInd);
			if (firstGray)
			{
				return;
			}
			// return;
		}
	}

	// std::cout << "\n fabsf(imageOdom.pose.pose.position.y) " << fabsf(imageOdom.pose.pose.position.y) << std::endl;
	// convert to opencv image
	cv_bridge::CvImagePtr cv_ptr;

	cv::Mat gray; // current image
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
		gray = cv_ptr->image;
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// std::cout << "\n gray.size() " << gray.size() << std::endl;

	//check if key has been found and if it hasnt try finding in new image
	if (!foundKeyImage)
	{
		// if (fabsf(imageOdom.pose.pose.position.y) < 0.25)
		// {
		// 	return;
		// }

		if (findFeatures(gray,t,imageOdom))
		{
			kgray = gray.clone();
			// pgray = gray.clone();
			keyOdom = imageOdom;
			tLast = t;
			foundKeyImage = true;

			//publish and subscribe
			startup();

			// cv_bridge::CvImage out_msg;
			// out_msg.header = msg->header; // Same timestamp and tf frame as input image
			// out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
			// out_msg.image = kgray; // Your cv::Mat
			// keyPub.publish(out_msg.toImageMsg());

		}
		// publish key image
		return;
	}

	{
		std::lock_guard<std::mutex> patchMutexGuard(patchMutex);

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

		//check if any remaining patches are shutdown, if so delete
		std::vector<cv::Point2f> pts;
		std::vector<PatchEstimator*> patchsIn;
		for (std::vector<PatchEstimator*>::iterator itP = patchs.begin(); itP != patchs.end(); itP++)
		{
			if (!(*itP)->patchShutdown)
			{
				patchsIn.push_back(*itP);
				if ((*itP)->depthEstimators.size() > 0)
				{
					//publish the image
					pcl::PointXYZRGB pt,ptHat;
					for (std::vector<DepthEstimator*>::iterator itD = (*itP)->depthEstimators.begin(); itD != (*itP)->depthEstimators.end(); itD++)
					{
						//get the points after update
						Eigen::Vector3f mci = (*itD)->mc;
						Eigen::Vector3f uci = mci/mci.norm();
						cv::Point2f cPti(fx*mci(0)+cx,fy*mci(1)+cy);
						cv::circle(gray, cPti, 10, cv::Scalar(150, 150, 150), -1);

						// Eigen::Vector3f mkiHat = depthEstimators.at(ii)->mk;
						// cv::Point2i kPti(int(fx*mkiHat(0)+cx),int(fy*mkiHat(1)+cy));
						// uint8_t colori = kimage.at<uint8_t>(kPti.y,kPti.x);

						Eigen::Vector3f pciHatICLExt = uci*((*itD)->dcHatICLExt);
						Eigen::Vector3f pciHatEKF = mci*((*itD)->zcHatEKF);

						// std::cout << "\n pciHatICLExt \n" << pciHatICLExt << std::endl;

						// if (saveExp)
						// {
						// 	// DataSave* dataSave = new DataSave(timeFromStart,pkc,qkc,pkcHat,qkcHat,pki,pkiHat,pics.at(ii),pciHat,pciHatEKF,pciHatLS,vc,wc);
						// 	DataSave* dataSave = new DataSave(timeFromStart,pkc,qkc,pkcHat,qkcHat,pki,pkiHatICL,pics.at(ii),pciHatICL,pciHatEKF,pciHatLS,pciHatICLExt,vc,wc);
						// 	data.push_back(dataSave);
						// }

						// //bring into world frame
						// pciHatICLExt = pcwHat + rotatevec(pciHatICLExt,qcwHat);
						// pciHatEKF = pcwHat + rotatevec(pciHatEKF,qcwHat);

						ptHat.x = pciHatICLExt(0);
						ptHat.y = pciHatICLExt(1);
						ptHat.z = pciHatICLExt(2);
						ptHat.r = 0;
						ptHat.g = 200;
						ptHat.b = 0;
						map->points.push_back(ptHat);

						ptHat.x = pciHatEKF(0);
						ptHat.y = pciHatEKF(1);
						ptHat.z = pciHatEKF(2);
						ptHat.r = 0;
						ptHat.g = 0;
						ptHat.b = 255;
						map->points.push_back(ptHat);

						// std::cout << "\n index " << ii << std::endl;
						// std::cout << "\n dkHat " << depthEstimators.at(ii)->dkHat << std::endl;
						// std::cout << "\n dcHat " << depthEstimators.at(ii)->dcHat << std::endl;
						// std::cout << "\n zcHatICLExt " << pciHatICLExt(2) << std::endl;
						// std::cout << "\n zcHatEKF " << pciHatEKF(2) << std::endl;
						// std::cout << "\n dk " << depthEstimators.at(ii)->pik.norm() << std::endl;
					}
				}
			}
			else
			{
				delete *itP;
			}
		}
		patchs = patchsIn;
		patchsIn.clear();
		map->width = map->points.size();
		pointCloudPub.publish(map);
		//publish lagged image and odom
		cv_bridge::CvImage out_msg;
		out_msg.header = msg->header; // Same timestamp and tf frame as input image
		out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
		out_msg.image = gray; // Your cv::Mat
		imageOutputPub.publish(out_msg.toImageMsg());
	}

	keyframeInDanger = (float(patchs.size())/float(patchIndMax)) <= 0.4;

	// //publish lagged image and odom
	// cv_bridge::CvImage out_msg;
	// out_msg.header = msg->header; // Same timestamp and tf frame as input image
	// out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
	// out_msg.image = gray; // Your cv::Mat

	{
		std::lock_guard<std::mutex> pubMutexGuard(pubMutex);
		if (patchs.size() > 0)
		{
			// imageOutputPub.publish(out_msg.toImageMsg());
			// odomPub.publish(imageOdom);
		}
	}

	float dt = (t-tLast).toSec();
	tLast = t;

	// if (keyframeShutdown)
	// {
	// 	odomSub.shutdown();
	// 	odomPub.shutdown();
	// 	grayPub.shutdown();
	// 	graySub.shutdown();
	// }

	if (patchs.size() < 1)
	{
		keyframeShutdown = true;
	}
}
