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
	nhp.param<float>("fG", fG, 1.0);
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
	nhp.param<int>("numberFeaturesToFindPerPart", numberFeaturesToFindPerPart, 1);
	nhp.param<int>("numberFeaturesPerPartRow", numberFeaturesPerPartRow, 1);
	nhp.param<int>("numberFeaturesPerPartCol", numberFeaturesPerPartCol, 1);
	nhp.param<int>("partitionSide", partitionSide, 1);
	nhp.param<bool>("saveExp", saveExp, false);
	nhp.param<std::string>("expName", expName, "exp");
	nhp.param<int>("patchSizeBase", patchSizeBase, 10);
	nhp.param<int>("checkSizeBase", checkSizeBase, 20);


	float pfix,pfiy,pfiz,qfiw,qfix,qfiy,qfiz;
	nhp.param<float>("pfix", pfix, 0.0);
	nhp.param<float>("pfiy", pfiy, 0.0);
	nhp.param<float>("pfiz", pfiz, 0.0);
	nhp.param<float>("qfiw", qfiw, 1.0);
	nhp.param<float>("qfix", qfix, 0.0);
	nhp.param<float>("qfiy", qfiy, 0.0);
	nhp.param<float>("qfiz", qfiz, 0.0);

	pfi = Eigen::Vector3f(pfix,pfiy,pfiz);

	qfi << qfiw,qfix,qfiy,qfiz;
	qfi /= qfi.norm();

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

	imageSub = it.subscribe(cameraName+"/image_undistort", 100, &Keyframe::imageCB,this);
	odomSub = nh.subscribe(cameraName+"/odom", 100, &Keyframe::odomCB,this);

	// ROS_WARN("keyframe needed subs time %3.7f",float(clock()-initializeKeyframeTime)/CLOCKS_PER_SEC);
}

void Keyframe::startup()
{
	// std::cout << "\n keyframe " << keyInd << " starting up \n";

	// clock_t initializeKeyframeTime = clock();

	// Publishers and subscribers

	// keyPub = it.advertise("/keyframe_"+std::to_string(keyInd)+"/key_image",1);
	imageOutputPub = it.advertise(cameraName+"/features",1);
	// odomPub = nh.advertise<nav_msgs::Odometry>("/keyframe_"+std::to_string(keyInd)+"/odom",1);
	pointCloudPub = nh.advertise<PointCloud> ("key_map", 1);

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
		// determine width and height of each partition
		int partitionWidth = imageWidth/partitionSide;
		int partitionHeight = imageHeight/partitionSide;
		if ((numberFeaturesPerPartRow*minDistance > partitionHeight) || (numberFeaturesPerPartCol*minDistance > partitionWidth))
		{
			ROS_ERROR("Feature config not possible");
			ros::shutdown();
		}

		// get the center of each partition and place uniform spacing of points using minDistance
		std::cout << "\n key \n";
		for (int ii = 0; ii < partitionSide*partitionSide; ii++)
		{
			std::vector<cv::Point2f> ptsii;
			int colc = ii%partitionSide*partitionWidth + partitionWidth/2;
			int rowc = ii/partitionSide*partitionHeight + partitionHeight/2;
			int coltl = colc - (numberFeaturesPerPartCol-1)*minDistance/2;
			int rowtl = rowc - (numberFeaturesPerPartRow-1)*minDistance/2;

			for (int jj = 0; jj < numberFeaturesPerPartRow; jj++)
			{
				int rowii = rowtl + jj*minDistance;
				for (int hh = 0; hh < numberFeaturesPerPartCol; hh++)
				{
					int colii = coltl + hh*minDistance;
					ptsii.push_back(cv::Point2f(colii,rowii));
				}
			}

			newPatch = new PatchEstimator(imageWidth,imageHeight,minFeaturesDanger,minFeaturesBad,keyInd,patchInd,gray,
				                            imageOdom,ptsii,fx,fy,cx,cy,zmin,zmax,t,fq,fp,ft,fn,fd,fG,cameraName,tau,saveExp,
																		expName,patchSizeBase,checkSizeBase,pfi,qfi);
			patchs.push_back(newPatch);
			patchIndMax = patchInd;
			patchInd++;
		}
	}
	catch (cv::Exception e)
	{
		ROS_ERROR("good features to track failed");
		return false;
	}

	// ros::shutdown();

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
		// std::lock_guard<std::mutex> patchMutexGuard(patchMutex);

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

		Eigen::Vector3f pcwHat(imageOdom.pose.pose.position.x,imageOdom.pose.pose.position.y,imageOdom.pose.pose.position.z);
		Eigen::Vector4f qcwHat(imageOdom.pose.pose.orientation.w,imageOdom.pose.pose.orientation.x,imageOdom.pose.pose.orientation.y,imageOdom.pose.pose.orientation.z);
		qcwHat /= qcwHat.norm();

		//check if any remaining patches are shutdown, if so delete
		std::vector<cv::Point2f> pts;
		std::vector<PatchEstimator*> patchsIn;
		for (std::vector<PatchEstimator*>::iterator itP = patchs.begin(); itP != patchs.end(); itP++)
		{
			patchMutex.lock();
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

						Eigen::Vector3f pciHatICLExt = pcwHat + rotatevec(uci*((*itD)->dcHatICLExt),qcwHat);
						Eigen::Vector3f pciHatEKF = pcwHat + rotatevec(mci*((*itD)->zcHatEKF),qcwHat);

						Eigen::Vector4f qcwInit(cos(3.1415/4.0),sin(3.1415/4.0),0.0,0.0);
						qcwInit /= qcwInit.norm();
						Eigen::Vector3f pbiHatICLExt = rotatevec(pciHatICLExt-rotatevec(pfi,getqInv(qfi)),getqInv(qcwInit));
						Eigen::Vector3f pbiHatEKF = rotatevec(pciHatEKF-rotatevec(pfi,getqInv(qfi)),getqInv(qcwInit));

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

						ptHat.x = pbiHatICLExt(0);
						ptHat.y = pbiHatICLExt(1);
						ptHat.z = pbiHatICLExt(2);
						ptHat.r = 0;
						ptHat.g = 200;
						ptHat.b = 0;
						map->points.push_back(ptHat);

						ptHat.x = pbiHatEKF(0);
						ptHat.y = pbiHatEKF(1);
						ptHat.z = pbiHatEKF(2);
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
			patchMutex.unlock();
		}
		patchs = patchsIn;
		patchsIn.clear();

		if (patchs.size() > 0)
		{
			map->width = map->points.size();
			pointCloudPub.publish(map);
			//publish lagged image and odom
			cv_bridge::CvImage out_msg;
			out_msg.header = msg->header; // Same timestamp and tf frame as input image
			out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
			out_msg.image = gray; // Your cv::Mat
			imageOutputPub.publish(out_msg.toImageMsg());
		}
	}

	keyframeInDanger = (float(patchs.size())/float(patchIndMax)) <= 0.4;

	// //publish lagged image and odom
	// cv_bridge::CvImage out_msg;
	// out_msg.header = msg->header; // Same timestamp and tf frame as input image
	// out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
	// out_msg.image = gray; // Your cv::Mat

	// {
	// 	std::lock_guard<std::mutex> pubMutexGuard(pubMutex);
	// 	if (patchs.size() > 0)
	// 	{
	// 		// imageOutputPub.publish(out_msg.toImageMsg());
	// 		// odomPub.publish(imageOdom);
	// 	}
	// }

	float dt = (t-tLast).toSec();
	tLast = t;


	if (keyframeShutdown)
	{
		pubMutex.lock();
		imageSub.shutdown();
		odomSub.shutdown();
		// odomPub.shutdown();
		imageOutputPub.shutdown();
		// pointCloudPub.shutdown();
		pubMutex.unlock();
	}

	if (patchs.size() < 1)
	{
		pubMutex.lock();
		keyframeShutdown = true;
		pubMutex.unlock();
	}
}
