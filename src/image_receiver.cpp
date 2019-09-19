#include <image_receiver.h>

ImageReceiver::~ImageReceiver()
{
	for (int ii = 0; ii < keyframes.size(); ii++)
	{
		delete keyframes.at(ii);
	}
}

ImageReceiver::ImageReceiver() : it(nh)
{
	// Parameters
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


	float pcbx,pcby,pcbz,qcbw,qcbx,qcby,qcbz;
	nhp.param<float>("pcbx", pcbx, 0.0);
	nhp.param<float>("pcby", pcby, 0.0);
	nhp.param<float>("pcbz", pcbz, 0.0);
	nhp.param<float>("qcbw", qcbw, 1.0);
	nhp.param<float>("qcbx", qcbx, 0.0);
	nhp.param<float>("qcby", qcby, 0.0);
	nhp.param<float>("qcbz", qcbz, 0.0);

	pcb = Eigen::Vector3f(pcbx,pcby,pcbz);

	qcb << qcbw,qcbx,qcby,qcbz;
	qcb /= qcb.norm();

	keyInd = 0;

	// Get camera parameters
	gotCamParam = false;
	camInfoSub = nh.subscribe(cameraName+"/camera_info",1,&ImageReceiver::camInfoCB,this);
	ROS_INFO("Waiting for camera parameters on topic %s/camera_info",cameraName.c_str());
	do
	{
		ros::spinOnce();
		ros::Duration(0.3).sleep();
	} while (!(ros::isShuttingDown()) && !gotCamParam);
	ROS_INFO("Got forward camera parameters");

	// Publishers
	undistortPub = it.advertise(cameraName+"/image_undistort",60);

	// Subscribers
	imageSub = it.subscribe(cameraName+"/image_raw", 60, &ImageReceiver::imageCB,this);
	keyframeSub = it.subscribe(cameraName+"/image_undistort", 60, &ImageReceiver::keyframeCB,this);
	odomSub = nh.subscribe(cameraName+"/odom", 10, &ImageReceiver::odomCB,this);

	fx = camMat.at<float>(0,0);
	fy = camMat.at<float>(1,1);
	cx = camMat.at<float>(0,2);
	cy = camMat.at<float>(1,2);
}

void ImageReceiver::odomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
	std::lock_guard<std::mutex> odomMutexGuard(odomMutex);
	odomSync.push_back(*msg);
}

void ImageReceiver::imageCB(const sensor_msgs::Image::ConstPtr& msg)
{
	clock_t processTime = clock();

	// convert to opencv image
	cv_bridge::CvImagePtr cv_ptr;
	ros::Time t = msg->header.stamp;
	// std::cout << "\n t forward " << t << std::endl;

	cv::Mat image; // current image
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
		image = cv_ptr->image;
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// undistort image
	cv::Mat imageUndistort(image.size(),CV_8UC1);
	cv::remap(image,imageUndistort,map1,map2,CV_INTER_LINEAR,cv::BORDER_CONSTANT,cv::Scalar(0,0,0));

	//publish image
	cv_bridge::CvImage undistort_msg;
	undistort_msg.header = msg->header; // Same timestamp and tf frame as input image
	undistort_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
	undistort_msg.image = imageUndistort; // Your cv::Mat
	undistortPub.publish(undistort_msg.toImageMsg());
}

//gray callback
void ImageReceiver::keyframeCB(const sensor_msgs::Image::ConstPtr& msg)
{
	clock_t processTime = clock();

	ros::Time t = msg->header.stamp;

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
			if (firstKey)
			{
				firstKey = false;
			}
		}
		else
		{

			// ROS_ERROR("FORWARD IMAGE NO NEW ODOM");

			if (firstKey)
			{
				return;
			}
			// return;
		}
	}

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
		//ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	//check if a new frame is needed
	// bool addframe = false;
	std::vector<bool> addKeytoPartition(partitionRows*partitionCols);
	for (int ii = 0; ii < addKeytoPartition.size(); ii++)
	{
		addKeytoPartition.at(ii) = true;
	}

	//if an active keyframe then check if moved or rotated away a certain amount, if so add another key frame
	float avgcPtx = 0.0;
	float avgcPty = 0.0;
	bool notFirst = false;
	bool tooFewFeatures = false;
	std::mutex imageMutex;
	if (keyframes.size() > 0)
	{
		// //determine how far from last key
		// Eigen::Vector3f lastpkw(lastKeyOdom.pose.pose.position.x,lastKeyOdom.pose.pose.position.y,lastKeyOdom.pose.pose.position.z);
		// Eigen::Vector4f lastqkw(lastKeyOdom.pose.pose.orientation.w,lastKeyOdom.pose.pose.orientation.x,lastKeyOdom.pose.pose.orientation.y,lastKeyOdom.pose.pose.orientation.z);
		//
		// Eigen::Vector3f pcw(imageOdom.pose.pose.position.x,imageOdom.pose.pose.position.y,imageOdom.pose.pose.position.z);
		// Eigen::Vector4f qcw(imageOdom.pose.pose.orientation.w,imageOdom.pose.pose.orientation.x,imageOdom.pose.pose.orientation.y,imageOdom.pose.pose.orientation.z);
		//
		// float pdiff = (pcw - lastpkw).norm();
		// float angdiff = 2.0*fabsf(acos(qcw(0)) - acos(lastqkw(0)));

		// if ((pdiff > 0.15) || (angdiff > 10.0*3.14/180.0))
		// {
		// 		addframe = true;
		// }

		//go through existing keyframes and determine if still active, if not delete
		std::vector<PatchEstimator*> keyframesIn;

		imageMutex.lock();
		for (int ii = 0; ii < keyframes.size(); ii++)
		{
			bool keepframe = false;


			if (!keyframes.at(ii)->firstImage)
			{
				// if (!addframe && (keyframes.size() < 2) && keyframes.at(ii)->keyframeInDanger)
				// {
				// 	addframe = true;
				// }

				if (!keyframes.at(ii)->patchShutdown)
				{
					// if (keyframes.at(ii)->depthEstimators.size() >= minFeaturesBad)
					// {
					// 	keepframe = true;
					// }
					keepframe = true;
				}
			}
			else
			{
				keepframe = true;
			}


			if (keepframe)
			{
				keyframesIn.push_back(keyframes.at(ii));
				// std::cout << "\n keyframes.at(ii)->currentAvgPartition " << keyframes.at(ii)->currentAvgPartition << std::endl;


				// if (!keyframes.at(ii)->firstImage)
				// {
					// avgcPtx += keyframes.at(ii)->avgcPtx;
					// avgcPty += keyframes.at(ii)->avgcPty;
					// notFirst = true;
					// addKeytoPartition.at(keyframes.at(ii)->currentAvgPartition) = false;
					// if (!tooFewFeatures)
					// {
					// 	if (keyframes.at(ii)->depthEstimators.size() > 30)
					// 	{
					//
					// 		tooFewFeatures = true;
					// 	}
					// }
				// }
				// else
				// {
				// 	if (notFirst)
				// 	{
				// 		notFirst = false;
				// 	}
				// }
			}
			else
			{
				delete keyframes.at(ii);
			}
		}
		keyframes = keyframesIn;
		keyframesIn.clear();
		imageMutex.unlock();
	}

	// bool notCentered = false;
	// if ((keyframes.size() > 0) && (partitionCols < 2))
	// {
	// 	avgcPtx /= float(keyframes.size());
	// 	avgcPty /= float(keyframes.size());
	//
	//
	// 	if ((avgcPtx < 0.3*imageWidth) || (avgcPtx > 0.7*imageWidth))
	// 	{
	// 		notCentered = true;
	// 	}
	// }

	// std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n not centered " << int(notCentered);

	// else
	// {
	// 	addframe = true;
	// }

	//ROS_WARN("match time %3.7f",float(clock()-processTime)/CLOCKS_PER_SEC);
	clock_t keyTime = clock();

	// add a new keyframe if needed and total is less than 3
	bool added = false;
	if (keyframes.size() < 1)
	{
		for(int ii = 0; ii < addKeytoPartition.size(); ii++)
		{
			if (addKeytoPartition.at(ii))
			{
				lastKeyOdom = imageOdom;
				newKeyframe = new PatchEstimator(imageWidth,imageHeight,partitionRows,partitionCols,minDistance,minFeaturesDanger,minFeaturesBad,keyInd,0,ii,
																			fx,fy,cx,cy,zmin,zmax,fq,fp,ft,fn,fd,fG,cameraName,tau,saveExp,
																			expName,patchSizeBase,checkSizeBase,pcb,qcb,numberFeaturesPerPartCol,numberFeaturesPerPartRow);
				keyframes.push_back(newKeyframe);
				added = true;
				keyInd++;
			}
		}
	}

	// std::cout << " keyframes.size() " << int(keyframes.size()) << " added " << int(added) << " notFirst " << notFirst;

	// bool centeradded = false;
	// if ((keyframes.size() < 2) && tooFewFeatures)
	// {
	// 	lastKeyOdom = imageOdom;
	// 	newKeyframe = new PatchEstimator(imageWidth,imageHeight,partitionRows,partitionCols,minDistance,minFeaturesDanger,minFeaturesBad,keyInd,0,0,
	// 																fx,fy,cx,cy,zmin,zmax,fq,fp,ft,fn,fd,fG,cameraName,tau,saveExp,
	// 																expName,patchSizeBase,checkSizeBase,pcb,qcb,numberFeaturesPerPartCol,numberFeaturesPerPartRow);
	// 	keyframes.push_back(newKeyframe);
	// 	centeradded = true;
	// 	keyInd++;
	// }


	// std::cout << " keyframes.size() " << int(keyframes.size()) << " centeradded " << int(centeradded) << "\n\n\n\n\n\n\n\n\n\n\n\n";

	// imageMutex.lock();
	// //clearn if messed up
	// for (int ii = 0; ii < keyframes.size(); ii++)
	// {
	// 	if (tooFewFeatures && (keyframes.at(0)->depthEstimators.size() < 10))
	// 	{
	// 		delete keyframes.at(0);
	// 		keyframes.erase(keyframes.begin());
	// 	}
	// }
	//
	// imageMutex.unlock();

	// if (addframe)
	// {
	// 	lastKeyOdom = imageOdom;
	// 	newKeyframe = new PatchEstimator(imageWidth,imageHeight,partitionSide,minDistance,minFeaturesDanger,minFeaturesBad,keyInd,0,0,
	// 																fx,fy,cx,cy,zmin,zmax,fq,fp,ft,fn,fd,fG,cameraName,tau,saveExp,
	// 																expName,patchSizeBase,checkSizeBase,pcb,qcb,numberFeaturesPerPartCol,numberFeaturesPerPartRow);
	// 	keyframes.push_back(newKeyframe);
	//
	// 	keyInd++;
	// }

	// ROS_WARN("key time %3.7f",float(clock()-keyTime)/CLOCKS_PER_SEC);

	// ROS_WARN("forward image reciever cb time %3.7f",float(clock()-processTime)/CLOCKS_PER_SEC);
}

// callback for getting camera intrinsic parameters
void ImageReceiver::camInfoCB(const sensor_msgs::CameraInfo::ConstPtr& camInfoMsg)
{
	//get camera info
	image_geometry::PinholeCameraModel cam_model;
	cam_model.fromCameraInfo(camInfoMsg);
	cv::Mat cam_calib_matTemp = cv::Mat(cam_model.fullIntrinsicMatrix());
	cam_calib_matTemp.convertTo(camMat,CV_32F);
	cam_model.distortionCoeffs().convertTo(distCoeffs,CV_32F);

	imageWidth = camInfoMsg->width;
	imageHeight = camInfoMsg->height;

	cv::Size imageSize(imageWidth,imageHeight);

	cv::initUndistortRectifyMap(camMat,distCoeffs,cv::Mat::eye(3,3,CV_32F),camMat,imageSize,CV_16SC2,map1,map2);

	// //get the tl corner of the centered roi applied to the images to find features
	// // int image_roi_x_tl = image_width*(1.0-image_roi_percent)/2;
	// int image_roi_x_tl = 0;
	// int image_roi_y_tl = image_roi_percent*imageHeight;
	// // int image_roi_y_tl = 0;
	//
	// //break the roi into the correct number of partitions
	// int image_roi_x_size = imageWidth/partitionCols;
	// // int image_roi_y_size = image_roi_percent*imageHeight/partitionRows;
	// int image_roi_y_size = image_roi_percent*imageHeight/partitionRows;
	//
	// cv::Size image_roi_size(image_roi_x_size,image_roi_y_size);
	// cv::Mat onesMat = cv::Mat::ones(image_roi_size,CV_8UC1);
	//
	// masks.resize((partitionRows)*(partitionCols));
	// int maskInd = 0;
	// for (int ii = 0; ii < partitionRows; ii++)
	// {
	// 	for (int jj = 0; jj < partitionCols; jj++)
	// 	{
	// 		cv::Mat maskii = cv::Mat::zeros(imageSize,CV_8UC1);
	// 		cv::Rect rectii(cv::Point(image_roi_x_tl+image_roi_x_size*jj,image_roi_y_tl+image_roi_y_size*ii),image_roi_size);
	// 		std::cout << "\n rectii " << rectii << std::endl;
	// 		onesMat.copyTo(maskii(rectii));
	// 		masks.at(maskInd) = maskii.clone();
	// 		maskInd++;
	// 	}
	// }
	//
	// std::cout << "masks size " << masks.size() << std::endl;
	// std::cout << "masks.at(0) size " << masks.at(0).size() << std::endl;

	//unregister subscriber
	camInfoSub.shutdown();
	gotCamParam = true;
}
