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
	nhp.param<float>("fq", fq, 1.0);
	nhp.param<float>("fp", fp, 1.0);
	nhp.param<float>("ft", ft, 1.0);
	nhp.param<float>("fn", fn, 1.0);
	nhp.param<float>("fd", fd, 1.0);
	nhp.param<float>("zmin", zmin, 1.0);
	nhp.param<float>("zmax", zmax, 1.0);
	nhp.param<int>("numberFeaturesToFindPerPart", numberFeaturesToFindPerPart, 100);
	nhp.param<int>("minFeaturesDanger", minFeaturesDanger, 50);
	nhp.param<int>("minFeaturesBad", minFeaturesBad, 25);

	nhp.param<int>("partitionRows", partitionRows, 2);
	nhp.param<int>("partitionCols", partitionCols, 2);

	nhp.param<float>("image_roi_percent", image_roi_percent, 1.0);

	float tau;
	nhp.param<float>("tau", tau, 1.0);

	nhp.param<bool>("saveExp", saveExp, false);
	nhp.param<std::string>("expName", expName, "exp");

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
	undistortPub = it.advertise(cameraName+"/image_undistort",1);

	// Subscribers
	imageSub = it.subscribe(cameraName+"/image_raw", 60, &ImageReceiver::imageCB,this);
	keyframeSub = it.subscribe(cameraName+"/image_undistort", 60, &ImageReceiver::keyframeCB,this);
	odomSub = nh.subscribe(cameraName+"/odom", 5, &ImageReceiver::odomCB,this);
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
			for (int ii = 0; ii <= minTimeInd; ii++)
			{
				odomSync.pop_front();
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
	bool addframe = false;

	//if an active keyframe then check if moved or rotated away a certain amount, if so add another key frame
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
		std::vector<Keyframe*> keyframesIn;
		for (int ii = 0; ii < keyframes.size(); ii++)
		{
			bool keepframe = false;

			if (keyframes.at(ii)->foundKeyImage)
			{
				if (!addframe && (keyframes.size() < 2) && keyframes.at(ii)->keyframeInDanger)
				{
					addframe = true;
				}

				if (!keyframes.at(ii)->keyframeShutdown)
				{
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
			}
			else
			{
				delete keyframes.at(ii);
			}
		}
		keyframes = keyframesIn;
		keyframesIn.clear();
	}
	else
	{
		addframe = true;
	}

	//ROS_WARN("match time %3.7f",float(clock()-processTime)/CLOCKS_PER_SEC);
	clock_t keyTime = clock();

	// add a new keyframe if needed
	if (addframe)
	{
		lastKeyOdom = imageOdom;
		newKeyframe = new Keyframe(keyInd, camMat, masks, imageWidth, imageHeight);
		keyframes.push_back(newKeyframe);

		keyInd++;
	}

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

	//get the tl corner of the centered roi applied to the images to find features
	// int image_roi_x_tl = image_width*(1.0-image_roi_percent)/2;
	int image_roi_x_tl = 0;
	int image_roi_y_tl = image_roi_percent*imageHeight;
	// int image_roi_y_tl = 0;

	//break the roi into the correct number of partitions
	int image_roi_x_size = imageWidth/partitionCols;
	// int image_roi_y_size = image_roi_percent*imageHeight/partitionRows;
	int image_roi_y_size = image_roi_percent*imageHeight/partitionRows;

	cv::Size image_roi_size(image_roi_x_size,image_roi_y_size);
	cv::Mat onesMat = cv::Mat::ones(image_roi_size,CV_8UC1);

	masks.resize((partitionRows)*(partitionCols));
	int maskInd = 0;
	for (int ii = 0; ii < partitionRows; ii++)
	{
		for (int jj = 0; jj < partitionCols; jj++)
		{
			cv::Mat maskii = cv::Mat::zeros(imageSize,CV_8UC1);
			cv::Rect rectii(cv::Point(image_roi_x_tl+image_roi_x_size*jj,image_roi_y_tl+image_roi_y_size*ii),image_roi_size);
			std::cout << "\n rectii " << rectii << std::endl;
			onesMat.copyTo(maskii(rectii));
			masks.at(maskInd) = maskii.clone();
			maskInd++;
		}
	}

	std::cout << "masks size " << masks.size() << std::endl;
	std::cout << "masks.at(0) size " << masks.at(0).size() << std::endl;

	//unregister subscriber
	camInfoSub.shutdown();
	gotCamParam = true;
}
