#include <patch_estimator.h>

PatchEstimator::~PatchEstimator()
{
	if (saveExp)
	{
		std::cout << "\nsaving\n";
		std::ofstream saveFile("/home/ncr/ncr_ws/src/icl_stationary/experiment/"+expName+".txt");
		if (saveFile.is_open())
		{
			std::cout << "\nopen\n";
			saveFile << "time,";
			saveFile << "pkcx," << "pkcy," << "pkcz,";
			saveFile << "qkcw," << "qkcx," << "qkcy," << "qkcz,";
			saveFile << "pkcICLx," << "pkcICLy," << "pkcICLz,";
			saveFile << "qkcICLw," << "qkcICLx," << "qkcICLy," << "qkcICLz,";
			saveFile << "pikx," << "piky," << "pikz,";
			saveFile << "pikICLx," << "pikICLy," << "pikICLz,";
			saveFile << "picx," << "picy," << "picz,";
			saveFile << "picICLx," << "picICLy," << "picICLz,";
			saveFile << "picEKFx," << "picEKFy," << "picEKFz,";
			saveFile << "picLSx," << "picLSy," << "picLSz,";
			saveFile << "picICLExtx," << "picICLExty," << "picICLExtz,";
			saveFile << "vx," << "vy," << "vz,";
			saveFile << "wx," << "wy," << "wz,";
			saveFile << "\n";

			for (int jj = 0; jj < data.size(); jj++)
			{
				float timej = data.at(jj)->time;
				Eigen::Vector3f pkcj = data.at(jj)->pkc;
				Eigen::Vector4f qkcj = data.at(jj)->qkc;
				Eigen::Vector3f pkcICLj = data.at(jj)->pkcICL;
				Eigen::Vector4f qkcICLj = data.at(jj)->qkcICL;
				Eigen::Vector3f pikj = data.at(jj)->pik;
				Eigen::Vector3f pikICLj = data.at(jj)->pikICL;
				Eigen::Vector3f picj = data.at(jj)->pic;
				Eigen::Vector3f picICLj = data.at(jj)->picICL;
				Eigen::Vector3f picEKFj = data.at(jj)->picEKF;
				Eigen::Vector3f picLSj = data.at(jj)->picLS;
				Eigen::Vector3f picICLExtj = data.at(jj)->picICLExt;
				Eigen::Vector3f vj = data.at(jj)->v;
				Eigen::Vector3f wj = data.at(jj)->w;

				std::cout << "\n picICLExtj \n" << picICLExtj << std::endl;

				saveFile << timej << ",";
				saveFile << pkcj(0) << "," << pkcj(1) << "," << pkcj(2) << ",";
				saveFile << qkcj(0) << "," << qkcj(1) << "," << qkcj(2) << "," << qkcj(3) << ",";
				saveFile << pkcICLj(0) << "," << pkcICLj(1) << "," << pkcICLj(2) << ",";
				saveFile << qkcICLj(0) << "," << qkcICLj(1) << "," << qkcICLj(2) << "," << qkcICLj(3) << ",";
				saveFile << pikj(0) << "," << pikj(1) << "," << pikj(2) << ",";
				saveFile << pikICLj(0) << "," << pikICLj(1) << "," << pikICLj(2) << ",";
				saveFile << picj(0) << "," << picj(1) << "," << picj(2) << ",";
				saveFile << picICLj(0) << "," << picICLj(1) << "," << picICLj(2) << ",";
				saveFile << picEKFj(0) << "," << picEKFj(1) << "," << picEKFj(2) << ",";
				saveFile << picLSj(0) << "," << picLSj(1) << "," << picLSj(2) << ",";
				saveFile << picICLExtj(0) << "," << picICLExtj(1) << "," << picICLExtj(2) << ",";
				saveFile << vj(0) << "," << vj(1) << "," << vj(2) << ",";
				saveFile << wj(0) << "," << wj(1) << "," << wj(2) << ",";
				saveFile << "\n";

				delete data.at(jj);
			}
			saveFile.close();
			std::cout << "\nclose\n";
		}
		std::cout << "\nsaved\n";
	}
}

PatchEstimator::PatchEstimator() : it(nh)
{}

PatchEstimator::PatchEstimator(int imageWidthInit, int imageHeightInit, int minFeaturesDangerInit, int minFeaturesBadInit, int keyIndInit,
															 int patchIndInit, cv::Mat& image, nav_msgs::Odometry imageOdom, std::vector<cv::Point2f> pts, float fxInit,
															 float fyInit, float cxInit, float cyInit, float zminInit, float zmaxInit, ros::Time t, float fq, float fp,
															 float ft, float fn, float fd, std::string cameraNameInit, float tauInit, bool saveExpInit, std::string expNameInit,
														   int patchSizeBaseInit,int checkSizeBaseInit) : it(nh)
{
	imageWidth = imageWidthInit;
	imageHeight = imageHeightInit;
	minFeaturesDanger = minFeaturesDangerInit;
	minFeaturesBad = minFeaturesBadInit;
	keyInd = keyIndInit;
	patchInd = patchIndInit;
	patchSizeBase = patchSizeBaseInit;
	checkSizeBase = checkSizeBaseInit;

	patchShutdown = false;
	firstOdomImageCB = true;
	tkcHat = Eigen::Vector3f::Zero();
	nkHat = Eigen::Vector3f(0.0,0.0,1.0);
	qkcHat = Eigen::Vector4f(1.0,0.0,0.0,0.0);
	pkcHat = Eigen::Vector3f::Zero();
	pckHat = Eigen::Vector3f::Zero();
 	qckHat = Eigen::Vector4f(1.0,0.0,0.0,0.0);
	fx = fxInit;
	fy = fyInit;
	cx = cxInit;
	cy = cyInit;
	camMat = cv::Mat::zeros(3,3,CV_32F);
	camMat.at<float>(0,0) = fx;
	camMat.at<float>(0,2) = cx;
	camMat.at<float>(1,1) = fy;
	camMat.at<float>(1,2) = cy;
	camMat.at<float>(2,2) = 1.0;

	camMatf = Eigen::Matrix3f::Zero();
	camMatf(0,0) = fx;
	camMatf(0,2) = cx;
	camMatf(1,1) = fy;
	camMatf(1,2) = cy;
	camMatf(2,2) = 1.0;

	Eigen::JacobiSVD<Eigen::MatrixXf> svdcamMatf(camMatf, Eigen::ComputeThinU | Eigen::ComputeThinV);
	camMatIf = svdcamMatf.solve(Eigen::Matrix3f::Identity());

	// GfLast = Eigen::Matrix<float,2,3>::Zero();
	// GfLast.block(0,0,2,2) = Eigen::Matrix2f::Identity();
	// GkfLast = Eigen::Matrix<float,2,3>::Zero();
	// GkfLast.block(0,0,2,2) = Eigen::Matrix2f::Identity();
	GfLast = Eigen::Matrix3f::Identity();
	GkfLast = Eigen::Matrix3f::Identity();
	// TfLast.block(0,0,2,2) = Eigen::Matrix2f::Identity();
	// TkfLast = Eigen::Matrix<float,2,3>::Zero();
	// TkfLast.block(0,0,2,2) = Eigen::Matrix2f::Identity();

	zmin = zminInit;
	zmax = zmaxInit;
	dkcHat = zmin;
	dkHat = zmax;
	tau = tauInit;

	pTau = 1.0/(2.0*M_PI*fp);
	qTau = 1.0/(2.0*M_PI*fq);
	tTau = 1.0/(2.0*M_PI*ft);
	nTau = 1.0/(2.0*M_PI*fn);
	dTau = 1.0/(2.0*M_PI*fd);

	keyOdom = imageOdom;
	kimage = image.clone();
	pimage = image.clone();
	tLast = t;
	tStart = t;
	odomSync.push_back(imageOdom);

	std::cout << "\n patch \n";
	for (int ii = 0; ii < pts.size(); ii++)
	{
		newDepthEstimator = new DepthEstimator(ii,Eigen::Vector3f((pts.at(ii).x-cx)/fx,(pts.at(ii).y-cy)/fy,1.0),tLast,zmin,zmax,(zmin+zmax)/2.0,tau,fx,fy,cx,cy);
		depthEstimators.push_back(newDepthEstimator);
		std::cout << ii << " ptix " << pts.at(ii).x << " ptiy " << pts.at(ii).y << std::endl;
	}

	cameraName = cameraNameInit;

	saveExp = saveExpInit;
	expName = expNameInit;

	// imagePub = it.advertise(cameraName+"/tracking_key"+std::to_string(keyInd)+"_patch"+std::to_string(patchInd),1);
	// imagePub = it.advertise(cameraName+"/test_image",1);
	imagePub2 = it.advertise(cameraName+"/test_image2",1);
	imageSub = it.subscribe(cameraName+"/image_undistort", 100, &PatchEstimator::imageCB,this);
	odomSub = nh.subscribe(cameraName+"/odom", 100, &PatchEstimator::odomCB,this);
	// odomPub = nh.advertise<nav_msgs::Odometry>(cameraName+"/odomHat",1);
	// odomDelayedPub = nh.advertise<nav_msgs::Odometry>(cameraName+"/odomDelayed", 1);
	// pointCloudPub = nh.advertise<PointCloud> ("wall_map", 1);
}

void PatchEstimator::odomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
	if (patchShutdown)
	{
		return;
	}
	std::lock_guard<std::mutex> odomMutexGuard(odomMutex);
	odomSync.push_back(*msg);
}

//image callback
void PatchEstimator::imageCB(const sensor_msgs::Image::ConstPtr& msg)
{
	ROS_WARN("IMAGE START");
	if (patchShutdown)
	{
		return;
	}

	clock_t callbackTime = clock();
	clock_t processTime = clock();
	std::cout << std::endl;

	// sync the times to find the closest
	ros::Time t = msg->header.stamp;

	{
		std::lock_guard<std::mutex> odomMutexGuard(odomMutex);
		std::vector<float> timeDiff;
		int numMatch = odomSync.size();

		// std::cout << "\n t key " << t << std::endl;

		std::cout << "\n numMatch " << numMatch << std::endl;
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

			std::cout << "\n odomSync size " << odomSync.size() << std::endl;
			if ((odomSync.size() > 1) && (minTimeInd < odomSync.size()-1))
			{
				for (int ii = 0; ii <= minTimeInd; ii++)
				{
					odomSync.pop_front();
				}
			}

			std::cout << "\n odomSync size " << odomSync.size() << std::endl;
			if (firstOdomImageCB)
			{
				firstOdomImageCB = false;
			}
		}
		else
		{
			ROS_ERROR("NO NEW IMAGE ODOM");
			if (firstOdomImageCB)
			{
				return;
			}
		}
	}

	Eigen::Vector3f pcw(imageOdom.pose.pose.position.x,imageOdom.pose.pose.position.y,imageOdom.pose.pose.position.z);
	Eigen::Vector4f qcw(imageOdom.pose.pose.orientation.w,imageOdom.pose.pose.orientation.x,imageOdom.pose.pose.orientation.y,imageOdom.pose.pose.orientation.z);

	// std::cout << "\n fabsf(imageOdom.pose.pose.position.y) " << fabsf(imageOdom.pose.pose.position.y) << std::endl;
	// convert to opencv image
	cv_bridge::CvImagePtr cv_ptr;
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

	// ROS_WARN("get image time %2.4f",float(clock()-processTime)/CLOCKS_PER_SEC);
	processTime = clock();

	// ROS_WARN("get checkerboard time %2.4f",float(clock()-processTime)/CLOCKS_PER_SEC);
	// processTime = clock();

	// float dt = std::min((t-tLast).toSec(),1.0/30.0);
	float dt = (t-tLast).toSec();
	float timeFromStart = (t-tStart).toSec();
	tLast = t;

	Eigen::Vector3f vc(imageOdom.twist.twist.linear.x,imageOdom.twist.twist.linear.y,imageOdom.twist.twist.linear.z);
	Eigen::Vector3f wc(imageOdom.twist.twist.angular.x,imageOdom.twist.twist.angular.y,imageOdom.twist.twist.angular.z);

	std::cout << "\n timeFromStart " << timeFromStart << std::endl;
	std::cout << "\n dt " << dt << std::endl;

	// find the features
	match(image,dt,vc,wc,t);

	// pckHat += (rotatevec(vc,qckHat)*dt);
	// qckHat += (0.5*B(qckHat)*wc*dt);
	// qckHat /= qckHat.norm();
	// pkcHat = rotatevec(-pckHat,getqInv(qckHat));
	// qkcHat = getqInv(qckHat);
	// qkcHat /= qkcHat.norm();
	pkcHat += ((-vc-getss(wc)*pkcHat)*dt);
	qkcHat += (-0.5*B(qkcHat)*wc*dt);
	qkcHat /= qkcHat.norm();
	pckHat = rotatevec(-pkcHat,getqInv(qkcHat));
	qckHat = getqInv(qkcHat);
	qckHat /= qckHat.norm();

	// std::cout << "\n dt \n" << dt << std::endl;
	// std::cout << "\n vc \n" << vc << std::endl;
	// std::cout << "\n wc \n" << wc << std::endl;
	// std::cout << "\n pkcHat \n" << pkcHat << std::endl;
	// std::cout << "\n qkcHat \n" << qkcHat << std::endl;

	// ros::shutdown();

	ROS_WARN("get match time %2.4f",float(clock()-processTime)/CLOCKS_PER_SEC);
	processTime = clock();

	pimage = image.clone();

	//send the pose estimate to the odom
	// Eigen::Vector4f qckHat = getqInv(qkcHat);
	// qckHat /= qckHat.norm();
	// Eigen::Vector3f pckHat = rotatevec(-pkcHat,qckHat);

	// std::cout << "\n pckHat \n" << pckHat << std::endl;
	// std::cout << "\n qckHat \n" << qckHat << std::endl;

	Eigen::Vector3f pkw(keyOdom.pose.pose.position.x,keyOdom.pose.pose.position.y,keyOdom.pose.pose.position.z);
	Eigen::Vector4f qkw(keyOdom.pose.pose.orientation.w,keyOdom.pose.pose.orientation.x,keyOdom.pose.pose.orientation.y,keyOdom.pose.pose.orientation.z);

	Eigen::Vector4f qcwHat = getqMat(qkw)*qckHat;
	qcwHat /= qcwHat.norm();

	Eigen::Vector3f pcwHat = rotatevec(pckHat,qkw)+pkw;

	Eigen::Vector3f pkc = rotatevec(pkw-pcw,getqInv(qcw));
	Eigen::Vector4f qkc = getqMat(getqInv(qcw))*qkw;
	qkc /= qkc.norm();

	// std::cout << "\n pkc \n" << pkc << std::endl;
	// std::cout << "\n qkc \n" << qkc << std::endl;

	// build and publish odom message
	nav_msgs::Odometry odomMsg;
	odomMsg.header.stamp = t;
	odomMsg.header.frame_id = "world";
	odomMsg.child_frame_id = "cameraHat";
	odomMsg.pose.pose.position.x = pcwHat(0);
	odomMsg.pose.pose.position.y = pcwHat(1);
	odomMsg.pose.pose.position.z = pcwHat(2);
	odomMsg.pose.pose.orientation.w = qcwHat(0);
	odomMsg.pose.pose.orientation.x = qcwHat(1);
	odomMsg.pose.pose.orientation.y = qcwHat(2);
	odomMsg.pose.pose.orientation.z = qcwHat(3);
	odomMsg.twist.twist.linear.x = vc(0);
	odomMsg.twist.twist.linear.y = vc(1);
	odomMsg.twist.twist.linear.z = vc(2);
	odomMsg.twist.twist.angular.x = wc(0);
	odomMsg.twist.twist.angular.y = wc(1);
	odomMsg.twist.twist.angular.z = wc(2);
	// odomPub.publish(odomMsg);

	nav_msgs::Odometry odomDelayedMsg;
	odomDelayedMsg.header.stamp = t;
	odomDelayedMsg.header.frame_id = "world";
	odomDelayedMsg.child_frame_id = "camera";
	odomDelayedMsg.pose.pose.position.x = pcw(0);
	odomDelayedMsg.pose.pose.position.y = pcw(1);
	odomDelayedMsg.pose.pose.position.z = pcw(2);
	odomDelayedMsg.pose.pose.orientation.w = qcw(0);
	odomDelayedMsg.pose.pose.orientation.x = qcw(1);
	odomDelayedMsg.pose.pose.orientation.y = qcw(2);
	odomDelayedMsg.pose.pose.orientation.z = qcw(3);
	odomDelayedMsg.twist.twist.linear.x = vc(0);
	odomDelayedMsg.twist.twist.linear.y = vc(1);
	odomDelayedMsg.twist.twist.linear.z = vc(2);
	odomDelayedMsg.twist.twist.angular.x = wc(0);
	odomDelayedMsg.twist.twist.angular.y = wc(1);
	odomDelayedMsg.twist.twist.angular.z = wc(2);
	// odomDelayedPub.publish(odomDelayedMsg);

	// if (depthEstimators.size() > 0)
	// {
	// 	//plot the points
	// 	// get all the points and draw them on the image
	// 	// std::cout << "\n wall 1 \n";
	// 	PointCloudRGB::Ptr map(new PointCloudRGB);
	// 	pcl_conversions::toPCL(msg->header.stamp,map->header.stamp);
	//
	// 	// std::cout << "\n wall 1 1 \n";
	// 	map->header.frame_id = "world";
	//
	// 	// std::cout << "\n wall 1 2 \n";
	// 	map->height = 1;
	// 	map->is_dense = true;
	// 	map->points.clear();
	//
	// 	//publish the image
	// 	pcl::PointXYZRGB pt,ptHat;
	// 	for (uint16_t ii = 0; ii < depthEstimators.size(); ii++)
	// 	{
	// 		//get the points after update
	// 		Eigen::Vector3f mci = depthEstimators.at(ii)->mc;
	// 		Eigen::Vector3f uci = mci/mci.norm();
	// 		cv::Point2f cPti(fx*mci(0)+cx,fy*mci(1)+cy);
	// 		cv::circle(image, cPti, 10, cv::Scalar(150, 150, 150), -1);
	//
	// 		// Eigen::Vector3f mkiHat = depthEstimators.at(ii)->mk;
	// 		// cv::Point2i kPti(int(fx*mkiHat(0)+cx),int(fy*mkiHat(1)+cy));
	// 		// uint8_t colori = kimage.at<uint8_t>(kPti.y,kPti.x);
	//
	// 		Eigen::Vector3f pciHatICLExt = uci*depthEstimators.at(ii)->dcHatICLExt;
	// 		Eigen::Vector3f pciHatEKF = mci*depthEstimators.at(ii)->zcHatEKF;
	//
	// 		// std::cout << "\n pciHatICLExt \n" << pciHatICLExt << std::endl;
	//
	// 		// if (saveExp)
	// 		// {
	// 		// 	// DataSave* dataSave = new DataSave(timeFromStart,pkc,qkc,pkcHat,qkcHat,pki,pkiHat,pics.at(ii),pciHat,pciHatEKF,pciHatLS,vc,wc);
	// 		// 	DataSave* dataSave = new DataSave(timeFromStart,pkc,qkc,pkcHat,qkcHat,pki,pkiHatICL,pics.at(ii),pciHatICL,pciHatEKF,pciHatLS,pciHatICLExt,vc,wc);
	// 		// 	data.push_back(dataSave);
	// 		// }
	//
	// 		// //bring into world frame
	// 		// pciHatICLExt = pcwHat + rotatevec(pciHatICLExt,qcwHat);
	// 		// pciHatEKF = pcwHat + rotatevec(pciHatEKF,qcwHat);
	//
	// 		ptHat.x = pciHatICLExt(0);
	// 		ptHat.y = pciHatICLExt(1);
	// 		ptHat.z = pciHatICLExt(2);
	// 		ptHat.r = 0;
	// 		ptHat.g = 200;
	// 		ptHat.b = 0;
	// 		map->points.push_back(ptHat);
	//
	// 		ptHat.x = pciHatEKF(0);
	// 		ptHat.y = pciHatEKF(1);
	// 		ptHat.z = pciHatEKF(2);
	// 		ptHat.r = 0;
	// 		ptHat.g = 0;
	// 		ptHat.b = 255;
	// 		map->points.push_back(ptHat);
	//
	// 		// std::cout << "\n index " << ii << std::endl;
	// 		// std::cout << "\n dkHat " << depthEstimators.at(ii)->dkHat << std::endl;
	// 		// std::cout << "\n dcHat " << depthEstimators.at(ii)->dcHat << std::endl;
	// 		// std::cout << "\n zcHatICLExt " << pciHatICLExt(2) << std::endl;
	// 		// std::cout << "\n zcHatEKF " << pciHatEKF(2) << std::endl;
	// 		// std::cout << "\n dk " << depthEstimators.at(ii)->pik.norm() << std::endl;
	//
	// 		if((fabsf(pciHatICLExt(2)) > 100.0) || std::isnan(pciHatICLExt(2)))
	// 		{
	// 			ros::shutdown();
	// 		}
	// 	}
	// 	map->width = map->points.size();
	// 	pointCloudPub.publish(map);
	// }

	ROS_WARN("get circle time %2.4f",float(clock()-processTime)/CLOCKS_PER_SEC);
	processTime = clock();

	// // publish key image
	// cv_bridge::CvImage out_msg;
	// out_msg.header = msg->header; // Same timestamp and tf frame as input image
	// out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
	// out_msg.image = image; // Your cv::Mat

	// {
		// std::lock_guard<std::mutex> pubMutexGuard(pubMutex);
		// imagePub.publish(out_msg.toImageMsg());
	// }

	// ROS_WARN("cb time %2.4f",float(clock()-callbackTime)/CLOCKS_PER_SEC);

	if (depthEstimators.size() <= minFeaturesBad)
	{
		for (std::vector<DepthEstimator*>::iterator it = depthEstimators.begin() ; it != depthEstimators.end(); it++)
		{
			delete *it;
		}
		depthEstimators.clear();
		// imagePub.shutdown();
		imagePub2.shutdown();
		imageSub.shutdown();
		odomSub.shutdown();
		// odomPub.shutdown();
		// odomDelayedPub.shutdown();
		// pointCloudPub.shutdown();
		patchShutdown = true;

		ROS_WARN("shutdown after imagesub");
	}

	ROS_WARN("IMAGE STOP cb time %2.4f",float(clock()-callbackTime)/CLOCKS_PER_SEC);
}

//finds the features in the previous image in the new image and matches the features
void PatchEstimator::match(cv::Mat& image, float dt, Eigen::Vector3f vc, Eigen::Vector3f wc, ros::Time t)
{
	std::lock_guard<std::mutex> featureMutexGuard(featureMutex);

	clock_t estimatorUpdateTime = clock();
	// ROS_WARN("depthEstimators size before predict %d",int(depthEstimators.size()));

	clock_t timeCheck = clock();

	//get the points from the previous image
	std::vector<cv::Point2f> pPts(depthEstimators.size()),cPts(depthEstimators.size()),kPts(depthEstimators.size());
	std::vector<cv::Point2f> kPtsInPred(depthEstimators.size()),cPtsInPred(depthEstimators.size());//,cPtPsInPred(depthEstimators.size());
	Eigen::Vector3f mcc;
	std::vector<cv::Point2f>::iterator itp = pPts.begin();
	std::vector<cv::Point2f>::iterator itc = cPts.begin();
	std::vector<cv::Point2f>::iterator itk = kPts.begin();
	cv::Mat GI;
	for (std::vector<DepthEstimator*>::iterator it = depthEstimators.begin() ; it != depthEstimators.end(); it++)
	{
		*itk = cv::Point2f((*it)->ptk(0),(*it)->ptk(1));
	  *itp = cv::Point2f(fx*((*it)->mc(0))+cx,fy*((*it)->mc(1))+cy);
	  mcc = (*it)->predict(vc,wc,dt);
	  *itc = cv::Point2f(fx*mcc(0)+cx,fy*mcc(1)+cy);
		itk++;
		itp++;
		itc++;
	}

	try
	{
		float kT = 0.03/(1.0/(2.0*M_PI*2.0)+ 0.03);
		cv::Mat inliersAffine;
		cv::Mat G = cv::findHomography(pPts, cPts, cv::RANSAC, 3.0, inliersAffine, 2000, 0.99);//calculate homography using RANSAC

		// Eigen::Matrix<float,3,3> Gkf,Gf;
		Eigen::Matrix<float,3,3> Gf;
		for (int ii = 0; ii < 9; ii++)
		{
			// Gkf(ii/3,ii%3) = Gk.at<double>(ii/3,ii%3);
			Gf(ii/3,ii%3) = G.at<double>(ii/3,ii%3);
		}
		//
		// if (!Gk.empty())
		// {
		// 	GkfLast += kT*(Gkf - GkfLast);
		// }

		if (!G.empty())
		{
			GfLast += kT*(Gf - GfLast);
		}
		GfLast *= (1.0/GfLast(2,2));

		for (int ii = 0; ii < 9; ii++)
		{
			// Gk.at<double>(ii/3,ii%3) = GkfLast(ii/3,ii%3);
			G.at<double>(ii/3,ii%3) = GfLast(ii/3,ii%3);
		}

		// cv::invert(G,GI,cv::DECOMP_SVD);

		std::cout << "\n Gp \n" << G << std::endl << std::endl;
		// std::cout << "\n Gk \n" << Gk << std::endl << std::endl;

		//correct points and remove the outliers
		int Gshift = std::round(fabsf(GfLast(0,2))+fabsf(GfLast(1,2)));
		std::vector<DepthEstimator*> depthEstimatorsInPred(depthEstimators.size());
		std::vector<DepthEstimator*>::iterator itDP = depthEstimators.begin();
		std::vector<DepthEstimator*>::iterator itDPPred = depthEstimatorsInPred.begin();
		std::vector<cv::Point2f>::iterator itcPred = cPtsInPred.begin();
		std::vector<cv::Point2f>::iterator itkPred = kPtsInPred.begin();
		assert(depthEstimators.size() > 0);
		// cv::Mat drawImage = image.clone();
		int patchSize = patchSizeBase+2*Gshift;
		int checkSize = checkSizeBase+2*Gshift;
		int resultSize = checkSize - patchSize + 1;
		int blurSize = 3;
		cv::Mat ppatch(patchSize,patchSize,CV_8UC1);
		cv::Mat cpatch(checkSize,checkSize,CV_8UC1);
		cv::Mat tmresult(resultSize,resultSize,CV_32F);
		cv::Mat tmresultBlur(resultSize,resultSize,CV_32F);
		double minResultVal, maxResultVal;
		cv::Point minResultPt, maxResultPt;
		int numPtsPred = 0;
		cv::Rect ppatchRectP,ppatchRectC,ppatchRectCLocal,checkRect;
		std::vector<cv::Point2f> ppatchPtsP(3),ppatchPtsC(3);
		ppatchPtsP.at(1) = cv::Point2f(patchSize,patchSize);//width and height
		ppatchPtsP.at(2) = cv::Point2f((patchSize-1.0)/2.0,(patchSize-1.0)/2.0);//width and height
		cv::Mat ppatchWarp(patchSize,patchSize,CV_8UC1);
		cv::Mat reducedWarp;

		// ROS_WARN("check 2 %2.5f",float(clock()-timeCheck)/CLOCKS_PER_SEC);
		timeCheck = clock();
		for (std::vector<cv::Point2f>::iterator itpp = pPts.begin(); itpp != pPts.end(); itpp++)
		{

			//get the  center, width, and height that will be searched and transform into new image
			ppatchPtsP.at(0) = cv::Point2f((*itpp).x,(*itpp).y);//center
			cv::perspectiveTransform(ppatchPtsP,ppatchPtsC,G);

			if (!G.empty() && (acos(qkcHat(0))*2.0 < 45.0*3.1415/180.0)
			 							 && ((ppatchPtsC.at(0).x-checkSize) > 0) && ((ppatchPtsC.at(0).x+checkSize) < imageWidth)
										 && ((ppatchPtsC.at(0).y-checkSize) > 0) && ((ppatchPtsC.at(0).y+checkSize) < imageHeight))
			//
			// if (!G.empty() && (acos(qkcHat(0))*2.0 < 45.0*3.1415/180.0)
			// 							 && (((*itpp).x-checkSize-Gshift) > 0) && (((*itpp).x+checkSize+Gshift) < imageWidth)
		  //                && (((*itpp).y-checkSize-Gshift) > 0) && (((*itpp).y+checkSize+Gshift) < imageHeight))
			{

				//get the patch to find and transform into new image
				ppatchRectP = cv::Rect(std::round((*itpp).x)-(patchSize-1)/2,std::round((*itpp).y)-(patchSize-1)/2,patchSize,patchSize);
				ppatch = pimage(ppatchRectP).clone();//patch

				//transform the patch
				cv::warpPerspective(ppatch.clone(),ppatchWarp,G,ppatch.size(),cv::INTER_LINEAR,cv::BORDER_CONSTANT,cv::Scalar(255,255,255));

				cv::Point2f patchCenterC(ppatchPtsC.at(0));
				cv::Point2f patchCenterCLocal(ppatchPtsC.at(2));
				float patchSizeC = std::min(ppatchPtsC.at(1).x,ppatchPtsC.at(1).y)-2*Gshift;

				ppatchRectCLocal.x = std::round(patchCenterCLocal.x-(patchSizeC-1)/2);
				ppatchRectCLocal.y = std::round(patchCenterCLocal.y-(patchSizeC-1)/2);
				ppatchRectCLocal.width = std::round(patchSizeC);
				ppatchRectCLocal.height = std::round(patchSizeC);
				reducedWarp = ppatchWarp(ppatchRectCLocal).clone();

				checkRect = cv::Rect(std::round(patchCenterC.x-(checkSize-1)/2.0),std::round(patchCenterC.y-(checkSize-1)/2.0),checkSize,checkSize);
				cpatch = image(checkRect).clone();
				// cv::matchTemplate(cpatch,ppatch,tmresult,cv::TM_SQDIFF_NORMED);
				// cv::matchTemplate(cpatch,reducedWarp,tmresult,cv::TM_CCORR_NORMED);
				cv::matchTemplate(cpatch,reducedWarp,tmresult,cv::TM_CCOEFF_NORMED);
				cv::GaussianBlur(tmresult,tmresultBlur,cv::Size(blurSize,blurSize),0);
				cv::minMaxLoc(tmresultBlur,&minResultVal,&maxResultVal,&minResultPt,&maxResultPt);

				//get the points
				*itDPPred = *itDP;
				*itcPred = cv::Point2f(checkRect.x+(ppatchRectCLocal.width-1)/2.0+maxResultPt.x,checkRect.y+(ppatchRectCLocal.height-1)/2.0+maxResultPt.y);
				*itkPred = cv::Point2f((*itDP)->ptk(0),(*itDP)->ptk(1));

				itDPPred++;
				itcPred++;
				itkPred++;
				numPtsPred++;
			}
			else
			{
				delete *itDP;
				if (G.empty())
				{
					std::cout << "\n G empty \n";
				}
			}
			itDP++;
		}
		depthEstimatorsInPred.resize(numPtsPred);
		cPtsInPred.resize(numPtsPred);
		kPtsInPred.resize(numPtsPred);
		depthEstimators = depthEstimatorsInPred;
		depthEstimatorsInPred.clear();
		pPts.clear();
		cPts.clear();
		kPts.clear();
	}
	catch (cv::Exception e)
	{
		ROS_ERROR("update failed");
	}


	cv::Mat Gk;
	std::vector<cv::Point2f> kcPtsInPred(depthEstimators.size()),kkPtsInPred(depthEstimators.size());
	try
	{
		float kT = 0.03/(1.0/(2.0*M_PI*2.0)+ 0.03);
		//correct points again using key and remove the outliers
		cv::Mat inliersAffinek;
		// Gk = cv::findHomography(kPtsInPred, cPtsInPred, cv::RANSAC, 3.0, inliersAffinek, 2000, 0.99);//calculate homography using RANSAC
		Gk = cv::findHomography(kPtsInPred, cPtsInPred, 0);//calculate homography using RANSAC
		Eigen::Matrix<float,3,3> Gkf;
		for (int ii = 0; ii < 9; ii++)
		{
			Gkf(ii/3,ii%3) = Gk.at<double>(ii/3,ii%3);
		}

		if (!Gk.empty())
		{
			GkfLast += kT*(Gkf - GkfLast);
		}
		GkfLast *= (1.0/GkfLast(2,2));

		for (int ii = 0; ii < 9; ii++)
		{
			Gk.at<double>(ii/3,ii%3) = GkfLast(ii/3,ii%3);
		}

		std::cout << "\n Gk \n" << Gk << std::endl << std::endl;
		int kGshift = std::round(fabsf(GkfLast(0,2))+fabsf(GkfLast(1,2)));

		std::vector<DepthEstimator*> kdepthEstimatorsInPred(depthEstimators.size());
		std::vector<DepthEstimator*>::iterator kitDP = depthEstimators.begin();
		std::vector<DepthEstimator*>::iterator kitDPPred = kdepthEstimatorsInPred.begin();
		std::vector<cv::Point2f>::iterator kitcPred = kcPtsInPred.begin();
		std::vector<cv::Point2f>::iterator kitkPred = kkPtsInPred.begin();
		// int kpatchSize = patchSizeBase+2*kGshift;
		// int kcheckSize = checkSizeBase+2*kGshift;
		int kpatchSize = patchSizeBase;
		int kcheckSize = checkSizeBase;
		std::cout << "\n kpatchSize " << kpatchSize << " kcheckSize " << kcheckSize << std::endl << std::endl;
		int kresultSize = kcheckSize - kpatchSize + 1;
		int blurSize = 3;
		cv::Mat kpatch(kpatchSize,kpatchSize,CV_8UC1);
		cv::Mat kcpatch(kcheckSize,kcheckSize,CV_8UC1);
		cv::Mat ktmresult(kresultSize,kresultSize,CV_32F);
		cv::Mat ktmresultBlur(kresultSize,kresultSize,CV_32F);
		double kminResultVal, kmaxResultVal;
		cv::Point kminResultPt, kmaxResultPt;
		int knumPtsPred = 0;
		cv::Rect kpatchRectP,kpatchRectC,kpatchRectCLocal,kcheckRect;
		std::vector<cv::Point2f> kpatchPtsP(3),kpatchPtsC(3);
		kpatchPtsP.at(1) = cv::Point2f(kpatchSize,kpatchSize);//width and height
		kpatchPtsP.at(2) = cv::Point2f((kpatchSize-1.0)/2.0,(kpatchSize-1.0)/2.0);//center width and height
		cv::Mat kpatchWarp(kpatchSize,kpatchSize,CV_8UC1);
		cv::Mat kreducedWarp;

		// ROS_WARN("check 2 %2.5f",float(clock()-timeCheck)/CLOCKS_PER_SEC);
		timeCheck = clock();
		for (std::vector<cv::Point2f>::iterator itkk = kPtsInPred.begin(); itkk != kPtsInPred.end(); itkk++)
		{
			//get the  center, width, and height that will be searched and transform into new image
			kpatchPtsP.at(0) = cv::Point2f((*itkk).x,(*itkk).y);//center in original
			cv::perspectiveTransform(kpatchPtsP,kpatchPtsC,Gk);
			cv::Point2f kpatchCenterC(kpatchPtsC.at(0));//center of patch in transformed image
			cv::Point2f kpatchCenterCLocal(kpatchPtsC.at(2));//transformed patch center in transformed patch
			int kpatchSizeC = std::round(std::min(kpatchPtsC.at(1).x,kpatchPtsC.at(1).y));//min of transformed patch width and height in transformed patch
			std::cout << "\n kpatchCenterCLocal.x " << kpatchCenterCLocal.x << " kpatchCenterCLocal.y " << kpatchCenterCLocal.y << std::endl;
			std::cout << "\n kpatchSizeC b " << kpatchSizeC << std::endl;
			std::cout << "\n reduce " << 2*(std::max(kpatchCenterCLocal.x,kpatchCenterCLocal.y)+1-(kpatchSizeC-1)/2) << std::endl;
			kpatchSizeC -= std::round(2*fabsf(std::max(kpatchCenterCLocal.x,kpatchCenterCLocal.y)+1-(kpatchSizeC-1)/2));//reduce patch size by adjusted center
			std::cout << "\n kpatchSizeC a " << kpatchSizeC << std::endl;

			if (!Gk.empty() && ((kpatchPtsC.at(0).x-kcheckSize) > 0) && ((kpatchPtsC.at(0).x+kcheckSize) < imageWidth)
										  && ((kpatchPtsC.at(0).y-kcheckSize) > 0) && ((kpatchPtsC.at(0).y+kcheckSize) < imageHeight))
			{

				//adjust size of patch in search

				kcheckRect = cv::Rect(std::round(kpatchCenterC.x-(kcheckSize-1)/2.0),std::round(kpatchCenterC.y-(kcheckSize-1)/2.0),kcheckSize,kcheckSize);
				std::cout << "\n knumPtsPred " << knumPtsPred << " kcheckRect " << kcheckRect;

				//get the patch to find and transform into new image
				kpatchRectP = cv::Rect(std::round((*itkk).x)-(kpatchSize-1)/2,std::round((*itkk).y)-(kpatchSize-1)/2,kpatchSize,kpatchSize);

				std::cout << " kpatchRectP " << kpatchRectP << std::endl << std::endl;
				std::cout << " kimage.size() " << kimage.size() << std::endl << std::endl;
				kpatch = kimage(kpatchRectP).clone();//patch
				std::cout << " kpatch.size() " << kpatch.size() << std::endl << std::endl;

				//transform the patch

				cv::warpPerspective(kpatch.clone(),kpatchWarp,Gk,kpatch.size(),cv::INTER_LINEAR,cv::BORDER_CONSTANT,cv::Scalar(255,255,255));
				// cv::warpPerspective(kpatch.clone(),kpatchWarp,Gk,cv::Size(kpatchPtsC.at(1).x,kpatchPtsC.at(1).y),cv::INTER_LINEAR,cv::BORDER_CONSTANT,cv::Scalar(255,255,255));
				std::cout << " kpatchWarp.size() " << kpatchWarp.size() << std::endl << std::endl;

				kpatchRectCLocal.x = fabsf(std::max(kpatchCenterCLocal.x,kpatchCenterCLocal.y)-(kpatchSizeC-1)/2);
				kpatchRectCLocal.y = fabsf(std::max(kpatchCenterCLocal.x,kpatchCenterCLocal.y)-(kpatchSizeC-1)/2);
				kpatchRectCLocal.width = kpatchSizeC;
				kpatchRectCLocal.height = kpatchSizeC;
				std::cout << " kpatchRectCLocal " << kpatchRectCLocal << std::endl << std::endl;

				kreducedWarp = kpatchWarp(kpatchRectCLocal).clone();
				std::cout << " kreducedWarp.size() " << kreducedWarp.size() << std::endl << std::endl;


				kcpatch = image(kcheckRect).clone();
				// cv::matchTemplate(cpatch,ppatch,tmresult,cv::TM_SQDIFF_NORMED);
				// cv::matchTemplate(cpatch,reducedWarp,tmresult,cv::TM_CCORR_NORMED);
				cv::matchTemplate(kcpatch,kreducedWarp,ktmresult,cv::TM_CCOEFF_NORMED);
				cv::GaussianBlur(ktmresult,ktmresultBlur,cv::Size(blurSize,blurSize),0);
				cv::minMaxLoc(ktmresultBlur,&kminResultVal,&kmaxResultVal,&kminResultPt,&kmaxResultPt);

				//get the points
				*kitcPred = cv::Point2f(kcheckRect.x+(kpatchRectCLocal.width-1)/2.0+kmaxResultPt.x,kcheckRect.y+(kpatchRectCLocal.height-1)/2.0+kmaxResultPt.y);
				*kitkPred = *itkk;
				*kitDPPred = *kitDP;

			 	kitcPred++;
			 	kitkPred++;;
				kitDPPred++;
				knumPtsPred++;
			}
			else
			{
				delete *kitDP;
				if (Gk.empty())
				{
					std::cout << "\n Gk empty \n";
				}
			}
			kitDP++;
		}
		kdepthEstimatorsInPred.resize(knumPtsPred);
		kcPtsInPred.resize(knumPtsPred);
		kkPtsInPred.resize(knumPtsPred);
		depthEstimators = kdepthEstimatorsInPred;
		kdepthEstimatorsInPred.clear();
		cPtsInPred.clear();
		kPtsInPred.clear();
	}
	catch (cv::Exception e)
	{
		ROS_ERROR("update k failed");
		ros::shutdown();
	}

	// ROS_WARN("keyframe %d patch %d pPts size before flow %d",keyInd,patchInd,int(pPts.size()));
	ROS_WARN("time for getting points %2.4f depthEstimators size after predict %d",float(clock()-estimatorUpdateTime)/CLOCKS_PER_SEC,int(depthEstimators.size()));
	estimatorUpdateTime = clock();

	// find the points using either approximated flow or looking for the board
  //use optical flow to find the features
	if (depthEstimators.size() > minFeaturesBad)
	{
		// cv::perspectiveTransform(cPtPsInPred,cPtsInPred,GI);
		estimatorUpdateTime = clock();
		//update the estimtators using the estimted points
		update(Gk,kkPtsInPred,kcPtsInPred,vc,wc,t,dt);
		ROS_WARN("time for estimator update call %2.4f",float(clock()-estimatorUpdateTime)/CLOCKS_PER_SEC);
		// estimatorUpdateTime = clock();
	}
	kcPtsInPred.clear();
	kkPtsInPred.clear();
	// cPtPsInPred.clear();
}

void PatchEstimator::update(cv::Mat& G, std::vector<cv::Point2f>& kPts, std::vector<cv::Point2f>& cPts, Eigen::Vector3f vc, Eigen::Vector3f wc, ros::Time t, float dt)
{
	// //predict the orientation and position
	// pkcHat += (-vc*dt);
	// qkcHat += (-0.5*B(qkcHat)*wc*dt);
	// qkcHat /= qkcHat.norm();


	// std::cout << "\n dt \n" << dt << std::endl;
	// std::cout << "\n vc \n" << vc << std::endl;
	// std::cout << "\n wc \n" << wc << std::endl;
	// std::cout << "\n pkcHat \n" << pkcHat << std::endl;
	// std::cout << "\n qkcHat \n" << qkcHat << std::endl;
	float kp = 0.03/(pTau + 0.03);
	float kq = 0.03/(qTau + 0.03);
	float kt = 0.03/(tTau + 0.03);
	float kn = 0.03/(nTau + 0.03 + 100.0*fabsf(qkcHat(2)));
	float kd = 0.03/(dTau + 0.03);

	// return;

	// add in if the the pattern is found and always searching hjaslfahdahdfhasdhasdas
	clock_t updateClock = clock();
	try
	{
		assert(kPts.size()>0);
		assert(cPts.size()>0);

		// cv::Mat inliersG;
		// cv::Mat G = cv::findHomography(kPts, cPts, cv::RANSAC, 3.0, inliersG, 2000, 0.99);//calculate homography using RANSAC
		// cv::Mat G = cv::findHomography(kPts, cPts, 0);//calculate homography using RANSAC

		std::cout << "\n G \n" << G << std::endl << std::endl;

		ROS_WARN("time for homog %2.4f",float(clock()-updateClock)/CLOCKS_PER_SEC);
		updateClock = clock();

		// estimate the homography
		Eigen::Vector4f qkc = qkcHat;
		Eigen::Vector3f nk = nkHat;
		Eigen::Vector3f tkc = tkcHat;

		if (!G.empty())
		{
			try
			{
				//find the solutions
				std::vector<cv::Mat> RkcH,tkcH,nkH;//holds the rotations, translations and normal vetors from decompose homography
				int numberHomogSols = cv::decomposeHomographyMat(G, camMat, RkcH, tkcH, nkH);// Decompose homography

				//check positive depth constraint on the inliers to find the solutions
				std::vector<Eigen::Vector3f> nks;
				std::vector<Eigen::Vector3f> tkcs;
				std::vector<Eigen::Vector4f> qkcs;
				std::vector<float> errors;

				// ROS_WARN("keyframe %d patch %d numberHomogSols %d",keyInd,patchInd,numberHomogSols);

				assert(numberHomogSols>0);
				assert(RkcH.size()>0);
				assert(tkcH.size()>0);
				assert(nkH.size()>0);

				for (int jj = 0; jj < numberHomogSols; jj++)
				{
					//convert normal to eigen
					Eigen::Vector3f nkj(nkH.at(jj).at<double>(0,0),nkH.at(jj).at<double>(1,0),nkH.at(jj).at<double>(2,0));

					// std::cout << "\n\n sol " << jj << std::endl;
					// std::cout << "\n nkjx " << nkj(0) << " nkjy " << nkj(1) << " nkjz " << nkj(2) << std::endl;

					if (nkj.norm() < 0.1)
					{
						nkj(0) = 0.0;
						nkj(1) = 0.0;
						nkj(2) = 1.0;
					}

					//if n^T*[0;0;1] > then solution in front of camera
					Eigen::Vector3f tkcj(tkcH.at(jj).at<double>(0,0),tkcH.at(jj).at<double>(1,0),tkcH.at(jj).at<double>(2,0));
					// std::cout << "\n tkcjx " << tkcj(0) << " tkcjy " << tkcj(1) << " tkcjz " << tkcj(2) << std::endl;
					if ((nkj(2) >= 0.0))
					{
						Eigen::Matrix3f Rkcj;
						for (int hh = 0; hh < 9; hh++)
						{
							Rkcj(hh/3,hh%3) = RkcH.at(jj).at<double>(hh/3,hh%3);
						}
						Eigen::Quaternionf qkcjq(Rkcj);// convert to quaternion
						Eigen::Vector4f qkcj(qkcjq.w(),qkcjq.x(),qkcjq.y(),qkcjq.z());
						qkcj /= qkcj.norm();

						if ((qkcHat + qkcj).norm() < (qkcHat - qkcj).norm())
						{
							qkcj *= -1.0;
						}

						// std::cout << "\n qkcjw " << qkcj(0) << " qkcjx " << qkcj(1) << " qkcjy " << qkcj(2) << " qkcjz " << qkcj(3) << std::endl;
						// std::cout << "\n tkcjx " << tkcj(0) << " tkcjy " << tkcj(1) << " tkcjz " << tkcj(2) << std::endl;
						// std::cout << "\n nkjx " << nkj(0) << " nkjy " << nkj(1) << " nkjz " << nkj(2) << std::endl;

						nks.push_back(nkj);
						tkcs.push_back(tkcj);
						qkcs.push_back(qkcj);
						if (tkcj.norm() > 0.001)
						{
							errors.push_back((qkcHat - qkcj).norm()+(pkcHat-tkcj).norm());
						}
						else
						{
							errors.push_back((qkcHat - qkcj).norm());
						}
					}
				}

				ROS_WARN("keyframe %d patch %d errors size %d",keyInd,patchInd,int(errors.size()));

				int minqkcsErrorInd = std::distance(errors.begin(),std::min_element(errors.begin(),errors.end()));
				qkc = qkcs.at(minqkcsErrorInd);
				nk = nks.at(minqkcsErrorInd);
				tkc = tkcs.at(minqkcsErrorInd);

				ROS_WARN("keyframe %d patch %d selected best",keyInd,patchInd);
			}
			catch (cv::Exception e)
			{
				ROS_ERROR("G failed");
			}
		}
		else
		{
			ROS_ERROR("G Empty");
		}

		// float kp = dt/(pTau + dt);
		// float kq = dt/(qTau + dt);
		// float kt = dt/(tTau + dt);
		// float kn = dt/(nTau + dt + 100.0*fabsf(qkc(2)));
		// float kd = dt/(dTau + dt);

		qkcHat += kq*(qkc - qkcHat);
		qkcHat /= qkcHat.norm();

		Eigen::Matrix3f RkcHat = getqRot(qkcHat);
		tkcHat += kt*(tkc - tkcHat);

		float alphank = 0.75;
		nk = (alphank*nk + (1.0-alphank)*(Eigen::Vector3f(0.0,0.0,1.0)));
		nk /= nk.norm();
		nkHat += kn*(nk - nkHat);
		nkHat /= nkHat.norm();

		ROS_WARN("time for update tnq %2.4f",float(clock()-updateClock)/CLOCKS_PER_SEC);
		updateClock = clock();

		Eigen::RowVector3f nkHatT = nkHat.transpose();
		Eigen::Matrix3f H = RkcHat+tkcHat*nkHatT;
		// Eigen::Matrix3f Gf = Eigen::Matrix3f::Zero();
		// for (int ii = 0; ii < 9; ii++)
		// {
		// 	Gf(ii/3,ii%3) = G.at<double>(ii/3,ii%3);
		// }
		//
		// // Eigen::Matrix3f H = camMatIf*Gf*camMatf;
		// // Eigen::Matrix<float,2,3> H12 = H.block(0,0,2,3);
		// // Eigen::RowVector3f H3 = H.block(2,0,1,3);
		//
		// // std::cout << "\nH\n" << H << std::endl;
		// // std::cout << "\nHc\n" << (camMatIf*Gf*camMatf) << std::endl;
		// // Eigen::RowVector3f Gp3 = Gpf.block(2,0,1,3);

		//remove the outliers
		std::vector<float> dkcs;
		assert(depthEstimators.size() > 0);
		assert(kPts.size() > 0);
		assert(cPts.size() > 0);

		// std::cout << "\n depthEstimators.size() " << depthEstimators.size() << " kPts.size() " << kPts.size() << " cPts.size() " << cPts.size() << std::endl << std::endl;

		ROS_WARN("time for check %2.4f",float(clock()-updateClock)/CLOCKS_PER_SEC);
		updateClock = clock();

		std::vector<DepthEstimator*> depthEstimatorsInHomog;
		// uint8_t* itIn = inliersG.data;
		std::vector<cv::Point2f>::iterator itc = cPts.begin();
		std::vector<cv::Point2f> kPtsC(kPts.size());
		if (!G.empty())
		{
			cv::perspectiveTransform(kPts,kPtsC,G);
		}
		std::vector<cv::Point2f>::iterator itkc = kPtsC.begin();
		float betakc = 0.2;
		for (std::vector<DepthEstimator*>::iterator it = depthEstimators.begin() ; it != depthEstimators.end(); it++)
		{
			if (!G.empty())
			{
				(*itc).x = betakc*(*itkc).x + (1.0-betakc)*(*itc).x;
				(*itc).y = betakc*(*itkc).y + (1.0-betakc)*(*itc).y;
			}
			float dkcHati = (*it)->update(H,Eigen::Vector3f(((*itc).x-cx)/fx,((*itc).y-cy)/fy,1.0),nkHatT,tkcHat,RkcHat,vc,wc,t,pkcHat,qkcHat);
			dkcs.push_back(dkcHati);
			depthEstimatorsInHomog.push_back(*it);
			// itIn++;
			itc++;
			itkc++;
			// std::cout << "\n ii " << ii << " stop\n";
		}
		depthEstimators = depthEstimatorsInHomog;
		depthEstimatorsInHomog.clear();

		ROS_WARN("time for estimators %2.4f",float(clock()-updateClock)/CLOCKS_PER_SEC);
		updateClock = clock();

		assert(depthEstimators.size() > 0);

		if(dkcs.size() > 0)
		{
			std::sort(dkcs.begin(),dkcs.end());
			float dkcMed = dkcs.at(0);
			if (dkcs.size()%2 == 0)
			{
				dkcMed = (dkcs.at(dkcs.size()/2-1)+dkcs.at(dkcs.size()/2))/2.0;
			}
			else
			{
				dkcMed = dkcs.at(dkcs.size()/2);
			}

			dkcHat += kd*(dkcMed - dkcHat);
			pkcHat += kp*(tkcHat*(dkcHat/tkcHat.norm()) - pkcHat);
		}

		ROS_WARN("dkcHat %2.1f",dkcHat);
	}
	catch (cv::Exception e)
	{
		ROS_ERROR("update failed");
	}
	//
	// std::cout << "\n after update \n";
	std::cout << "\n tkcHat \n" << tkcHat << std::endl;
	std::cout << "\n qkcHat \n" << qkcHat << std::endl;
	std::cout << "\n rotation \n" << 2.0*acos(qkcHat(0))*180/3.14 << std::endl;
	std::cout << "\n pkcHat \n" << pkcHat << std::endl;
	// std::cout << "\n dkHat " << dkHat << std::endl;
	std::cout << "\n dkcHat " << dkcHat << std::endl;
	std::cout << "\n vc \n" << vc << std::endl;
	std::cout << "\n wc \n" << wc << std::endl;



}
