#include <patch_estimator.h>

PatchEstimator::~PatchEstimator()
{
	destroyLock.lock();
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

	std::cout << "\n hid10 \n";
	imagePub.shutdown();
	std::cout << "\n hid11 \n";
	// imagePub2.shutdown();
	imageSub.shutdown();
	std::cout << "\n hid12 \n";
	odomSub.shutdown();
	std::cout << "\n hid13 \n";
	roiSub.shutdown();
	std::cout << "\n hid14 \n";
	roiPub.shutdown();
	std::cout << "\n hid15 \n";
	poseDeltaPub.shutdown();
	std::cout << "\n hid16 \n";
	wallPub.shutdown();
	std::cout << "\n hid17 \n";
	pointCloudPub.shutdown();
	std::cout << "\n hid18 \n";

	chessboardPub.shutdown();
	std::cout << "\n hid19 \n";
	// chessboardSub.shutdown();
	ROS_WARN("destroying \n");

	destroyLock.unlock();

	// ros::shutdown();
}

PatchEstimator::PatchEstimator() : it(nh)
{}

PatchEstimator::PatchEstimator(int imageWidthInit, int imageHeightInit, int partitionRowsInit, int partitionColsInit, int minDistanceInit, int minFeaturesDangerInit,
															 int minFeaturesBadInit, int keyIndInit, int patchIndInit, int partitionIndInit, float fxInit, float fyInit,
															 float cxInit, float cyInit, float zminInit, float zmaxInit, float fq, float fp, float ft, float fn, float fd,
															 float fG, std::string cameraNameInit, float tauInit, bool saveExpInit, std::string expNameInit,
														   int patchSizeBaseInit,int checkSizeBaseInit,Eigen::Vector3f pcbInit, Eigen::Vector4f qcbInit,
															 int numberFeaturesPerPartColInit, int numberFeaturesPerPartRowInit) : it(nh)
{
	imageWidth = imageWidthInit;
	imageHeight = imageHeightInit;
	partitionRows = partitionRowsInit;
	partitionCols = partitionColsInit;
	minDistance = minDistanceInit;
	numberFeaturesPerPartCol = numberFeaturesPerPartColInit;
	numberFeaturesPerPartRow = numberFeaturesPerPartRowInit;
	allPtsKnown = false;
	landmarkView = false;

	minFeaturesDanger = minFeaturesDangerInit;
	minFeaturesBad = minFeaturesBadInit;
	keyInd = keyIndInit;
	patchInd = patchIndInit;
	partitionInd = partitionIndInit;
	currentAvgPartition = partitionInd;
	patchSizeBase = patchSizeBaseInit;
	checkSizeBase = checkSizeBaseInit;
	pcb = pcbInit;
	qcb = qcbInit;
	numLandmarkCheck = 0;

	// Eigen::Vector4f qcbx(cos(0.5*M_PI/2),sin(-0.5*M_PI/2),0.0,0.0);
	// Eigen::Vector4f qcby(cos(-0.5*5.0*M_PI/180.0),0.0,sin(-0.5*5.0*M_PI/180.0),0.0);
	// qcb = getqMat(qcbx)*qcby;
	// qcb /= qcb.norm();

	patchShutdown = false;
	firstOdomImageCB = true;
	tkcHat = Eigen::Vector3f::Zero();
	nkHat = Eigen::Vector3f(0.0,0.0,1.0);
	qkcHat = Eigen::Vector4f(1.0,0.0,0.0,0.0);
	pkcHat = Eigen::Vector3f::Zero();
	pckHat = Eigen::Vector3f::Zero();
 	qckHat = Eigen::Vector4f(1.0,0.0,0.0,0.0);
	tkcDotEstimator.initialize(3);
	qkcDotEstimator.initialize(4);
	wcbHat = Eigen::Vector3f::Zero();

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
	camMatD = cv::Mat::zeros(3,3,CV_64F);
	camMatD.at<double>(0,0) = fx;
	camMatD.at<double>(0,2) = cx;
	camMatD.at<double>(1,1) = fy;
	camMatD.at<double>(1,2) = cy;
	camMatD.at<double>(2,2) = 1.0;

	camMatf = Eigen::Matrix3f::Zero();
	camMatf(0,0) = fx;
	camMatf(0,2) = cx;
	camMatf(1,1) = fy;
	camMatf(1,2) = cy;
	camMatf(2,2) = 1.0;

	Gkcum = cv::Mat::eye(3,3,CV_64F);

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
	GTau = 1.0/(2.0*M_PI*fG);

	// keyOdom = imageOdom;
	// kimage = image.clone();
	// pimage = image.clone();
	// tLast = t;
	// tStart = t;
	// odomSync.push_back(imageOdom);

	// std::cout << "\n patch \n";
	// for (int ii = 0; ii < pts.size(); ii++)
	// {
	// 	newDepthEstimator = new DepthEstimator(ii,Eigen::Vector3f((pts.at(ii).x-cx)/fx,(pts.at(ii).y-cy)/fy,1.0),tLast,zmin,zmax,(zmin+zmax)/2.0,tau,fx,fy,cx,cy);
	// 	depthEstimators.push_back(newDepthEstimator);
	// 	std::cout << ii << " ptix " << pts.at(ii).x << " ptiy " << pts.at(ii).y << std::endl;
	// 	std::cout << ii << " mcx " << depthEstimators.at(ii)->mc(0) << " mcy " << depthEstimators.at(ii)->mc(1) << std::endl;
	// 	std::cout << ii << " mkx " << depthEstimators.at(ii)->mk(0) << " mky " << depthEstimators.at(ii)->mk(1) << std::endl;
	// }

	cameraName = cameraNameInit;

	saveExp = saveExpInit;
	expName = expNameInit;

	// imagePub = it.advertise(cameraName+"/tracking_key"+std::to_string(keyInd)+"_patch"+std::to_string(patchInd),1);
	// imagePub = it.advertise(cameraName+"/test_image",1);
	// poseDeltaPub = nh.advertise<icl_multiple_stationary::PoseDelta>(cameraName+"/pose_delta",10);
	// roiPub = nh.advertise<icl_multiple_stationary::Roi>(cameraName+"/"+std::to_string(keyInd)+"/"+std::to_string(patchInd)+"/roi",10);
	// wallPub = nh.advertise<icl_multiple_stationary::Wall>("/wall_points",10);
	// imagePub2 = it.advertise(cameraName+"/test_image2",1);
	imageSub = it.subscribe(cameraName+"/image_undistort", 100, &PatchEstimator::imageCB,this);
	odomSub = nh.subscribe(cameraName+"/odom", 100, &PatchEstimator::odomCB,this);
	// roiSub = nh.subscribe(cameraName+"/"+std::to_string(keyInd)+"/"+std::to_string(patchInd)+"/roi",10, &PatchEstimator::roiCB,this);

	// odomPub = nh.advertise<nav_msgs::Odometry>(cameraName+"/odomHat",1);
	// odomDelayedPub = nh.advertise<nav_msgs::Odometry>(cameraName+"/odomDelayed", 1);
	// pointCloudPub = nh.advertise<PointCloud> ("patch_map", 1);
	firstImage = true;
}

void PatchEstimator::initialize(cv::Mat& image, nav_msgs::Odometry imageOdom, ros::Time t)
{
	imagePub = it.advertise(cameraName+"/test_image",1);
	poseDeltaPub = nh.advertise<icl_multiple_stationary::PoseDelta>(cameraName+"/pose_delta",1);
	roiPub = nh.advertise<icl_multiple_stationary::Roi>(cameraName+"/"+std::to_string(keyInd)+"/"+std::to_string(patchInd)+"/roi",1);
	wallPub = nh.advertise<icl_multiple_stationary::Wall>("/wall_points",1);
	roiSub = nh.subscribe(cameraName+"/"+std::to_string(keyInd)+"/"+std::to_string(patchInd)+"/roi",1, &PatchEstimator::roiCB,this);
	pointCloudPub = nh.advertise<PointCloud>("patch_map", 1);
	chessboardPub = nh.advertise<std_msgs::Time>(cameraName+"/"+std::to_string(keyInd)+"/"+std::to_string(patchInd)+"/chessboard", 1);
	chessboardSub = nh.subscribe(cameraName+"/"+std::to_string(keyInd)+"/"+std::to_string(patchInd)+"/chessboard",1, &PatchEstimator::chessboardCB,this);

	keyOdom = imageOdom;
	kimage = image.clone();
	pimage = image.clone();
	tLast = t;
	tStart = t;
	tkcDotEstimator.update(Eigen::Vector3f::Zero(),t);
	qkcDotEstimator.update(Eigen::Vector4f(1.0,0.0,0.0,0.0),t);

	int partitionWidth = imageWidth/partitionCols;
	int partitionHeight = imageHeight/partitionRows;

	// get the center of each partition and place uniform spacing of points using minDistance
	std::vector<cv::Point2f> pts;
	int colc = partitionInd%partitionCols*partitionWidth + partitionWidth/2;
	int rowc = partitionInd/partitionCols*partitionHeight + partitionHeight/2;
	int coltl = colc - (numberFeaturesPerPartCol-1)*minDistance/2;
	int rowtl = rowc - (numberFeaturesPerPartRow-1)*minDistance/2;

	for (int jj = 0; jj < numberFeaturesPerPartRow; jj++)
	{
		int rowii = rowtl + jj*minDistance;
		for (int hh = 0; hh < numberFeaturesPerPartCol; hh++)
		{
			int colii = coltl + hh*minDistance;
			pts.push_back(cv::Point2f(colii,rowii));
		}
	}

	std::cout << "\n patch \n";
	for (int ii = 0; ii < pts.size(); ii++)
	{
		newDepthEstimator = new DepthEstimator(ii,Eigen::Vector3f((pts.at(ii).x-cx)/fx,(pts.at(ii).y-cy)/fy,1.0),tLast,zmin,zmax,(zmin+zmax)/2.0,tau,fx,fy,cx,cy);
		depthEstimators.push_back(newDepthEstimator);
		std::cout << ii << " ptix " << pts.at(ii).x << " ptiy " << pts.at(ii).y << std::endl;
		std::cout << ii << " mcx " << depthEstimators.at(ii)->mc(0) << " mcy " << depthEstimators.at(ii)->mc(1) << std::endl;
		std::cout << ii << " mkx " << depthEstimators.at(ii)->mk(0) << " mky " << depthEstimators.at(ii)->mk(1) << std::endl;
	}
}

void PatchEstimator::odomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
	odomCBMutex.lock();
	if (patchShutdown)
	{
		odomCBMutex.unlock();
		return;
	}
	odomSync.push_back(*msg);
	odomCBMutex.unlock();
}

//roi CB
void PatchEstimator::roiCB(const icl_multiple_stationary::Roi::ConstPtr& msg)
{
	roiMutex.lock();
	ROS_WARN("\n\n ************** ROI START ************** \n\n");
	if (patchShutdown)
	{
		roiMutex.unlock();
		return;
	}

	clock_t processTime = clock();
	ros::Time t = msg->header.stamp;
	Eigen::Vector3f pcw(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
	Eigen::Vector4f qcw(msg->pose.orientation.w,msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z);
	qcw /= qcw.norm();

	Eigen::Vector3f pkw(keyOdom.pose.pose.position.x,keyOdom.pose.pose.position.y,keyOdom.pose.pose.position.z);
	Eigen::Vector4f qkw(keyOdom.pose.pose.orientation.w,keyOdom.pose.pose.orientation.x,keyOdom.pose.pose.orientation.y,keyOdom.pose.pose.orientation.z);
	qkw /= qkw.norm();

	//estimate the plane using the points by normalizing by the z element of the normal

	// Eigen::Vector3f pkw(keyOdom.pose.pose.position.x,keyOdom.pose.pose.position.y,keyOdom.pose.pose.position.z);
	// Eigen::Vector4f qkw(keyOdom.pose.pose.orientation.w,keyOdom.pose.pose.orientation.x,keyOdom.pose.pose.orientation.y,keyOdom.pose.pose.orientation.z);
	int numberPts = int(depthEstimators.size());
	Eigen::MatrixXf XY1ICL = Eigen::MatrixXf::Ones(numberPts,3);
	Eigen::MatrixXf XY1TICL = Eigen::MatrixXf::Ones(3,numberPts);
	Eigen::VectorXf NZICL = Eigen::VectorXf::Zero(numberPts);
	Eigen::MatrixXf XY1EKF = Eigen::MatrixXf::Ones(numberPts,3);
	Eigen::MatrixXf XY1TEKF = Eigen::MatrixXf::Ones(3,numberPts);
	Eigen::VectorXf NZEKF = Eigen::VectorXf::Zero(numberPts);
	// Eigen::Matrix<float,numberPts,3> XZ1 = Eigen::Matrix<float,numberPts,3>::Ones();
	// Eigen::Matrix<float,3,numberPts> XZ1T = Eigen::Matrix<float,3,numberPts>::Ones();
	// Eigen::Matrix<float,numberPts,1> NY = Eigen::Matrix<float,numberPts,1>::Zeros();
	// Eigen::Matrix<float,numberPts,3> YZ1 = Eigen::Matrix<float,numberPts,3>::Ones();
	// Eigen::Matrix<float,3,numberPts> YZ1T = Eigen::Matrix<float,3,numberPts>::Ones();
	// Eigen::Matrix<float,numberPts,1> NX = Eigen::Matrix<float,numberPts,1>::Zeros();

	int minx,maxx,miny,maxy;
	bool firstPt = true;
	int itIdx = 0;
	float avgNumSaved = 0.0;

	PointCloudRGB kcloud;
	kcloud.clear();
	kcloud.height = 1;
	kcloud.width = 9;
	kcloud.is_dense = true;
	kcloud.resize(9);
	PointCloudRGB::iterator kcloudIt = kcloud.begin();

	PointCloudRGB ccloud;
	ccloud.clear();
	ccloud.height = 1;
	ccloud.width = 9;
	ccloud.is_dense = true;
	ccloud.resize(9);
	PointCloudRGB::iterator ccloudIt = ccloud.begin();

	for (std::vector<DepthEstimator*>::iterator itD = depthEstimators.begin(); itD != depthEstimators.end(); itD++)
	{
		//add each point
		avgNumSaved += float((*itD)->depthEstimatorICLExt.numSaved);
		Eigen::Vector3f mic = (*itD)->current();
		Eigen::Vector3f uic = mic/mic.norm();
		Eigen::Vector3f picICL = uic*((*itD)->dcHatICLExt);
		Eigen::Vector3f piwICLc = pcw + rotatevec(picICL,qcw);
		(*ccloudIt).x = piwICLc(0);
		(*ccloudIt).y = piwICLc(1);
		(*ccloudIt).z = piwICLc(2);
		(*ccloudIt).r = 0;
		(*ccloudIt).g = 0;
		(*ccloudIt).b = 255;
		ccloudIt++;

		Eigen::Vector3f uik = (*itD)->uk;
		Eigen::Vector3f pikICL = uik*((*itD)->dkHatICLExt);
		Eigen::Vector3f piwICLk = pkw + rotatevec(pikICL,qkw);
		(*kcloudIt).x = piwICLk(0);
		(*kcloudIt).y = piwICLk(1);
		(*kcloudIt).z = piwICLk(2);
		(*kcloudIt).r = 0;
		(*kcloudIt).g = 255;
		(*kcloudIt).b = 0;
		kcloudIt++;

		Eigen::Vector3f micEKF = (*itD)->mc;
		Eigen::Vector3f picEKF = micEKF*((*itD)->zcHatEKF);
		XY1ICL(itIdx,0) = picICL(0);
		XY1ICL(itIdx,1) = picICL(1);
		XY1TICL(0,itIdx) = picICL(0);
		XY1TICL(1,itIdx) = picICL(1);
		NZICL(itIdx) = -picICL(2);

		XY1EKF(itIdx,0) = picEKF(0);
		XY1EKF(itIdx,1) = picEKF(1);
		XY1TEKF(0,itIdx) = picEKF(0);
		XY1TEKF(1,itIdx) = picEKF(1);
		NZEKF(itIdx) = -picEKF(2);
		// XZ1(itIdx,0) = pic(0);
		// XZ1(itIdx,1) = pic(2);
		// XZ1T(0,itIdx) = pic(0);
		// XZ1T(1,itIdx) = pic(2);
		// NY(itIdx) = -pic(1);
		// YZ1(itIdx,0) = pic(1);
		// YZ1(itIdx,1) = pic(2);
		// YZ1T(0,itIdx) = pic(1);
		// YZ1T(1,itIdx) = pic(2);
		// NX(itIdx) = -pic(0);
		itIdx++;

		cv::Point2f ptic(fx*mic(0)+cx,fy*mic(1)+cy);

		// Eigen::Vector3f mkiHat = depthEstimators.at(ii)->mk;
		// cv::Point2i kPti(int(fx*mkiHat(0)+cx),int(fy*mkiHat(1)+cy));
		// uint8_t colori = kimage.at<uint8_t>(kPti.y,kPti.x);
		if (firstPt)
		{
			minx = ptic.x;
			maxx = ptic.x;
			miny = ptic.y;
			maxy = ptic.y;
			firstPt = false;
		}
		else
		{
			if (minx > ptic.x)
			{
				minx = ptic.x;
			}
			if (maxx < ptic.x)
			{
				maxx = ptic.x;
			}
			if (miny > ptic.y)
			{
				miny = ptic.y;
			}
			if (maxy < ptic.y)
			{
				maxy = ptic.y;
			}
		}
	}

	avgNumSaved /= float(numberPts);

	int cols = maxx-minx;
	int rows = maxy-miny;
	int coltl = minx;
	int rowtl = miny;


	if ((coltl >= 0) && (rowtl >= 0) && ((coltl+cols) < pimage.cols) && ((rowtl+rows) < pimage.rows))
	{
		try
		{
			cv::Mat roiimage = pimage(cv::Rect(coltl,rowtl,cols,rows)).clone();
			int reduceFactor = 25;
			int colsReduce = std::round(cols/reduceFactor);
			int rowsReduce = std::round(rows/reduceFactor);
			cv::Mat roiimageReduce(cv::Size(colsReduce,rowsReduce),CV_8U);
			cv::resize(roiimage,roiimageReduce,roiimageReduce.size(),fx,fy,cv::INTER_AREA);

			roiMutex.unlock();

			Eigen::Matrix3f XYXYICL = XY1TICL*XY1ICL;
			Eigen::Vector3f XYNZICL = XY1TICL*NZICL;
			float XYXYdet = float(XYXYICL.determinant());
			Eigen::Matrix3f XYXYEKF = XY1TEKF*XY1EKF;
			Eigen::Vector3f XYNZEKF = XY1TEKF*NZEKF;
			// Eigen::Matrix3f XZXZ = XZ1T*XZ1;
			// float XZXZdet = float(XZXZ.determinant());
			// Eigen::Matrix3f YZYZ = YZ1T*YZ1;
			// float YZYZdet = float(YZYZ.determinant());

			if (XYXYdet > 0.001)
			{
				Eigen::JacobiSVD<Eigen::MatrixXf> svdXYXYICL(XYXYICL, Eigen::ComputeThinU | Eigen::ComputeThinV);
				Eigen::Vector3f dnzICL = svdXYXYICL.solve(XYNZICL);
				Eigen::Vector3f ncICL(dnzICL(0),dnzICL(1),1.0);
				ncICL /= ncICL.norm();
				float nxICL = ncICL(0);
				float nyICL = ncICL(1);
				float nzICL = ncICL(2);
				float dcICL = -dnzICL(2)*nzICL;

				Eigen::JacobiSVD<Eigen::MatrixXf> svdXYXYEKF(XYXYEKF, Eigen::ComputeThinU | Eigen::ComputeThinV);
				Eigen::Vector3f dnzEKF = svdXYXYEKF.solve(XYNZEKF);
				Eigen::Vector3f ncEKF(dnzEKF(0),dnzEKF(1),1.0);
				ncEKF /= ncEKF.norm();
				float nxEKF = ncEKF(0);
				float nyEKF = ncEKF(1);
				float nzEKF = ncEKF(2);
				float dcEKF = -dnzEKF(2)*nzEKF;

				ROS_WARN("roi get normal time %2.4f",float(clock()-processTime)/CLOCKS_PER_SEC);
				processTime = clock();

				//get the estimated point for each pixel in the roi
				int plotInd = 0;

				// PointCloudRGB::Ptr map(new PointCloudRGB);
				// pcl_conversions::toPCL(msg->header.stamp,map->header.stamp);
				// map->header.frame_id = "world";
				// map->height = rows;
				// map->width = cols;
				// map->is_dense = true;
				// map->points.clear();
				// map->points.resize(rows*cols);

				// std::vector<geometry_msgs::Point32> wallPts(colsReduce*rowsReduce);
				// std::vector<uint8_t> wallColors(colsReduce*rowsReduce);
				// std::vector<geometry_msgs::Point32>::iterator ptIt = wallPts.begin();
				// std::vector<uint8_t>::iterator colIt = wallColors.begin();
				PointCloudRGB cloud;
				cloud.clear();
				cloud.height = 1;
				cloud.width = colsReduce*rowsReduce;
				cloud.is_dense = true;
				cloud.resize(colsReduce*rowsReduce);
				PointCloudRGB::iterator cloudIt = cloud.begin();
				pcl::PointXYZRGB ptxyz;

				//convert into body frame
				// Eigen::Vector3f pbw = rotatevec(pcw,qcb)-pcb;
				// pbw(2) = 0.0;
				// Eigen::Vector4f qbw = getqMat(qcw)*getqInv(qcb);
				// qbw /= qbw.norm();
				// qbw(1) = 0.0;
				// qbw(2) = 0.0;
				// qbw /= qbw.norm();
				// qcw = getqMat(qbw)*qcb;
				// qcw /= qcw.norm();
				// Eigen::Vector3f pcbw = rotatevec(pcb,qbw);


				int rowj = 0;
				int colj = 0;

				// *colIt = uint8_t(*itI);
				float mxj = 0;
				float myj = 0;
				// Eigen::Vector4f pjw4(0.0,0.0,0.0,0.0);
				// Eigen::Vector3f pjwc(0.0,0.0,0.0);
				// Eigen::Matrix4f qjw4M;
				// float xb
				// float xc = pcwInitcbInit(0);
				// float yc = pcwInitcbInit(1);
				// float zc = pcwInitcbInit(2);
				// float qw = qcwcwInit(0);
				// float qx = qcwcwInit(1);
				// float qy = qcwcwInit(2);
				// float qz = qcwcwInit(3);
				// float qwI = qcwcwInitI(0);
				// float qxI = qcwcwInitI(1);
				// float qyI = qcwcwInitI(2);
				// float qzI = qcwcwInitI(3);
				float xjcICL = 0;
				float yjcICL = 0;
				float zjcICL = 0;
				Eigen::Vector3f piw(0.0,0.0,0.0);
				Eigen::Vector3f pic(0.0,0.0,0.0);
				// float wjwICL = 0;
				// float xjwICL = 0;
				// float yjwICL = 0;
				// float zjwICL = 0;
				// float xjcEKF = 0;
				// float yjcEKF = 0;
				// float zjcEKF = 0;
				// float wjwEKF = 0;
				// float xjwEKF = 0;
				// float yjwEKF = 0;
				// float zjwEKF = 0;
				// Eigen::Vector3f pjw(0.0,0.0,0.0);
				for (cv::MatIterator_<uchar> itI = roiimageReduce.begin<uchar>(); itI != roiimageReduce.end<uchar>(); itI++)
				{
					rowj = rowtl + reduceFactor*(plotInd/colsReduce);
					colj = coltl + reduceFactor*(plotInd%colsReduce);

					// *colIt = uint8_t(*itI);
					mxj = (colj - cx)/fx;
					myj = (rowj - cy)/fy;
					zjcICL = dcICL/(nxICL*mxj+nyICL*myj+nzICL);
					xjcICL = zjcICL*mxj;
					yjcICL = zjcICL*myj;

					pic(0) = xjcICL;
					pic(1) = yjcICL;
					pic(2) = zjcICL;

					// zjcEKF = dcEKF/(nxEKF*mxj+nyEKF*myj+nzEKF);
					// xjcEKF = zjcEKF*mxj;
					// yjcEKF = zjcEKF*myj;
					// Eigen::Vector3f pjc(zjc*mxj,zjc*myj,zjc);

					// //rotate the point into the world frame
					// Eigen::Vector3f pjw = pcw + rotatevec(pjc,qcw);
					//
					// Eigen::Vector4f qcwInit(cos(3.1415/4.0),sin(3.1415/4.0),0.0,0.0);
					// qcwInit /= qcwInit.norm();
					// pjw = rotatevec(pjw-rotatevec(pcb,getqInv(qcb)),getqInv(qcwInit));

					//rotate the point into the world frame
					// pjw4 = getqMat(qcwcwInitMatR*pjwc)*qcwcwInitI;

					// wjwICL = -qx*xjcICL-qy*yjcICL-qz*zjcICL;
					// xjwICL = qw*xjcICL-qz*yjcICL+qy*zjcICL;
					// yjwICL = qz*xjcICL+qw*yjcICL-qx*zjcICL;
					// zjwICL = -qy*xjcICL+qx*yjcICL+qw*zjcICL;

					// wjwEKF = -qx*xjcEKF-qy*yjcEKF-qz*zjcEKF;
					// xjwEKF = qw*xjcEKF-qz*yjcEKF+qy*zjcEKF;
					// yjwEKF = qz*xjcEKF+qw*yjcEKF-qx*zjcEKF;
					// zjwEKF = -qy*xjcEKF+qx*yjcEKF+qw*zjcEKF;
					piw = pcw + rotatevec(pic,qcw);

					// std::cout << "piwHatx " << piw(0) << ", piwHaty " << piw(1) << ", piwHatz " << piw(2) << std::endl;

					ptxyz.x = piw(0);
					ptxyz.y = piw(1);
					ptxyz.z = piw(2);
					ptxyz.r = (*itI);
					ptxyz.g = (*itI);
					ptxyz.b = std::min((*itI)+100,255);
					*cloudIt = ptxyz;
					cloudIt++;

					// ptxyz.x = xc+xjwEKF*qwI+wjwEKF*qxI-zjwEKF*qyI+yjwEKF*qzI;
					// ptxyz.y = yc+yjwEKF*qwI+zjwEKF*qxI+wjwEKF*qyI-xjwEKF*qzI;
					// ptxyz.z = zc+zjwEKF*qwI-yjwEKF*qxI+xjwEKF*qyI+wjwEKF*qzI;
					// ptxyz.r = (*itI);
					// ptxyz.g = (*itI);
					// ptxyz.b = std::min((*itI)+100,255);
					// *cloudIt = ptxyz;
					// cloudIt++;

					// geometry_msgs::Point32 pjwMsg;
					// pjwMsg.x = pjw(0);
					// pjwMsg.y = pjw(1);
					// pjwMsg.z = pjw(2);
					// *ptIt = pjwMsg;
					plotInd++;
					// ptIt++;
					// colIt++;
				}

				ROS_WARN("roi get points time %2.4f, plotInd %d",float(clock()-processTime)/CLOCKS_PER_SEC,plotInd);
				processTime = clock();

				// map->points = cloud;
				//publish the features

				PointCloudRGB::Ptr mappatch(new PointCloudRGB);
				(*mappatch).clear();
				pcl_conversions::toPCL(msg->header.stamp,mappatch->header.stamp);

				// std::cout << "\n wall 1 1 \n";
				mappatch->header.frame_id = "world";

				// std::cout << "\n wall 1 2 \n";
				mappatch->height = 1;
				mappatch->is_dense = true;
				mappatch->points.clear();
				*mappatch += ccloud;
				*mappatch += kcloud;
				mappatch->width = mappatch->points.size();


				PointCloudRGB::Ptr map(new PointCloudRGB);
				(*map).clear();
				pcl_conversions::toPCL(msg->header.stamp,map->header.stamp);

				// std::cout << "\n wall 1 1 \n";
				map->header.frame_id = "world";

				// std::cout << "\n wall 1 2 \n";
				map->height = 1;
				map->is_dense = true;
				map->points.clear();
				*map += cloud;
				map->width = map->points.size();

				ROS_WARN("patch map->width %d",int(map->width));
				// std::cout << "\n wall 3 \n";


				// sensor_msgs::PointCloud2 cloudMsg;

				// wallMsg.colors = wallColors;
				// wallMsg.rows = rows;
				// wallMsg.cols = cols;

				roiMutex.lock();
				ROS_WARN("ROI checking");
				if (!patchShutdown)
				{
					ROS_WARN("ROI patch not shutdown");
					// if (allPtsKnown && (acos(nyICL) > (70.0*3.1415/180.0)) && (acos(nxICL) > (70.0*3.1415/180.0)))
					if (allPtsKnown && (avgNumSaved >= 1) && (acos(nzICL) < (30.0*3.1415/180.0)))
					{
						icl_multiple_stationary::Wall wallMsg;
						wallMsg.header.stamp = t;
						// wallMsg.wallPts = wallPts;
						pcl::toROSMsg(cloud, wallMsg.cloud);
						wallMsg.keyInd = keyInd;
						wallMsg.patchInd = patchInd;
						wallMsg.pose = msg->pose;
						wallPub.publish(wallMsg);

						icl_multiple_stationary::PoseDelta poseDeltaMsg;
						poseDeltaMsg.header.stamp = t;
						poseDeltaMsg.pose = msg->pose;
						poseDeltaMsg.poseHat = msg->poseHat;
						poseDeltaMsg.landmarkView = landmarkView;
						poseDeltaPub.publish(poseDeltaMsg);
						ROS_WARN("published poseDelta");
					}
					pointCloudPub.publish(mappatch);
				}
				else
				{
					ROS_ERROR("ROI patchShutdown");
				}
				roiMutex.unlock();

				ROS_WARN("roi send time %2.4f",float(clock()-processTime)/CLOCKS_PER_SEC);
				processTime = clock();
			}
		}
		catch (cv::Exception e)
		{
			ROS_ERROR("wall points failed");
			roiMutex.unlock();
		}
	}
	else
	{
		roiMutex.unlock();
		ROS_ERROR("wall points failed roi to large");
	}

	ROS_WARN("\n\n ************** ROI STOP ************** \n\n");
}

//image callback
void PatchEstimator::imageCB(const sensor_msgs::Image::ConstPtr& msg)
{
	odomMutex.lock();
	ROS_WARN("\n\n ************** IMAGE START ************** \n\n");
	if (patchShutdown)
	{
		odomMutex.unlock();
		return;
	}
	odomMutex.unlock();

	clock_t callbackTime = clock();
	clock_t processTime = clock();
	// std::cout << std::endl;

	// sync the times to find the closest
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

	if (firstImage)
	{
		initialize(image,imageOdom,t);
		firstImage = false;
	}

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

	// pckHat += (rotatevec(vc,qckHat)*dt);
	// qckHat += (0.5*B(qckHat)*wc*dt);
	// qckHat /= qckHat.norm();
	// pkcHat = rotatevec(-pckHat,getqInv(qckHat));
	// qkcHat = getqInv(qckHat);
	// qkcHat /= qkcHat.norm();
	Eigen::Vector3f pkpHat = pkcHat;
	Eigen::Vector3f pckH = -rotatevec(pkcHat,getqInv(qkcHat)) + rotatevec(vc,getqInv(qkcHat))*dt;
	pkcHat = -rotatevec(pckH,qkcHat);

	// pkcHat += ((-vc-getss((wc))*pkcHat)*dt);
	Eigen::Vector4f qkpHat = qkcHat;
	qkcHat += (-0.5*B(qkcHat)*(wc)*dt);
	qkcHat /= qkcHat.norm();

	Eigen::Vector4f qpcHat = getqMat(qkcHat)*getqInv(qkpHat);
	qpcHat /= qpcHat.norm();
	Eigen::Vector3f ppcHat = pkcHat - rotatevec(pkpHat,qpcHat);

	Eigen::Vector3f pkw(keyOdom.pose.pose.position.x,keyOdom.pose.pose.position.y,keyOdom.pose.pose.position.z);
	Eigen::Vector4f qkw(keyOdom.pose.pose.orientation.w,keyOdom.pose.pose.orientation.x,keyOdom.pose.pose.orientation.y,keyOdom.pose.pose.orientation.z);

	Eigen::Vector4f qkc = getqMat(getqInv(qcw))*qkw;
	qkc /= qkc.norm();
	Eigen::Vector3f pkc = rotatevec(pkw-pcw,getqInv(qcw));

	// find the features
	match(image,dt,vc,wc,t,ppcHat,qpcHat,pkc,qkc);

	pubMutex.lock();
	cv::Mat drawImage = image.clone();

	// cv::Mat imageGrad;
	// // cv::Sobel(ppatch,pSobel,CV_32F,1,1,3);
	//
	// /// Generate grad_x and grad_y
	// cv::Mat imageGrad_x, imageGrad_y, imageGrad_xabs, imageGrad_yabs;
	//
	// /// Gradient X
	// //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
	// cv::Sobel(drawImage, imageGrad_x, CV_32F, 1, 0, 5);
	// cv::Sobel(drawImage, imageGrad_y, CV_32F, 0, 1, 5);
	// cv::convertScaleAbs(imageGrad_x, imageGrad_xabs);
	// cv::convertScaleAbs(imageGrad_y, imageGrad_yabs );
	// cv::addWeighted( imageGrad_xabs, 0.5, imageGrad_yabs, 0.5, 0, imageGrad );

	// cv::Mat pCompare,cCompare;
	// cv::addWeighted( ppatch, 0.5, pSobel, 0.5, 0, pCompare);
	// cv::addWeighted( cpatch, 0.5, cSobel, 0.5, 0, cCompare);

	if (depthEstimators.size() > minFeaturesBad)
	{

		for (uint16_t ii = 0; ii < depthEstimators.size(); ii++)
		{
			//get the points after update
			Eigen::Vector3f mci = depthEstimators.at(ii)->mc;
			// Eigen::Vector3f uci = mci/mci.norm();
			cv::Point2f cPti(fx*mci(0)+cx,fy*mci(1)+cy);

			cv::circle(drawImage, cPti, 10, cv::Scalar(50, 50, 50), -1);

			// Eigen::Vector3f mkiHat = depthEstimators.at(ii)->mk;
			// cv::Point2i kPti(int(fx*mkiHat(0)+cx),int(fy*mkiHat(1)+cy));
			// uint8_t colori = kimage.at<uint8_t>(kPti.y,kPti.x);

			// Eigen::Vector3f pciHatICLExt = uci*depthEstimators.at(ii)->dcHatICLExt;
			// Eigen::Vector3f pciHatEKF = mci*depthEstimators.at(ii)->zcHatEKF;

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

			// ptHat.x = pciHatICLExt(0);
			// ptHat.y = pciHatICLExt(1);
			// ptHat.z = pciHatICLExt(2);
			// ptHat.r = 0;
			// ptHat.g = 200;
			// ptHat.b = 0;
			// map->points.push_back(ptHat);
			//
			// ptHat.x = pciHatEKF(0);
			// ptHat.y = pciHatEKF(1);
			// ptHat.z = pciHatEKF(2);
			// ptHat.r = 0;
			// ptHat.g = 0;
			// ptHat.b = 255;
			// map->points.push_back(ptHat);

			// std::cout << "\n index " << ii << std::endl;
			// std::cout << "\n dkHat " << depthEstimators.at(ii)->dkHat << std::endl;
			// std::cout << "\n dcHat " << depthEstimators.at(ii)->dcHat << std::endl;
			// std::cout << "\n zcHatICLExt " << pciHatICLExt(2) << std::endl;
			// std::cout << "\n zcHatEKF " << pciHatEKF(2) << std::endl;
			// std::cout << "\n dk " << depthEstimators.at(ii)->pik.norm() << std::endl;

			// if((fabsf(pciHatICLExt(2)) > 100.0) || std::isnan(pciHatICLExt(2)))
			// {
			// 	ros::shutdown();
			// }
		}

		// map->width = map->points.size();
		// pointCloudPub.publish(map);
	}
	pubMutex.unlock();

	// ROS_WARN("get circle time %2.4f",float(clock()-processTime)/CLOCKS_PER_SEC);
	// processTime = clock();

	// publish key image
	cv_bridge::CvImage out_msg;
	out_msg.header = msg->header; // Same timestamp and tf frame as input image
	out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
	out_msg.image = drawImage; // Your cv::Mat

	pubMutex.lock();
	imagePub.publish(out_msg.toImageMsg());
	int depthEstimatorsSize = depthEstimators.size();
	pubMutex.unlock();

	// ROS_WARN("cb time %2.4f",float(clock()-callbackTime)/CLOCKS_PER_SEC);

	if ((depthEstimatorsSize <= minFeaturesBad))// || (inlierRatio <= 0.26))
	{
		pubMutex.lock();

		std::cout << "\n hid8 \n";
		for (std::vector<DepthEstimator*>::iterator itD = depthEstimators.begin() ; itD != depthEstimators.end(); itD++)
		{
			delete *itD;
		}
		std::cout << "\n hid9 \n";
		depthEstimators.clear();
		std::cout << "\n hid10 \n";

		patchShutdown = true;
		pubMutex.unlock();

		// odomPub.shutdown();
		// odomDelayedPub.shutdown();
		// pointCloudPub.shutdown();
		// patchShutdown = true;

		ROS_WARN("shutdown after imagesub");
	}
	else
	{
		std::cout << "\n hi9 \n";

		// std::cout << "\n dt \n" << dt << std::endl;
		// std::cout << "\n vc \n" << vc << std::endl;
		// std::cout << "\n wc \n" << wc << std::endl;
		// std::cout << "\n pkcHat \n" << pkcHat << std::endl;
		// std::cout << "\n qkcHat \n" << qkcHat << std::endl;

		// ros::shutdown();

		ROS_WARN("get match time %2.4f",float(clock()-processTime)/CLOCKS_PER_SEC);
		processTime = clock();

		pubMutex.lock();
		pimage = image.clone();

		//send the pose estimate to the odom
		// Eigen::Vector4f qckHat = getqInv(qkcHat);
		// qckHat /= qckHat.norm();
		// Eigen::Vector3f pckHat = rotatevec(-pkcHat,qckHat);
		qckHat = getqInv(qkcHat/qkcHat.norm());
		qckHat /= qckHat.norm();

		pckHat = rotatevec(-pkcHat,qckHat);

		// std::cout << "\n pckHat \n" << pckHat << std::endl;
		// std::cout << "\n qckHat \n" << qckHat << std::endl;

		Eigen::Vector4f qcwHat = getqMat(qkw)*qckHat;
		qcwHat /= qcwHat.norm();

		Eigen::Vector3f pcwHat = pkw + rotatevec(pckHat,qkw);

		// Eigen::Vector3f pckbHat = rotatevec(pckHat,qcb);
		// Eigen::Vector4f qckbHat = getqMat(getqMat(qcb)*qckHat)*getqInv(qcb);
		// qckbHat /= qckbHat.norm();
		//
		// std::cout << "\n pckbHat \n" << pckbHat << std::endl;
		// std::cout << "\n qckbHat \n" << qckbHat << std::endl;

		Eigen::Vector4f qbwHat = getqMat(qcwHat)*getqInv(qcb);
		qbwHat /= qbwHat.norm();
		// qbwHat(1) = 0.0;
		// qbwHat(2) = 0.0;
		// qbwHat /= qbwHat.norm();

		Eigen::Vector3f pbwHat = pcwHat - rotatevec(pcb,qbwHat);
		// pbwHat(2) = 0.0;

		Eigen::Vector4f qbw = getqMat(qcw)*getqInv(qcb);
		qbw /= qbw.norm();
		// qbw(1) = 0.0;
		// qbw(2) = 0.0;
		// qbw /= qbw.norm();
		Eigen::Vector3f pbw = pcw - rotatevec(pcb,qbw);
		// pbw(2) = 0.0;

		// Eigen::Vector3f pbwk = rotatevec(pkw,qcb)-pcb;
		// pbwk(2) = 0.0;
		// Eigen::Vector4f qbwk = getqMat(qkw)*getqInv(qcb);
		// qbwk /= qbwk.norm();
		// qbwk(1) = 0.0;
		// qbwk(2) = 0.0;
		// qbwk /= qbwk.norm();

		std::cout << "\n pbwHat \n" << pbwHat << std::endl;
		std::cout << "\n qbwHat \n" << qbwHat << std::endl;
		// std::cout << "\n pbwk \n" << pbwk << std::endl;
		// std::cout << "\n qbwk \n" << qbwk << std::endl;
		std::cout << "\n pbw \n" << pbw << std::endl;
		std::cout << "\n qbw \n" << qbw << std::endl;
		// std::cout << "\n pcw \n" << pcw << std::endl;
		// std::cout << "\n qcw \n" << qcw << std::endl;
		// std::cout << "\n pkw \n" << pkw << std::endl;
		// std::cout << "\n qkw \n" << qkw << std::endl;

		// Eigen::Vector3f pkc = rotatevec(pkw-pcw,getqInv(qcw));
		// Eigen::Vector4f qkc = getqMat(getqInv(qcw))*qkw;
		// qkc /= qkc.norm();

		icl_multiple_stationary::Roi roiMsg;
		roiMsg.header.stamp = t;
		roiMsg.pose = imageOdom.pose.pose;
		roiMsg.poseHat.position.x = pcwHat(0);
		roiMsg.poseHat.position.y = pcwHat(1);
		roiMsg.poseHat.position.z = pcwHat(2);
		roiMsg.poseHat.orientation.w = qcwHat(0);
		roiMsg.poseHat.orientation.x = qcwHat(1);
		roiMsg.poseHat.orientation.y = qcwHat(2);
		roiMsg.poseHat.orientation.z = qcwHat(3);
		roiPub.publish(roiMsg);

		if (!landmarkView)
		{
			std_msgs::Time tMsg;
			tMsg.data = t;
			chessboardPub.publish(tMsg);
		}

		pubMutex.unlock();
	}

	ROS_WARN("IMAGE STOP cb time %2.4f",float(clock()-callbackTime)/CLOCKS_PER_SEC);
}

void PatchEstimator::chessboardCB(const std_msgs::Time::ConstPtr& msg)
{
	chessboardMutex.lock();
	if (patchShutdown)
	{
		chessboardMutex.unlock();
		return;
	}

	if (numLandmarkCheck < 10)
	{
		numLandmarkCheck++;
		chessboardMutex.unlock();
		return;
	}
	else
	{
		numLandmarkCheck = 0;
	}

	clock_t chessTime = clock();
	ROS_WARN("chessboard time %2.4f",float(clock()-chessTime)/CLOCKS_PER_SEC);
	cv::Size patternSize(8,6);
	std::vector<cv::Point2f> ptsCheckerBoard;
	landmarkView = cv::findChessboardCorners(pimage,patternSize,ptsCheckerBoard,cv::CALIB_CB_FAST_CHECK);
	if (landmarkView)
	{
		std::cout << "\n saw landmark \n";
		chessboardSub.shutdown();
	}
	chessboardMutex.unlock();
}

//finds the features in the previous image in the new image and matches the features
void PatchEstimator::match(cv::Mat& image, float dt, Eigen::Vector3f vc, Eigen::Vector3f wc, ros::Time t, Eigen::Vector3f ppcHat, Eigen::Vector4f qpcHat, Eigen::Vector3f pkc, Eigen::Vector4f qkc)
{
	// std::lock_guard<std::mutex> featureMutexGuard(featureMutex);

	clock_t estimatorUpdateTime = clock();
	ROS_WARN("patch %d start",patchInd);

	clock_t timeCheck = clock();

	featureMutex.lock();
	int numberPts = int(depthEstimators.size());
	featureMutex.unlock();
	Eigen::MatrixXf XY1ICL = Eigen::MatrixXf::Ones(numberPts,3);
	Eigen::MatrixXf XY1ICLk = Eigen::MatrixXf::Ones(numberPts,3);
	Eigen::MatrixXf XY1TICL = Eigen::MatrixXf::Ones(3,numberPts);
	Eigen::MatrixXf XY1TICLk = Eigen::MatrixXf::Ones(3,numberPts);
	Eigen::VectorXf NZICL = Eigen::VectorXf::Zero(numberPts);
	Eigen::VectorXf NZICLk = Eigen::VectorXf::Zero(numberPts);
	int itIdx = 0;

	//get the points from the previous image
	std::vector<cv::Point2f> pPts(numberPts),cPts(numberPts),kPts(numberPts);
	std::vector<cv::Point2f> kPtsInPred(numberPts),cPtsInPred(numberPts);//,cPtPsInPred(depthEstimators.size());
	Eigen::Vector3f mpp,mcc;
	std::vector<cv::Point2f>::iterator itp = pPts.begin();
	std::vector<cv::Point2f>::iterator itc = cPts.begin();
	std::vector<cv::Point2f>::iterator itk = kPts.begin();
	float avgNumSaved = 0.0;
	float avgNumThrown = 0.0;

	featureMutex.lock();
	for (std::vector<DepthEstimator*>::iterator itD = depthEstimators.begin() ; itD != depthEstimators.end(); itD++)
	{
		*itk = cv::Point2f((*itD)->ptk(0),(*itD)->ptk(1));
		mpp = (*itD)->current();
	  *itp = cv::Point2f(fx*mpp(0)+cx,fy*mpp(1)+cy);
	  mcc = (*itD)->predict(vc,wc,dt,pkc,qkc);
	  *itc = cv::Point2f(fx*mcc(0)+cx,fy*mcc(1)+cy);
		avgNumSaved += float((*itD)->depthEstimatorICLExt.numSaved);
		avgNumThrown += float((*itD)->depthEstimatorICLExt.numThrown);

		std::cout << " ctix " << (*itc).x << " ctiy " << (*itc).y << std::endl;

		//add each point
		Eigen::Vector3f uip = mpp/mpp.norm();
		Eigen::Vector3f pipICL = uip*((*itD)->dcHatICLExt);
		Eigen::Vector3f mik = (*itD)->mk;
		Eigen::Vector3f uik = mik/mik.norm();
		Eigen::Vector3f pikICL = uik*((*itD)->dkHatICLExt);

		XY1ICL(itIdx,0) = pipICL(0);
		XY1ICL(itIdx,1) = pipICL(1);
		XY1TICL(0,itIdx) = pipICL(0);
		XY1TICL(1,itIdx) = pipICL(1);
		NZICL(itIdx) = -pipICL(2);
		XY1ICLk(itIdx,0) = pikICL(0);
		XY1ICLk(itIdx,1) = pikICL(1);
		XY1TICLk(0,itIdx) = pikICL(0);
		XY1TICLk(1,itIdx) = pikICL(1);
		NZICLk(itIdx) = -pikICL(2);
		itIdx++;

		itk++;
		itp++;
		itc++;
	}
	featureMutex.unlock();
	avgNumSaved /= float(numberPts);
	avgNumThrown /= float(numberPts);

	Eigen::Matrix3f XYXYICL = XY1TICL*XY1ICL;
	Eigen::Vector3f XYNZICL = XY1TICL*NZICL;
	Eigen::Matrix3f XYXYICLk = XY1TICLk*XY1ICLk;
	Eigen::Vector3f XYNZICLk = XY1TICLk*NZICLk;
	float XYXYdet = float(XYXYICL.determinant());
	Eigen::Vector3f np(0.0,0.0,1.0),nk(0.0,0.0,1.0);
	float dp = 1.0;
	float dk = 1.0;
	Eigen::Matrix3f GICLf = Eigen::Matrix3f::Identity();
	cv::Mat GICL(cv::Size(3,3),CV_64F);

	if (XYXYdet > 0.001)
	{
		Eigen::JacobiSVD<Eigen::MatrixXf> svdXYXYICL(XYXYICL, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::Vector3f dnzICL = svdXYXYICL.solve(XYNZICL);
		np = Eigen::Vector3f(dnzICL(0),dnzICL(1),1.0);
		np /= np.norm();
		dp = -dnzICL(2)*np(2);
		Eigen::JacobiSVD<Eigen::MatrixXf> svdXYXYICLk(XYXYICLk, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::Vector3f dnzICLk = svdXYXYICLk.solve(XYNZICLk);
		nk = Eigen::Vector3f(dnzICLk(0),dnzICLk(1),1.0);
		nk /= nk.norm();
		dk = -dnzICLk(2)*nk(2);
		GICLf = camMatf*((ppcHat*np.transpose())/dp + getqRot(qpcHat))*camMatIf;
		GICLf /= GICLf(2,2);
	}

	Eigen::Vector3f nkRot = rotatevec(nk,qkcHat);
	nkRot /= nkRot.norm();
	float nDifAngRot = 180.0/3.1415*acos(float(np.transpose()*nkRot));
	float nDifAng = 180.0/3.1415*acos(float(np.transpose()*nk));
	float qAng = 180.0/3.1415*acos(qkcHat(0))*2.0;

	// std::cout << "\n XY1ICL \n" << XY1ICL << std::endl;
	// std::cout << "\n XY1ICLk \n" << XY1ICLk << std::endl;
	// std::cout << "\n NZICL \n" << NZICL << std::endl;
	// std::cout << "\n NZICLk \n" << NZICLk << std::endl;
	std::cout << "\n dp " << dp << std::endl;
	std::cout << "\n dk " << dk << std::endl;
	std::cout << "\n npx " << np(0) << " npy " << np(1) << " npz " << np(2) << std::endl;
	std::cout << "\n nkx " << nk(0) << " nky " << nk(1) << " nkz " << nk(2) << std::endl;
	std::cout << "\n nkRotx " << nkRot(0) << " nkRoty " << nkRot(1) << " nkRotz " << nkRot(2) << std::endl;
	std::cout << "\n nDifAng " << nDifAng << std::endl;
	std::cout << "\n nDifAngRot " << nDifAngRot << std::endl;
	std::cout << "\n qAng " << qAng << std::endl;
	std::cout << "\n avgNumSaved " << avgNumSaved << std::endl;
	std::cout << "\n avgNumThrown " << avgNumThrown << std::endl;
	std::cout << "\n ppcHat \n" << ppcHat << std::endl;
	std::cout << "\n qpcHat \n" << qpcHat << std::endl;
	std::cout << "\n GICLf \n" << GICLf << std::endl;

	bool normGood = true;
	if (allPtsKnown && (avgNumSaved >= 1))
	{
		normGood = (nDifAngRot < 30.0);
	}

	bool angGood = qAng < 45.0;

	if (normGood && angGood)
	{
		try
		{
			float kT = 0.03/(GTau + 0.03);

			// cv::Mat inliersAffinek;
			// // cv::Mat T = cv::estimateAffine2D(pPts, cPts, inliersAffinek, cv::RANSAC, 4.0, 2000, 0.99, 10);//calculate affine transform using RANSAC
			// // cv::Mat T = cv::estimateAffinePartial2D(pPts, cPts, inliersAffine, cv::RANSAC, 4.0, 2000, 0.99, 10);//calculate affine transform using RANSAC
			// // cv::Mat Tk = cv::estimateAffine2D(kPts, cPts, inliersAffinek, cv::RANSAC, 4.0, 2000, 0.99, 10);//calculate affine transform using RANSAC
			// cv::Mat Gk = cv::findHomography(kPts, cPts, cv::RANSAC, 5.0, inliersAffinek, 2000, 0.99);//calculate homography using RANSAC
			// for (std::vector<cv::Point2f>::iterator itpp = cPts.begin(); itpp != cPts.end(); itpp++)
			// {
			// 	std::cout << "c cptx " << (*itpp).x << " cpty " << (*itpp).y << std::endl;
			// }
			// cv::Mat Gk = cv::findHomography(kPts, pPts, cv::RANSAC, 5.0, inliersAffinek, 2000, 0.99);//calculate homography using RANSAC
			// cv::Mat Gk = cv::findHomography(pPts,kPts,0);//calculate homography using RANSAC

			cv::Mat inliersAffine;
			// for (std::vector<cv::Point2f>::iterator itpp = cPts.begin(); itpp != cPts.end(); itpp++)
			// {
			// 	std::cout << "b cptx " << (*itpp).x << " cpty " << (*itpp).y << std::endl;
			// }
			// cv::Mat G = cv::estimateAffine2D(pPts, cPts, inliersAffine, cv::RANSAC, 4.0, 2000, 0.99, 10);//calculate affine transform using RANSAC
			// cv::Mat G = cv::findHomography(pPts, cPts, cv::RANSAC, 0.05, inliersAffine, 2000, 0.99);//calculate homography using RANSAC
			// cv::Mat G = cv::findHomography(pPts, cPts, cv::RANSAC, 0.05, inliersAffine, 2000, 0.99);//calculate homography using RANSAC
			// cv::Mat G = cv::findHomography(kPts, cPts, cv::RANSAC, 0.05, inliersAffine, 2000, 0.99);//calculate homography using RANSAC
			cv::Mat G = cv::findHomography(pPts, cPts, 0);

			// cv::Mat G = cv::findHomography(pPts, cPts, cv::LMEDS, 4.0, inliersAffine, 2000, 0.99);//calculate homography using RANSAC
			// cv::Mat G = cv::findHomography(pPts, cPts, 0);//calculate homography using RANSAC
			// std::cout << std::endl;
			// for (std::vector<cv::Point2f>::iterator itpp = cPts.begin(); itpp != cPts.end(); itpp++)
			// {
			// 	std::cout << "a cptx " << (*itpp).x << " cpty " << (*itpp).y << std::endl;
			// }

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
				GICL.at<double>(ii/3,ii%3) = GICLf(ii/3,ii%3);
			}

			Gkcum *= G.clone();
			Gkcum /= Gkcum.at<double>(2,2);

			// std::vector<cv::Point2f> ppts;
			// ppts.clear();
			findPoints(image,kPts,pPts,cPts,G);
			kPtsInPred = kPts;
			cPtsInPred = cPts;

			// G = cv::findHomography(pPts, cPtsInPred, cv::RANSAC, 0.05, inliersAffine, 2000, 0.99);//calculate homography using RANSAC
			// for (int ii = 0; ii < 9; ii++)
			// {
			// 	// Gkf(ii/3,ii%3) = Gk.at<double>(ii/3,ii%3);
			// 	Gf(ii/3,ii%3) = G.at<double>(ii/3,ii%3);
			// }
			//
			// if (!Gk.empty())
			// {
			// 	GkfLast += kT*(Gkf - GkfLast);
			// }

			// if (!G.empty())
			// {
			// 	GfLast += kT*(Gf - GfLast);
			// }
			// GfLast *= (1.0/GfLast(2,2));

		}
		catch (cv::Exception e)
		{
			ROS_ERROR("update failed");
			featureMutex.lock();
			for (std::vector<DepthEstimator*>::iterator itD = depthEstimators.begin() ; itD != depthEstimators.end(); itD++)
			{
				delete *itD;
			}

			// cPtsInPred.resize(numPtsPred);
			// ROS_WARN("depthEstimators size after predict1 %d",int(depthEstimators.size()));
			// ROS_WARN("depthEstimatorsInPred size after predict1 %d",int(depthEstimatorsInPred.size()));
			depthEstimators.clear();
			featureMutex.unlock();
			pPts.clear();
			cPts.clear();
			kPts.clear();
		}
	}
	else
	{
		ROS_ERROR("planes off");
		featureMutex.lock();
		for (std::vector<DepthEstimator*>::iterator itD = depthEstimators.begin() ; itD != depthEstimators.end(); itD++)
		{
			delete *itD;
		}
		// cPtsInPred.resize(numPtsPred);
		// ROS_WARN("depthEstimators size after predict1 %d",int(depthEstimators.size()));
		// ROS_WARN("depthEstimatorsInPred size after predict1 %d",int(depthEstimatorsInPred.size()));
		depthEstimators.clear();
		featureMutex.unlock();
		pPts.clear();
		cPts.clear();
		kPts.clear();
	}

	featureMutex.lock();
	// ROS_WARN("keyframe %d patch %d pPts size before flow %d",keyInd,patchInd,int(pPts.size()));
	ROS_WARN("time for getting points %2.4f depthEstimators size after predict %d",float(clock()-estimatorUpdateTime)/CLOCKS_PER_SEC,int(depthEstimators.size()));
	estimatorUpdateTime = clock();

	// find the points using either approximated flow or looking for the board
  //use optical flow to find the features

	if (depthEstimators.size() > minFeaturesBad)
	{
		std::cout << "\n hi7 \n";
		// cv::perspectiveTransform(cPtPsInPred,cPtsInPred,GI);
		estimatorUpdateTime = clock();
		//update the estimtators using the estimted points
		update(image,kPtsInPred,cPtsInPred,vc,wc,t,dt,pkc,qkc);
		ROS_WARN("time for estimator update call %2.4f",float(clock()-estimatorUpdateTime)/CLOCKS_PER_SEC);
		// estimatorUpdateTime = clock();
	}
	featureMutex.unlock();
	cPtsInPred.clear();
	kPtsInPred.clear();
	// cPtPsInPred.clear();
}

void PatchEstimator::findPoints(cv::Mat& image, std::vector<cv::Point2f>& kPts, std::vector<cv::Point2f>& pPts, std::vector<cv::Point2f>& cPts, cv::Mat& G)
{
	bool useKey = false;
	int patchScalar = 1;
	if (pPts.empty())
	{
		pPts = kPts;
		useKey = true;
		patchScalar = 2;
		std::cout << "\n\n\n *********** find points start key *********** \n";
	}
	else
	{
		std::cout << "\n\n\n *********** find points start *********** \n";
	}

	// std::cout << "\n G " << G << std::endl;

	cv::Mat GI;
	cv::invert(G,GI,cv::DECOMP_SVD);
	GI /= GI.at<double>(2,2);

	//correct points and remove the outliers
	std::vector<cv::Point2f> kPtsInPred(depthEstimators.size()),cPtsInPred(depthEstimators.size());//,cPtPsInPred(depthEstimators.size());
	std::vector<DepthEstimator*> depthEstimatorsInPred(depthEstimators.size());
	std::vector<DepthEstimator*>::iterator itDP = depthEstimators.begin();
	std::vector<DepthEstimator*>::iterator itDPPred = depthEstimatorsInPred.begin();
	std::vector<cv::Point2f>::iterator itcPred = cPtsInPred.begin();
	std::vector<cv::Point2f>::iterator itkPred = kPtsInPred.begin();
	std::vector<cv::Point2f>::iterator itc = cPts.begin();
	assert(depthEstimators.size() > 0);
	int patchSize = patchScalar*patchSizeBase;
	int checkSize = patchScalar*checkSizeBase;
	int resultSize = checkSize - patchSize + 1;
	int blurSize = 5;
	cv::Mat ppatch;
	cv::Mat cpatch;
	cv::Mat ppatchBoundP,ppatchBoundC;
	cv::Mat tmresult;
	cv::Mat tmresultBlur;
	double minResultVal, maxResultVal;
	cv::Point minResultPt, maxResultPt;
	int checkSizeMax = checkSize;
	// int Gshift = std::round(fabsf(GfLast(0,2))+fabsf(GfLast(1,2)));

	// ROS_WARN("check 2 %2.5f",float(clock()-timeCheck)/CLOCKS_PER_SEC);
	clock_t timeCheck = clock();

	// *      c00*xi + c01*yi + c02
	// * ui = ---------------------
	// *      c20*xi + c21*yi + c22
	// *
	// *      c10*xi + c11*yi + c12
	// * vi = ---------------------
	// *      c20*xi + c21*yi + c22

	int numPtsPred = 0;
	// int itppIdx = 0;
	bool isFirstPt = true;
	cv::Rect ppatchRectP,ppatchRectC,pcheckRect;
	std::vector<cv::Point2f> ppatchCornersP(4),ppatchCornersC(4),ppatchBoundCornersP(7),ppatchBoundCornersC(7);
	std::vector<cv::Point2f> ppatchPtsP(3),ppatchPtsC(3);
	//first element will be the transformed point itself
	ppatchPtsP.at(1) = cv::Point2f(patchSize,patchSize);//transformed width and height of local patch
	ppatchPtsP.at(2) = cv::Point2f((patchSize-1.0)/2.0,(patchSize-1.0)/2.0);//transformed center of local patch
	cv::Mat ppatchWarp;
	float sizeNormalizer = 0.0;
	float pptx = 0.0;
	float ppty = 0.0;
	float cptx = 0.0;
	float cpty = 0.0;
	float Dtlx = 0.0;
	float Dtly = 0.0;
	std::cout << "\n G " << G << std::endl;


	for (std::vector<cv::Point2f>::iterator itpp = pPts.begin(); itpp != pPts.end(); itpp++)
	{
		try
		{
			std::cout << "\n\n\n --------- next start --------- \n";
			// get center point in new image, find the patch box around it, transform into previous image,
			// check to ensure it fits in previous image
			// if patch does fit, extract the patch then transform extracted patch back and perform search
			// if patch does not fit, skip the point so partition is deleted

			std::cout << "\n hi1 \n";

			//predicted patch center in new image
			pptx = (*itpp).x;
			ppty = (*itpp).y;
			cptx = (*itc).x;
			cpty = (*itc).y;
			// sizeNormalizer = G.at<double>(2,0)*pptx + G.at<double>(2,1)*ppty + G.at<double>(2,2);
			// cptx = (G.at<double>(0,0)*pptx + G.at<double>(0,1)*ppty + G.at<double>(0,2))/sizeNormalizer;
			// cpty = (G.at<double>(1,0)*pptx + G.at<double>(1,1)*ppty + G.at<double>(1,2))/sizeNormalizer;

			//four corners around the predicted center
			ppatchCornersC.at(0) = cv::Point2f(cptx-patchSize/2.0,cpty-patchSize/2.0);//tl
			ppatchCornersC.at(1) = cv::Point2f(cptx+patchSize/2.0,cpty-patchSize/2.0);//tr
			ppatchCornersC.at(2) = cv::Point2f(cptx+patchSize/2.0,cpty+patchSize/2.0);//br
			ppatchCornersC.at(3) = cv::Point2f(cptx-patchSize/2.0,cpty+patchSize/2.0);//bl

			pcheckRect = cv::Rect(cptx-checkSize/2.0,cpty-checkSize/2.0,checkSize,checkSize);

			//transform four corners into previous image and find bounding rectangle
			cv::perspectiveTransform(ppatchCornersC,ppatchCornersP,GI);
			ppatchRectP = cv::boundingRect(ppatchCornersP);

			std::cout << "\n hi2 \n";

			//assign the new corners as the bounding rect corners
			ppatchBoundCornersP.at(0) = cv::Point2f(ppatchRectP.x,ppatchRectP.y);//tl
			ppatchBoundCornersP.at(1) = cv::Point2f(ppatchRectP.x+ppatchRectP.width,ppatchRectP.y);//tr
			ppatchBoundCornersP.at(2) = cv::Point2f(ppatchRectP.x+ppatchRectP.width,ppatchRectP.y+ppatchRectP.height);//br
			ppatchBoundCornersP.at(3) = cv::Point2f(ppatchRectP.x,ppatchRectP.y+ppatchRectP.height);//bl
			ppatchBoundCornersP.at(4) = cv::Point2f(ppatchRectP.x+ppatchRectP.width/2.0,ppatchRectP.y+ppatchRectP.height/2.0);//center in image
			ppatchBoundCornersP.at(5) = cv::Point2f(ppatchRectP.width/2.0,ppatchRectP.height/2.0);//center of subimage
			ppatchBoundCornersP.at(6) = cv::Point2f(pptx-ppatchRectP.x,ppty-ppatchRectP.y);//point in subimage

			bool warpGood = false;
			if ((ppatchBoundCornersP.at(0).x > 0) && (ppatchBoundCornersP.at(0).y > 0)
						&& (ppatchBoundCornersP.at(0).x < (imageWidth)) && (ppatchBoundCornersP.at(0).y < (imageHeight))
						&& (ppatchBoundCornersP.at(1).x > 0) && (ppatchBoundCornersP.at(1).y > 0)
						&& (ppatchBoundCornersP.at(1).x < (imageWidth)) && (ppatchBoundCornersP.at(1).y < (imageHeight))
						&& (ppatchBoundCornersP.at(2).x > 0) && (ppatchBoundCornersP.at(2).y > 0)
						&& (ppatchBoundCornersP.at(2).x < (imageWidth)) && (ppatchBoundCornersP.at(2).y < (imageHeight))
						&& (ppatchBoundCornersP.at(3).x > 0) && (ppatchBoundCornersP.at(3).y > 0)
						&& (ppatchBoundCornersP.at(3).x < (imageWidth)) && (ppatchBoundCornersP.at(3).y < (imageHeight)))
			{
				if (useKey)
				{
					ppatchBoundP = kimage(ppatchRectP).clone();
				}
				else
				{
					ppatchBoundP = pimage(ppatchRectP).clone();
				}

				std::cout << "\n hi3 \n";

				//extract and transform bounding patch
				cv::warpPerspective(ppatchBoundP,ppatchBoundC,G,ppatchBoundP.size(),cv::INTER_LINEAR,cv::BORDER_CONSTANT,cv::Scalar(255,255,255));

				//transform the bounding corners back to remove the excess of bounding image
				cv::perspectiveTransform(ppatchBoundCornersP,ppatchBoundCornersC,G);

				//relative position of the center of the patch to the center of the transformed subimage
				float cDx = ppatchBoundCornersC.at(6).x;
				float cDy = ppatchBoundCornersC.at(6).y;

				//check if the corners will be inside subimage, if not find the largest that will fit with correct center
				float patchSizeWidthC = patchSize;
				float patchSizeHeightC = patchSize;
				if (int(cDx-patchSizeWidthC/2.0) <= 0)
				{
					patchSizeWidthC -= 2.0*fabsf(cDx-patchSizeWidthC/2.0);
				}
				if (int(cDx+patchSizeWidthC/2.0) >= ppatchBoundC.cols)
				{
					patchSizeWidthC -= 2.0*fabsf((cDx+patchSizeWidthC/2.0) - ppatchBoundC.cols);
				}
				if (int(cDy-patchSizeHeightC/2.0) <= 0.0)
				{
					patchSizeHeightC -= 2.0*fabsf(cDy-patchSizeHeightC/2.0);
				}
				if (int(cDy+patchSizeHeightC/2.0) >= ppatchBoundC.rows)
				{
					patchSizeHeightC -= 2.0*fabsf((cDy+patchSizeHeightC/2.0)-ppatchBoundC.rows);
				}

				int patchSizeC = std::min(patchSizeWidthC,patchSizeHeightC);
				ppatchRectC.x = int(cDx-patchSizeC/2.0);
				ppatchRectC.y = int(cDy-patchSizeC/2.0);
				ppatchRectC.width = patchSizeC;
				ppatchRectC.height = patchSizeC;

				std::cout << "\n ppatchRectC " << ppatchRectC << std::endl;
				std::cout << "\n cDx " << cDx << " cDy " << cDy << " patchSize " << patchSize << "  ppatchBoundC.cols " <<  ppatchBoundC.cols << "  ppatchBoundC.rows " <<  ppatchBoundC.rows << std::endl;


				if ((ppatchRectC.x >= 0) && (ppatchRectC.y >= 0) && (ppatchRectC.x < ppatchBoundC.cols) && (ppatchRectC.y < ppatchBoundC.rows)
						&& ((ppatchRectC.x+ppatchRectC.width) < (ppatchBoundC.cols)) && ((ppatchRectC.y+ppatchRectC.height) < (ppatchBoundC.rows))
					  && (ppatchRectC.width > 0) && (ppatchRectC.height > 0))
				{
					warpGood = true;
				}
			}

			//if the bounding rectangle points are within the previous image
			if ((acos(qkcHat(0))*2.0 < 45.0*3.1415/180.0) && warpGood
					&& (pcheckRect.x > 0) && (pcheckRect.y > 0)
					&& ((pcheckRect.x+checkSize) < (imageWidth)) && ((pcheckRect.y+checkSize) < (imageHeight)))
			{
				std::cout << "\n hi33 \n";

				ppatch = ppatchBoundC(ppatchRectC).clone();

				// ppatch = ppatchBoundC.clone();

				std::cout << "\n hi4 \n";

				//extract the check image
				cpatch = image(pcheckRect).clone();

				std::cout << "\n hi44 \n";

				cv::Mat pSobel,cSobel;
				// cv::Sobel(ppatch,pSobel,CV_32F,1,1,3);

				/// Generate grad_x and grad_y
				cv::Mat pgrad_x, pgrad_y, cgrad_x, cgrad_y;
				cv::Mat pabs_grad_x, pabs_grad_y, cabs_grad_x, cabs_grad_y;
				/// Gradient X
				//Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
				cv::Sobel( ppatch, pgrad_x, CV_32F, 1, 0, 3);
				cv::Sobel( ppatch, pgrad_y, CV_32F, 0, 1, 3);
				cv::convertScaleAbs( pgrad_x, pabs_grad_x );
				cv::convertScaleAbs( pgrad_y, pabs_grad_y );
				cv::addWeighted( pabs_grad_x, 0.5, pabs_grad_y, 0.5, 0, pSobel );
				cv::Sobel( cpatch, cgrad_x, CV_32F, 1, 0, 3);
				cv::Sobel( cpatch, cgrad_y, CV_32F, 0, 1, 3);
				cv::convertScaleAbs( cgrad_x, cabs_grad_x );
				cv::convertScaleAbs( cgrad_y, cabs_grad_y );
				cv::addWeighted( cabs_grad_x, 0.5, cabs_grad_y, 0.5, 0, cSobel );
				/// Gradient Y
				//Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );

				cv::Mat pCompare,cCompare;
				cv::addWeighted( ppatch, 0.75, pSobel, 0.25, 0, pCompare);
				cv::addWeighted( cpatch, 0.75, cSobel, 0.25, 0, cCompare);

				/// Total Gradient (approximate)

				// std::cout << "\n ppatchRectCLocal \n" << ppatchRectCLocal << std::endl;



				std::cout << "\n cpatch.size() " << cpatch.size() << " ppatch.size() " << ppatch.size() << std::endl;

				//use the patch as template and match in check patch
				// cv::matchTemplate(cpatch,ppatch,tmresult,cv::TM_SQDIFF_NORMED);
				// cv::matchTemplate(cpatch,reducedWarp,tmresult,cv::TM_CCORR_NORMED);
				cv::matchTemplate(cCompare,pCompare,tmresult,cv::TM_CCOEFF_NORMED);

				std::cout << "\n tmresult.size() " << tmresult.size() << std::endl;

				cv::GaussianBlur(tmresult,tmresultBlur,cv::Size(blurSize,blurSize),0);
				cv::minMaxLoc(tmresult,&minResultVal,&maxResultVal,&minResultPt,&maxResultPt);

				std::cout << "\n hi5 \n";

				// if (isFirstPt)
				// {
				// 	// cv_bridge::CvImage out_msg;
				// 	// out_msg.header.stamp = t; // Same timestamp and tf frame as input image
				// 	// out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
				// 	// {
				// 	// 	cv::circle(cpatch,cv::Point2f((ppatchRectCLocal.width-1)/2.0+maxResultPt.x,(ppatchRectCLocal.height-1)/2.0+maxResultPt.y),5,cv::Scalar(255,255,255),-1);
				// 	// 	out_msg.image = cpatch; // Your cv::Mat
				// 	// 	std::lock_guard<std::mutex> pubMutexGuard(pubMutex);
				// 	// 	imagePub.publish(out_msg.toImageMsg());
				// 	// }
				// 	cv_bridge::CvImage out_msg2;
				// 	out_msg2.header.stamp = t; // Same timestamp and tf frame as input image
				// 	out_msg2.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
				// 	// cv::Rect blendCenter(std::round(maxResultPt.x),
				// 	//                      std::round(maxResultPt.y),
				// 	// 										 ppatchRectCLocal.width,ppatchRectCLocal.height);
				// 	cv::Mat reducedWarpBlend;
				// 	cv::addWeighted(reducedWarp,0.6,reducedWarpSobel,0.4,20.0,reducedWarpBlend);
				// 	out_msg2.image = reducedWarpBlend; // Your cv::Mat
				// 	{
				// 		std::lock_guard<std::mutex> pubMutexGuard(pubMutex);
				// 		imagePub2.publish(out_msg2.toImageMsg());
				// 	}
				// 	isFirstPt = false;
				// }

				// std::cout << "\n ppatch \n" << ppatch << std::endl;
				// std::cout << "\n cpatch \n" << cpatch << std::endl;
				// std::cout << "\n tmresult \n" << tmresult << std::endl;
				// std::cout << "\n tmresultBlur \n" << tmresultBlur << std::endl;
				// std::cout << "\n minResultVal \n" << minResultVal << std::endl;
				// std::cout << "\n minResultPt \n" << minResultPt << std::endl;
				// std::cout << "\n maxResultVal \n" << maxResultVal << std::endl;
				// std::cout << "\n fitx " << (ppatchRectCLocal.width-1)/2.0+maxResultPt.x
				//           << "\n fity " << (ppatchRectCLocal.height-1)/2.0+maxResultPt.y << std::endl;

				//get the points
				*itDPPred = *itDP;

				// std::cout << "\n maxx " << maxResultPt.x << " maxy " << maxResultPt.y << std::endl;
				// std::cout << "\n maxResultValx " << maxResultVal << std::endl;
				//
				// std::cout << "\n mx " << pcheckRect.x+ppatchRectC.width/2.0+maxResultPt.x << " my " << pcheckRect.y+ppatchRectC.height/2.0+maxResultPt.y << std::endl;

				// *itcPred = cv::Point2f(pcheckRect.x+(ppatchRectC.width)/2.0+maxResultPt.x,pcheckRect.y+ppatchRectC.height/2.0+maxResultPt.y);

				std::cout << "\n maxx " << maxResultPt.x << " maxy " << maxResultPt.y << std::endl;
				std::cout << "\n pptx " << pptx << " ppty " << ppty << std::endl;
				std::cout << "\n cptx " << cptx << " cpty " << cpty << std::endl;
				std::cout << "\n (*itc).x " << (*itc).x << " (*itc).y " << (*itc).y << std::endl;

				// std::cout << "\n maxResultValx " << maxResultVal << std::endl;
				//
				std::cout << "\n mx " << pcheckRect.x+ppatchRectC.width/2.0+maxResultPt.x << " my " << pcheckRect.y+ppatchRectC.height/2.0+maxResultPt.y << std::endl;


				float alphapr = 0.9;
				// float avgx = (1.0-alphapr)*cptx+alphapr*(pcheckRect.x+ppatchRectC.width/2.0+maxResultPt.x);
				// float avgy = (1.0-alphapr)*cpty+alphapr*(pcheckRect.y+ppatchRectC.height/2.0+maxResultPt.y);
				float avgx = (1.0-alphapr)*cptx+alphapr*(cptx-checkSize/2.0+ppatchRectC.width/2.0+maxResultPt.x);
				float avgy = (1.0-alphapr)*cpty+alphapr*(cpty-checkSize/2.0+ppatchRectC.width/2.0+maxResultPt.y);
				// if (ppatchRectC.width%2 == 0)
				// {
				// 	avgx += alphapr;
				// 	avgy += alphapr;
				// 	// *itcPred = cv::Point2f(std::round(pcheckRect.x+ppatchRectC.width/2.0+maxResultPt.x),std::round(pcheckRect.y+ppatchRectC.height/2.0+maxResultPt.y));
				// }
				// else
				// {
				// 	*itcPred = cv::Point2f(std::round(pcheckRect.x+ppatchRectC.width/2.0+maxResultPt.x+1.0),std::round(pcheckRect.y+ppatchRectC.height/2.0+maxResultPt.y+1.0));
				// }
				*itcPred = cv::Point2f(avgx,avgy);




				std::cout << "\n pcheckRect.x " << pcheckRect.x << " pcheckRect.y " << pcheckRect.y << std::endl;
				std::cout << "\n ppatchRectC.width/2.0 " << ppatchRectC.width/2.0 << " ppatchRectC.width/2.0 " << ppatchRectC.width/2.0 << std::endl;
				std::cout << "\n (*itcPred).x " << (*itcPred).x << " (*itcPred).y " << (*itcPred).y << std::endl;

				// *itcPred = cv::Point2f(std::round(ppatchRectC.x)+(ppatchRectC.width-1)/2-(checkSize-1)/2+(patchSize-1)/2+maxResultPt.x,std::round(ppatchRectC.y)+(ppatchRectC.height-1)/2-(checkSize-1)/2+(patchSize-1)/2+maxResultPt.y);
				// *itcPPred = cv::Point2f(std::round((*itpp).x)-(checkSize-1)/2+(patchSize-1)/2+maxResultPt.x,std::round((*itpp).y)-(checkSize-1)/2+(patchSize-1)/2+maxResultPt.y);
				// *itcPPred = cv::Point2f(std::round(pPtsRectCPtsP.at(0).x+pPtsRecSearchtlDiff.x+itppIdx*checkSize+(patchSize-1)/2+maxResultPt.x),std::round(pPtsRectCPtsP.at(0).y+pPtsRecSearchtlDiff.y+itppIdx*checkSize+(patchSize-1)/2+maxResultPt.y));
				*itkPred = cv::Point2f((*itDP)->ptk(0),(*itDP)->ptk(1));

				itDPPred++;
				itcPred++;
				itkPred++;
				numPtsPred++;
				// itppIdx++;
				// ROS_WARN("check 7 %2.5f",float(clock()-timeCheck)/CLOCKS_PER_SEC);
				// timeCheck = clock();
				std::cout << "\n --------- next end --------- \n\n\n";
			}
			else
			{
				delete *itDP;
				if (G.empty())
				{
					std::cout << "\n G empty \n";
				}
			}
		}
		catch (cv::Exception e)
		{
			delete *itDP;
			std::cout << "\n patch failed \n";
		}

		itDP++;
		itc++;
	}

	depthEstimatorsInPred.resize(numPtsPred);
	cPtsInPred.resize(numPtsPred);
	kPtsInPred.resize(numPtsPred);
	// cPtsInPred.resize(numPtsPred);
	// ROS_WARN("depthEstimators size after predict1 %d",int(depthEstimators.size()));
	// ROS_WARN("depthEstimatorsInPred size after predict1 %d",int(depthEstimatorsInPred.size()));
	depthEstimators = depthEstimatorsInPred;
	depthEstimatorsInPred.clear();
	// cv::cornerSubPix(image,cPtsInPred,cv::Size(3,3),cv::Size(-1,-1),cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
	cPts = cPtsInPred;
	kPts = kPtsInPred;
	cPtsInPred.clear();
	kPtsInPred.clear();

	if (useKey)
	{
		std::cout << "\n *********** find points stop key *********** \n\n\n";
	}
	else
	{
		std::cout << "\n *********** find points stop *********** \n\n\n";
	}
}

void PatchEstimator::update(cv::Mat& image, std::vector<cv::Point2f>& kPts, std::vector<cv::Point2f>& cPts, Eigen::Vector3f vc, Eigen::Vector3f wc, ros::Time t, float dt, Eigen::Vector3f pkc, Eigen::Vector4f qkc)
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

		Eigen::Vector4f qkcPnP;
		Eigen::Vector3f tkcPnP;
		bool usePnP = false;
		if (allPtsKnown)
		{
			std::cout << "\n tryingPnP \n" << std::endl;
			std::vector<cv::Point3f> kPts3(depthEstimators.size());
			std::vector<cv::Point3f>::iterator kPts3it = kPts3.begin();
			for (std::vector<DepthEstimator*>::iterator itD = depthEstimators.begin() ; itD != depthEstimators.end(); itD++)
			{
				float dkHati = (*itD)->dkHatICLExt;
				(*kPts3it).x = (*itD)->uk(0)*dkHati;
				(*kPts3it).y = (*itD)->uk(1)*dkHati;
				(*kPts3it).z = (*itD)->uk(2)*dkHati;
				kPts3it++;
			}

			Eigen::Matrix3f RkcHatPnPf = getqRot(qkc);
			cv::Mat RkcHatPnP;
			cv::eigen2cv(RkcHatPnPf,RkcHatPnP);
			cv::Mat rvec(cv::Size(1,3),CV_32F);
			cv::Mat tvec(cv::Size(1,3),CV_32F);
			cv::Rodrigues(RkcHatPnP,rvec);
			cv::eigen2cv(pkc,tvec);
			std::cout << "rvecPnPb \n" << rvec << std::endl;
			std::cout << "tvecPnPb \n" << tvec << std::endl;
			usePnP = cv::solvePnP(kPts3,cPts,camMat,cv::Mat(),rvec,tvec,true,cv::SOLVEPNP_ITERATIVE);
			// usePnP = cv::solvePnPRansac(kPts3,cPts,camMat,cv::Mat(),rvec,tvec,true,1000,1.0,0.99,cv::Mat(),cv::SOLVEPNP_ITERATIVE);
			std::cout << "rvecPnPa \n" << rvec << std::endl;
			std::cout << "tvecPnPa \n" << tvec << std::endl;


			if (usePnP)
			{
				cv::Rodrigues(rvec,RkcHatPnP);
				cv::cv2eigen(RkcHatPnP,RkcHatPnPf);
				Eigen::Quaternionf qkcPnPq(RkcHatPnPf);// convert to quaternion
				qkcPnP << qkcPnPq.w(),qkcPnPq.x(),qkcPnPq.y(),qkcPnPq.z();
				cv::cv2eigen(tvec,tkcPnP);
				float tkcPnPNorm = tkcPnP.norm();
				if (tkcPnPNorm>0.001)
				{
					tkcPnP /= tkcPnPNorm;
				}
				std::cout << std::endl;
				std::cout << "dkcPnP \n" << tkcPnPNorm << std::endl;
				std::cout << "tkcPnP \n" << tkcPnP << std::endl;
				std::cout << "qkcPnP \n" << qkcPnP << std::endl;
			}
		}
		else
		{
			std::cout << "\n tryingPnP current \n" << std::endl;
			std::vector<cv::Point3f> cPts3(depthEstimators.size());
			std::vector<cv::Point3f>::iterator cPts3it = cPts3.begin();
			std::vector<cv::Point2f>::iterator cPtsit = cPts.begin();
			Eigen::Vector3f uci;
			for (std::vector<DepthEstimator*>::iterator itD = depthEstimators.begin() ; itD != depthEstimators.end(); itD++)
			{
				uci = Eigen::Vector3f(((*cPtsit).x-cx)/fx,((*cPtsit).y-cy)/fy,1.0);
				uci /= uci.norm();
				float dcHati = (*itD)->dcHatICLExt;
				(*cPts3it).x = uci(0)*dcHati;
				(*cPts3it).y = uci(1)*dcHati;
				(*cPts3it).z = uci(2)*dcHati;
				cPts3it++;
				cPtsit++;
			}

			Eigen::Matrix3f RckHatPnPf = getqRot(getqInv(qkc));
			cv::Mat RckHatPnP;
			cv::eigen2cv(RckHatPnPf,RckHatPnP);
			cv::Mat rvec(cv::Size(1,3),CV_32F);
			cv::Mat tvec(cv::Size(1,3),CV_32F);
			cv::Rodrigues(RckHatPnP,rvec);
			Eigen::Vector3f pck = -rotatevec(pkc,getqInv(qkc));
			cv::eigen2cv(pck,tvec);
			std::cout << "rvecPnPcb \n" << rvec << std::endl;
			std::cout << "tvecPnPcb \n" << tvec << std::endl;
			usePnP = cv::solvePnP(cPts3,kPts,camMat,cv::Mat(),rvec,tvec,true,cv::SOLVEPNP_ITERATIVE);
			// usePnP = cv::solvePnPRansac(kPts3,cPts,camMat,cv::Mat(),rvec,tvec,true,1000,1.0,0.99,cv::Mat(),cv::SOLVEPNP_ITERATIVE);
			std::cout << "rvecPnPca \n" << rvec << std::endl;
			std::cout << "tvecPnPca \n" << tvec << std::endl;


			if (usePnP)
			{
				cv::Rodrigues(rvec,RckHatPnP);
				cv::cv2eigen(RckHatPnP,RckHatPnPf);
				Eigen::Quaternionf qckPnPq(RckHatPnPf);// convert to quaternion
				Eigen::Vector4f qckPnP;
				qckPnP << qckPnPq.w(),qckPnPq.x(),qckPnPq.y(),qckPnPq.z();
				qkcPnP = getqInv(qckPnP);
				qkcPnP /= qkcPnP.norm();
				Eigen::Vector3f tckPnP;
				cv::cv2eigen(tvec,tckPnP);
				tkcPnP = -rotatevec(tckPnP,qkcPnP);
				float tkcPnPNorm = tkcPnP.norm();
				if (tkcPnPNorm>0.001)
				{
					tkcPnP /= tkcPnPNorm;
				}
				std::cout << std::endl;
				std::cout << "dkcPnP \n" << tkcPnPNorm << std::endl;
				std::cout << "tkcPnP \n" << tkcPnP << std::endl;
				std::cout << "qkcPnP \n" << qkcPnP << std::endl;
			}
		}

		// cv::Mat inliersG;
		// cv::Mat G = cv::findHomography(kPts, cPts, cv::RANSAC, 0.05, inliersG1, 2000, 0.99);//calculate homography using RANSAC
		cv::Mat G = cv::findHomography(kPts, cPts, 0);//calculate homography using RANSAC
		// cv::Mat F = cv::findFundamentalMat(kPts, cPts,cv::FM_RANSAC,0.05,0.99);
		cv::Mat F = cv::findFundamentalMat(kPts, cPts,cv::FM_8POINT);
		cv::Mat E = camMatD.t()*F*camMatD;
		cv::Mat R1,R2,tt;
		cv::decomposeEssentialMat(E,R1,R2,tt);

		// std::cout << "\n G \n" << G << std::endl << std::endl;

		float kT = 0.03/(GTau + 0.03);
		Eigen::Matrix<float,3,3> Gf;
		for (int ii = 0; ii < 9; ii++)
		{
			Gf(ii/3,ii%3) = G.at<double>(ii/3,ii%3);
		}

		if (!G.empty())
		{
			GkfLast += kT*(Gf - GkfLast);
		}

		GkfLast *= (1.0/GkfLast(2,2));

		for (int ii = 0; ii < 9; ii++)
		{
			G.at<double>(ii/3,ii%3) = GkfLast(ii/3,ii%3);
		}

		ROS_WARN("time for homog %2.4f",float(clock()-updateClock)/CLOCKS_PER_SEC);
		updateClock = clock();

		// estimate the homography
		Eigen::Vector4f qkcEst = qkcHat;
		// Eigen::Vector3f nk(0.0,0.0,1.0);
		Eigen::Vector3f tkcEst = tkcHat;

		if (!G.empty())
		{
			try
			{
				//find the solutions
				std::vector<cv::Mat> RkcH,tkcH,nkH,RkcHcum,tkcHcum,nkHcum;//holds the rotations, translations and normal vetors from decompose homography
				int numberHomogSols = cv::decomposeHomographyMat(G, camMatD, RkcH, tkcH, nkH);// Decompose homography
				int numberHomogSolscum = cv::decomposeHomographyMat(Gkcum, camMatD, RkcHcum, tkcHcum, nkHcum);// Decompose homography

				//check positive depth constraint on the inliers to find the solutions
				// std::vector<Eigen::Vector3f> nks;
				std::vector<Eigen::Vector3f> tkcs;
				std::vector<Eigen::Vector4f> qkcs;
				std::vector<float> errors;

				if (usePnP)
				{
					if ((qkc + qkcPnP).norm() < (qkc - qkcPnP).norm())
					{
						qkcPnP *= -1.0;
					}

					qkcs.push_back(qkcPnP);
					if ((tkcPnP.norm() > 0.001) && (pkc.norm() > 0.001))
					{
						errors.push_back((qkc - qkcPnP).norm()+(pkc/pkc.norm()-tkcPnP).norm());
						tkcs.push_back(tkcPnP);
					}
					else
					{
						errors.push_back((qkc - qkcPnP).norm());
						tkcs.push_back(Eigen::Vector3f::Zero());
					}
				}

				if (!F.empty())
				{
					Eigen::Matrix3f Rkc1,Rkc2;
					for (int hh = 0; hh < 9; hh++)
					{
						Rkc1(hh/3,hh%3) = R1.at<double>(hh/3,hh%3);
						Rkc2(hh/3,hh%3) = R2.at<double>(hh/3,hh%3);
					}
					Eigen::Quaternionf qkcq1(Rkc1);// convert to quaternion
					Eigen::Quaternionf qkcq2(Rkc2);// convert to quaternion
					Eigen::Vector4f qkc1(qkcq1.w(),qkcq1.x(),qkcq1.y(),qkcq1.z());
					Eigen::Vector4f qkc2(qkcq2.w(),qkcq2.x(),qkcq2.y(),qkcq2.z());
					qkc1 /= qkc1.norm();
					qkc2 /= qkc2.norm();
					Eigen::Vector3f tkct(tt.at<double>(0,0),tt.at<double>(1,0),tt.at<double>(2,0));

					if ((qkc + qkc1).norm() < (qkc - qkc1).norm())
					{
						qkc1 *= -1.0;
					}

					if ((qkc + qkc2).norm() < (qkc - qkc2).norm())
					{
						qkc2 *= -1.0;
					}

					qkcs.push_back(qkc1);
					qkcs.push_back(qkc2);
					if ((tkct.norm() > 0.001) && (pkc.norm() > 0.001))
					{
						errors.push_back((qkc - qkc1).norm()+(pkc/pkc.norm()-tkct/tkct.norm()).norm());
						errors.push_back((qkc - qkc2).norm()+(pkc/pkc.norm()-tkct/tkct.norm()).norm());
						tkcs.push_back(tkct/tkct.norm());
						tkcs.push_back(tkct/tkct.norm());
					}
					else
					{
						errors.push_back((qkc - qkc1).norm());
						errors.push_back((qkc - qkc2).norm());
						tkcs.push_back(Eigen::Vector3f::Zero());
						tkcs.push_back(Eigen::Vector3f::Zero());
					}
				}

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

						if ((qkc + qkcj).norm() < (qkc - qkcj).norm())
						{
							qkcj *= -1.0;
						}

						// std::cout << "\n qkcjw " << qkcj(0) << " qkcjx " << qkcj(1) << " qkcjy " << qkcj(2) << " qkcjz " << qkcj(3) << std::endl;
						// std::cout << "\n tkcjx " << tkcj(0) << " tkcjy " << tkcj(1) << " tkcjz " << tkcj(2) << std::endl;
						// std::cout << "\n nkjx " << nkj(0) << " nkjy " << nkj(1) << " nkjz " << nkj(2) << std::endl;

						// nks.push_back(nkj);

						qkcs.push_back(qkcj);
						if ((tkcj.norm() > 0.001) && (pkc.norm() > 0.001))
						{
							errors.push_back((qkc - qkcj).norm()+(pkc/pkc.norm()-tkcj/tkcj.norm()).norm());
							tkcs.push_back(tkcj/tkcj.norm());
						}
						else
						{
							errors.push_back((qkc - qkcj).norm());
							tkcs.push_back(Eigen::Vector3f::Zero());
						}
					}
				}

				// for (int jj = 0; jj < numberHomogSolscum; jj++)
				// {
				// 	//convert normal to eigen
				// 	Eigen::Vector3f nkj(nkHcum.at(jj).at<double>(0,0),nkHcum.at(jj).at<double>(1,0),nkHcum.at(jj).at<double>(2,0));
				//
				// 	// std::cout << "\n\n sol " << jj << std::endl;
				// 	// std::cout << "\n nkjx " << nkj(0) << " nkjy " << nkj(1) << " nkjz " << nkj(2) << std::endl;
				//
				// 	if (nkj.norm() < 0.1)
				// 	{
				// 		nkj(0) = 0.0;
				// 		nkj(1) = 0.0;
				// 		nkj(2) = 1.0;
				// 	}
				//
				// 	//if n^T*[0;0;1] > then solution in front of camera
				// 	Eigen::Vector3f tkcj(tkcHcum.at(jj).at<double>(0,0),tkcHcum.at(jj).at<double>(1,0),tkcHcum.at(jj).at<double>(2,0));
				// 	// std::cout << "\n tkcjx " << tkcj(0) << " tkcjy " << tkcj(1) << " tkcjz " << tkcj(2) << std::endl;
				// 	if ((nkj(2) >= 0.0))
				// 	{
				// 		Eigen::Matrix3f Rkcj;
				// 		for (int hh = 0; hh < 9; hh++)
				// 		{
				// 			Rkcj(hh/3,hh%3) = RkcHcum.at(jj).at<double>(hh/3,hh%3);
				// 		}
				// 		Eigen::Quaternionf qkcjq(Rkcj);// convert to quaternion
				// 		Eigen::Vector4f qkcj(qkcjq.w(),qkcjq.x(),qkcjq.y(),qkcjq.z());
				// 		qkcj /= qkcj.norm();
				//
				// 		if ((qkcHat + qkcj).norm() < (qkcHat - qkcj).norm())
				// 		{
				// 			qkcj *= -1.0;
				// 		}
				//
				// 		// std::cout << "\n qkcjw " << qkcj(0) << " qkcjx " << qkcj(1) << " qkcjy " << qkcj(2) << " qkcjz " << qkcj(3) << std::endl;
				// 		// std::cout << "\n tkcjx " << tkcj(0) << " tkcjy " << tkcj(1) << " tkcjz " << tkcj(2) << std::endl;
				// 		// std::cout << "\n nkjx " << nkj(0) << " nkjy " << nkj(1) << " nkjz " << nkj(2) << std::endl;
				//
				// 		// nks.push_back(nkj);
				//
				// 		qkcs.push_back(qkcj);
				// 		if ((tkcj.norm() > 0.001) && (pkcHat.norm() > 0.001))
				// 		{
				// 			errors.push_back((qkcHat - qkcj).norm()+(pkcHat/pkcHat.norm()-tkcj/tkcj.norm()).norm());
				// 			tkcs.push_back(tkcj/tkcj.norm());
				// 		}
				// 		else
				// 		{
				// 			errors.push_back((qkcHat - qkcj).norm());
				// 			tkcs.push_back(Eigen::Vector3f::Zero());
				// 		}
				// 	}
				// }

				// ROS_WARN("keyframe %d patch %d errors size %d",keyInd,patchInd,int(errors.size()));

				int minqkcsErrorInd = std::distance(errors.begin(),std::min_element(errors.begin(),errors.end()));
				qkcEst = qkcs.at(minqkcsErrorInd);
				// nk = nks.at(minqkcsErrorInd);
				tkcEst = tkcs.at(minqkcsErrorInd);

				// ROS_WARN("keyframe %d selected best as %d",keyInd,minqkcsErrorInd);
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

		// qkcHat += kq*(qkc - qkcHat);
		// qkcHat /= qkcHat.norm();

		std::cout << "\n qkc \n" << qkc << std::endl;
		std::cout << "\n tkc \n" << pkc/pkc.norm() << std::endl;
		std::cout << "\n qkcEst \n" << qkcEst << std::endl;
		std::cout << "\n tkcEst \n" << tkcEst << std::endl;

		//use kinematric constraints, can only rotate about z, can only translate camera in xy plane
		// qkc(1) = 0.0;
		// qkc(3) = 0.0;
		qkcEst /= qkcEst.norm();

		// Eigen::Vector4f qkw(keyOdom.pose.pose.orientation.w,keyOdom.pose.pose.orientation.x,keyOdom.pose.pose.orientation.y,keyOdom.pose.pose.orientation.z);
		// Eigen::Vector4f qck = getqInv(qkc);
		// Eigen::Vector4f qbwm = getqMat(getqMat(qkw)*getqInv(qkc))*getqInv(qcb);
		// qbwm /= qbwm.norm();
		// qbwm(1) = 0.0;
		// qbwm(2) = 0.0;
		// qbwm /= qbwm.norm();
		// Eigen::Vector4f qckm = getqMat(getqMat(getqInv(qkw))*qbwm)*qcb;
		// qckm /= qckm.norm();
		// qkc = getqInv(qckm);


		// Eigen::Vector4f qbk = getqMat(getqMat(qcb)*qck)*;
		// qbk /= qbk.norm();
		// qbk(1) = 0.0;
		// qbk(2) = 0.0;
		// qbk /= qbk.norm();
		// qck = getqMat(getqMat(getqInv(qcb))*qbk)*qcb;
		// qck /= qck.norm();
		// qkc = getqInv(qck);

		if (tkcEst.norm() > 0.001)
		{
			// Eigen::Vector3f tckm = rotatevec(rotatevec(-tkc,qckm),qkw);
			// tkc(1) = 0.0;
			if (tkcEst.norm() > 0.001)
			{
				tkcEst /= tkcEst.norm();
				// Eigen::Vector3f tkct = -rotatevec(rotatevec(tckm,getqInv(qkw)),qkc);
				// tkc = tkct/tkct.norm();
			}
		}

		Eigen::Matrix<float,8,1> xHatq = qkcDotEstimator.update(qkcEst,t);
		qkcHat = xHatq.segment(0,4);
		// qkcHat(1) = 0.0;
		// qkcHat(3) = 0.0;
		qkcHat /= qkcHat.norm();
		Eigen::Vector4f qkcHatDot = xHatq.segment(4,4);
		Eigen::Vector3f wcb = -2.0*B(qkcHat).transpose()*qkcHatDot - wc;
		wcbHat += 0.25*(wcb-wcbHat);

		Eigen::Matrix3f RkcHat = getqRot(qkcHat);

		Eigen::Matrix<float,6,1> xHatt = tkcDotEstimator.update(tkcEst,t);
	  tkcHat = xHatt.segment(0,3);

		if (tkcHat.norm() > 0.001)
		{
			// tkcHat(1) = 0.0;
			if (tkcHat.norm() > 0.001)
			{
				tkcHat /= tkcHat.norm();
			}
		}
	  // Eigen::Vector3f ucDot = xHatt.segment(3,3);

		// tkcHat += kt*(tkc - tkcHat);

		// float alphank = 0.75;
		// nk = (alphank*nk + (1.0-alphank)*(Eigen::Vector3f(0.0,0.0,1.0)));
		// nk /= nk.norm();
		// nkHat += kn*(nk - nkHat);
		// nkHat /= nkHat.norm();

		ROS_WARN("time for update tnq %2.4f",float(clock()-updateClock)/CLOCKS_PER_SEC);
		updateClock = clock();

		// Eigen::RowVector3f nkHatT = nkHat.transpose();
		// Eigen::Matrix3f H = RkcHat+tkcHat*nkHatT;
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
		// std::vector<cv::Point2f> kPtsC(kPts.size());
		// if (!G.empty())
		// {
		// 	cv::perspectiveTransform(kPts,kPtsC,G);
		// }
		// std::vector<cv::Point2f>::iterator itkc = kPtsC.begin();
		float betakc = 0.0;
		// cv::MatIterator_<uchar> itIn = inliersG.begin<uchar>();
		float avgcPtx = 0.0;
		float avgcPty = 0.0;
		bool allPtsKnownIn = true;
		for (std::vector<DepthEstimator*>::iterator itD = depthEstimators.begin() ; itD != depthEstimators.end(); itD++)
		{
			// if (!G.empty())
			// {
			// 	(*itc).x = betakc*(*itkc).x + (1.0-betakc)*(*itc).x;
			// 	(*itc).y = betakc*(*itkc).y + (1.0-betakc)*(*itc).y;
			// }
			float dkcHati = (*itD)->update(Eigen::Vector3f(((*itc).x-cx)/fx,((*itc).y-cy)/fy,1.0),tkcHat,RkcHat,vc,wc,t,pkc,qkc);
			dkcs.push_back(dkcHati);
			depthEstimatorsInHomog.push_back(*itD);
			// if (bool(*itIn))
			// {
			// 	inlierRatio += 1.0;
			// }
			// inlierRatio += 1.0;

			allPtsKnownIn = allPtsKnownIn&&(*itD)->depthEstimatorICLExt.dkKnown;

			avgcPtx += (*itc).x;
			avgcPty += (*itc).y;

			// itIn++;
			itc++;
			// itkc++;
			// std::cout << "\n ii " << ii << " stop\n";
		}
		allPtsKnown = allPtsKnownIn;
		depthEstimators = depthEstimatorsInHomog;
		depthEstimatorsInHomog.clear();
		// inlierRatio /= depthEstimators.size();
		avgcPtx /= float(depthEstimators.size());
		avgcPty /= float(depthEstimators.size());

		//find which partition the average is in
		int partitionWidth = imageWidth/partitionCols;
		int partitionHeight = imageHeight/partitionRows;
		bool avgRowFound = false;
		int avgRowInd = 0;
		while (!avgRowFound)
		{
			int top = avgRowInd*partitionHeight;
			int bottom = (avgRowInd+1)*partitionHeight;
			if ((top <= avgcPty) && (avgcPty <= bottom))
			{
				avgRowFound = true;
			}
			else
			{
				avgRowInd++;
			}
		}

		bool avgColFound = false;
		int avgColInd = 0;
		while (!avgColFound)
		{
			int left = avgColInd*partitionWidth;
			int right = (avgColInd+1)*partitionWidth;
			if ((left <= avgcPtx) && (avgcPtx <= right))
			{
				avgColFound = true;
			}
			else
			{
				avgColInd++;
			}
		}

		currentAvgPartition = partitionCols*avgRowInd + avgColInd;

		ROS_WARN("time for estimators %2.4f, avg partition is %d, all points known %d",float(clock()-updateClock)/CLOCKS_PER_SEC,currentAvgPartition,int(allPtsKnown));
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

			if (tkcHat.norm() > 0.001)
			{
				pkcHat += kp*(tkcHat*(dkcHat/tkcHat.norm()) - pkcHat);
			}
		}

		// ROS_WARN("dkcHat %2.1f",dkcHat);
	}
	catch (cv::Exception e)
	{
		ROS_ERROR("update failed");
	}
	//
	// std::cout << "\n after update \n";
	std::cout << "\n tkcHat \n" << tkcHat << std::endl;
	std::cout << "\n qkcHat \n" << qkcHat << std::endl;
	// std::cout << "\n rotation \n" << 2.0*acos(qkcHat(0))*180/3.14 << std::endl;
	std::cout << "\n pkcHat \n" << pkcHat << std::endl;
	// std::cout << "\n dkHat " << dkHat << std::endl;
	std::cout << "\n dkcHat " << dkcHat << std::endl;
	// std::cout << "\n vc \n" << vc << std::endl;
	// std::cout << "\n wc \n" << wc << std::endl;

}
