#include <patch_estimator.h>

PatchEstimator::~PatchEstimator()
{
	destroyLock.lock();
	// if (saveExp)
	// {
	// 	std::cout << "\nsaving\n";
	// 	std::ofstream saveFile("/home/ncr/ncr_ws/src/icl_stationary/experiment/"+expName+".txt");
	// 	if (saveFile.is_open())
	// 	{
	// 		std::cout << "\nopen\n";
	// 		saveFile << "time,";
	// 		saveFile << "pkcx," << "pkcy," << "pkcz,";
	// 		saveFile << "qkcw," << "qkcx," << "qkcy," << "qkcz,";
	// 		saveFile << "pkcICLx," << "pkcICLy," << "pkcICLz,";
	// 		saveFile << "qkcICLw," << "qkcICLx," << "qkcICLy," << "qkcICLz,";
	// 		saveFile << "pikx," << "piky," << "pikz,";
	// 		saveFile << "pikICLx," << "pikICLy," << "pikICLz,";
	// 		saveFile << "picx," << "picy," << "picz,";
	// 		saveFile << "picICLx," << "picICLy," << "picICLz,";
	// 		saveFile << "picEKFx," << "picEKFy," << "picEKFz,";
	// 		saveFile << "picLSx," << "picLSy," << "picLSz,";
	// 		saveFile << "picICLExtx," << "picICLExty," << "picICLExtz,";
	// 		saveFile << "vx," << "vy," << "vz,";
	// 		saveFile << "wx," << "wy," << "wz,";
	// 		saveFile << "\n";
	//
	// 		for (int jj = 0; jj < data.size(); jj++)
	// 		{
	// 			float timej = data.at(jj)->time;
	// 			Eigen::Vector3f pkcj = data.at(jj)->pkc;
	// 			Eigen::Vector4f qkcj = data.at(jj)->qkc;
	// 			Eigen::Vector3f pkcICLj = data.at(jj)->pkcICL;
	// 			Eigen::Vector4f qkcICLj = data.at(jj)->qkcICL;
	// 			Eigen::Vector3f pikj = data.at(jj)->pik;
	// 			Eigen::Vector3f pikICLj = data.at(jj)->pikICL;
	// 			Eigen::Vector3f picj = data.at(jj)->pic;
	// 			Eigen::Vector3f picICLj = data.at(jj)->picICL;
	// 			Eigen::Vector3f picEKFj = data.at(jj)->picEKF;
	// 			Eigen::Vector3f picLSj = data.at(jj)->picLS;
	// 			Eigen::Vector3f picICLExtj = data.at(jj)->picICLExt;
	// 			Eigen::Vector3f vj = data.at(jj)->v;
	// 			Eigen::Vector3f wj = data.at(jj)->w;
	//
	// 			std::cout << "\n picICLExtj \n" << picICLExtj << std::endl;
	//
	// 			saveFile << timej << ",";
	// 			saveFile << pkcj(0) << "," << pkcj(1) << "," << pkcj(2) << ",";
	// 			saveFile << qkcj(0) << "," << qkcj(1) << "," << qkcj(2) << "," << qkcj(3) << ",";
	// 			saveFile << pkcICLj(0) << "," << pkcICLj(1) << "," << pkcICLj(2) << ",";
	// 			saveFile << qkcICLj(0) << "," << qkcICLj(1) << "," << qkcICLj(2) << "," << qkcICLj(3) << ",";
	// 			saveFile << pikj(0) << "," << pikj(1) << "," << pikj(2) << ",";
	// 			saveFile << pikICLj(0) << "," << pikICLj(1) << "," << pikICLj(2) << ",";
	// 			saveFile << picj(0) << "," << picj(1) << "," << picj(2) << ",";
	// 			saveFile << picICLj(0) << "," << picICLj(1) << "," << picICLj(2) << ",";
	// 			saveFile << picEKFj(0) << "," << picEKFj(1) << "," << picEKFj(2) << ",";
	// 			saveFile << picLSj(0) << "," << picLSj(1) << "," << picLSj(2) << ",";
	// 			saveFile << picICLExtj(0) << "," << picICLExtj(1) << "," << picICLExtj(2) << ",";
	// 			saveFile << vj(0) << "," << vj(1) << "," << vj(2) << ",";
	// 			saveFile << wj(0) << "," << wj(1) << "," << wj(2) << ",";
	// 			saveFile << "\n";
	//
	// 			delete data.at(jj);
	// 		}
	// 		saveFile.close();
	// 		std::cout << "\nclose\n";
	// 	}
	// 	std::cout << "\nsaved\n";
	// }

	ROS_ERROR("\n\n starting destroy \n\n");

	// std::cout << "\n hid10 \n";
	imagePub.shutdown();
	// std::cout << "\n hid11 \n";
	// imagePub2.shutdown();
	imageSub.shutdown();
	// std::cout << "\n hid12 \n";
	odomSub.shutdown();
	// std::cout << "\n hid13 \n";
	roiPub.shutdown();
	roiSub.shutdown();
	// std::cout << "\n hid14 \n";
	// std::cout << "\n hid15 \n";
	poseDeltaPub.shutdown();
	// std::cout << "\n hid16 \n";
	wallPub.shutdown();
	// std::cout << "\n hid17 \n";
	pointCloudPub.shutdown();
	// std::cout << "\n hid18 \n";

	chessboardPub.shutdown();
	chessboardSub.shutdown();

	// std::cout << "\n hid19 \n";
	// chessboardSub.shutdown();
	ROS_ERROR("\n\n destroyed \n\n");

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
	qkcHat = Eigen::Vector4f(1.0,0.0,0.0,0.0);
	pkcHat = Eigen::Vector3f::Zero();
	pkp = Eigen::Vector3f::Zero();
	qkp = Eigen::Vector4f(1.0,0.0,0.0,0.0);
	firstkp = true;

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

	Eigen::JacobiSVD<Eigen::MatrixXf> svdcamMatf(camMatf, Eigen::ComputeThinU | Eigen::ComputeThinV);
	camMatIf = svdcamMatf.solve(Eigen::Matrix3f::Identity());

	GfLast = Eigen::Matrix<float,2,3>::Zero();
	GfLast.block(0,0,2,2) = Eigen::Matrix2f::Identity();

	GkfLast = Eigen::Matrix3f::Identity();

	zmin = zminInit;
	zmax = zmaxInit;
	dkcHat = 0.01;
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
	imageSub = it.subscribe(cameraName+"/image_undistort", 30, &PatchEstimator::imageCB,this);
	odomSub = nh.subscribe(cameraName+"/odom", 30, &PatchEstimator::odomCB,this);
	// roiSub = nh.subscribe(cameraName+"/"+std::to_string(keyInd)+"/"+std::to_string(patchInd)+"/roi",10, &PatchEstimator::roiCB,this);

	// odomPub = nh.advertise<nav_msgs::Odometry>(cameraName+"/odomHat",1);
	// odomDelayedPub = nh.advertise<nav_msgs::Odometry>(cameraName+"/odomDelayed", 1);
	// pointCloudPub = nh.advertise<PointCloud> ("patch_map", 1);
	firstImage = true;
}

bool PatchEstimator::initialize(cv::Mat& image, nav_msgs::Odometry imageOdom, ros::Time t)
{
	int partitionWidth = imageWidth/partitionCols;
	int partitionHeight = imageHeight/partitionRows;

	// get the center of each partition and place uniform spacing of points using minDistance
	std::vector<cv::Point2f> pts;
	int colc = partitionInd%partitionCols*partitionWidth + partitionWidth/2;
	int rowc = partitionInd/partitionCols*partitionHeight + partitionHeight/2 - minDistance;
	int coltl = colc - (numberFeaturesPerPartCol-1)*minDistance/2;
	int rowtl = rowc - (numberFeaturesPerPartRow-1)*minDistance/2;
	// int partitionWidth = imageWidth;
	// int partitionHeight = imageHeight;

	// // get the center of each partition and place uniform spacing of points using minDistance
	// std::vector<cv::Point2f> pts;
	// int colc = partitionWidth/2;
	// int rowc = partitionHeight/2 - minDistance;
	// int coltl = colc - (numberFeaturesPerPartCol-1)*minDistance/2;
	// int rowtl = rowc - (numberFeaturesPerPartRow-1)*minDistance/2;

	for (int jj = 0; jj < numberFeaturesPerPartRow; jj++)
	{
		int rowii = rowtl + jj*minDistance;
		for (int hh = 0; hh < numberFeaturesPerPartCol; hh++)
		{
			int colii = coltl + hh*minDistance;
			pts.push_back(cv::Point2f(colii,rowii));
		}
	}

	//find the minimum enclosing rectangle
	cv::Rect ptsRect = cv::boundingRect(pts);
	cv::Mat onesMat = cv::Mat::ones(ptsRect.size(),CV_8UC1);
	cv::Mat mask = cv::Mat::zeros(cv::Size(imageWidth,imageHeight),CV_8UC1);
	onesMat.copyTo(mask(ptsRect));
	std::vector<cv::Point2f> ptCorners;

	std::cout << "\n ptsRect \n" << ptsRect << std::endl;
	std::cout << "\n onesMat.size() \n" << onesMat.size() << std::endl;

	//find the best corners in the masked image
	cv::goodFeaturesToTrack(image,ptCorners,100,0.01,100,mask,7);
	std::cout << "\n ptCorners.size() " << ptCorners.size() << std::endl;
	std::cout << "\n minFeaturesBad " << minFeaturesBad << std::endl;

	std::cout << "\n numberFeaturesPerPartRow*numberFeaturesPerPartCol " << numberFeaturesPerPartRow*numberFeaturesPerPartCol << std::endl;


	if (ptCorners.size() <= minFeaturesBad)
	{
		return false;
	}

	cv::cornerSubPix(image,ptCorners,cv::Size(7,7),cv::Size(-1,-1),cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 0.001));

	imagePub = it.advertise(cameraName+"/test_image",1);
	poseDeltaPub = nh.advertise<icl_multiple_stationary::PoseDelta>(cameraName+"/pose_delta",1);
	roiPub = nh.advertise<icl_multiple_stationary::Roi>(cameraName+"/"+std::to_string(keyInd)+"/"+std::to_string(patchInd)+"/roi",1);
	wallPub = nh.advertise<icl_multiple_stationary::Wall>("/wall_points",1);
	roiSub = nh.subscribe(cameraName+"/"+std::to_string(keyInd)+"/"+std::to_string(patchInd)+"/roi",1, &PatchEstimator::roiCB,this);
	pointCloudPub = nh.advertise<PointCloud>("patch_map", 1);
	chessboardPub = nh.advertise<std_msgs::Time>(cameraName+"/"+std::to_string(keyInd)+"/"+std::to_string(patchInd)+"/chessboard", 5);
	chessboardSub = nh.subscribe(cameraName+"/"+std::to_string(keyInd)+"/"+std::to_string(patchInd)+"/chessboard",5, &PatchEstimator::chessboardCB,this);

	keyOdom = imageOdom;
	kimage = image.clone();
	pimage = image.clone();
	tLast = t;
	tStart = t;
	// tkcDotEstimator.update(Eigen::Vector3f::Zero(),t);
	// qkcDotEstimator.update(Eigen::Vector4f(1.0,0.0,0.0,0.0),t);

	std::cout << "\n patch \n";
	for (int ii = 0; ii < ptCorners.size(); ii++)
	{
		newDepthEstimator = new DepthEstimator(ii,Eigen::Vector3f((ptCorners.at(ii).x-cx)/fx,(ptCorners.at(ii).y-cy)/fy,1.0),tLast,zmin,zmax,zmax,tau,fx,fy,cx,cy);
		depthEstimators.push_back(newDepthEstimator);
		// std::cout << ii << " ptix " << pts.at(ii).x << " ptiy " << pts.at(ii).y << std::endl;
		std::cout << ii << " ptCix " << ptCorners.at(ii).x << " ptCiy " << ptCorners.at(ii).y << std::endl;
		avgcPtx += ptCorners.at(ii).x;
		avgcPty += ptCorners.at(ii).y;
		// std::cout << ii << " mcx " << depthEstimators.at(ii)->mc(0) << " mcy " << depthEstimators.at(ii)->mc(1) << std::endl;
		// std::cout << ii << " mkx " << depthEstimators.at(ii)->mk(0) << " mky " << depthEstimators.at(ii)->mk(1) << std::endl;
	}
	avgcPtx /= float(ptCorners.size());
	avgcPty /= float(ptCorners.size());
	depthEstimatorsSaved = depthEstimators;
	return true;
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
	// Eigen::MatrixXf XY1ICL = Eigen::MatrixXf::Ones(numberPts,3);
	// Eigen::MatrixXf XY1TICL = Eigen::MatrixXf::Ones(3,numberPts);
	// Eigen::VectorXf NZICL = Eigen::VectorXf::Zero(numberPts);
	// Eigen::MatrixXf XY1EKF = Eigen::MatrixXf::Ones(numberPts,3);
	// Eigen::MatrixXf XY1TEKF = Eigen::MatrixXf::Ones(3,numberPts);
	// Eigen::VectorXf NZEKF = Eigen::VectorXf::Zero(numberPts);
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
	kcloud.width = numberPts;
	kcloud.is_dense = true;
	kcloud.resize(numberPts);
	PointCloudRGB::iterator kcloudIt = kcloud.begin();

	PointCloudRGB ccloud;
	ccloud.clear();
	ccloud.height = 1;
	ccloud.width = numberPts;
	ccloud.is_dense = true;
	ccloud.resize(numberPts);
	PointCloudRGB::iterator ccloudIt = ccloud.begin();

	std::vector<uint8_t> dkKnowns(numberPts,0),inds(numberPts);
	std::vector<uint8_t>::iterator dkKnownsIt = dkKnowns.begin();
	std::vector<uint8_t>::iterator indsIt = inds.begin();

	for (std::vector<DepthEstimator*>::iterator itD = depthEstimators.begin(); itD != depthEstimators.end(); itD++)
	{
		//add each point
		avgNumSaved += float((*itD)->depthEstimatorICLExt.numSaved);
		*dkKnownsIt = (*itD)->depthEstimatorICLExt.dkKnown;
		*indsIt = (*itD)->depthInd;
		dkKnownsIt++;
		indsIt++;

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
		// XY1ICL(itIdx,0) = picICL(0);
		// XY1ICL(itIdx,1) = picICL(1);
		// XY1TICL(0,itIdx) = picICL(0);
		// XY1TICL(1,itIdx) = picICL(1);
		// NZICL(itIdx) = -picICL(2);
		//
		// XY1EKF(itIdx,0) = picEKF(0);
		// XY1EKF(itIdx,1) = picEKF(1);
		// XY1TEKF(0,itIdx) = picEKF(0);
		// XY1TEKF(1,itIdx) = picEKF(1);
		// NZEKF(itIdx) = -picEKF(2);
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


		// cv::Point2f ptic(fx*mic(0)+cx,fy*mic(1)+cy);

		// Eigen::Vector3f mkiHat = depthEstimators.at(ii)->mk;
		// cv::Point2i kPti(int(fx*mkiHat(0)+cx),int(fy*mkiHat(1)+cy));
		// uint8_t colori = kimage.at<uint8_t>(kPti.y,kPti.x);
		// if (firstPt)
		// {
		// 	minx = ptic.x;
		// 	maxx = ptic.x;
		// 	miny = ptic.y;
		// 	maxy = ptic.y;
		// 	firstPt = false;
		// }
		// else
		// {
		// 	if (minx > ptic.x)
		// 	{
		// 		minx = ptic.x;
		// 	}
		// 	if (maxx < ptic.x)
		// 	{
		// 		maxx = ptic.x;
		// 	}
		// 	if (miny > ptic.y)
		// 	{
		// 		miny = ptic.y;
		// 	}
		// 	if (maxy < ptic.y)
		// 	{
		// 		maxy = ptic.y;
		// 	}
		// }
	}

	avgNumSaved /= float(numberPts);
	//
	// int cols = maxx-minx;
	// int rows = maxy-miny;
	// int coltl = minx;
	// int rowtl = miny;


	// if ((coltl >= 0) && (rowtl >= 0) && ((coltl+cols) < pimage.cols) && ((rowtl+rows) < pimage.rows))
	if (true)
	{
		try
		{
			// cv::Mat roiimage = pimage(cv::Rect(coltl,rowtl,cols,rows)).clone();
			// int reduceFactor = 25;
			// int colsReduce = std::round(cols/reduceFactor);
			// int rowsReduce = std::round(rows/reduceFactor);
			// cv::Mat roiimageReduce(cv::Size(colsReduce,rowsReduce),CV_8U);
			// cv::resize(roiimage,roiimageReduce,roiimageReduce.size(),fx,fy,cv::INTER_AREA);

			roiMutex.unlock();

			// Eigen::Matrix3f XYXYICL = XY1TICL*XY1ICL;
			// Eigen::Vector3f XYNZICL = XY1TICL*NZICL;
			// float XYXYdet = float(XYXYICL.determinant());
			// Eigen::Matrix3f XYXYEKF = XY1TEKF*XY1EKF;
			// Eigen::Vector3f XYNZEKF = XY1TEKF*NZEKF;
			// Eigen::Matrix3f XZXZ = XZ1T*XZ1;
			// float XZXZdet = float(XZXZ.determinant());
			// Eigen::Matrix3f YZYZ = YZ1T*YZ1;
			// float YZYZdet = float(YZYZ.determinant());

			if (true)
			{
				// Eigen::JacobiSVD<Eigen::MatrixXf> svdXYXYICL(XYXYICL, Eigen::ComputeThinU | Eigen::ComputeThinV);
				// Eigen::Vector3f dnzICL = svdXYXYICL.solve(XYNZICL);
				// Eigen::Vector3f ncICL(dnzICL(0),dnzICL(1),1.0);
				// ncICL /= ncICL.norm();
				// float nxICL = ncICL(0);
				// float nyICL = ncICL(1);
				// float nzICL = ncICL(2);
				// float dcICL = -dnzICL(2)*nzICL;
				//
				// Eigen::JacobiSVD<Eigen::MatrixXf> svdXYXYEKF(XYXYEKF, Eigen::ComputeThinU | Eigen::ComputeThinV);
				// Eigen::Vector3f dnzEKF = svdXYXYEKF.solve(XYNZEKF);
				// Eigen::Vector3f ncEKF(dnzEKF(0),dnzEKF(1),1.0);
				// ncEKF /= ncEKF.norm();
				// float nxEKF = ncEKF(0);
				// float nyEKF = ncEKF(1);
				// float nzEKF = ncEKF(2);
				// float dcEKF = -dnzEKF(2)*nzEKF;
				//
				// ROS_WARN("roi get normal time %2.4f",float(clock()-processTime)/CLOCKS_PER_SEC);
				// processTime = clock();
				//
				// //get the estimated point for each pixel in the roi
				// int plotInd = 0;
				// PointCloudRGB cloud;
				// cloud.clear();
				// cloud.height = 1;
				// cloud.width = colsReduce*rowsReduce;
				// cloud.is_dense = true;
				// cloud.resize(colsReduce*rowsReduce);
				// PointCloudRGB::iterator cloudIt = cloud.begin();
				// pcl::PointXYZRGB ptxyz;
				//
				// int rowj = 0;
				// int colj = 0;
				//
				// float mxj = 0;
				// float myj = 0;
				// float xjcICL = 0;
				// float yjcICL = 0;
				// float zjcICL = 0;
				// Eigen::Vector3f piw(0.0,0.0,0.0);
				// Eigen::Vector3f pic(0.0,0.0,0.0);
				//
				// for (cv::MatIterator_<uchar> itI = roiimageReduce.begin<uchar>(); itI != roiimageReduce.end<uchar>(); itI++)
				// {
				// 	rowj = rowtl + reduceFactor*(plotInd/colsReduce);
				// 	colj = coltl + reduceFactor*(plotInd%colsReduce);
				//
				// 	// *colIt = uint8_t(*itI);
				// 	mxj = (colj - cx)/fx;
				// 	myj = (rowj - cy)/fy;
				// 	zjcICL = dcICL/(nxICL*mxj+nyICL*myj+nzICL);
				// 	xjcICL = zjcICL*mxj;
				// 	yjcICL = zjcICL*myj;
				//
				// 	pic(0) = xjcICL;
				// 	pic(1) = yjcICL;
				// 	pic(2) = zjcICL;
				//
				// 	piw = pcw + rotatevec(pic,qcw);
				//
				// 	ptxyz.x = piw(0);
				// 	ptxyz.y = piw(1);
				// 	ptxyz.z = piw(2);
				// 	ptxyz.r = (*itI);
				// 	ptxyz.g = (*itI);
				// 	ptxyz.b = std::min((*itI)+100,255);
				// 	*cloudIt = ptxyz;
				// 	cloudIt++;
				//
				// 	plotInd++;
				// }

				// ROS_WARN("roi get points time %2.4f, plotInd %d",float(clock()-processTime)/CLOCKS_PER_SEC,plotInd);
				// processTime = clock();

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


				// PointCloudRGB::Ptr map(new PointCloudRGB);
				// (*map).clear();
				// pcl_conversions::toPCL(msg->header.stamp,map->header.stamp);
				//
				// // std::cout << "\n wall 1 1 \n";
				// map->header.frame_id = "world";
				//
				// // std::cout << "\n wall 1 2 \n";
				// map->height = 1;
				// map->is_dense = true;
				// map->points.clear();
				// // *map += cloud;
				// *map += ccloud;
				// map->width = map->points.size();
				//
				// ROS_WARN("patch map->width %d",int(map->width));
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
					icl_multiple_stationary::Wall wallMsg;
					wallMsg.header.stamp = t;
					// wallMsg.wallPts = wallPts;
					pcl::toROSMsg(ccloud, wallMsg.cloud);
					// pcl::toROSMsg(kcloud, wallMsg.cloud);
					wallMsg.keyInd = keyInd;
					wallMsg.patchInd = patchInd;
					wallMsg.pose = msg->pose;
					wallMsg.allPtsKnown = allPtsKnown;
					wallMsg.dkKnowns = dkKnowns;
					wallMsg.inds = inds;
					wallPub.publish(wallMsg);
					// if (allPtsKnown && (acos(nyICL) > (70.0*3.1415/180.0)) && (acos(nxICL) > (70.0*3.1415/180.0)))
					// if (allPtsKnown && (avgNumSaved >= 1) && (acos(nzICL) < (30.0*3.1415/180.0)))
					if ((allPtsKnown && (avgNumSaved >= 1)) || landmarkView)
					{

						icl_multiple_stationary::PoseDelta poseDeltaMsg;
						poseDeltaMsg.header.stamp = t;
						poseDeltaMsg.pose = msg->pose;
						poseDeltaMsg.poseHat = msg->poseHat;
						poseDeltaMsg.poseBody = msg->poseBody;
						poseDeltaMsg.poseBodyHat = msg->poseBodyHat;
						poseDeltaMsg.landmarkView = landmarkView;
						poseDeltaMsg.keyInd = keyInd;
						poseDeltaMsg.pRatio = float(depthEstimators.size())/float(depthEstimatorsSaved.size());
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
	qcw /= qcw.norm();

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
		if (initialize(image,imageOdom,t))
		{
			ppw = pcw;
			qpw = qcw;
			firstImage = false;
			ROS_INFO("initialized");
			return;
		}
		else
		{
			ROS_ERROR("not enough features");
			return;
		}

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

	Eigen::Vector3f pkcHatDot = -getss(wc)*pkcHat-vc;
	Eigen::Vector4f qkcHatDot = -0.5*B(qkcHat)*wc;

	if (tkcHat.norm()>0.01)
	{
		// tkcHat /= tkcHat.norm();
		Eigen::Vector3f tkcHatDot = -getss(wc)*tkcHat + (1.0/dkcHat)*(tkcHat*tkcHat.transpose() - Eigen::Matrix3f::Identity())*vc;
		// Eigen::Vector3f ucEstDot = -getss(w)*ucEst + (1.0/dcHat)*(ucEst*ucEstT - Eigen::Matrix3f::Identity())*v;
		float dkcHatDot = -float(tkcHat.transpose()*vc);
		dkcHat += (dkcHatDot*dt);
		if (dkcHat < 0.01)
		{
			dkcHat = 0.01;
		}
		tkcHat += (tkcHatDot*dt);

		if (tkcHat.norm() > 0.01)
		{
			tkcHat /= tkcHat.norm();
		}
	}

	pkcHat += (pkcHatDot*dt);
	qkcHat += (qkcHatDot*dt);
	qkcHat /= qkcHat.norm();

	Eigen::Vector3f pkw(keyOdom.pose.pose.position.x,keyOdom.pose.pose.position.y,keyOdom.pose.pose.position.z);
	Eigen::Vector4f qkw(keyOdom.pose.pose.orientation.w,keyOdom.pose.pose.orientation.x,keyOdom.pose.pose.orientation.y,keyOdom.pose.pose.orientation.z);

	if ((pcw-ppw).norm() < 0.001)
	{
		Eigen::Vector4f qcwDot = 0.5*B(qcw)*wc;
		Eigen::Vector3f pcwDot = rotatevec(vc,qcw);
		pcw += pcwDot*dt;
		qcw += qcwDot*dt;
		qcw /= qcw.norm();
	}

	Eigen::Vector4f qpc = getqMat(getqInv(qcw))*qpw;
	qpc /= qpc.norm();
	Eigen::Vector3f ppc = rotatevec(ppw-pcw,qcw);

	ppw = pcw;
	qpw = qcw;

	Eigen::Vector4f qkc = getqMat(getqInv(qcw))*qkw;
	qkc /= qkc.norm();
	Eigen::Vector3f pkc = rotatevec(pkw-pcw,getqInv(qcw));

	// find the features
	match(image,dt,vc,wc,t,ppc,qpc,pkc,qkc);

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

			cv::circle(drawImage, cPti, 10, cv::Scalar(250, 250, 250), -1);

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
	//
	pubMutex.lock();
	imagePub.publish(out_msg.toImageMsg());
	int depthEstimatorsSize = depthEstimators.size();
	pubMutex.unlock();

	// ROS_WARN("cb time %2.4f",float(clock()-callbackTime)/CLOCKS_PER_SEC);

	if ((depthEstimatorsSize <= minFeaturesBad))// || (inlierRatio <= 0.26))
	{
		pubMutex.lock();

		// std::cout << "\n hid8 \n";
		for (std::vector<DepthEstimator*>::iterator itD = depthEstimators.begin() ; itD != depthEstimators.end(); itD++)
		{
			delete *itD;
		}
		// std::cout << "\n hid9 \n";
		depthEstimators.clear();
		// std::cout << "\n hid10 \n";

		patchShutdown = true;
		roiPub.shutdown();
		roiSub.shutdown();
		chessboardPub.shutdown();
		chessboardSub.shutdown();
		pubMutex.unlock();

		// odomPub.shutdown();
		// odomDelayedPub.shutdown();
		// pointCloudPub.shutdown();
		// patchShutdown = true;

		ROS_WARN("shutdown after imagesub");
	}
	else
	{
		// std::cout << "\n hi9 \n";

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
		qkcHat /= qkcHat.norm();
		qckHat = getqInv(qkcHat);
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

		// std::cout << "\n pbwHat \n" << pbwHat << std::endl;
		// std::cout << "\n qbwHat \n" << qbwHat << std::endl;
		// std::cout << "\n pbwk \n" << pbwk << std::endl;
		// std::cout << "\n qbwk \n" << qbwk << std::endl;
		// std::cout << "\n pbw \n" << pbw << std::endl;
		// std::cout << "\n qbw \n" << qbw << std::endl;
		// std::cout << "\n pcw \n" << pcw << std::endl;
		// std::cout << "\n qcw \n" << qcw << std::endl;
		// std::cout << "\n pkw \n" << pkw << std::endl;
		// std::cout << "\n qkw \n" << qkw << std::endl;

		// Eigen::Vector3f pkc = rotatevec(pkw-pcw,getqInv(qcw));
		// Eigen::Vector4f qkc = getqMat(getqInv(qcw))*qkw;
		// qkc /= qkc.norm();

		if (!patchShutdown)
		{
			icl_multiple_stationary::Roi roiMsg;
			roiMsg.header.stamp = t;
			roiMsg.pose.position.x = pcw(0);
			roiMsg.pose.position.y = pcw(1);
			roiMsg.pose.position.z = pcw(2);
			roiMsg.pose.orientation.w = qcw(0);
			roiMsg.pose.orientation.x = qcw(1);
			roiMsg.pose.orientation.y = qcw(2);
			roiMsg.pose.orientation.z = qcw(3);
			roiMsg.poseHat.position.x = pcwHat(0);
			roiMsg.poseHat.position.y = pcwHat(1);
			roiMsg.poseHat.position.z = pcwHat(2);
			roiMsg.poseHat.orientation.w = qcwHat(0);
			roiMsg.poseHat.orientation.x = qcwHat(1);
			roiMsg.poseHat.orientation.y = qcwHat(2);
			roiMsg.poseHat.orientation.z = qcwHat(3);
			roiMsg.poseBody.position.x = pbw(0);
			roiMsg.poseBody.position.y = pbw(1);
			roiMsg.poseBody.position.z = pbw(2);
			roiMsg.poseBody.orientation.w = qbw(0);
			roiMsg.poseBody.orientation.x = qbw(1);
			roiMsg.poseBody.orientation.y = qbw(2);
			roiMsg.poseBody.orientation.z = qbw(3);
			roiMsg.poseBodyHat.position.x = pbwHat(0);
			roiMsg.poseBodyHat.position.y = pbwHat(1);
			roiMsg.poseBodyHat.position.z = pbwHat(2);
			roiMsg.poseBodyHat.orientation.w = qbwHat(0);
			roiMsg.poseBodyHat.orientation.x = qbwHat(1);
			roiMsg.poseBodyHat.orientation.y = qbwHat(2);
			roiMsg.poseBodyHat.orientation.z = qbwHat(3);
			roiPub.publish(roiMsg);

			if (!landmarkView)
			{
				std_msgs::Time tMsg;
				tMsg.data = t;
				chessboardPub.publish(tMsg);
			}
		}

		pubMutex.unlock();
	}

	ROS_WARN("IMAGE STOP cb time %2.4f",float(clock()-callbackTime)/CLOCKS_PER_SEC);
}

void PatchEstimator::chessboardCB(const std_msgs::Time::ConstPtr& msg)
{
	clock_t chessTime = clock();
	chessboardMutex.lock();
	if (patchShutdown)
	{
		chessboardMutex.unlock();
		return;
	}

	if (numLandmarkCheck < 3)
	{
		numLandmarkCheck++;
		chessboardMutex.unlock();
		return;
	}
	else
	{
		numLandmarkCheck = 0;
	}
	cv::Mat image = pimage.clone();
	chessboardMutex.unlock();
	// cv::Mat imageThresh(image.size(),image.type());
	//
	// cv::threshold(image,imageThresh,225,255,CV_THRESH_BINARY);
	cv::Size patternSize(8,6);
	std::vector<cv::Point2f> ptsCheckerBoard;
	bool landmarkViewNew = cv::findChessboardCorners(image,patternSize,ptsCheckerBoard,cv::CALIB_CB_FAST_CHECK);
	// cv::Mat imageThresh;
	// cv::threshold(image,imageThresh,225,255,cv::THRESH_BINARY);
	// std::vector<std::vector<cv::Point>> contours;
	// std::vector<cv::Vec4i> heirchy;
	// cv::findContours(imageThresh,contours,heirchy,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
	//
	// // publish key image
	// cv_bridge::CvImage out_msg;
	// out_msg.header.stamp = msg->data; // Same timestamp and tf frame as input image
	// out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
	// out_msg.image = imageThresh; // Your cv::Mat

	// pubMutex.lock();
	// imagePub.publish(out_msg.toImageMsg());
	// pubMutex.unlock();

	if (landmarkViewNew)
	{
		chessboardMutex.lock();
		landmarkView = landmarkViewNew;
		// std::cout << "\n saw landmark \n";
		chessboardSub.shutdown();
		chessboardMutex.unlock();
	}
	ROS_WARN("chessboard time %2.4f",float(clock()-chessTime)/CLOCKS_PER_SEC);
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

	//get the points from the previous image
	std::vector<cv::Point2f> pPts(numberPts),cPts(numberPts),kPts(numberPts);
	std::vector<cv::Point2f> kPtsInPred(numberPts),cPtsInPred(numberPts);//,cPtPsInPred(depthEstimators.size());
	std::vector<DepthEstimator*> depthEstimatorsInH;
	std::vector<cv::Point2f> pPtsInH,cPtsInH,kPtsInH;
	Eigen::Vector3f mpp,mcc;
	std::vector<cv::Point2f>::iterator itp = pPts.begin();
	std::vector<cv::Point2f>::iterator itc = cPts.begin();
	std::vector<cv::Point2f>::iterator itk = kPts.begin();
	float avgNumSaved = 0.0;
	float avgNumThrown = 0.0;
	float avgdkKnown = 0.0;
	int numberdkKnown = 0;
	int numWithSaved = 0;

	std::vector<cv::Point2f> flows(pPts.size());
	std::vector<cv::Point2f>::iterator itf = flows.begin();
	cv::Point2f avgFlow(0.0,0.0);
	featureMutex.lock();
	for (std::vector<DepthEstimator*>::iterator itD = depthEstimators.begin() ; itD != depthEstimators.end(); itD++)
	{
		*itk = cv::Point2f((*itD)->ptk(0),(*itD)->ptk(1));
		mpp = (*itD)->current();
	  *itp = cv::Point2f(fx*mpp(0)+cx,fy*mpp(1)+cy);
	  mcc = (*itD)->predict(vc,wc,dt,pkc,qkc);
	  *itc = cv::Point2f(fx*mcc(0)+cx,fy*mcc(1)+cy);
		*itf = ((*itc) - (*itp));
		avgFlow += (*itf);

		avgNumSaved += float((*itD)->depthEstimatorICLExt.numSaved);
		avgNumThrown += float((*itD)->depthEstimatorICLExt.numThrown);

		if ((*itD)->depthEstimatorICLExt.dkKnown)
		{
			avgdkKnown += 1.0;
			numberdkKnown += 1;
		}

		if ((*itD)->depthEstimatorICLExt.numSaved > 0)
		{
			numWithSaved += 1;
		}


		itk++;
		itp++;
		itc++;
		itf++;
	}
	// if (cPts.size() > 0)
	// {
	// 	// cv::cornerSubPix(image,cPts,cv::Size(3,3),cv::Size(-1,-1),cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
	// 	cv::cornerSubPix(pimage,pPts,cv::Size(3,3),cv::Size(-1,-1),cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
	// }

	featureMutex.unlock();

	if (numberPts > 0)
	{
		avgNumSaved /= float(numberPts);
		avgNumThrown /= float(numberPts);
		avgFlow /= float(numberPts);
		avgdkKnown /= float(numberPts);
	}

	// std::vector<float> flowxs(pPts.size()),flowys(pPts.size());
	// for (int ii = 0; ii < cPts.size(); ii++)
	// {
	// 	if (dt > 0)
	// 	{
	// 		flowxs.at(ii) = (cPts.at(ii).x-pPts.at(ii).x)/dt;
	// 		flowys.at(ii) = (cPts.at(ii).y-pPts.at(ii).y)/dt;
	// 	}
	// 	else
	// 	{
	// 		flowxs.at(ii) = 0.0;
	// 		flowys.at(ii) = 0.0;
	// 	}
	// }

	// std::vector<float> flowxsSort = flowxs;
	// std::vector<float> flowysSort = flowys;
	// std::sort(flowxsSort.begin(),flowxsSort.end());
	// std::sort(flowysSort.begin(),flowysSort.end());
	// float flowxMed = flowxsSort.at(0);
	// float flowyMed = flowysSort.at(0);
	// if (flowxs.size()%2 == 0)
	// {
	// 	flowxMed = (flowxsSort.at(flowxsSort.size()/2-1)+flowxsSort.at(flowxsSort.size()/2))/2.0;
	// 	flowyMed = (flowysSort.at(flowysSort.size()/2-1)+flowysSort.at(flowysSort.size()/2))/2.0;
	// }
	// else
	// {
	// 	flowxMed = flowxsSort.at(flowxsSort.size()/2);
	// 	flowyMed = flowysSort.at(flowysSort.size()/2);
	// }

	float qAng = 180.0/3.1415*acos(qkc(0))*2.0;
	cv::Rect currentBound = cv::boundingRect(cPts);

	std::cout << "\n flowAvgx " << avgFlow.x << ", flowAvgy " << avgFlow.y << std::endl;

	float flowchi2 = 6.63; //chi^2 for 99%
	float flowsig = 3.0;
	float flowsig2 = flowsig*flowsig;

	std::vector<DepthEstimator*> depthEstimatorsInFlow;
	std::vector<cv::Point2f> pPtsInFlow,cPtsInFlow,kPtsInFlow;
	// flowDiffPt.x = flowxMed*dt;
	// flowDiffPt.y = flowyMed*dt;
	//remove outlier flows

	featureMutex.lock();
	std::cout << "\n num points before flow " << depthEstimators.size() << " avgNumSaved " << avgNumSaved << " numWithSaved " << numWithSaved << " avgdkKnown " << avgdkKnown << " numberdkKnown " << numberdkKnown << std::endl;
	float timeFromStart = (t-tStart).toSec();
	for (int ii = 0; ii < flows.size(); ii++)
	{
		cv::Point2f flowdiffi = avgFlow - flows.at(ii);
		float chiTestVal = (flowdiffi.x*flowdiffi.x + flowdiffi.y*flowdiffi.y)/flowsig2;
		std::cout << " flowx " << flows.at(ii).x << " flowy " << flows.at(ii).y << " chiVal " << chiTestVal << " badCount " << depthEstimators.at(ii)->badCount << " numSaved " << depthEstimators.at(ii)->depthEstimatorICLExt.numSaved  << std::endl;
		// if ((chiTestVal < flowchi2) || (timeFromStart < 0.25))
		// {
		// 	pPtsInFlow.push_back(pPts.at(ii));
		// 	// cPtsInFlow.push_back(pPts.at(ii)+flowDiffPt);
		// 	cPtsInFlow.push_back(cPts.at(ii));
		// 	kPtsInFlow.push_back(kPts.at(ii));
		// 	depthEstimatorsInFlow.push_back(depthEstimators.at(ii));
		// }
		// else
		// {
		// 	delete depthEstimators.at(ii);
		// }
		if ((timeFromStart < 0.25))
		{
			pPtsInFlow.push_back(pPts.at(ii));
			cPtsInFlow.push_back(cPts.at(ii));
			kPtsInFlow.push_back(kPts.at(ii));
			depthEstimatorsInFlow.push_back(depthEstimators.at(ii));
		}
		else
		{
			// if ((depthEstimators.at(ii)->badCount > 1) || ( (avgdkKnown > 0.6) && !depthEstimators.at(ii)->depthEstimatorICLExt.dkKnown))
			// if (((depthEstimators.at(ii)->badCount > 1) || (numWithSaved > (minFeaturesBad+5))) && (depthEstimators.at(ii)->depthEstimatorICLExt.numSaved < 1))
			if ((depthEstimators.at(ii)->badCount > 1) || ((numWithSaved > (minFeaturesBad+5)) && (depthEstimators.at(ii)->depthEstimatorICLExt.numSaved < 1)))
			{
				delete depthEstimators.at(ii);
			}
			else
			{
				if (chiTestVal < flowchi2)
				{
					pPtsInFlow.push_back(pPts.at(ii));
					cPtsInFlow.push_back(cPts.at(ii));
					kPtsInFlow.push_back(kPts.at(ii));
					depthEstimatorsInFlow.push_back(depthEstimators.at(ii));
				}
				else
				{
					pPtsInFlow.push_back(pPts.at(ii));
					cPtsInFlow.push_back(pPts.at(ii)+avgFlow);
					kPtsInFlow.push_back(kPts.at(ii));
					depthEstimatorsInFlow.push_back(depthEstimators.at(ii));
				}
			}
		}
	}
	pPts = pPtsInFlow;
	cPts = cPtsInFlow;
	kPts = kPtsInFlow;
	depthEstimators = depthEstimatorsInFlow;

	featureMutex.unlock();

	std::cout << "\n num points after flow " << depthEstimatorsInFlow.size() << std::endl;
	// std::cout << "\n medDiffx " << flowDiffPt.x << ", medDiffy " << flowDiffPt.y << std::endl;
	std::cout << "\n qAng " << qAng << std::endl;
	std::cout << "\n currentBound.width " << currentBound.width << std::endl;
	std::cout << "\n currentBound.height " << currentBound.height << std::endl;
	std::cout << "\n avgNumSaved " << avgNumSaved << std::endl;
	std::cout << "\n avgNumThrown " << avgNumThrown << std::endl;

	// if (normGood && angGood)
	if ((qAng < 15.0) && (currentBound.width > (numberFeaturesPerPartCol-1)*50) && (currentBound.height > (numberFeaturesPerPartRow-1)*50) && (depthEstimatorsInFlow.size() > minFeaturesBad))
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
			// std::vector<cv::Point2f> pPtsTemp = pPts;
			// std::vector<cv::Point2f> cPtsTemp = cPts;
			// cv::Mat G;
			// cv::Mat G = cv::estimateAffine2D(pPtsTemp, cPtsTemp, inliersAffine, cv::RANSAC, 5.0, 2000, 0.99, 10);//calculate affine transform using RANSAC
			// cv::Mat G = cv::estimateAffine2D(pPtsTemp, cPtsTemp, inliersAffine, cv::LMEDS);//calculate affine transform using RANSAC
			cv::Mat G = (cv::Mat_<double>(2,3) << 1.0, 0.0, avgFlow.x, 0.0, 1.0, avgFlow.y);
			// std::cout << "\n Gaff \n" << G << std::endl;
			// cv::Mat G32;
			// G.convertTo(G32,CV_32F);
			//
			// double corrECC = cv::findTransformECC(pimage(currentBound),image(currentBound),G32);
			// G32.convertTo(G,CV_64F);
			// cv::Mat GG = cv::findHomography(pPts, cPts, cv::RANSAC, 8.0, inliersAffine, 2000, 0.99);//calculate homography using RANSAC
			// cv::Mat G = cv::Mat(2,3,CV_64F);
			// for (int ii = 0; ii < 6; ii++)
			// {
			// 	G.at<double>(ii/3,ii%3) = GG.at<double>(ii/3,ii%3);
			// }

			//find G using previous and current image only
			// {

				// //find the best corners in the masked image
				// std::vector<cv::Point2f> pcorners,ccorners;
				// cv::goodFeaturesToTrack(pimage,pcorners,25,0.001,30,cv::Mat(),3);
				// cv::goodFeaturesToTrack(image,ccorners,25,0.001,30,cv::Mat(),3);
				// cv::cornerSubPix(pimage,pcorners,cv::Size(3,3),cv::Size(-1,-1),cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 0.1));
				// cv::cornerSubPix(image,ccorners,cv::Size(3,3),cv::Size(-1,-1),cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 0.1));
				//
				// if (pcorners.size() != ccorners.size())
				// {
				// 	int sizenew = std::min(int(pcorners.size()),int(ccorners.size()));
				// 	pcorners.resize(sizenew);
				// 	ccorners.resize(sizenew);
				// }
				//
				// std::cout << "\n p.size() " << pcorners.size() << " c.size() " << ccorners.size() << std::endl;
				//
				// // find the closest for the match
				// std::vector<cv::Point2f> ccornersMatch(ccorners.size());
				// for (int ii = 0; ii < pcorners.size(); ii++)
				// {
				// 	float diffjx = ccorners.at(0).x-pcorners.at(ii).x;
				// 	float diffjy = ccorners.at(0).y-pcorners.at(ii).y;
				// 	float diffjnorm = sqrtf(diffjx*diffjx+diffjy*diffjy);
				// 	float diffmin = diffjnorm;
				// 	int diffminInd = 0;
				// 	for (int jj = 1; jj < ccorners.size(); jj++)
				// 	{
				// 		diffjx = ccorners.at(jj).x-pcorners.at(ii).x;
				// 		diffjy = ccorners.at(jj).y-pcorners.at(ii).y;
				// 		diffjnorm = sqrtf(diffjx*diffjx+diffjy*diffjy);
				// 		if (diffjnorm < diffmin)
				// 		{
				// 			diffmin = diffjnorm;
				// 			diffminInd = jj;
				// 		}
				// 	}
				// 	ccornersMatch.at(ii) = ccorners.at(diffminInd);
				// 	std::cout << ii << " diffmin " << diffmin;
				// 	std::cout << ", ctox-ptox " << ccornersMatch.at(ii).x-pcorners.at(ii).x;
				// 	std::cout << ", ctoy-ptoy " << ccornersMatch.at(ii).y-pcorners.at(ii).y;
				// 	std::cout << std::endl;
				// }
				//
				// cv::Mat inliersCorners;
				// cv::Mat GCorners = cv::estimateAffine2D(pcorners,ccornersMatch,inliersCorners, cv::RANSAC, 4.0, 2000, 0.99, 10);//calculate affine transform using RANSAC
				// G = GCorners;
				// for (int ii = 0; ii < ccornersMatch.size(); ii++)
				// {
				// 	std::cout << ii << " " << int(inliersCorners.at<uchar>(ii)) << std::endl;
				// }
				// std::cout << "\n GCorners \n" << GCorners << std::endl;

		    // std::vector<cv::KeyPoint> pkeypoints, ckeypoints;
		    // cv::Mat pdescriptors, cdescriptors;
				// //int nfeatures=500, float scaleFactor=1.2f, int nlevels=8, int edgeThreshold=31, int firstLevel=0, int WTA_K=2, int scoreType=ORB::HARRIS_SCORE, int patchSize=31
		    // cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create(100,1.2,1);
		    // cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
				//
		    // cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
		    // detector->detect(pimage,pkeypoints);
		    // detector->detect(image,ckeypoints);
		    // descriptor->compute(pimage,pkeypoints,pdescriptors);
		    // descriptor->compute(image,ckeypoints,cdescriptors);
				//
		    // std::vector<cv::DMatch> matches;
		    // matcher->match(cdescriptors,pdescriptors,matches);
				//
				// std::vector<cv::Point2f> pPtsOrb(matches.size()),cPtsOrb(matches.size());
				// for (int ii = 0; ii < matches.size(); ii++)
				// {
				// 	std::cout << ii << ", distance " << matches.at(ii).distance;
				// 	// std::cout << ", imgIdx " << matches.at(ii).imgIdx;
				// 	// std::cout << ", queryIdx " << matches.at(ii).queryIdx;
				// 	std::cout << ", trainIdx " << matches.at(ii).trainIdx;
				// 	pPtsOrb.at(ii) = pkeypoints.at(matches.at(ii).trainIdx).pt;
				// 	cPtsOrb.at(ii) = ckeypoints.at(matches.at(ii).queryIdx).pt;
				// 	std::cout << ", ctox-ptox " << cPtsOrb.at(ii).x-pPtsOrb.at(ii).x;
				// 	std::cout << ", ctoy-ptoy " << cPtsOrb.at(ii).y-pPtsOrb.at(ii).y;
				// 	std::cout << std::endl;
				// }
				// cv::Mat inliersOrb;
				// cv::Mat Gorb = cv::estimateAffine2D(pPtsOrb,cPtsOrb,inliersOrb, cv::RANSAC, 4.0, 2000, 0.99, 10);//calculate affine transform using RANSAC
				// for (int ii = 0; ii < pPtsOrb.size(); ii++)
				// {
				// 	std::cout << ii << " " << int(inliersOrb.at<uchar>(ii)) << std::endl;
				// }
				// std::cout << "\n Gorb \n" << Gorb << std::endl;
				// G = Gorb;
			// }

			// for (int ii = 0; ii < cPts.size(); ii++)
			// {
			// 	std::cout << " ptix " << pPts.at(ii).x << " ptiy " << pPts.at(ii).y << std::endl;
			// 	std::cout << " ctix " << cPts.at(ii).x << " ctiy " << cPts.at(ii).y << std::endl;
			// }
			// cv::Mat G = cv::findHomography(pPts, cPts, cv::RANSAC, 0.05, inliersAffine, 2000, 0.99);//calculate homography using RANSAC
			// cv::Mat G = cv::findHomography(pPts, cPts, cv::RANSAC, 3.0, inliersAffine, 2000, 0.99);//calculate homography using RANSAC
			// cv::Mat G = cv::findHomography(kPts, cPts, cv::RANSAC, 0.05, inliersAffine, 2000, 0.99);//calculate homography using RANSAC
			// cv::Mat G = cv::findHomography(pPts, cPts, 0);

			// cv::Mat G = cv::findHomography(pPts, cPts, cv::LMEDS, 4.0, inliersAffine, 2000, 0.99);//calculate homography using RANSAC
			// cv::Mat G = cv::findHomography(pPts, cPts, 0);//calculate homography using RANSAC
			// std::cout << std::endl;
			// for (std::vector<cv::Point2f>::iterator itpp = cPts.begin(); itpp != cPts.end(); itpp++)
			// {
			// 	std::cout << "a cptx " << (*itpp).x << " cpty " << (*itpp).y << std::endl;
			// }
			// cv::Mat G = cv::Mat::eye(3,3,CV_64F);

			// // Eigen::Matrix<float,3,3> Gkf,Gf;
			// Eigen::Matrix<float,2,3> Gf;
			// for (int ii = 0; ii < 6; ii++)
			// {
			// 	Gf(ii/3,ii%3) = G.at<double>(ii/3,ii%3);
			// }
			//
			// if (!G.empty())
			// {
			// 	GfLast += kT*(Gf - GfLast);
			// }
			//
			// for (int ii = 0; ii < 6; ii++)
			// {
			// 	G.at<double>(ii/3,ii%3) = GfLast(ii/3,ii%3);
			// }

			float sigEst = 5;
			float sigEst2 = sigEst*sigEst;
			// float chi2 = 3.84; //chi^2 for 95%
			// float chi2 = 6.63; //chi^2 for 99%
			Eigen::Vector3f pPtif;
			Eigen::Vector2f cPtif,cPtiEst,cPtiD;
			float chiTestValGICL = 0.0;
			float chiTestValG = 0.0;
			float normalizei = 0.0;
			featureMutex.lock();
			std::cout << "\n cpts.size() inlier before " << cPts.size() << std::endl;
			for (int ii = 0; ii < cPts.size(); ii++)
			{
				// if (inliersAffine.at<uchar>(ii) || (timeFromStart < 0.25))
				if (true)
				{
					depthEstimatorsInH.push_back(depthEstimators.at(ii));
					pPtsInH.push_back(pPts.at(ii));
					cPtsInH.push_back(cPts.at(ii));
					kPtsInH.push_back(kPts.at(ii));
				}
				else
				{
					delete depthEstimators.at(ii);
				}
				// for (std::vector<cv::Point2f>::iterator itpp = cPts.begin(); itpp != cPts.end(); itpp++)
				// {
				// 	std::cout << "b cptx " << (*itpp).x << " cpty " << (*itpp).y << std::endl;
				// }
				// cv::Mat G = cv::estimateAffine2D(pPts, cPts, inliersAffine, cv::RANSAC, 4.0, 2000, 0.99, 10);//calculate affine transform using RANSAC
				// cv::Mat G = cv::findHomography(pPts, cPts, cv::RANSAC, 0.05, inliersAffine, 2000, 0.99);//calculate homography using RANSAC
				// cv::Mat G = cv::findHomography(pPts, cPts, cv::RANSAC, 0.05, inliersAffine, 2000, 0.99);
			}
			depthEstimators = depthEstimatorsInH;
			featureMutex.unlock();
			chiTestValGICL /= float(cPtsInH.size());
			chiTestValG /= float(cPtsInH.size());
			pPts = pPtsInH;
			cPts = cPtsInH;
			kPts = kPtsInH;
			std::cout << "\n cpts.size() inlier after " << cPts.size() << std::endl;
			//
			// Eigen::Matrix3f Goutf = ((sigEst2+chiTestValGICL)*GfLast+(sigEst2+chiTestValG)*GICLf)/((sigEst2+chiTestValGICL)+(sigEst2+chiTestValG));
			// Goutf /= Goutf(2,2);
			//
			// cv::Mat Gout(cv::Size(3,3),CV_64F);
			// for (int ii = 0; ii < 9; ii++)
			// {
			// 	Gout.at<double>(ii/3,ii%3) = Goutf(ii/3,ii%3);
			// }

			std::cout << "\n G \n" << G << std::endl;
			// std::cout << "\n GICL \n" << GICL << std::endl;
			// std::cout << "\n Gout \n" << Gout << std::endl;
			// std::cout << "\n chiTestValGICL " << chiTestValGICL << std::endl;
			// std::cout << "\n chiTestValG " << chiTestValG << std::endl;
			// cv::Mat imageWarp(image.size(),image.type());
			// cv::warpAffine(image,imageWarp,G,image.size(),cv::INTER_LINEAR,cv::BORDER_CONSTANT,cv::Scalar(255,255,255));
			//

			// std::vector<cv::Point2f> ppts;
			// ppts.clear();
			featureMutex.lock();
			findPoints(image,kPts,pPts,cPts,G,pkc);
			featureMutex.unlock();
			kPtsInPred = kPts;
			cPtsInPred = cPts;

			// //check if any of the features are too close
			// std::vector<cv::Point2f> cPtsInPredToCheck = cPtsInPred;
			// float minDistanceCheck = minDistance;
			// float minDistanceCheckNew = minDistance;
			// while (cPtsInPredToCheck.size() > 1)
			// {
			// 	for (int jjToCheck = 0; jjToCheck < cPtsInPredToCheck.size()-1; jjToCheck++)
			// 	{
			// 		minDistanceCheckNew = sqrtf(std::pow(cPtsInPredToCheck.at(jjToCheck).x-cPtsInPredToCheck.back().x,2.0)+std::pow(cPtsInPredToCheck.at(jjToCheck).y-cPtsInPredToCheck.back().y,2.0));
			// 		if (minDistanceCheckNew < minDistanceCheck)
			// 		{
			// 			minDistanceCheck = minDistanceCheckNew;
			// 		}
			// 	}
			// 	cPtsInPredToCheck.pop_back();
			// }
			//
			// if (minDistanceCheck < 10)
			// {
			// 	ROS_ERROR("min distance failed");
			// 	featureMutex.lock();
			// 	for (std::vector<DepthEstimator*>::iterator itD = depthEstimators.begin() ; itD != depthEstimators.end(); itD++)
			// 	{
			// 		delete *itD;
			// 	}
			//
			// 	depthEstimators.clear();
			// 	featureMutex.unlock();
			// 	pPts.clear();
			// 	cPts.clear();
			// 	kPts.clear();
			// 	kPtsInPred.clear();
			// 	cPtsInPred.clear();
			// }
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
		if ((qAng >= 30.0))
		{
			ROS_ERROR("angle off");
		}
		if ((currentBound.width <= (numberFeaturesPerPartCol-1)*50) || (currentBound.height <= (numberFeaturesPerPartRow-1)*50))
		{
			ROS_ERROR("area to small");
		}
		if ((depthEstimators.size() <= minFeaturesBad))
		{
			ROS_ERROR("too few features");
		}

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

void PatchEstimator::findPoints(cv::Mat& image, std::vector<cv::Point2f>& kPts, std::vector<cv::Point2f>& pPts, std::vector<cv::Point2f>& cPts, cv::Mat& G, Eigen::Vector3f pkc)
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
	std::cout << "\n size before " << cPts.size() << std::endl;

	cv::Mat GI;
	cv::invertAffineTransform(G,GI);

	//correct points and remove the outliers
	std::vector<cv::Point2f> kPtsInPred(depthEstimators.size()),cPtsInPred(depthEstimators.size()),pPtsInPred(depthEstimators.size());//,cPtPsInPred(depthEstimators.size());
	std::vector<cv::Point2f> flowsInPred(pPts.size());
	std::vector<DepthEstimator*> depthEstimatorsInPred(depthEstimators.size());
	std::vector<DepthEstimator*>::iterator itDP = depthEstimators.begin();
	std::vector<DepthEstimator*>::iterator itDPPred = depthEstimatorsInPred.begin();
	std::vector<cv::Point2f>::iterator itcPred = cPtsInPred.begin();
	std::vector<cv::Point2f>::iterator itpPred = pPtsInPred.begin();
	std::vector<cv::Point2f>::iterator itkPred = kPtsInPred.begin();
	std::vector<cv::Point2f>::iterator itfPred = flowsInPred.begin();

	std::vector<cv::Point2f>::iterator itc = cPts.begin();
	std::vector<cv::Point2f>::iterator itk = kPts.begin();
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
	// std::cout << "\n G " << G << std::endl;

	cv::Point2f avgFlow(0.0,0.0);
	for (std::vector<cv::Point2f>::iterator itpp = pPts.begin(); itpp != pPts.end(); itpp++)
	{
		try
		{
			// std::cout << "\n\n\n --------- next start --------- \n";
			// get center point in new image, find the patch box around it, transform into previous image,
			// check to ensure it fits in previous image
			// if patch does fit, extract the patch then transform extracted patch back and perform search
			// if patch does not fit, skip the point so partition is deleted

			// std::cout << "\n hi1 \n";

			//predicted patch center in new image
			pptx = (*itpp).x;
			ppty = (*itpp).y;
			cptx = (*itc).x;
			cpty = (*itc).y;

			// std::cout << "\n pptx " << pptx << " ppty " << ppty << std::endl;
			// std::cout << "\n cptx " << cptx << " cpty " << cpty << std::endl;
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
			cv::transform(ppatchCornersC,ppatchCornersP,GI);
			ppatchRectP = cv::boundingRect(ppatchCornersP);

			// std::cout << "\n GI \n" << GI << std::endl;

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

				// std::cout << "\n hi3 \n";

				//extract and transform bounding patch
				cv::warpAffine(ppatchBoundP,ppatchBoundC,G,ppatchBoundP.size(),cv::INTER_LINEAR,cv::BORDER_CONSTANT,cv::Scalar(255,255,255));

				//transform the bounding corners back to remove the excess of bounding image
				cv::transform(ppatchBoundCornersP,ppatchBoundCornersC,G);

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

				// std::cout << "\n ppatchRectC " << ppatchRectC << std::endl;
				// std::cout << "\n cDx " << cDx << " cDy " << cDy << " patchSize " << patchSize << "  ppatchBoundC.cols " <<  ppatchBoundC.cols << "  ppatchBoundC.rows " <<  ppatchBoundC.rows << std::endl;


				if ((ppatchRectC.x >= 0) && (ppatchRectC.y >= 0) && (ppatchRectC.x < ppatchBoundC.cols) && (ppatchRectC.y < ppatchBoundC.rows)
						&& ((ppatchRectC.x+ppatchRectC.width) < (ppatchBoundC.cols)) && ((ppatchRectC.y+ppatchRectC.height) < (ppatchBoundC.rows))
					  && (ppatchRectC.width > 0) && (ppatchRectC.height > 0))
				{
					warpGood = true;
				}
			}

			//if the bounding rectangle points are within the previous image
			// if ((acos(qkcHat(0))*2.0 < 45.0*3.1415/180.0) && warpGood
			if (warpGood
					&& (pcheckRect.x > 0) && (pcheckRect.y > 0)
					&& ((pcheckRect.x+checkSize) < (imageWidth)) && ((pcheckRect.y+checkSize) < (imageHeight)))
			{
				// std::cout << "\n hi33 \n";

				ppatch = ppatchBoundC(ppatchRectC).clone();

				// ppatch = ppatchBoundC.clone();

				// std::cout << "\n hi4 \n";

				//extract the check image
				cpatch = image(pcheckRect).clone();

				// std::cout << "\n hi44 \n";

				// cv::Mat pSobel,cSobel;
				// cv::preCornerDetect(ppatch, pSobel, 5);
				// cv::preCornerDetect(cpatch, cSobel, 5);
				// cv::Mat ppatch32,cpatch32;
				// ppatch.convertTo(ppatch32,CV_32F);
				// cpatch.convertTo(cpatch32,CV_32F);
				//
				// /// Generate grad_x and grad_y
				// cv::Mat pgrad_x, pgrad_y, cgrad_x, cgrad_y;
				// cv::Mat pabs_grad_x, pabs_grad_y, cabs_grad_x, cabs_grad_y;
				// /// Gradient X
				// //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
				// cv::Sobel( ppatch, pgrad_x, CV_ 32F, 1, 0, 3);
				// cv::Sobel( ppatch, pgrad_y, CV_32F, 0, 1, 3);
				// cv::convertScaleAbs( pgrad_x, pabs_grad_x );
				// cv::convertScaleAbs( pgrad_y, pabs_grad_y );
				// cv::addWeighted( pabs_grad_x, 0.5, pabs_grad_y, 0.5, 0, pSobel );
				// cv::Sobel( cpatch, cgrad_x, CV_32F, 1, 0, 3);
				// cv::Sobel( cpatch, cgrad_y, CV_32F, 0, 1, 3);
				// cv::convertScaleAbs( cgrad_x, cabs_grad_x );
				// cv::convertScaleAbs( cgrad_y, cabs_grad_y );
				// cv::addWeighted( cabs_grad_x, 0.5, cabs_grad_y, 0.5, 0, cSobel );
				// /// Gradient Y
				// //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
				//
				// cv::Mat pCompare,cCompare;
				// cv::addWeighted( ppatch32, 0.75, pSobel, 0.25, 0, pCompare);
				// cv::addWeighted( cpatch32, 0.75, cSobel, 0.25, 0, cCompare);

				/// Total Gradient (approximate)

				// std::cout << "\n ppatchRectCLocal \n" << ppatchRectCLocal << std::endl;



				// std::cout << "\n cpatch.size() " << cpatch.size() << " ppatch.size() " << ppatch.size() << std::endl;

				//use the patch as template and match in check patch
				// cv::matchTemplate(cpatch,ppatch,tmresult,cv::TM_SQDIFF_NORMED);
				// cv::matchTemplate(cpatch,reducedWarp,tmresult,cv::TM_CCORR_NORMED);
				// cv::matchTemplate(cCompare,pCompare,tmresult,cv::TM_CCOEFF_NORMED);
				cv::matchTemplate(cpatch,ppatch,tmresult,cv::TM_CCOEFF_NORMED);
				// cv::matchTemplate(cSobel,pSobel,tmresult,cv::TM_CCOEFF_NORMED);

				// std::cout << "\n tmresult.size() " << tmresult.size() << std::endl;

				// cv::GaussianBlur(tmresult,tmresultBlur,cv::Size(blurSize,blurSize),0);
				cv::minMaxLoc(tmresult,&minResultVal,&maxResultVal,&minResultPt,&maxResultPt);

				// std::cout << "\n hi5 \n";

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



				// std::cout << "\n maxx " << maxResultPt.x << " maxy " << maxResultPt.y << std::endl;
				// std::cout << "\n maxResultValx " << maxResultVal << std::endl;
				//
				// std::cout << "\n mx " << pcheckRect.x+ppatchRectC.width/2.0+maxResultPt.x << " my " << pcheckRect.y+ppatchRectC.height/2.0+maxResultPt.y << std::endl;

				// *itcPred = cv::Point2f(pcheckRect.x+(ppatchRectC.width)/2.0+maxResultPt.x,pcheckRect.y+ppatchRectC.height/2.0+maxResultPt.y);

				// std::cout << "\n maxx " << maxResultPt.x << " maxy " << maxResultPt.y << std::endl;
				// std::cout << "\n pptx " << pptx << " ppty " << ppty << std::endl;
				// std::cout << "\n cptx " << cptx << " cpty " << cpty << std::endl;
				// std::cout << "\n (*itc).x " << (*itc).x << " (*itc).y " << (*itc).y << std::endl;

				// std::cout << "\n maxResultValx " << maxResultVal << std::endl;
				//
				// std::cout << "\n mx " << pcheckRect.x+ppatchRectC.width/2.0+maxResultPt.x << " my " << pcheckRect.y+ppatchRectC.height/2.0+maxResultPt.y << std::endl;


				float sigEst = 5.0;
				float sigEst2 = sigEst*sigEst;
			  // float chi2 = 3.84; //chi^2 for 95%
			  float chi2 = 6.63; //chi^2 for 99%
				float cptxEst = cptx-checkSize/2.0+ppatchRectC.width/2.0+maxResultPt.x;
				float cptyEst = cpty-checkSize/2.0+ppatchRectC.width/2.0+maxResultPt.y;
			  Eigen::Vector2f cPtD = Eigen::Vector2f(cptx - cptxEst,cpty - cptyEst);
			  float chiTestVal = (cPtD(0)*cPtD(0) + cPtD(1)*cPtD(1))/sigEst2;

				float sigProj = 20.0;
				float sigProj2 = sigProj*sigProj;
				float sig2Sum = sigEst2+sigProj2;
				// float chi2 = 3.84; //chi^2 for 95%

				// float cPtAlpha = 1.0/(2.0 + chiTestVal);

				// float avgx = (1.0-alphapr)*cptx+alphapr*(pcheckRect.x+ppatchRectC.width/2.0+maxResultPt.x);
				// float avgy = (1.0-alphapr)*cpty+alphapr*(pcheckRect.y+ppatchRectC.height/2.0+maxResultPt.y);
				// float avgx = (sigEst2/sig2Sum)*cptx+(sigProj2/sig2Sum)*cptxEst;
				// float avgy = (sigEst2/sig2Sum)*cpty+(sigProj2/sig2Sum)*cptyEst;
				float avgx = cptxEst;
				float avgy = cptyEst;

				std::cout << "\n shiftx " << (avgx-pptx) << " shifty " << (avgy-ppty);
				// std::cout << "\n pptx " << pptx << " ppty " << ppty;
				// std::cout << " cptx " << cptx << " cpty " << cpty;
				// std::cout << " cptxEst " << cptxEst << " cptyEst " << cptyEst;
				// std::cout << "\n avgx " << avgx << " avgy " << avgy << std::endl;
				std::cout << " maxResultVal " << maxResultVal << " minResultVal " << minResultVal << " chiTestVal " << chiTestVal << std::endl;

				// if ((maxResultVal > 0.6) && (minResultVal < 0.3))
				if ((maxResultVal > 0.4))
				{
						*itcPred = cv::Point2f(avgx,avgy);
						*itDPPred = *itDP;
						*itkPred = *itk;
						*itfPred = ((*itcPred)-(*itpp));
						*itpPred = *itpp;
						avgFlow += (*itfPred);

						itDPPred++;
						itcPred++;
						itpPred++;
						itkPred++;
						itfPred++;
						numPtsPred++;
				}
				else
				{
					delete *itDP;
					std::cout << "\n patch prob to low\n";
				}
			}
			else
			{
				delete *itDP;
				std::cout << "\n check to large for previous \n";
			}
		}
		catch (cv::Exception e)
		{
			delete *itDP;
			std::cout << "\n patch failed \n";
		}

		itDP++;
		itc++;
		itk++;

	}

	if (numPtsPred > 0)
	{
		avgFlow /= float(numPtsPred);
	}

	std::cout << "\n avgShiftx " << avgFlow.x << " avgShifty " << avgFlow.y << std::endl;
	depthEstimatorsInPred.resize(numPtsPred);
	cPtsInPred.resize(numPtsPred);
	pPtsInPred.resize(numPtsPred);
	kPtsInPred.resize(numPtsPred);
	flowsInPred.resize(numPtsPred);
	// cPtsInPred.resize(numPtsPred);
	// ROS_WARN("depthEstimators size after predict1 %d",int(depthEstimators.size()));
	// ROS_WARN("depthEstimatorsInPred size after predict1 %d",int(depthEstimatorsInPred.size()));
	depthEstimators = depthEstimatorsInPred;
	depthEstimatorsInPred.clear();

	// cv::cornerSubPix(image,cPtsInPred,cv::Size(3,3),cv::Size(-1,-1),cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
	pPts = pPtsInPred;
	cPts = cPtsInPred;
	if (cPts.size() > 0)
	{
		float sigPred = 3.0;
		float sigPred2 = sigPred*sigPred;
		// float chi2 = 3.84; //chi^2 for 95%
		float chi2 = 6.63; //chi^2 for 99%
		cv::Point2f cPtD(0.0,0.0);
		float chiTestVal = 0.0;
		//adjust points outside of average flow
		for (int ii = 0; ii < flowsInPred.size(); ii++)
		{
			cPtD = flowsInPred.at(ii) - avgFlow;
			chiTestVal = (cPtD.x*cPtD.x + cPtD.y*cPtD.y)/sigPred2;
			if (chiTestVal > chi2)
			{
				cPts.at(ii) = pPts.at(ii)+avgFlow;
				depthEstimators.at(ii)->badCount++;
			}
		}
		cv::cornerSubPix(image,cPts,cv::Size(3,3),cv::Size(-1,-1),cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 0.001));
	}

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

		Eigen::Vector4f qkcPnPc;
		Eigen::Vector3f tkcPnPc;
		Eigen::Vector4f qkcPnPk;
		Eigen::Vector3f tkcPnPk;
		bool usePnPc = false;
		bool usePnPk = false;

		// std::cout << "\n tryingPnPk \n" << std::endl;
		std::vector<cv::Point3f> kPts3(depthEstimators.size()),cPts3(depthEstimators.size());
		std::vector<cv::Point2f> cPtsNorm(depthEstimators.size()),kPtsNorm(depthEstimators.size());
		std::vector<cv::Point2f> kPtsTemp(depthEstimators.size()),cPtsTemp(depthEstimators.size());
		std::vector<cv::Point3f>::iterator kPts3it = kPts3.begin();
		std::vector<cv::Point3f>::iterator cPts3it = cPts3.begin();
		std::vector<cv::Point2f>::iterator kPtsit = kPts.begin();
		std::vector<cv::Point2f>::iterator cPtsit = cPts.begin();
		std::vector<cv::Point2f>::iterator kPtsNormit = kPtsNorm.begin();
		std::vector<cv::Point2f>::iterator cPtsNormit = cPtsNorm.begin();
		std::vector<cv::Point2f>::iterator kPtsTempit = kPtsTemp.begin();
		std::vector<cv::Point2f>::iterator cPtsTempit = cPtsTemp.begin();
		Eigen::Vector3f uci,uki;
		float DdcHati = 0.0;
		for (std::vector<DepthEstimator*>::iterator itD = depthEstimators.begin() ; itD != depthEstimators.end(); itD++)
		{
			float dkHati = (*itD)->dkHatICLExt;
			float dcHati = (*itD)->dcHatICLExt;
			*kPtsNormit = cv::Point2f(((*kPtsit).x-cx)/fx,((*kPtsit).y-cy)/fy);
			*cPtsNormit = cv::Point2f(((*cPtsit).x-cx)/fx,((*cPtsit).y-cy)/fy);
			*kPtsTempit = *kPtsit;
			*cPtsTempit = *cPtsit;
			uki = Eigen::Vector3f(((*kPtsit).x-cx)/fx,((*kPtsit).y-cy)/fy,1.0);
			uki /= uki.norm();
			uci = Eigen::Vector3f(((*cPtsit).x-cx)/fx,((*cPtsit).y-cy)/fy,1.0);
			uci /= uci.norm();
			DdcHati = -float(uci.transpose()*vc)*dt;
			(*kPts3it).x = uki(0)*dkHati;
			(*kPts3it).y = uki(1)*dkHati;
			(*kPts3it).z = uki(2)*dkHati;
			(*cPts3it).x = uci(0)*(dcHati+DdcHati);
			(*cPts3it).y = uci(1)*(dcHati+DdcHati);
			(*cPts3it).z = uci(2)*(dcHati+DdcHati);
			kPtsNormit++;
			cPtsNormit++;
			kPtsTempit++;
			cPtsTempit++;
			kPts3it++;
			cPts3it++;
			kPtsit++;
			cPtsit++;
		}

		{
			Eigen::Matrix3f RkcHatPnPf = getqRot(qkc);
			cv::Mat RkcHatPnP;
			cv::eigen2cv(RkcHatPnPf,RkcHatPnP);
			cv::Mat rvec(cv::Size(1,3),CV_32F);
			cv::Mat tvec(cv::Size(1,3),CV_32F);
			cv::Rodrigues(RkcHatPnP,rvec);
			cv::eigen2cv(pkc,tvec);
			// std::cout << "rvecPnPkb \n" << rvec << std::endl;
			// std::cout << "tvecPnPkb \n" << tvec << std::endl;
			// usePnPk = cv::solvePnP(kPts3,cPts,camMat,cv::Mat(),rvec,tvec,true,cv::SOLVEPNP_ITERATIVE);
			int PnPkIn = 3.0;
			while (!usePnPk && (PnPkIn < 5))
			{
				usePnPk = cv::solvePnPRansac(kPts3,cPts,camMat,cv::noArray(),rvec,tvec,true,100,float(PnPkIn),0.99);
				PnPkIn++;
			}

			// std::cout << "rvecPnPka \n" << rvec << std::endl;
			// std::cout << "tvecPnPka \n" << tvec << std::endl;


			if (usePnPk)
			{
				cv::Rodrigues(rvec,RkcHatPnP);
				cv::cv2eigen(RkcHatPnP,RkcHatPnPf);
				Eigen::Quaternionf qkcPnPq(RkcHatPnPf);// convert to quaternion
				qkcPnPk << qkcPnPq.w(),qkcPnPq.x(),qkcPnPq.y(),qkcPnPq.z();
				qkcPnPk /= qkcPnPk.norm();
				cv::cv2eigen(tvec,tkcPnPk);
				float tkcPnPNorm = tkcPnPk.norm();
				if (tkcPnPNorm>0.001)
				{
					tkcPnPk /= tkcPnPNorm;
				}
				// std::cout << std::endl;
				std::cout << "dkcPnPk \n" << tkcPnPNorm << std::endl;
				// std::cout << "dkcPnPk \n" << tkcPnPNorm << std::endl;
				// std::cout << "tkcPnPk \n" << tkcPnPk << std::endl;
				// std::cout << "qkcPnPk \n" << qkcPnPk << std::endl;
			}
		}

		ROS_WARN("time for pnp %2.4f",float(clock()-updateClock)/CLOCKS_PER_SEC);
		updateClock = clock();

		// cv::Mat TA;
		// cv::estimateAffine3D(kPts3,cPts3,TA,cv::noArray(),3.0,0.99);
		//
		// std::cout << "\n TA \n" << TA << std::endl;


		{
			// std::cout << "\n tryingPnP current \n" << std::endl;
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
			// std::cout << "rvecPnPcb \n" << rvec << std::endl;
			// std::cout << "tvecPnPcb \n" << tvec << std::endl;
			// usePnPc = cv::solvePnP(cPts3,kPts,camMat,cv::Mat(),rvec,tvec,true,cv::SOLVEPNP_ITERATIVE);
			// usePnPc = cv::solvePnPRansac(cPts3,kPts,camMat,cv::Mat(),rvec,tvec,true,100,3.0);
			int PnPcIn = 3.0;
			while (!usePnPc && (PnPcIn < 5))
			{
				usePnPc = cv::solvePnPRansac(cPts3,kPts,camMat,cv::noArray(),rvec,tvec,true,100,float(PnPcIn),0.99);
				PnPcIn++;
			}
			// std::cout << "rvecPnPca \n" << rvec << std::endl;
			// std::cout << "tvecPnPca \n" << tvec << std::endl;

			if (usePnPc)
			{
				cv::Rodrigues(rvec,RckHatPnP);
				cv::cv2eigen(RckHatPnP,RckHatPnPf);
				Eigen::Quaternionf qckPnPq(RckHatPnPf);// convert to quaternion
				Eigen::Vector4f qckPnP;
				qckPnP << qckPnPq.w(),qckPnPq.x(),qckPnPq.y(),qckPnPq.z();
				qkcPnPc = getqInv(qckPnP);
				qkcPnPc /= qkcPnPc.norm();
				Eigen::Vector3f tckPnP;
				cv::cv2eigen(tvec,tckPnP);
				tkcPnPc = -rotatevec(tckPnP,qkcPnPc);
				float tkcPnPNorm = tkcPnPc.norm();
				if (tkcPnPNorm>0.001)
				{
					tkcPnPc /= tkcPnPNorm;
				}
				std::cout << std::endl;
				// std::cout << "dkcPnPc \n" << tkcPnPNorm << std::endl;
				// std::cout << "tkcPnPc \n" << tkcPnPc << std::endl;
				// std::cout << "qkcPnPc \n" << qkcPnPc << std::endl;
			}
		}

		// std::vector<cv::Point2f> kPtsTemp = kPts;
		// std::vector<cv::Point2f> cPtsTemp = cPts;
		// cv::Mat G = cv::estimateAffine2D(pPtsTemp, cPtsTemp, inliersAffine, cv::RANSAC, 5.0, 2000, 0.99, 10);//calculate affine transform using RANSAC
		// cv::Mat inliersGA;
		// cv::Mat GAtemp = cv::estimateAffine2D(kPtsTemp, cPtsTemp, inliersGA, cv::LMEDS);//calculate affine transform using RANSAC
		// cv::Mat GA = (cv::Mat_<double>(3,3) << GAtemp.at<double>(0,0),GAtemp.at<double>(0,1),GAtemp.at<double>(0,2),GAtemp.at<double>(1,0),GAtemp.at<double>(1,1),GAtemp.at<double>(1,2),0.0,0.0,1.0);
		// cv::Mat inliersG1;
		// cv::Mat G = cv::findHomography(kPts, cPts, cv::RANSAC, 3.0);//calculate homography using RANSAC
		// cv::Mat G = cv::findHomography(kPts,cPts,cv::LMEDS);//calculate homography using RANSAC
		// cv::Mat G = cv::findHomography(kPts,cPts,cv::RANSAC,0.05,cv::noArray(),1000,0.99);//calculate homography using RANSAC
		cv::Mat G = cv::findHomography(kPts,cPts,0);//calculate homography using RANSAC

		ROS_WARN("time for homog %2.4f",float(clock()-updateClock)/CLOCKS_PER_SEC);
		updateClock = clock();
		// float G11 = G.at<double>(0,0);
		// float G12 = G.at<double>(0,1);
		// float G13 = G.at<double>(0,2);
		// float G21 = G.at<double>(2,0);
		// float G22 = G.at<double>(2,1);
		// float G23 = G.at<double>(2,2);
		// float G31 = G.at<double>(3,0);
		// float G32 = G.at<double>(3,1);
		// float G33 = G.at<double>(3,2);

		// std::vector<cv::Point2f> cPtsNorm,kPtsNorm;
		// cv::undistortPoints(cPts,cPtsNorm,camMat,cv::Mat());
		// cv::undistortPoints(kPts,kPtsNorm,camMat,cv::Mat());
		// cv::Mat EE = cv::findEssentialMat(kPtsNorm,cPtsNorm,1.0,cv::Point2d(0.0,0.0),cv::LMEDS);
		cv::Mat EE = cv::findEssentialMat(kPtsNorm,cPtsNorm,1.0,cv::Point2d(0.0,0.0),cv::RANSAC,0.99,1.0);

		ROS_WARN("time for essential %2.4f",float(clock()-updateClock)/CLOCKS_PER_SEC);
		updateClock = clock();

		// cv::Mat F = cv::findFundamentalMat(kPts, cPts,cv::FM_LMEDS);
		// cv::Mat F = cv::findFundamentalMat(kPts, cPts,cv::FM_RANSAC,1.0,0.99);
		// cv::Mat E = cv::findFundamentalMat(kPtsNorm, cPtsNorm,cv::FM_LMEDS);
		// cv::Mat E = cv::findFundamentalMat(kPtsNorm,cPtsNorm,cv::FM_8POINT);
		// cv::Mat F = cv::findFundamentalMat(kPts,cPts,cv::FM_8POINT);

		// std::cout << "\n E \n" << E << std::endl;

		//normalize F
		// cv::Mat w,u,vt;
		// cv::SVD::compute(E, w, u, vt);

		// std::cout << "\n w \n" << w << "\n u \n" << u << "\n vt \n" << vt << std::endl;
		// w.at<double>(2,2) = 0.0;
		// cv::Mat ww = cv::Mat::diag(w);
		// cv::Mat Fp = u*ww*vt;
		//
		// std::cout << "\n Fp \n" << Fp << std::endl;
		// float F11 = F.at<double>(0,0);
		// float F12 = F.at<double>(0,1);
		// float F13 = F.at<double>(0,2);
		// float F21 = F.at<double>(2,0);
		// float F22 = F.at<double>(2,1);
		// float F23 = F.at<double>(2,2);
		// float F31 = F.at<double>(3,0);
		// float F32 = F.at<double>(3,1);
		// float F33 = F.at<double>(3,2);



		// //find which has lower reprojection error
		// std::vector<float> Gerror(kPts.size());
		// std::vector<float> Ferror(kPts.size());
		// for (int ii = 0; i < kPts.size(); ii++)
		// {
		// 	Gerror.at(ii) = fabsf((G11*kPts.at(ii).x+G12*kPts.at(ii).y+G13)/(G31*kPts.at(ii).x+G32*kPts.at(ii).y+G33));
		// }

		std::cout << "\n G \n" << G << std::endl;
		// std::cout << "\n GA \n" << GA << std::endl;

		// cv::Mat E = camMatD.t()*F*camMatD;
		// float ENorm = cv::norm(E);
		// E /= ENorm;
		// std::cout << "\n E " << E << std::endl;
		// std::cout << "\n EE " << EE << std::endl;

		// cv::Mat wE,uE,vtE;
		// cv::SVD::compute(E, wE, uE, vtE);
		// cv::Mat WE = cv::Mat::eye(3,3,CV_64F);
		// WE.at<double>(2,2) = 0.0;
		// E = uE*WE*vtE;
		// //
		// std::cout << "\n E \n" << E << std::endl;
		std::cout << "\n EE \n" << EE << std::endl;

		// cv::Mat wEN,uEN,vtEN;
		// cv::SVD::compute(E, wEN, uEN, vtEN);

		// cv::Mat wEE,uEE,vtEE;
		// cv::SVD::compute(EE, wEE, uEE, vtEE);

		// std::cout << "\n wE \n" << wE << "\n uE \n" << uE << "\n vtE \n" << vtE << std::endl;
		// std::cout << "\n wEN \n" << wEN << "\n uEN \n" << uEN << "\n vtEN \n" << vtEN << std::endl;
		// std::cout << "\n wEE \n" << wEE << "\n uEE \n" << uEE << "\n vtEE \n" << vtEE << std::endl;
		//
		// cv::Mat WW = cv::Mat::zeros(3,3,CV_64F);
		// WW.at<double>(0,1) = -1.0;
		// WW.at<double>(1,0) = 1.0;
		// WW.at<double>(2,2) = 1.0;
		// cv::Mat WWI = WW.t();
		// cv::Mat REN = uEN*WWI*vtEN;
		// cv::Mat tEN = uEN*WW*WE*uEN.t();

		// Eigen::Matrix3f RENf;
		// cv::cv2eigen(REN,RENf);
		// Eigen::Quaternionf qENq(RENf);// convert to quaternion
		// Eigen::Vector4f qENf(qENq.w(),qENq.x(),qENq.y(),qENq.z());
		// qENf /= qENf.norm();
		// Eigen::Vector3f tENf;
		// cv::cv2eigen(tEN,tENf);
		//
		// std::cout << "\n qEN " << qENf.transpose() << std::endl;
		// std::cout << "\n tEN " << tENf.transpose() << std::endl;

		// cv::Mat R1,R2,tt;
		// cv::decomposeEssentialMat(E,R1,R2,tt);
		cv::Mat R3,R4,tt2;
		cv::decomposeEssentialMat(EE,R3,R4,tt2);

		// std::cout << "\n G \n" << G << std::endl << std::endl;

		// float kT = 0.03/(GTau + 0.03);
		// Eigen::Matrix<float,3,3> Gf;
		// for (int ii = 0; ii < 9; ii++)
		// {
		// 	Gf(ii/3,ii%3) = G.at<double>(ii/3,ii%3);
		// }
		//
		// if (!G.empty())
		// {
		// 	GkfLast += kT*(Gf - GkfLast);
		// }
		//
		// GkfLast *= (1.0/GkfLast(2,2));
		//
		// for (int ii = 0; ii < 9; ii++)
		// {
		// 	G.at<double>(ii/3,ii%3) = GkfLast(ii/3,ii%3);
		// }

		// estimate the homography
		Eigen::Vector4f qkcEst = qkcHat;
		// Eigen::Vector3f nk(0.0,0.0,1.0);
		Eigen::Vector3f tkcEst = tkcHat;

		if (!G.empty())
		{
			try
			{
				//find the solutions
				std::vector<cv::Mat> RkcH,tkcH,nkH;//holds the rotations, translations and normal vetors from decompose homography
				// std::vector<cv::Mat> RkcA,tkcA,nkA;//holds the rotations, translations and normal vetors from decompose homography
				int numberHomogSols = cv::decomposeHomographyMat(G, camMatD, RkcH, tkcH, nkH);// Decompose homography
				// int numberAffineSols = cv::decomposeHomographyMat(GA, camMatD, RkcA, tkcA, nkA);// Decompose homography

				//check positive depth constraint on the inliers to find the solutions
				// std::vector<Eigen::Vector3f> nks;
				std::vector<Eigen::Vector3f> tkcs;
				std::vector<Eigen::Vector4f> qkcs;
				std::vector<float> errors;
				std::vector<int> types;
				// std::vector<Eigen::Matrix3f> Ekcs;

				if (usePnPk)
				{
					if ((qkc + qkcPnPk).norm() < (qkc - qkcPnPk).norm())
					{
						qkcPnPk *= -1.0;
					}

					qkcs.push_back(qkcPnPk);
					if ((tkcPnPk.norm() > 0.001) && (pkc.norm() > 0.001))
					{
						errors.push_back((qkc - qkcPnPk).norm()+(pkc/pkc.norm()-tkcPnPk).norm());
						// errors.push_back((pkc/pkc.norm()-tkcPnPk).norm());
						tkcs.push_back(tkcPnPk);
						// Eigen::Matrix3f Fkci = camMatf
						// Ekcs.push_back(getss(tkcPnPk)*getqRot(qkcPnPk));
					}
					else
					{
						errors.push_back((qkc - qkcPnPk).norm());
						tkcs.push_back(Eigen::Vector3f::Zero());
						// Ekcs.push_back(Eigen::Matrix3f::Zero());
					}
					types.push_back(0);

					std::cout << "\n qkcwPNPK " << qkcPnPk(0) << " qkcxPNPK " << qkcPnPk(1) << " qkcyPNPK " << qkcPnPk(2) << " qkczPNPK " << qkcPnPk(3) << std::endl;
					std::cout << "\n tkcxPNPK " << tkcPnPk(0) << " tkcyPNPK " << tkcPnPk(1) << " tkczPNPK " << tkcPnPk(2) << std::endl;
					// std::cout << "\n EkcPNPK " << Ekcs.back() << std::endl;
				}

				if (usePnPc)
				{
					if ((qkc + qkcPnPc).norm() < (qkc - qkcPnPc).norm())
					{
						qkcPnPc *= -1.0;
					}

					qkcs.push_back(qkcPnPc);
					if ((tkcPnPc.norm() > 0.001) && (pkc.norm() > 0.001))
					{
						errors.push_back((qkc - qkcPnPc).norm()+(pkc/pkc.norm()-tkcPnPc).norm());
						tkcs.push_back(tkcPnPc);
						// Ekcs.push_back(getss(tkcPnPc)*getqRot(qkcPnPc));
					}
					else
					{
						errors.push_back((qkc - qkcPnPc).norm());
						tkcs.push_back(Eigen::Vector3f::Zero());
						// Ekcs.push_back(Eigen::Matrix3f::Zero());
					}
					types.push_back(0);

					std::cout << "\n qkcwPNPC " << qkcPnPc(0) << " qkcxPNPC " << qkcPnPc(1) << " qkcyPNPC " << qkcPnPc(2) << " qkczPNPC " << qkcPnPc(3) << std::endl;
					std::cout << "\n tkcxPNPC " << tkcPnPc(0) << " tkcyPNPC " << tkcPnPc(1) << " tkczPNPC " << tkcPnPc(2) << std::endl;
					// std::cout << "\n EkcPNPC " << Ekcs.back() << std::endl;
				}

				// if (!E.empty())
				// {
				// 	Eigen::Matrix3f Rkc1,Rkc2;
				// 	for (int hh = 0; hh < 9; hh++)
				// 	{
				// 		Rkc1(hh/3,hh%3) = R1.at<double>(hh/3,hh%3);
				// 		Rkc2(hh/3,hh%3) = R2.at<double>(hh/3,hh%3);
				// 	}
				// 	Eigen::Quaternionf qkcq1(Rkc1);// convert to quaternion
				// 	Eigen::Quaternionf qkcq2(Rkc2);// convert to quaternion
				// 	Eigen::Vector4f qkc1(qkcq1.w(),qkcq1.x(),qkcq1.y(),qkcq1.z());
				// 	Eigen::Vector4f qkc2(qkcq2.w(),qkcq2.x(),qkcq2.y(),qkcq2.z());
				// 	qkc1 /= qkc1.norm();
				// 	qkc2 /= qkc2.norm();
				// 	Eigen::Vector3f tkct(tt.at<double>(0,0),tt.at<double>(1,0),tt.at<double>(2,0));
				//
				// 	if ((qkc + qkc1).norm() < (qkc - qkc1).norm())
				// 	{
				// 		qkc1 *= -1.0;
				// 	}
				//
				// 	if ((qkc + qkc2).norm() < (qkc - qkc2).norm())
				// 	{
				// 		qkc2 *= -1.0;
				// 	}
				//
				// 	qkcs.push_back(qkc1);
				// 	qkcs.push_back(qkc1);
				// 	qkcs.push_back(qkc2);
				// 	qkcs.push_back(qkc2);
				// 	Eigen::Vector3f tkcF = Eigen::Vector3f::Zero();
				// 	if ((tkct.norm() > 0.001) && (pkc.norm() > 0.001))
				// 	{
				// 		tkcF = tkct/tkct.norm();
				// 		// errors.push_back((qkc - qkc1).norm()+(pkc/pkc.norm()-tkcF).norm());
				// 		// errors.push_back((qkc - qkc2).norm()+(pkc/pkc.norm()-tkcF).norm());
				// 		errors.push_back((qkc - qkc1).norm()+(pkc/pkc.norm()-tkcF).norm());
				// 		errors.push_back((qkc - qkc1).norm()+(pkc/pkc.norm()+tkcF).norm());
				// 		errors.push_back((qkc - qkc2).norm()+(pkc/pkc.norm()-tkcF).norm());
				// 		errors.push_back((qkc - qkc2).norm()+(pkc/pkc.norm()+tkcF).norm());
				// 		// errors.push_back((pkc/pkc.norm()-tkcF).norm());
				// 		// errors.push_back((pkc/pkc.norm()+tkcF).norm());
				// 		// errors.push_back((pkc/pkc.norm()-tkcF).norm());
				// 		// errors.push_back((pkc/pkc.norm()+tkcF).norm());
				// 		tkcs.push_back(tkcF);
				// 		tkcs.push_back(-tkcF);
				// 		tkcs.push_back(tkcF);
				// 		tkcs.push_back(-tkcF);
				// 		// errors.push_back((qkc - qkc1).norm()+(pkc/pkc.norm()+tkcF).norm());
				// 		// errors.push_back((qkc - qkc2).norm()+(pkc/pkc.norm()+tkcF).norm());
				//
				// 		// Ekcs.push_back(getss(tkcF)*getqRot(qkc1));
				// 		// Ekcs.push_back(getss(tkcF)*getqRot(qkc2));
				// 		// Ekcs.push_back(getss(-tkcF)*getqRot(qkc1));
				// 		// Ekcs.push_back(getss(-tkcF)*getqRot(qkc2));
				// 	}
				// 	else
				// 	{
				// 		errors.push_back((qkc - qkc1).norm());
				// 		errors.push_back((qkc - qkc1).norm());
				// 		errors.push_back((qkc - qkc2).norm());
				// 		errors.push_back((qkc - qkc2).norm());
				// 		tkcs.push_back(Eigen::Vector3f::Zero());
				// 		tkcs.push_back(Eigen::Vector3f::Zero());
				// 		tkcs.push_back(Eigen::Vector3f::Zero());
				// 		tkcs.push_back(Eigen::Vector3f::Zero());
				// 		// Ekcs.push_back(Eigen::Matrix3f::Zero());
				// 		// Ekcs.push_back(Eigen::Matrix3f::Zero());
				// 		// Ekcs.push_back(Eigen::Matrix3f::Zero());
				// 		// Ekcs.push_back(Eigen::Matrix3f::Zero());
				// 	}
				// 	std::cout << "\n qkcwF1 " << qkc1(0) << " qkcxF1 " << qkc1(1) << " qkcyF1 " << qkc1(2) << " qkczF1 " << qkc1(3) << std::endl;
				// 	std::cout << "\n qkcwF2 " << qkc2(0) << " qkcxF2 " << qkc2(1) << " qkcyF2 " << qkc2(2) << " qkczF2 " << qkc2(3) << std::endl;
				// 	std::cout << "\n tkcxF " << tkcF(0) << " tkcyF " << tkcF(1) << " tkczF " << tkcF(2) << std::endl;
				// 	// std::cout << "\n EkcjF1 " << Ekcs.at(Ekcs.size()-4) << std::endl;
				// 	// std::cout << "\n EkcjF2 " << Ekcs.at(Ekcs.size()-3) << std::endl;
				// 	// std::cout << "\n EkcjF3 " << Ekcs.at(Ekcs.size()-2) << std::endl;
				// 	// std::cout << "\n EkcjF4 " << Ekcs.at(Ekcs.size()-1) << std::endl;
				// 	// std::cout << "\n EkcjF11 " << getqRot(qkc1)*getss(tkcF) << std::endl;
				// }

				if (!EE.empty())
				{
					Eigen::Matrix3f Rkc1,Rkc2;
					for (int hh = 0; hh < 9; hh++)
					{
						Rkc1(hh/3,hh%3) = R3.at<double>(hh/3,hh%3);
						Rkc2(hh/3,hh%3) = R4.at<double>(hh/3,hh%3);
					}
					Eigen::Quaternionf qkcq1(Rkc1);// convert to quaternion
					Eigen::Quaternionf qkcq2(Rkc2);// convert to quaternion
					Eigen::Vector4f qkc1(qkcq1.w(),qkcq1.x(),qkcq1.y(),qkcq1.z());
					Eigen::Vector4f qkc2(qkcq2.w(),qkcq2.x(),qkcq2.y(),qkcq2.z());
					qkc1 /= qkc1.norm();
					qkc2 /= qkc2.norm();
					Eigen::Vector3f tkct(tt2.at<double>(0,0),tt2.at<double>(1,0),tt2.at<double>(2,0));

					if ((qkc + qkc1).norm() < (qkc - qkc1).norm())
					{
						qkc1 *= -1.0;
					}

					if ((qkc + qkc2).norm() < (qkc - qkc2).norm())
					{
						qkc2 *= -1.0;
					}

					qkcs.push_back(qkc1);
					qkcs.push_back(qkc1);
					qkcs.push_back(qkc2);
					qkcs.push_back(qkc2);
					Eigen::Vector3f tkcF = Eigen::Vector3f::Zero();
					if ((tkct.norm() > 0.001) && (pkc.norm() > 0.001))
					{
						tkcF = tkct/tkct.norm();
						errors.push_back((qkc - qkc1).norm()+(pkc/pkc.norm()-tkcF).norm());
						errors.push_back((qkc - qkc1).norm()+(pkc/pkc.norm()+tkcF).norm());
						errors.push_back((qkc - qkc2).norm()+(pkc/pkc.norm()-tkcF).norm());
						errors.push_back((qkc - qkc2).norm()+(pkc/pkc.norm()+tkcF).norm());
						// errors.push_back((pkc/pkc.norm()-tkcF).norm());
						// errors.push_back((pkc/pkc.norm()+tkcF).norm());
						// errors.push_back((pkc/pkc.norm()-tkcF).norm());
						// errors.push_back((pkc/pkc.norm()+tkcF).norm());
						tkcs.push_back(tkcF);
						tkcs.push_back(-tkcF);
						tkcs.push_back(tkcF);
						tkcs.push_back(-tkcF);
					}
					else
					{
						errors.push_back((qkc - qkc1).norm());
						errors.push_back((qkc - qkc1).norm());
						errors.push_back((qkc - qkc2).norm());
						errors.push_back((qkc - qkc2).norm());
						tkcs.push_back(Eigen::Vector3f::Zero());
						tkcs.push_back(Eigen::Vector3f::Zero());
						tkcs.push_back(Eigen::Vector3f::Zero());
						tkcs.push_back(Eigen::Vector3f::Zero());
					}
					types.push_back(1);
					types.push_back(1);
					types.push_back(1);
					types.push_back(1);
					std::cout << "\n qkcwE1 " << qkc1(0) << " qkcxE1 " << qkc1(1) << " qkcyE1 " << qkc1(2) << " qkczE1 " << qkc1(3) << std::endl;
					std::cout << "\n qkcwE2 " << qkc2(0) << " qkcxE2 " << qkc2(1) << " qkcyE2 " << qkc2(2) << " qkczE2 " << qkc2(3) << std::endl;
					std::cout << "\n tkcxE " << tkcF(0) << " tkcyE " << tkcF(1) << " tkczE " << tkcF(2) << std::endl;
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

					if (nkj.norm() < 0.001)
					{
						nkj(0) = 0.0;
						nkj(1) = 0.0;
						nkj(2) = 1.0;
					}

					//if n^T*[0;0;1] > then solution in front of camera
					Eigen::Vector3f tkcj(tkcH.at(jj).at<double>(0,0),tkcH.at(jj).at<double>(1,0),tkcH.at(jj).at<double>(2,0));
					// std::cout << "\n tkcjx " << tkcj(0) << " tkcjy " << tkcj(1) << " tkcjz " << tkcj(2) << std::endl;
					if (nkj(2) > 0)
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

						// nks.push_back(nkj);

						qkcs.push_back(qkcj);
						Eigen::Vector3f tkcjN = Eigen::Vector3f::Zero();
						if ((tkcj.norm() > 0.001) && (pkc.norm() > 0.001))
						{
							tkcjN = tkcj/tkcj.norm();
							errors.push_back((qkc - qkcj).norm()+(pkc/pkc.norm()-tkcjN).norm());
							// errors.push_back((pkc/pkc.norm()-tkcjN).norm());
							tkcs.push_back(tkcjN);
							// Ekcs.push_back(getss(tkcjN)*getqRot(qkcj));
						}
						else
						{
							errors.push_back((qkc - qkcj).norm());
							tkcs.push_back(Eigen::Vector3f::Zero());
							// Ekcs.push_back(Eigen::Matrix3f::Zero());
						}
						types.push_back(2);

						std::cout << "\n qkcjwH " << qkcj(0) << " qkcjxH " << qkcj(1) << " qkcjyH " << qkcj(2) << " qkcjzH " << qkcj(3) << std::endl;
						std::cout << "\n tkcjxH " << tkcjN(0) << " tkcjyH " << tkcjN(1) << " tkcjzH " << tkcjN(2) << std::endl;
						// std::cout << "\n EkcjH " << Ekcs.back() << std::endl;
						// std::cout << "\n nkjx " << nkj(0) << " nkjy " << nkj(1) << " nkjz " << nkj(2) << std::endl;
					}
				}

				// for (int jj = 0; jj < numberAffineSols; jj++)
				// {
				// 	//convert normal to eigen
				// 	Eigen::Vector3f nkj(nkA.at(jj).at<double>(0,0),nkA.at(jj).at<double>(1,0),nkA.at(jj).at<double>(2,0));
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
				// 	Eigen::Vector3f tkcj(tkcA.at(jj).at<double>(0,0),tkcA.at(jj).at<double>(1,0),tkcA.at(jj).at<double>(2,0));
				// 	// std::cout << "\n tkcjx " << tkcj(0) << " tkcjy " << tkcj(1) << " tkcjz " << tkcj(2) << std::endl;
				// 	if (true)
				// 	{
				// 		Eigen::Matrix3f Rkcj;
				// 		for (int hh = 0; hh < 9; hh++)
				// 		{
				// 			Rkcj(hh/3,hh%3) = RkcA.at(jj).at<double>(hh/3,hh%3);
				// 		}
				// 		Eigen::Quaternionf qkcjq(Rkcj);// convert to quaternion
				// 		Eigen::Vector4f qkcj(qkcjq.w(),qkcjq.x(),qkcjq.y(),qkcjq.z());
				// 		qkcj /= qkcj.norm();
				//
				// 		if ((qkc + qkcj).norm() < (qkc - qkcj).norm())
				// 		{
				// 			qkcj *= -1.0;
				// 		}
				//
				// 		// nks.push_back(nkj);
				//
				// 		qkcs.push_back(qkcj);
				// 		Eigen::Vector3f tkcjN = Eigen::Vector3f::Zero();
				// 		if ((tkcj.norm() > 0.001) && (pkc.norm() > 0.001))
				// 		{
				// 			tkcjN = tkcj/tkcj.norm();
				// 			// errors.push_back((qkc - qkcj).norm()+(pkc/pkc.norm()-tkcjN).norm());
				// 			errors.push_back((pkc/pkc.norm()-tkcjN).norm());
				// 			tkcs.push_back(tkcjN);
				// 			// Ekcs.push_back(getss(tkcjN)*getqRot(qkcj));
				// 		}
				// 		else
				// 		{
				// 			errors.push_back((qkc - qkcj).norm());
				// 			tkcs.push_back(Eigen::Vector3f::Zero());
				// 			// Ekcs.push_back(Eigen::Matrix3f::Zero());
				// 		}
				//
				// 		std::cout << "\n qkcjwA " << qkcj(0) << " qkcjxA " << qkcj(1) << " qkcjyA " << qkcj(2) << " qkcjzA " << qkcj(3) << std::endl;
				// 		std::cout << "\n tkcjxA " << tkcjN(0) << " tkcjyA " << tkcjN(1) << " tkcjzA " << tkcjN(2) << std::endl;
				// 		// std::cout << "\n EkcjH " << Ekcs.back() << std::endl;
				// 		// std::cout << "\n nkjx " << nkj(0) << " nkjy " << nkj(1) << " nkjz " << nkj(2) << std::endl;
				// 	}
				// }

				// int minqkcsErrorInd = std::distance(errors.begin(),std::min_element(errors.begin(),errors.end()));

				//find the minimum reprojection error
				// std::vector<float> projErrors(qkcs.size(),0.0);
				std::vector<float> euclidErrors(qkcs.size(),0.0);
				std::vector<Eigen::Vector2f> cPtsB(cPts.size());
				std::vector<Eigen::Vector3f> piksB(cPts.size());
				std::vector<int> isB(cPts.size());
				std::vector<int> negativeDepths(qkcs.size(),0);
				if (false)
				{
					for (int ii = 0; ii < cPts.size(); ii++)
					{
						Eigen::Vector3f kPti3(kPts3.at(ii).x,kPts3.at(ii).y,kPts3.at(ii).z);
						piksB.at(ii) = kPti3;
						cPtsB.at(ii) = Eigen::Vector2f(cPts.at(ii).x,cPts.at(ii).y);
						isB.at(ii) = depthEstimators.at(ii)->depthInd;
						// Eigen::Vector3f cPti3(cPts3.at(ii).x,cPts3.at(ii).y,cPts3.at(ii).z);
						Eigen::Vector3f cPti3Proj(0.0,0.0,0.0);
						// Eigen::Vector2f cPtiProj(0.0,0.0);
						// Eigen::Vector2f DcPti(0.0,0.0);
						Eigen::Vector3f DcPti3(0.0,0.0,0.0);
						for (int jj = 0; jj < qkcs.size(); jj++)
						{
							cPti3Proj = tkcs.at(jj)*dkcHat + rotatevec(kPti3,qkcs.at(jj));
							// cPtiProj(0) = (cPti3Proj(0)/cPti3Proj(2))*fx+cx;
							// cPtiProj(1) = (cPti3Proj(1)/cPti3Proj(2))*fy+cy;
							// std::cout << ii << " " << jj << " " << cPts3.at(ii).x << " " << cPts3.at(ii).y << " " << cPts3.at(ii).z;
							// std::cout << " " << cPti3Proj(0) << " " << cPti3Proj(1) << " " << cPti3Proj(2) << std::endl;
							std::cout << ii << " " << jj << " " << cPts3.at(ii).z;
							std::cout << " " << cPti3Proj(2) << std::endl;
							// std::cout << " " << cPts.at(ii).x << " " << cPts.at(ii).y;
							// std::cout << " " << cPtiProj(0) << " " << cPtiProj(1) << std::endl;
							// DcPti(0) = cPtiProj(0) - cPts.at(ii).x;
							// DcPti(1) = cPtiProj(1) - cPts.at(ii).y;
							DcPti3(0) = cPti3Proj(0) - cPts3.at(ii).x;
							DcPti3(1) = cPti3Proj(1) - cPts3.at(ii).y;
							DcPti3(2) = cPti3Proj(2) - cPts3.at(ii).z;
							// projErrors.at(jj) += float(DcPti.norm());
							euclidErrors.at(jj) += float(DcPti3.norm());
							if (cPti3Proj(2) < 0.1)
							{
								negativeDepths.at(jj)++;
							}
						}
					}
				}

				int minqkcsErrorInd = 0;
				int minFkcsErrorInd = 0;
				std::cout << std::endl;
				// std::cout << "p0: " << errors.at(0) << " eu0: " << euclidErrors.at(0) << std::endl;
				std::vector<int> minEsts;
				std::vector<float> minEstWeights;
				float minEstWeightSum = 0.0;
				for (int ii = 0; ii < errors.size(); ii++)
				{
					std::cout << "p" << ii << ": " << errors.at(ii) << " eu" << ii << ": " << euclidErrors.at(ii) << std::endl;

					if (false)
					{
						if (euclidErrors.at(ii) < float(cPts.size()*0.2))
						{
							float scalei = 1.0;
							if (types.at(ii) == 2)
							{
								scalei = 0.1;
							}
							minEsts.push_back(ii);
							std::cout << "\n use " << ii << " ";
							minEstWeights.push_back(1.0/(1.0+scalei*euclidErrors.at(ii)*euclidErrors.at(ii)));
							std::cout << minEstWeights.back() << std::endl;
							minEstWeightSum += minEstWeights.back();
						}
						if (euclidErrors.at(ii) < euclidErrors.at(minqkcsErrorInd))
						{
							minqkcsErrorInd = ii;
						}
					}
					else
					{
						if (errors.at(ii) < 0.1)
						{
							float scalei = 1.0;
							if (types.at(ii) == 2)
							{
								scalei = 0.1;
							}
							minEsts.push_back(ii);
							std::cout << "\n use " << ii << " ";
							minEstWeights.push_back(1.0/(1.0+scalei*errors.at(ii)*errors.at(ii)));
							std::cout << minEstWeights.back() << std::endl;
							minEstWeightSum += minEstWeights.back();
						}
						if (errors.at(ii) < errors.at(minqkcsErrorInd))
						{
							minqkcsErrorInd = ii;
						}
					}

				}
				std::cout << std::endl;

				std::cout << "\n weightSum " << minEstWeightSum << std::endl;

				// int minqkcsErrorInd = 0;
				// int minFkcsErrorInd = 0;
				// std::cout << std::endl;
				// std::cout << "p0: " << errors.at(0) << " pr0: " << projErrors.at(0) << " eu0: " << euclidErrors.at(0) << std::endl;
				// for (int ii = 1; ii < errors.size(); ii++)
				// {
				// 	std::cout << "p" << ii << ": " << errors.at(ii) << " pr" << ii << ": " << projErrors.at(ii) << " eu" << ii << ": " << euclidErrors.at(ii) << std::endl;
				// 	// if (allPtsKnown)
				// 	// {
				// 	// 	if (euclidErrors.at(ii) < euclidErrors.at(minqkcsErrorInd))
				// 	// 	{
				// 	// 		minqkcsErrorInd = ii;
				// 	// 	}
				// 	// }
				// 	// else
				// 	// {
				// 	// 	if (errors.at(ii) < errors.at(minqkcsErrorInd))
				// 	// 	{
				// 	// 		minqkcsErrorInd = ii;
				// 	// 	}
				// 	// }
				//
				// 	if (errors.at(ii) < errors.at(minqkcsErrorInd))
				// 	{
				// 		minqkcsErrorInd = ii;
				// 	}
				// }
				// std::cout << std::endl;


				if (minEsts.size() > 0)
				{
					if (minEsts.size() > 1)
					{
						qkcEst = Eigen::Vector4f::Zero();
						tkcEst = Eigen::Vector3f::Zero();
						for (int ii = 0; ii < minEsts.size(); ii++)
						{
							qkcEst += (minEstWeights.at(ii)/minEstWeightSum)*qkcs.at(minEsts.at(ii));
							tkcEst += (minEstWeights.at(ii)/minEstWeightSum)*tkcs.at(minEsts.at(ii));
						}
					}
					else
					{
						qkcEst = qkcs.at(minEsts.at(0));
						tkcEst = tkcs.at(minEsts.at(0));
					}
				}
				else
				{
					Eigen::Vector3f tkc = Eigen::Vector3f::Zero();
					if (pkc.norm() > 0.001)
					{
						tkc =  pkc/pkc.norm();
					}
					// qkcEst = qkcs.at(minqkcsErrorInd);
					// tkcEst = tkcs.at(minqkcsErrorInd);
					qkcEst = qkc;
					tkcEst = tkc;
				}

				std::cout << "\n qkcs.size " << qkcs.size() << std::endl;
				std::cout << "\n tkcs.size " << tkcs.size() << std::endl;
				std::cout << "\n minqkcsErrorInd " << minqkcsErrorInd << std::endl;
				std::cout << "\n qkcEstb " << qkcEst.transpose() << std::endl;
				std::cout << "\n tkcEstb " << tkcEst.transpose() << std::endl;

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

		Eigen::Vector3f tkc = Eigen::Vector3f::Zero();
		if (pkc.norm() > 0.001)
		{
			tkc =  pkc/pkc.norm();
		}

		// if (pkc.norm()>0.05)
		// {
		// 	bundleNew = new BundleSave(qkcEst,tkcEst*dkcHat,cPtsB,piksB,isB);
		// 	bundleSaves.push_back(bundleNew);
		// 	while (bundleSaves.size() > 5)
		// 	{
		// 		delete bundleSaves.front();
		// 		bundleSaves.pop_front();
		// 	}
		//
		// 	//adjust all the points in bundles to match the new one
		// 	std::vector<Eigen::Vector2f> cPtsBi(cPts.size());
		// 	std::vector<Eigen::Vector3f> piksBi(cPts.size());
		// 	for (int ii = 0; ii < bundleSaves.size()-1; ii++)
		// 	{
		// 		int kk = 0;
		// 		for (int jj = 0; jj < bundleSaves.at(ii)->is.size(); jj++)
		// 		{
		// 			//if the i matches then save the point
		// 			if (bundleSaves.at(ii)->is.at(jj) == isB.at(kk))
		// 			{
		// 				cPtsBi.at(kk) = bundleSaves.at(ii)->cPts.at(jj);
		// 				piksBi.at(kk) = bundleSaves.at(ii)->piks.at(jj);
		// 				kk++;
		// 			}
		// 		}
		// 		bundleSaves.at(ii)->cPts = cPtsBi;
		// 		bundleSaves.at(ii)->piks = piksBi;
		// 		bundleSaves.at(ii)->is = isB;
		// 	}
		//
		// 	//perform the bundle adjustment to approximate current pose estimate
		// 	std::vector<Eigen::MatrixXf> JsB(bundleSaves.size());
		// 	std::vector<Eigen::VectorXf> bsB(bundleSaves.size());
		// 	std::vector<Eigen::VectorXf> etasB(bundleSaves.size());
		// 	std::vector<Eigen::VectorXf> psB(bundleSaves.size());
		// 	Eigen::MatrixXf Jetan = Eigen::MatrixXf::Zero(2*cPts.size(),7);
		// 	Eigen::MatrixXf Jpn = Eigen::MatrixXf::Zero(2*cPts.size(),3*cPts.size());
		// 	Eigen::VectorXf bn = Eigen::VectorXf::Zero(2*cPts.size());
		// 	Eigen::VectorXf etan = Eigen::VetorXf::Zero(7);
		// 	Eigen::VectorXf pn = Eigen::VetorXf::Zero(3*cPTs.size());
		// 	for (int ii = 0; ii < bundleSaves.size(); ii++)
		// 	{
		// 		etaB.segment(0,4) = bundleSaves.at(ii)->qkc;
		// 		etaB.segment(4,3) = bundleSaves.at(ii)->pkc;
		// 		etasB.at(ii) = etaB;
		// 		JkB.block(2*kk,0,2,10) = localBundleJacobian(piksBi.at(kk), bundleSaves.at(ii)->qkc, bundleSaves.at(ii)->pkc);
		// 		bkB.block(2*kk,2) = cPtsBi.at(kk);
		// 	}
		// }


		if (false)
		{
			int numPts = kPts.size();
			std::vector<bool> projIn(kPts.size(),false);
			int numProjIn = 0;

			//find all the points within the inlier group
			float sigProj = 3.0;
			// float chi2 = 3.84; //chi^2 for 95%
			float chi2 = 6.63; //chi^2 for 99%
			float chiTestVal = 0.0;
			float xkcEst = pkc(0);
			float zkcEst = pkc(2);
			float qwEst = qkc(0);
			float qyEst = qkc(2);
			float xik = 0;
			float yik = 0;
			float zik = 0;
			float xic = 0;
			float yic = 0;
			float zic = 0;
			float cPtix = 0;
			float cPtiy = 0;
			float cDiffix = 0;
			float cDiffiy = 0;

			int numAttempt = 0;
			while (numProjIn < 5)
			{
				numProjIn = 0;
				float sigProj2 = std::pow(1.1,numAttempt)*sigProj*sigProj;
				//adjust points outside of average flow
				for (int ii = 0; ii < numPts; ii++)
				{
					xik = kPts3.at(ii).x;
					yik = kPts3.at(ii).y;
					zik = kPts3.at(ii).z;
					xic = xkcEst + xik -2.0*qyEst*qyEst*xik + 2.0*qyEst*qwEst*zik;
					yic = yik;
					zic = zkcEst - 2.0*qyEst*qwEst*xik + zik - 2.0*qyEst*qyEst*zik;
					cPtix = (fx*xic/zic+cx);
					cPtiy = (fy*yic/zic+cy);
					cDiffix = cPts.at(ii).x - cPtix;
					cDiffiy = cPts.at(ii).y - cPtiy;
					chiTestVal = (cDiffix*cDiffix + cDiffiy*cDiffiy)/sigProj2;
					std::cout << ii << " cPts.at(ii).x  " << cPts.at(ii).x << " cPts.at(ii).y  " << cPts.at(ii).y;
					std::cout << " cPtix " << cPtix << " cPtiy  " << cPtiy << " chiTestVal " << chiTestVal << std::endl;
					if (chiTestVal < chi2)
					{
						projIn.at(ii) = true;
						numProjIn++;
					}
				}
				numAttempt++;
			}

			std::cout << "\n numAttempt " << numAttempt << " numProjIn " << numProjIn << " sigProj2 " << std::pow(1.1,numAttempt-1)*sigProj*sigProj << std::endl;

			// std::vector<cv::Point3f>::iterator cPts3it = cPts3.begin();

			// std::vector<DepthEstimator*>::iterator depthsit = depthEstimators.begin();
			Eigen::VectorXf Eb,Xb;
			int bundleIterations = 0;
			Eigen::MatrixXf Jb = Eigen::MatrixXf::Zero(2*numProjIn,4);
			Eigen::VectorXf fb = Eigen::VectorXf::Zero(2*numProjIn);
			Eigen::VectorXf bb = Eigen::VectorXf::Zero(2*numProjIn);
			Xb = Eigen::VectorXf::Zero(4);
			// std::vector<Eigen::Vector3f> piks(numPts);
			// std::vector<Eigen::Vector3f>::iterator piksit = piks.begin();
			// Eigen::Matrix<float,2,7> JbNorm = Eigen::Matrix<float,2,7>::Zero();

			// Eigen::Matrix3f REst = getqRot(qkcEst);
			Eigen::Vector2f fbi(0.0,0.0);
			int iiIn = 0;
			for (int ii = 0; ii < numPts; ii++)
			{
				// *piksit = Eigen::Vector3f((*kPts3it).x,(*kPts3it).y,(*kPts3it).z);
				// Jb.block(ii*2,0,2,7) = localBundleJacobianPnP(fx,fy,*piksit,qkcEst,tkcEst,dkcHat);
				// fb.segment(ii*2,2) = localBundleJacobianPnPProject(fx,fy,cx,cy,*piksit,qkcEst,tkcEst,dkcHat);
				// bb.segment(ii*2,2) = Eigen::Vector2f((*cPtsit).x,(*cPtsit).y);

				if (projIn.at(ii))
				{
					std::cout << ii << std::endl;
					Jb.block(iiIn*2,0,2,4) = localBundleJacobianWMR(fx,fy,cx,cy,kPts3.at(ii).x,kPts3.at(ii).y,kPts3.at(ii).z,qkcEst(0),qkcEst(2),tkcEst(0)*dkcHat,tkcEst(2)*dkcHat,fbi);
					fb.segment(iiIn*2,2) = fbi;
					bb.segment(iiIn*2,2) = Eigen::Vector2f(cPts.at(ii).x,cPts.at(ii).y);
					iiIn++;
				}

				// depthsit++;
				// kPts3it++;
				// cPtsit++;
				// piksit++;
			}
			Xb(0) = qkcEst(0);
			Xb(1) = qkcEst(2);
			Xb(2) = tkcEst(0)*dkcHat;
			Xb(3) = tkcEst(2)*dkcHat;

			// Jb.block(3*numPts,0,2,7) = 1000.0*localBundleJacobianNorm(qkcEst,tkcEst);
			// fb.segment(3*numPts,2) = 1000.0*Eigen::Vector2f(float(qkcEst.transpose()*qkcEst)-1.0,float(tkcEst.transpose()*tkcEst)-1.0);
			// bb.segment(3*numPts,2) = Eigen::Vector2f(0.0,0.0);

			Eigen::VectorXf Xb0 = Xb;

			std::cout << "\n Xb0 \n" << Xb0.transpose() << std::endl;
			Eb = bb - fb;
			Eigen::VectorXf Eb0 = Eb;
			std::cout << "\n Eb0 \n" << Eb.transpose() << std::endl;
			Eigen::MatrixXf IIb = Eigen::MatrixXf::Identity(4,4);
			Eigen::MatrixXf JbT = Jb.transpose();
			// Eigen::MatrixXf JbJb = JbT*Jb + 0.0001*IIb;
			Eigen::MatrixXf JbJb = JbT*Jb;
			Eigen::MatrixXf JbEb = JbT*Eb;
			Eigen::VectorXf DXb = JbJb.fullPivHouseholderQr().solve(JbEb);
			// while ((Eb.norm()/(2.0*numPts) > 1.0) && !ros::isShuttingDown() && (bundleIterations < 10) && (DXb.norm() > 0.0000001))
			while (!ros::isShuttingDown() && (bundleIterations < 100) && (DXb.norm() > 0.0001))
			{
				//determine the new delta and update Xb
				if (bundleIterations > 0)
				{
					JbT = Jb.transpose();
					JbJb = JbT*Jb;
					JbEb = JbT*Eb;
					DXb = JbJb.fullPivHouseholderQr().solve(JbEb);
				}

				Xb += 0.75*DXb;

				Eigen::Vector4f qkcb(Xb(0),0.0,Xb(1),0.0);
				Eigen::Vector3f pkcb(Xb(2),0.0,Xb(3));
				// kPts3it = kPts3.begin();
				// Eigen::Matrix3f Rkcb = getqRot(qkcb);

				//update the jacobian and projection
				iiIn = 0;
				for (int ii = 0; ii < numPts; ii++)
				{
					// Jb.block(ii*2,0,2,7) = localBundleJacobianPnP(fx,fy,*piksit,qkcb,tkcb,dkcHat);
					// fb.segment(ii*2,2) = localBundleJacobianPnPProject(fx,fy,cx,cy,*piksit,qkcb,tkcb,dkcHat);
					// Jb.block(ii*3,0,3,7) = localBundleJacobian(Eigen::Vector3f((*kPts3it).x,(*kPts3it).y,(*kPts3it).z),qkcb,dkcHat);
					// fb.segment(ii*3,3) = tkcb*dkcHat+rotatevec(Eigen::Vector3f((*kPts3it).x,(*kPts3it).y,(*kPts3it).z),qkcb);
					// Jb.block(ii*3,0,3,7) = localBundleJacobian(Eigen::Vector3f((*kPts3it).x,(*kPts3it).y,(*kPts3it).z),qkcb,dkcHat);
					// fb.segment(ii*3,3) = tkcb*dkcHat+Rkcb*Eigen::Vector3f((*kPts3it).x,(*kPts3it).y,(*kPts3it).z);
					if (projIn.at(ii))
					{
						Jb.block(iiIn*2,0,2,4) = localBundleJacobianWMR(fx,fy,cx,cy,kPts3.at(ii).x,kPts3.at(ii).y,kPts3.at(ii).z,Xb(0),Xb(1),Xb(2),Xb(3),fbi);
						fb.segment(iiIn*2,2) = fbi;
						iiIn++;
					}
					// piksit++;
					// kPts3it++;
				}
				// Jb.block(3*numPts,0,2,7) = 1000.0*localBundleJacobianNorm(qkcb,tkcb);
				// fb.segment(3*numPts,2) = 1000.0*Eigen::Vector2f(float(qkcb.transpose()*qkcb)-1.0,float(tkcb.transpose()*tkcb)-1.0);

				//update the error
				Eb = bb - fb;
				// std::cout << "\n DXb " << DXb.transpose() << std::endl;
				std::cout << "\n DXb.norm() " << DXb.norm() << std::endl;
				// std::cout << "\n Eb " << Eb.transpose() << std::endl;
				std::cout << "\n Eb.norm() " << Eb.norm() << std::endl;
				// std::cout << "\n Xb0 " << Xb0.transpose() << std::endl;
				// std::cout << "\n Xb " << Xb.transpose() << std::endl;

				bundleIterations++;
			}

			std::cout << "\n bundleIterations " << bundleIterations << std::endl;
			std::cout << "\n Eb0 " << Eb0.transpose() << std::endl;
			std::cout << "\n Eb0.norm() " << Eb0.norm() << std::endl;
			std::cout << "\n Eb " << Eb.transpose() << std::endl;
			std::cout << "\n Eb.norm() " << Eb.norm() << std::endl;
			std::cout << "\n Xb0 " << Xb0.transpose() << std::endl;
			std::cout << "\n Xb " << Xb.transpose() << std::endl;
			qkcEst = Eigen::Vector4f(Xb(0),0.0,Xb(1),0.0);
			qkcEst /= qkcEst.norm();
			tkcEst = Eigen::Vector3f(Xb(2),0.0,Xb(3));
			tkcEst /= tkcEst.norm();
		}

		// Eigen::Matrix<float,8,1> xHatq = qkcDotEstimator.update(qkcEst,t,qkc);
		// qkcHat = xHatq.segment(0,4)/xHatq.segment(0,4).norm();
		qkcEst(1) = 0.0;
		qkcEst(3) = 0.0;
		qkcEst /= qkcEst.norm();
		//
		if (tkcEst.norm() > 0.001)
		{
			tkcEst(1) = 0.0;
			tkcEst /= tkcEst.norm();
		}

		qkcEst /= qkcEst.norm();

		Eigen::Matrix<float,8,1> xHatq;
		Eigen::Vector4f qEst;
		if ((qkcDotEstimator.firstUpdate) || (pkc.norm() < 0.1))
		{
			xHatq = qkcDotEstimator.update(qkc,t);
			qEst = xHatq.segment(0,4)/xHatq.segment(0,4).norm();
			qkcHat +=  kq*(xHatq.segment(0,4)-qkcHat);
			qkcHat /= qkcHat.norm();
		}
		else
		{
			xHatq = qkcDotEstimator.update(qkcEst,t);
			qEst = xHatq.segment(0,4)/xHatq.segment(0,4).norm();
			qkcHat +=  kq*(xHatq.segment(0,4)-qkcHat);
			qkcHat /= qkcHat.norm();
		}

		Eigen::Vector3f wEst = -2.0*B(qEst).transpose()*xHatq.segment(4,4);
		Eigen::Matrix3f RkcHat = getqRot(qkcHat);
		Eigen::Matrix3f Rkc = getqRot(qkc);

		// Eigen::Matrix<float,6,1> xHatt = tkcDotEstimator.update(tkcEst,t,tkc);
		if (pkc.norm() > 0.005)
		{
			std::cout << "\n hi 1 \n";
			if (tkcDotEstimator.firstUpdate || (pkc.norm() < 0.1))
			{
				if (tkcDotEstimator.firstUpdate)
				{
					tkcHat = tkc;
				}
				//
				// std::cout << "\n hi 2 \n";
				// std::cout << "\n tkc " << tkc.transpose() << std::endl;
				Eigen::Matrix<float,6,1> xHatt = tkcDotEstimator.update(tkc,t);
				// std::cout << "\n xHatt " << xHatt.transpose() << std::endl;
				// std::cout << "\n tkc " << tkc.transpose() << std::endl;

				// tkcHat = xHatt.segment(0,3);
				tkcHat +=  kq*(xHatt.segment(0,3)-tkcHat);
			}
			else
			{
				std::cout << "\n hi 3 \n";
				Eigen::Matrix<float,6,1> xHatt = tkcDotEstimator.update(tkcEst,t);
				// tkcHat = xHatt.segment(0,3);
				tkcHat +=  kq*(xHatt.segment(0,3)-tkcHat);
			}
		}



		if (tkcHat.norm() > 0.005)
		{
			tkcHat /= tkcHat.norm();
		}

		std::cout << "\n qkc " << qkc.transpose() << std::endl;
		std::cout << "\n tkc " << tkc.transpose() << std::endl;
		std::cout << "\n pkc " << pkc.transpose() << std::endl;
		std::cout << "\n qkcEst " << qkcEst.transpose() << std::endl;
		std::cout << "\n tkcEst " << tkcEst.transpose() << std::endl;
		std::cout << "\n qkcHat " << qkcHat.transpose() << std::endl;
		std::cout << "\n tkcHat " << tkcHat.transpose() << std::endl;
		std::cout << "\n pkcHat " << pkcHat.transpose() << std::endl;
		std::cout << "\n w " << wc.transpose() << std::endl;
		std::cout << "\n wEst " << wEst.transpose() << std::endl;

		ROS_WARN("time for update uq %2.4f",float(clock()-updateClock)/CLOCKS_PER_SEC);
		updateClock = clock();

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
		// float avgcPtx = 0.0;
		// float avgcPty = 0.0;
		bool allPtsKnownIn = true;
		float dkcAvg = 0.0;
		for (std::vector<DepthEstimator*>::iterator itD = depthEstimators.begin() ; itD != depthEstimators.end(); itD++)
		{
			// if (!G.empty())
			// {
			// 	(*itc).x = betakc*(*itkc).x + (1.0-betakc)*(*itc).x;
			// 	(*itc).y = betakc*(*itkc).y + (1.0-betakc)*(*itc).y;
			// }
			// float dkcHati = (*itD)->update(Eigen::Vector3f(((*itc).x-cx)/fx,((*itc).y-cy)/fy,1.0),tkcHat,RkcHat,vc,wc,t,pkc,qkc);
			float dkcHati = (*itD)->update(Eigen::Vector3f(((*itc).x-cx)/fx,((*itc).y-cy)/fy,1.0),tkc,Rkc,vc,wc,t,pkc,qkc);
			// dkcs.push_back(dkcHati);
			dkcAvg += dkcHati;
			// depthEstimatorsInHomog.push_back(*itD);
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
		// depthEstimators = depthEstimatorsInHomog;
		// depthEstimatorsInHomog.clear();
		// inlierRatio /= depthEstimators.size();
		avgcPtx /= float(depthEstimators.size());
		avgcPty /= float(depthEstimators.size());

		dkcAvg /= float(depthEstimators.size());

		//find which partition the average is in
		int partitionWidth = imageWidth/partitionCols;
		int partitionHeight = imageHeight/partitionRows;
		// bool avgRowFound = false;
		int avgRowInd = 0;
		// while (!avgRowFound)
		// {
		// 	int top = avgRowInd*partitionHeight;
		// 	int bottom = (avgRowInd+1)*partitionHeight;
		// 	if ((top <= avgcPty) && (avgcPty <= bottom))
		// 	{
		// 		avgRowFound = true;
		// 	}
		// 	else
		// 	{
		// 		avgRowInd++;
		// 	}
		// }

		// bool avgColFound = false;
		int avgColInd = 0;
		// while (!avgColFound)
		// {
		// 	int left = avgColInd*partitionWidth;
		// 	int right = (avgColInd+1)*partitionWidth;
		// 	if ((left <= avgcPtx) && (avgcPtx <= right))
		// 	{
		// 		avgColFound = true;
		// 	}
		// 	else
		// 	{
		// 		avgColInd++;
		// 	}
		// }

		currentAvgPartition = partitionCols*avgRowInd + avgColInd;

		ROS_WARN("time for estimators %2.4f, avg partition is %d, all points known %d, avgcPtx %3.4f",float(clock()-updateClock)/CLOCKS_PER_SEC,currentAvgPartition,int(allPtsKnown),avgcPtx);
		updateClock = clock();

		assert(depthEstimators.size() > 0);

		// if(dkcs.size() > 0)
		// {
		// 	std::sort(dkcs.begin(),dkcs.end());
		// 	float dkcMed = dkcs.at(0);
		// 	if (dkcs.size()%2 == 0)
		// 	{
		// 		dkcMed = (dkcs.at(dkcs.size()/2-1)+dkcs.at(dkcs.size()/2))/2.0;
		// 	}
		// 	else
		// 	{
		// 		dkcMed = dkcs.at(dkcs.size()/2);
		// 	}
		//
		// 	dkcHat += kd*(dkcMed - dkcHat);
		//
		// 	if (tkcHat.norm() > 0.001)
		// 	{
		// 		pkcHat += kp*(tkcHat*(dkcHat/tkcHat.norm()) - pkcHat);
		// 	}
		// }
		dkcHat += kd*(dkcAvg - dkcHat);

		if (tkcHat.norm() > 0.001)
		{
			pkcHat += kp*(tkcHat*(dkcHat/tkcHat.norm()) - pkcHat);
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
