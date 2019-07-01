#include <odom_estimator.h>

OdomEstimator::OdomEstimator()
{
	// Parameters
	ros::NodeHandle nhp("~");
	nhp.param<std::string>("bodyName", bodyName, "turtle");
	nhp.param<std::string>("cameraName", cameraName, "camera");

	float pfix,pfiy,pfiz,qfiw,qfix,qfiy,qfiz,qmew,qmex,qmey,qmez;
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

	float fpi,fqi,fvi,fwi;
	nhp.param<float>("fpi", fpi, 1.0);
	nhp.param<float>("fqi", fqi, 1.0);
	nhp.param<float>("fvi", fvi, 1.0);
	nhp.param<float>("fwi", fwi, 1.0);
	piTau = 1.0/(2.0*M_PI*fqi);
	qiTau = 1.0/(2.0*M_PI*fqi);
	viTau = 1.0/(2.0*M_PI*fvi);
	wiTau = 1.0/(2.0*M_PI*fwi);

	//publisher
  camOdomPub = nh.advertise<nav_msgs::Odometry>(cameraName+"/odom",1);
	bodyOdomPub = nh.advertise<nav_msgs::Odometry>(cameraName+"/body/odom",1);

	// Subscriber
	velSub = nh.subscribe(bodyName+"/odom",5,&OdomEstimator::velCB,this);
	poseDeltaSub = nh.subscribe(cameraName+"/pose_delta",100, &OdomEstimator::poseDeltaCB,this);

	pcwHat = Eigen::Vector3f::Zero();
	qcwHat = Eigen::Vector4f::Zero();
	qcwHat(0) = 1.0;
	vcHat = Eigen::Vector3f::Zero();
	wcHat = Eigen::Vector3f::Zero();
}

void OdomEstimator::poseDeltaCB(const icl_multiple_stationary::PoseDelta::ConstPtr& msg)
{
	std::lock_guard<std::mutex> poseDeltaMutexGuard(poseDeltaMutex);
	poseDeltas.push_back(msg);
}

void OdomEstimator::velCB(const nav_msgs::Odometry::ConstPtr& msg)
{
	// //check to make sure mocap recieved
	// if (firstPose || firstMarker)
	// {
	// 	ROS_ERROR("NO MOCAP");
	// 	return;
	// }

	//velocity is of turtlebot in body frame of turtlebot x forward, y to left, z up
	// odom pose is in ENU, odom twist is in body
	ros::Time t = msg->header.stamp;
	Eigen::Vector3f vi(msg->twist.twist.linear.x,msg->twist.twist.linear.y,msg->twist.twist.linear.z);
	Eigen::Vector3f wi(msg->twist.twist.angular.x,msg->twist.twist.angular.y,msg->twist.twist.angular.z);

	Eigen::Vector3f vc = rotatevec((vi+getss(wi)*pfi),getqInv(qfi));
	Eigen::Vector3f wc = rotatevec(wi,getqInv(qfi));

	if (firstVel)
	{
		tVelLast = t;
		vcHat = vc;
		wcHat = wc;
		firstVel = false;
	}

	float dt = (t-tVelLast).toSec();
	tVelLast = t;

	// get the low pass gains
	float kvi = dt/(viTau+dt);
	float kwi = dt/(wiTau+dt);

	// get the linear velocuty in world
	Eigen::Vector3f vcHatRot = rotatevec(vcHat,qcwHat);

	// get the low pass gains
	float kpi = dt/(piTau + dt);
	float kqi = dt/(qiTau + dt);

	// predict the estimates forward
	// pcwHat += (dt*vcHatRot + kp*(Eigen::Vector3f(0.0,pcw(1)-pcwHat(1),0.0)));
	pcwHat += (dt*vcHatRot);
	qcwHat += (dt*0.5*B(qcwHat)*wcHat);

	vcHat += kvi*(vc-vcHat);
	wcHat += kwi*(wc-wcHat);

	poseDeltaMutex.lock();
	// get the pose from each keyframe and average all together to get estimate
	std::deque<icl_multiple_stationary::PoseDelta::ConstPtr> poseDeltasRemove = poseDeltas;
	poseDeltas.clear();
	poseDeltaMutex.unlock();
	int numMeas = poseDeltasRemove.size();
	if (numMeas > 0)
	{
		// weight the estimates
		Eigen::Vector3f pcwTildeSum = Eigen::Vector3f::Zero();
		Eigen::Vector4f qcwTildeSum = Eigen::Vector4f::Zero();
		for (std::deque<icl_multiple_stationary::PoseDelta::ConstPtr>::iterator it = poseDeltasRemove.begin(); it != poseDeltasRemove.end(); it++)
		{
			// get the poses at time i
			// geometry_msgs::Pose posei = poseDeltaSyncBuff.at(i).pose;
			// geometry_msgs::Pose poseHati = poseDeltaSyncBuff.at(i).poseHat;
			// int keyIndi = poseDeltaSyncBuff.at(i).keyInd;

			//convert what camera thought at time i to eigen
			Eigen::Vector3f pcwi((*it)->pose.position.x,(*it)->pose.position.y,(*it)->pose.position.z);
			Eigen::Vector4f qcwi((*it)->pose.orientation.w,(*it)->pose.orientation.x,(*it)->pose.orientation.y,(*it)->pose.orientation.z);
			qcwi /= qcwi.norm();

			//convert what estimate was at time i to eigen
			Eigen::Vector3f pcwHati((*it)->poseHat.position.x,(*it)->poseHat.position.y,(*it)->poseHat.position.z);
			Eigen::Vector4f qcwHati((*it)->poseHat.orientation.w,(*it)->poseHat.orientation.x,(*it)->poseHat.orientation.y,(*it)->poseHat.orientation.z);
			qcwHati /= qcwHati.norm();

			// get the difference between the estimate now and time i
			Eigen::Vector3f pcwcwi = pcwHat - pcwi;
			Eigen::Vector4f qcwcwi = getqMat(getqInv(qcwi))*qcwHat;
			qcwcwi /= qcwcwi.norm();

			// new measure is what the camera thought at time i plus the difference
			Eigen::Vector3f pcw = pcwHati + pcwcwi;
			Eigen::Vector4f qcw = getqMat(qcwHati)*qcwcwi;
			qcw /= qcw.norm();

			// get the error for time i
			Eigen::Vector3f pcwTildei = pcw - pcwHat;
			Eigen::Vector4f qcwTildei = qcwi - qcwHat;

			// add what the camera thought to the sum weighted by the 1-alpha discounted further by ratio
			pcwTildeSum += pcwTildei;
			qcwTildeSum += qcwTildei;

			// std::cout << "\n pcw \n" << pcw << std::endl;
			// std::cout << "\n qcw \n" << qcw << std::endl;
			// std::cout << "\n pRatioi \n" << pRatioi << std::endl;
			// std::cout << "\n keyIndi \n" << keyIndi << std::endl;
			// delete *it;
		}

		// new estimate is the weighted average
		pcwHat += dt*kpi/numMeas*pcwTildeSum;
		qcwHat += dt*kqi/numMeas*qcwTildeSum;
	}

	poseDeltasRemove.clear();

	float qcwHatNorm = qcwHat.norm();
	Eigen::Vector4f qcwInit(cos(3.1415/4.0),sin(3.1415/4.0),0.0,0.0);
	qcwInit /= qcwInit.norm();
	Eigen::Vector3f pbwHat = rotatevec(pcwHat-rotatevec(pfi,getqInv(qfi)),getqInv(qcwInit));
	Eigen::Vector4f qbwHat = getqMat(getqMat(getqInv(qcwInit))*qcwHat/qcwHatNorm)*getqInv(qfi);
	qbwHat /= qbwHat.norm();

	// build and publish odom message for camera
	nav_msgs::Odometry camOdomMsg;
	camOdomMsg.header.stamp = t;
	camOdomMsg.header.frame_id = "world";
	camOdomMsg.child_frame_id = "camera";
	camOdomMsg.pose.pose.position.x = pcwHat(0);
	camOdomMsg.pose.pose.position.y = pcwHat(1);
	camOdomMsg.pose.pose.position.z = pcwHat(2);
	camOdomMsg.pose.pose.orientation.w = qcwHat(0)/qcwHatNorm;
	camOdomMsg.pose.pose.orientation.x = qcwHat(1)/qcwHatNorm;
	camOdomMsg.pose.pose.orientation.y = qcwHat(2)/qcwHatNorm;
	camOdomMsg.pose.pose.orientation.z = qcwHat(3)/qcwHatNorm;
	camOdomMsg.twist.twist.linear.x = vcHat(0);
	camOdomMsg.twist.twist.linear.y = vcHat(1);
	camOdomMsg.twist.twist.linear.z = vcHat(2);
	camOdomMsg.twist.twist.angular.x = wcHat(0);
	camOdomMsg.twist.twist.angular.y = wcHat(1);
	camOdomMsg.twist.twist.angular.z = wcHat(2);
	camOdomPub.publish(camOdomMsg);

	// build and publish odom message for body
	nav_msgs::Odometry bodyOdomMsg;
	bodyOdomMsg.header.stamp = t;
	bodyOdomMsg.header.frame_id = "world";
	bodyOdomMsg.child_frame_id = "body";
	bodyOdomMsg.pose.pose.position.x = pbwHat(0);
	bodyOdomMsg.pose.pose.position.y = pbwHat(1);
	bodyOdomMsg.pose.pose.position.z = pbwHat(2);
	bodyOdomMsg.pose.pose.orientation.w = qbwHat(0);
	bodyOdomMsg.pose.pose.orientation.x = qbwHat(1);
	bodyOdomMsg.pose.pose.orientation.y = qbwHat(2);
	bodyOdomMsg.pose.pose.orientation.z = qbwHat(3);
	bodyOdomMsg.twist.twist.linear.x = vcHat(0);
	bodyOdomMsg.twist.twist.linear.y = vcHat(1);
	bodyOdomMsg.twist.twist.linear.z = vcHat(2);
	bodyOdomMsg.twist.twist.angular.x = wcHat(0);
	bodyOdomMsg.twist.twist.angular.y = wcHat(1);
	bodyOdomMsg.twist.twist.angular.z = wcHat(2);
	bodyOdomPub.publish(bodyOdomMsg);
}
