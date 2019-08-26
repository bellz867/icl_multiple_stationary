#include <odom_estimator.h>

OdomEstimator::OdomEstimator()
{
	// Parameters
	ros::NodeHandle nhp("~");
	nhp.param<std::string>("bodyName", bodyName, "turtle");
	nhp.param<std::string>("cameraName", cameraName, "camera");
	nhp.param<bool>("useMocap", useMocap, false);


	float pcbx,pcby,pcbz,qcbw,qcbx,qcby,qcbz,qmew,qmex,qmey,qmez;
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

	// Eigen::Vector4f qcbxx(cos(0.5*M_PI/2),sin(-0.5*M_PI/2),0.0,0.0);
	// Eigen::Vector4f qcbyy(cos(-0.5*5.0*M_PI/180.0),0.0,sin(-0.5*5.0*M_PI/180.0),0.0);
	// qcb = getqMat(qcbxx)*qcbyy;
	// qcb /= qcb.norm();

	float fp,fq,fv,fw;
	nhp.param<float>("fp", fp, 1.0);
	nhp.param<float>("fq", fq, 1.0);
	nhp.param<float>("fv", fv, 1.0);
	nhp.param<float>("fw", fw, 1.0);
	pTau = 1.0/(2.0*M_PI*fq);
	qTau = 1.0/(2.0*M_PI*fq);
	vTau = 1.0/(2.0*M_PI*fv);
	wTau = 1.0/(2.0*M_PI*fw);

	pbwHat = Eigen::Vector3f::Zero();
	qbwHat = Eigen::Vector4f::Zero();
	qbwHat(0) = 1.0;
	vbHat = Eigen::Vector3f::Zero();
	wbHat = Eigen::Vector3f::Zero();

	cvHat = 0.975;
	cwHat = 1.025;
	kcv = 1.0;
	kcw = 1.0;

	// Get initial odom from mocap
	if (useMocap)
	{
		gotInitialPose = false;
		initialPoseSub = nh.subscribe("/mocap/body/odom",1,&OdomEstimator::mocapPoseCB,this);
		ROS_INFO("Waiting for initial pose from mocap on topic /mocap/body/odom");
		do
		{
			ros::spinOnce();
			ros::Duration(0.1).sleep();
		} while (!(ros::isShuttingDown()) && !gotInitialPose);
		ROS_INFO("Got initial pose");
	}

	//publisher
	camOdomPub = nh.advertise<nav_msgs::Odometry>(cameraName+"/odom",1);
	bodyOdomPub = nh.advertise<nav_msgs::Odometry>(cameraName+"/body/odom",1);

	// Subscriber
	velSub = nh.subscribe(bodyName+"/odom",5,&OdomEstimator::velCB,this);
	poseDeltaSub = nh.subscribe(cameraName+"/pose_delta",100, &OdomEstimator::poseDeltaCB,this);
}

void OdomEstimator::mocapPoseCB(const nav_msgs::Odometry::ConstPtr& msg)
{
	ros::Time t = msg->header.stamp;
	pbwMocap = Eigen::Vector3f(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
	qbwMocap = Eigen::Vector4f(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
	qbwMocap /= qbwMocap.norm();

	if (!gotInitialPose)
	{
		pbwHat = pbwMocap;
		qbwHat = qbwMocap;
		vbHat = Eigen::Vector3f(msg->twist.twist.linear.x,msg->twist.twist.linear.y,msg->twist.twist.linear.z);
		wbHat = Eigen::Vector3f(msg->twist.twist.angular.x,msg->twist.twist.angular.y,msg->twist.twist.angular.z);
		pDotEstimator.initialize(3);
		qDotEstimator.initialize(4);
		vDotEstimator.initialize(3);
		wDotEstimator.initialize(3);
		tVelLast = t;
		gotInitialPose = true;
	}
	// initialPoseSub.shutdown();

}

void OdomEstimator::poseDeltaCB(const icl_multiple_stationary::PoseDelta::ConstPtr& msg)
{
	std::lock_guard<std::mutex> poseDeltaMutexGuard(poseDeltaMutex);
	poseDeltas.push_back(msg);
}

void OdomEstimator::velCB(const nav_msgs::Odometry::ConstPtr& msg)
{
	//velocity is of turtlebot in body frame of turtlebot x forward, y to left, z up
	ros::Time t = msg->header.stamp;
	Eigen::Matrix<float,6,1> vDott = vDotEstimator.update(Eigen::Vector3f(msg->twist.twist.linear.x,msg->twist.twist.linear.y,msg->twist.twist.linear.z),t);
	Eigen::Matrix<float,6,1> wDott = wDotEstimator.update(Eigen::Vector3f(msg->twist.twist.angular.x,msg->twist.twist.angular.y,msg->twist.twist.angular.z),t);
	Eigen::Vector3f vb = vDott.segment(0,3);
	Eigen::Vector3f wb = wDott.segment(0,3);

	// Eigen::Vector3f vb(msg->twist.twist.linear.x,msg->twist.twist.linear.y,msg->twist.twist.linear.z);
	// Eigen::Vector3f wb(msg->twist.twist.angular.x,msg->twist.twist.angular.y,msg->twist.twist.angular.z);

	float dt = (t-tVelLast).toSec();
	tVelLast = t;

	// get the low pass gains
	float kp = dt/(pTau + dt);
	float kq = dt/(qTau + dt);
	float kv = dt/(vTau+dt);
	float kw = dt/(wTau+dt);

	vbHat += kv*(cvHat*vb-vbHat);
	wbHat += kw*(cwHat*wb-wbHat);

	// predict the estimates forward
	pbwHat += (rotatevec(vbHat,qbwHat)*dt);
	qbwHat += (0.5*B(qbwHat)*wbHat*dt);

	// // get the linear velocuty in world
	// Eigen::Vector3f vcHatRot = rotatevec(vcHat,qcwHat);

	poseDeltaMutex.lock();
	Eigen::Vector3f pbwMocapLast = pbwMocap;
	Eigen::Vector4f qbwMocapLast = qbwMocap;

	// get the pose from each keyframe, rotate into body of turtlebot, and average all together to get estimate
	std::deque<icl_multiple_stationary::PoseDelta::ConstPtr> poseDeltasRemove = poseDeltas;
	poseDeltas.clear();
	poseDeltaMutex.unlock();
	int numMeas = poseDeltasRemove.size();
	//check if one saw landmark
	bool landmarkView = false;
	// int numMeas = 0;
	if (numMeas > 0)
	{
		// weight the estimates
		Eigen::Vector3f pbwTildeSum = Eigen::Vector3f::Zero();
		Eigen::Vector4f qbwTildeSum = Eigen::Vector4f::Zero();

		for (std::deque<icl_multiple_stationary::PoseDelta::ConstPtr>::iterator it = poseDeltasRemove.begin(); it != poseDeltasRemove.end(); it++)
		{
			if (!landmarkView)
			{
				landmarkView = (*it)->landmarkView;
			}
		}

		if (!landmarkView)
		{
			for (std::deque<icl_multiple_stationary::PoseDelta::ConstPtr>::iterator it = poseDeltasRemove.begin(); it != poseDeltasRemove.end(); it++)
			{
				// get the poses at time i
				// geometry_msgs::Pose posei = poseDeltaSyncBuff.at(i).pose;
				// geometry_msgs::Pose poseHati = poseDeltaSyncBuff.at(i).poseHat;
				// int keyIndi = poseDeltaSyncBuff.at(i).keyInd;
				// bool landmarkViewi = (*it)->landmarkView;

				Eigen::Vector3f pbwTildei = Eigen::Vector3f::Zero();
				Eigen::Vector4f qbwTildei = Eigen::Vector4f::Zero();

				//convert what camera thought at time i to eigen
				Eigen::Vector3f pcwi((*it)->pose.position.x,(*it)->pose.position.y,(*it)->pose.position.z);
				Eigen::Vector4f qcwi((*it)->pose.orientation.w,(*it)->pose.orientation.x,(*it)->pose.orientation.y,(*it)->pose.orientation.z);
				qcwi /= qcwi.norm();

				//convert what estimate was at time i to eigen
				Eigen::Vector3f pcwHati((*it)->poseHat.position.x,(*it)->poseHat.position.y,(*it)->poseHat.position.z);
				Eigen::Vector4f qcwHati((*it)->poseHat.orientation.w,(*it)->poseHat.orientation.x,(*it)->poseHat.orientation.y,(*it)->poseHat.orientation.z);
				qcwHati /= qcwHati.norm();

				//convert into body frame
				// Eigen::Vector4f qbwi = getqMat(qcwi)*getqInv(qcb);
				// qbwi /= qbwi.norm();
				// qbwi(1) = 0.0;
				// qbwi(2) = 0.0;
				// qbwi /= qbwi.norm();
				// Eigen::Vector3f pbwi = pcwi - rotatevec(pcb,qbwi);
				// pbwi(2) = 0.0;
				Eigen::Vector4f qbwi = getqMat(qcwi)*getqInv(qcb);
				qbwi /= qbwi.norm();
				Eigen::Vector3f pbwi = pcwi - rotatevec(pcb,qbwi);

				// Eigen::Vector4f qbwHati = getqMat(qcwHati)*getqInv(qcb);
				// qbwHati /= qbwHati.norm();
				// qbwHati(1) = 0.0;
				// qbwHati(2) = 0.0;
				// qbwHati /= qbwHati.norm();
				// Eigen::Vector3f pbwHati = pcwHati - rotatevec(pcb,qbwHati);
				// pbwHati(2) = 0.0;
				Eigen::Vector4f qbwHati = getqMat(qcwHati)*getqInv(qcb);
				qbwHati /= qbwHati.norm();
				Eigen::Vector3f pbwHati = pcwHati - rotatevec(pcb,qbwHati);

				// get the difference between the estimate now and time i
				Eigen::Vector3f pbwbwi = pbwHat - pbwi;
				Eigen::Vector4f qbwbwi = getqMat(getqInv(qbwi))*qbwHat;
				qbwbwi /= qbwbwi.norm();

				// new measure is what the camera thought at time i plus the difference
				Eigen::Vector3f pbw = pbwHati + pbwbwi;
				Eigen::Vector4f qbw = getqMat(qbwHati)*qbwbwi;
				qbw /= qbw.norm();

				// get the error for time i
				pbwTildei = pbw - pbwHat;
				qbwTildei = qbwi - qbwHat;

				// add what the camera thought to the sum
				pbwTildeSum += pbwTildei;
				qbwTildeSum += qbwTildei;

				// std::cout << "\n pbwi \n" << pbwi << std::endl;
				// std::cout << "\n qbwi \n" << qbwi << std::endl;
				// std::cout << "\n pbwHati \n" << pbwHati << std::endl;
				// std::cout << "\n qbwHati \n" << qbwHati << std::endl;
				// std::cout << "\n pbwHat \n" << pbwHat << std::endl;
				// std::cout << "\n qbwHat \n" << qbwHat << std::endl;
				// std::cout << "\n pRatioi \n" << pRatioi << std::endl;
				// std::cout << "\n keyIndi \n" << keyIndi << std::endl;
				// delete *it;
			}
			// new estimate is the weighted average
			pbwHat += dt*kp/numMeas*pbwTildeSum;
			qbwHat += dt*kq/numMeas*qbwTildeSum;
		}
		else
		{
			// pbwTildeSum = 10.0*(pbwMocapLast - pbwHat);
			// qbwTildeSum = 10.0*(qbwMocapLast - qbwHat);
			pbwHat = pbwMocapLast;
			qbwHat = qbwMocapLast;
		}


	}

	poseDeltasRemove.clear();
	// qbwHat /= qbwHat.norm();
	// qbwHat(1) = 0.0;
	// qbwHat(2) = 0.0;
	// qbwHat /= qbwHat.norm();
	qbwHat /= qbwHat.norm();

	// pcwHat = rotatevec(pbwHat + pcb,getqInv(qcb));
	// qcwHat = getqMat(qbwHat)*qcb;
	pcwHat = pbwHat + rotatevec(pcb,qbwHat);
	qcwHat = getqMat(qbwHat)*qcb;
	qcwHat /= qcwHat.norm();


	Eigen::Matrix<float,6,1> pHatt = pDotEstimator.update(pbwHat,t);
	Eigen::Vector3f pHatDot = pHatt.segment(3,3);
	Eigen::Vector3f vHat = rotatevec(pHatDot,getqInv(qbwHat));
	// std::cout << "\n dt " << dt << std::endl;
	std::cout << "\n vHat \n" << vHat << std::endl;
	// std::cout << "\n vb \n" << vb << std::endl;

	// std::cout << std::endl;
	// std::cout << "cvb " << cvHat << ", cwb " << cwHat << std::endl;

	float cvHatDot = kcv*(float(vb.transpose()*vHat) - float(vb.transpose()*vb)*cvHat);


	Eigen::Matrix<float,8,1> qHatt = qDotEstimator.update(qbwHat,t);
	// std::cout << "\n qHatt \n" << qHatt << std::endl;
	Eigen::Vector4f qHatDot = qHatt.segment(4,4);
	Eigen::Vector3f wHat = 2.0*B(qbwHat).transpose()*qHatDot;
	std::cout << "\n wHat \n" << wHat << std::endl;
	// std::cout << "\n wb \n" << wb << std::endl;
	float cwHatDot = kcw*(float(wb.transpose()*wHat) - float(wb.transpose()*wb)*cwHat);

	vcHat = rotatevec((vbHat+getss(wbHat)*pcb),getqInv(qcb));
	wcHat = rotatevec(wbHat,getqInv(qcb));
	// if (landmarkView)
	// {
	// 	vcHat = rotatevec((vbHat+getss(wbHat)*pcb),getqInv(qcb));
	// 	wcHat = rotatevec(wbHat,getqInv(qcb));
	// }
	// else
	// {
	// 	vcHat = rotatevec((vHat+getss(wHat)*pcb),getqInv(qcb));
	// 	wcHat = rotatevec(wHat,getqInv(qcb));
	// }


	// std::cout << std::endl;
	// if (numMeas > 0)
	// {
	// 	cvHat += cvHatDot*dt;
	// 	cwHat += cwHatDot*dt;
	// }
	// std::cout << "cva " << cvHat << ", cwa " << cwHat << std::endl;

	// build and publish odom message for camera
	nav_msgs::Odometry camOdomMsg;
	camOdomMsg.header.stamp = t;
	camOdomMsg.header.frame_id = "world";
	camOdomMsg.child_frame_id = "camera";
	camOdomMsg.pose.pose.position.x = pcwHat(0);
	camOdomMsg.pose.pose.position.y = pcwHat(1);
	camOdomMsg.pose.pose.position.z = pcwHat(2);
	camOdomMsg.pose.pose.orientation.w = qcwHat(0);
	camOdomMsg.pose.pose.orientation.x = qcwHat(1);
	camOdomMsg.pose.pose.orientation.y = qcwHat(2);
	camOdomMsg.pose.pose.orientation.z = qcwHat(3);
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
	bodyOdomMsg.twist.twist.linear.x = vbHat(0);
	bodyOdomMsg.twist.twist.linear.y = vbHat(1);
	bodyOdomMsg.twist.twist.linear.z = vbHat(2);
	bodyOdomMsg.twist.twist.angular.x = wbHat(0);
	bodyOdomMsg.twist.twist.angular.y = wbHat(1);
	bodyOdomMsg.twist.twist.angular.z = wbHat(2);
	bodyOdomPub.publish(bodyOdomMsg);
}
