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
  odomPub = nh.advertise<nav_msgs::Odometry>(cameraName+"/odom",1);

	// Subscriber
	velSub = nh.subscribe(bodyName+"/odom",5,&OdomEstimator::velCB,this);

	pcwHat = Eigen::Vector3f::Zero();
	qcwHat = Eigen::Vector4f::Zero();
	qcwHat(0) = 1.0;
	vcHat = Eigen::Vector3f::Zero();
	wcHat = Eigen::Vector3f::Zero();
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

	vcHat += kvi*(vc-vcHat);
	wcHat += kwi*(wc-wcHat);

	// build and publish odom message
	nav_msgs::Odometry odomMsg;
	odomMsg.header.stamp = t;
	odomMsg.header.frame_id = "world";
	odomMsg.child_frame_id = "camera";
	odomMsg.pose.pose.position.x = pcwHat(0);
	odomMsg.pose.pose.position.y = pcwHat(1);
	odomMsg.pose.pose.position.z = pcwHat(2);
	odomMsg.pose.pose.orientation.w = qcwHat(0);
	odomMsg.pose.pose.orientation.x = qcwHat(1);
	odomMsg.pose.pose.orientation.y = qcwHat(2);
	odomMsg.pose.pose.orientation.z = qcwHat(3);
	odomMsg.twist.twist.linear.x = vcHat(0);
	odomMsg.twist.twist.linear.y = vcHat(1);
	odomMsg.twist.twist.linear.z = vcHat(2);
	odomMsg.twist.twist.angular.x = wcHat(0);
	odomMsg.twist.twist.angular.y = wcHat(1);
	odomMsg.twist.twist.angular.z = wcHat(2);
	odomPub.publish(odomMsg);
}
