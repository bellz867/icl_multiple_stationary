#include <mocap_odom_estimator.h>

MocapOdomEstimator::MocapOdomEstimator()
{
	// Parameters
	ros::NodeHandle nhp("~");

	float pcbx,pcby,pcbz,qcbw,qcbx,qcby,qcbz,qmew,qmex,qmey,qmez;
	nhp.param<float>("pcbx", pcbx, 0.0);
	nhp.param<float>("pcby", pcby, 0.0);
	nhp.param<float>("pcbz", pcbz, 0.0);
	nhp.param<float>("qcbw", qcbw, 1.0);
	nhp.param<float>("qcbx", qcbx, 0.0);
	nhp.param<float>("qcby", qcby, 0.0);
	nhp.param<float>("qcbz", qcbz, 0.0);

	pcb << pcbx,pcby,pcbz;

	qcb << qcbw,qcbx,qcby,qcbz;
	qcb /= qcb.norm();

	// nhp.param<std::string>("bodyName", bodyName, "/vrpn_client_node/turtlebot");

	//publisher
	bodyOdomPub = nh.advertise<nav_msgs::Odometry>("/mocap/body/odom",1);
	camOdomPub =  nh.advertise<nav_msgs::Odometry>("/mocap/camera/odom",1);

	// Subscriber
	velSub = nh.subscribe("/odom",5,&MocapOdomEstimator::velCB,this);
	poseSub = nh.subscribe("/vrpn_client_node/turtlebot/pose",5, &MocapOdomEstimator::poseCB,this);

	XHat = Eigen::VectorXf::Zero(7);//px,py,pz,qw,qx,qy,qz
	P = Eigen::MatrixXf::Zero(7,7);
	Q = Eigen::MatrixXf::Zero(7,7);
	R = Eigen::MatrixXf::Zero(7,7);
	F = Eigen::MatrixXf::Identity(7,7);
	H = Eigen::MatrixXf::Zero(7,7);

	P(0,0) = 1.0;//px
	P(1,1) = 1.0;//py
	P(2,2) = 1.0;//pz
	P(3,3) = 0.1;//qw
	P(4,4) = 0.1;//qx
	P(5,5) = 0.1;//qy
	P(6,6) = 0.1;//qz

	Q(0,0) = 2.5;//px
	Q(1,1) = 2.5;//py
	Q(2,2) = 0.7;//pz
	Q(3,3) = 0.7;//qw
	Q(4,4) = 0.7;//qx
	Q(5,5) = 0.7;//qy
	Q(6,6) = 0.7;//qz

	R(0,0) = 200.0;//px
	R(1,1) = 200.0;//py
	R(2,2) = 200.0;//pz
	R(3,3) = 2.0*M_PI;//qw
	R(4,4) = 2.0*M_PI;//qx
	R(5,5) = 2.0*M_PI;//qy
	R(6,6) = 2.0*M_PI;//qz

	p = Eigen::Vector3f::Zero();
	pLast = Eigen::Vector3f::Zero();
	q = Eigen::Vector4f::Zero();
	vx = 0.0;
	wz = 0.0;

	firstVel = true;
	firstPose = true;

	std::cout << "\n started mocap odom \n";
}

void MocapOdomEstimator::poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	Eigen::Vector3f pn(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
	Eigen::Vector4f qn(msg->pose.orientation.w,msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z);

	poseMutex.lock();

	p = pn;
	q = qn;

	if (firstPose)
	{
		firstPose = false;
	}

	poseMutex.unlock();
}

void MocapOdomEstimator::velCB(const nav_msgs::Odometry::ConstPtr& msg)
{
	// std::cout << "\n mocap odom vel \n";
	velMutex.lock();

	if (firstPose)
	{
		velMutex.unlock();
		return;
	}

	//velocity is of turtlebot in body frame of turtlebot x forward, y to left, z up
	ros::Time t = msg->header.stamp;
	vx = msg->twist.twist.linear.x;
	wz = msg->twist.twist.angular.z;
	Eigen::Vector3f v(vx,0,0);
	Eigen::Vector3f w(0,0,wz);

	if (firstVel)
	{
		XHat.segment(0,3) = p;
		XHat.segment(3,4) = q;

		tLast = t;
		firstVel = false;
	}

	float dt = (t-tLast).toSec();
	tLast = t;

	//predict
	float pxHat = XHat(0);
	float pyHat = XHat(1);
	float pzHat = XHat(2);
	float qwHat = XHat(3);
	float qxHat = XHat(4);
	float qyHat = XHat(5);
	float qzHat = XHat(6);

	Eigen::VectorXf DXHat = Eigen::VectorXf::Zero(7);
	DXHat.segment(0,3) = rotatevec(v,XHat.segment(3,4));
	DXHat.segment(3,4) = 0.5*B(XHat.segment(3,4))*w;

	//process jacobian
	F(0,3) = 2.0*qwHat*vx*dt;
	F(0,4) = 2.0*qxHat*vx*dt;
	F(0,5) = -2.0*qyHat*vx*dt;
	F(0,6) = -2.0*qzHat*vx*dt;

	F(1,3) = 2.0*qzHat*vx*dt;
	F(1,4) = 2.0*qyHat*vx*dt;
	F(1,5) = 2.0*qxHat*vx*dt;
	F(1,6) = 2.0*qwHat*vx*dt;

	F(2,3) = -2.0*qyHat*vx*dt;
	F(2,4) = 2.0*qzHat*vx*dt;
	F(2,5) = -2.0*qwHat*vx*dt;
	F(2,6) = 2.0*qxHat*vx*dt;

	F(3,3) = 0;
	F(3,4) = 0;
	F(3,5) = 0;
	F(3,6) = -0.5*wz*dt;

	F(4,3) = 0;
	F(4,4) = 0;
	F(4,5) = 0.5*wz*dt;
	F(4,6) = 0;

	F(5,3) = 0;
	F(5,4) = -0.5*wz*dt;
	F(5,5) = 0;
	F(5,6) = 0;

	F(6,3) = 0.5*wz*dt;
	F(6,4) = 0;
	F(6,5) = 0;
	F(6,6) = 0;

	XHat += (DXHat*dt);
	P = (F*P*F.transpose() + Q);

	// std::cout << "\n DXHat " << DXHat << std::endl;
	// std::cout << "\n P " << P << std::endl;

	float pDiff = (p-pLast).norm();
	bool measGood = true;
	if (pDiff < 0.01)
	{
		measGood = false;
	}
	else
	{
		pLast = p;
	}

	Eigen::Vector4f qHatp = XHat.segment(3,4);
	qHatp /= qHatp.norm();
	if ((qHatp+q).norm() < (qHatp-q).norm())
	{
		q *= -1.0;
	}

	Eigen::VectorXf Z = Eigen::VectorXf::Zero(7);
	Z.segment(0,3) = p;
	Z.segment(3,4) = q;
	velMutex.unlock();

	float distance = (Z-XHat).norm();

	// std::cout << "\n XHat " << XHat << std::endl;
	// std::cout << "\n Z " << Z << std::endl;

	if (measGood)
	{
		// std::cout << "\n F " << F << std::endl;
		Eigen::MatrixXf S = P + (1.0+25*distance)*R;
		Eigen::JacobiSVD<Eigen::MatrixXf> svdS(S, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::MatrixXf SI = svdS.solve(Eigen::MatrixXf::Identity(7,7));

		// std::cout << "\n S " << S << std::endl;
		Eigen::MatrixXf K = P*SI;
		XHat += (K*(Z-XHat));
		P -= (K*P);
	}

	// std::cout << "\n XHat " << XHat << std::endl;

	// build and publish odom message for body
	float normqHat = XHat.segment(3,4).norm();
	// std::cout << "\n normqHat " << normqHat << std::endl;
	nav_msgs::Odometry bodyOdomMsg;
	bodyOdomMsg.header.stamp = t;
	bodyOdomMsg.header.frame_id = "world";
	bodyOdomMsg.child_frame_id = "body_mocap";
	bodyOdomMsg.pose.pose.position.x = XHat(0);
	bodyOdomMsg.pose.pose.position.y = XHat(1);
	bodyOdomMsg.pose.pose.position.z = XHat(2);
	bodyOdomMsg.pose.pose.orientation.w = XHat(3)/normqHat;
	bodyOdomMsg.pose.pose.orientation.x = XHat(4)/normqHat;
	bodyOdomMsg.pose.pose.orientation.y = XHat(5)/normqHat;
	bodyOdomMsg.pose.pose.orientation.z = XHat(6)/normqHat;
	bodyOdomMsg.twist.twist.linear.x = vx;
	bodyOdomMsg.twist.twist.linear.y = 0.0;
	bodyOdomMsg.twist.twist.linear.z = 0.0;
	bodyOdomMsg.twist.twist.angular.x = 0.0;
	bodyOdomMsg.twist.twist.angular.y = 0.0;
	bodyOdomMsg.twist.twist.angular.z = wz;
	bodyOdomPub.publish(bodyOdomMsg);

	Eigen::Vector3f pbwHat = XHat.segment(0,3);
	Eigen::Vector4f qbwHat = (1.0/normqHat)*XHat.segment(3,4);
	Eigen::Vector3f pcwHat = pbwHat + rotatevec(pcb,qbwHat);
	Eigen::Vector4f qcwHat = getqMat(qbwHat)*qcb;
	qcwHat /= qcwHat.norm();

	Eigen::Vector3f vbHat(vx,0,0);
	Eigen::Vector3f wbHat(0,0,wz);
	Eigen::Vector3f vcHat = rotatevec(vbHat+getss(wbHat)*pcb,getqInv(qcb));
	Eigen::Vector3f wcHat = rotatevec(wbHat,getqInv(qcb));

	nav_msgs::Odometry camOdomMsg;
	camOdomMsg.header.stamp = t;
	camOdomMsg.header.frame_id = "world";
	camOdomMsg.child_frame_id = "body_mocap";
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
}
