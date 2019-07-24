#include <mocap_odom_estimator.h>

MocapOdomEstimator::MocapOdomEstimator()
{
	// Parameters
	ros::NodeHandle nhp("~");
	// nhp.param<std::string>("bodyName", bodyName, "/vrpn_client_node/turtlebot");

	//publisher
	bodyOdomPub = nh.advertise<nav_msgs::Odometry>("/mocap/body/odom",1);

	// Subscriber
	velSub = nh.subscribe("/odom",5,&MocapOdomEstimator::velCB,this);
	poseSub = nh.subscribe("/vrpn_client_node/turtlebot/pose",5, &MocapOdomEstimator::poseCB,this);

	XHat = Eigen::VectorXf::Zero(8);//px,py,qw,qz,vx,wz,ax,alz
	P = Eigen::MatrixXf::Zero(8,8);
	Q = Eigen::MatrixXf::Zero(8,8);
	R = Eigen::MatrixXf::Zero(6,6);
	F = Eigen::MatrixXf::Identity(8,8);
	H = Eigen::MatrixXf::Zero(6,8);
	H.block(0,0,6,6) = Eigen::MatrixXf::Identity(6,6);
	HT = H.transpose();

	P(0,0) = 1.0;//px
	P(1,1) = 1.0;//py
	P(2,2) = 0.1;//qw
	P(3,3) = 0.1;//qz
	P(4,4) = 0.1;//vx
	P(5,5) = 0.1;//wz
	P(6,6) = 1.0;//ax
	P(7,7) = 1.0;//alz

	Q(0,0) = 2.5;//px
	Q(1,1) = 2.5;//py
	Q(2,2) = 0.7;//qw
	Q(3,3) = 0.7;//qz
	Q(4,4) = 0.01;//vx
	Q(5,5) = 0.01;//wz
	Q(6,6) = 1.0;//ax
	Q(7,7) = 1.0;//alz

	R(0,0) = 5.0;//px
	R(1,1) = 5.0;//py
	R(2,2) = 0.7;//qw
	R(3,3) = 0.7;//qz
	R(4,4) = 0.001;//vx
	R(5,5) = 0.001;//wz

	px = 0.0;
	py = 0.0;
	qw = 0.0;
	qz = 0.0;
	vx = 0.0;
	wz = 0.0;

	firstVel = true;
	firstPose = true;

	std::cout << "\n started mocap odom \n";
}

void MocapOdomEstimator::poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	float pxn = msg->pose.position.x;
	float pyn = msg->pose.position.y;
	float qwn = msg->pose.orientation.w;
	float qzn = msg->pose.orientation.z;

	float normq = sqrtf(pxn*pxn+pyn*pyn);
	if (normq < 0.01)
	{
		return;
	}

	poseMutex.lock();

	px = pxn;
	py = pyn;
	qw = qwn;
	qz = qzn;

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

	if (firstVel)
	{
		XHat(0) = px;
		XHat(1) = py;
		XHat(2) = qw;
		XHat(3) = qz;
		XHat(4) = vx;
		XHat(5) = wz;
		tLast = t;
		firstVel = false;
	}

	float dt = (t-tLast).toSec();
	tLast = t;

	//predict
	float pxHat = XHat(0);
	float pyHat = XHat(1);
	float qwHat = XHat(2);
	float qzHat = XHat(3);
	float vxHat = XHat(4);
	float wzHat = XHat(5);
	float aHat = XHat(6);
	float alHat = XHat(7);

	Eigen::VectorXf DXHat = Eigen::VectorXf::Zero(8);
	DXHat(0) = (qwHat*vxHat*qwHat - qzHat*vxHat*qzHat)*dt;
	DXHat(1) = 2.0*qzHat*vxHat*qwHat*dt;
	DXHat(2) = -0.5*qzHat*wzHat*dt;
	DXHat(3) = 0.5*qwHat*wzHat*dt;
	DXHat(4) = aHat*dt;
	DXHat(5) = alHat*dt;

	XHat += DXHat;
	P = (F*P*F.transpose() + Q);

	// std::cout << "\n DXHat " << DXHat << std::endl;
	// std::cout << "\n P " << P << std::endl;

	Eigen::VectorXf Z = Eigen::VectorXf::Zero(6);
	if (((fabs(px-XHat(0)) < 0.01) && (fabs(py-XHat(1)) < 0.01))
	     || (fabs(px-XHat(0)) > 0.5)
			 || (fabs(py-XHat(1)) > 0.5)
			 || (fabs(acos(qw)*2.0 - acos(qwHat)*2.0) > 30.0*M_PI/180.0)
			 || (fabs(acos(qz)*2.0 - acos(qzHat)*2.0) > 30.0*M_PI/180.0))
	{
		Z(0) = XHat(0);
		Z(1) = XHat(1);
		Z(2) = XHat(2);
		Z(3) = XHat(3);
	}
	else
	{
		Z(0) = px;
		Z(1) = py;
		Z(2) = qw;
		Z(3) = qz;
	}

	Z(4) = vx;
	Z(5) = wz;
	velMutex.unlock();

	if (sqrtf((qwHat+Z(2))*(qwHat+Z(2))+(qzHat+Z(3))*(qzHat+Z(3))) < sqrtf((qwHat-Z(2))*(qwHat-Z(2))+(qzHat-Z(3))*(qzHat-Z(3))))
	{
		Z(2) = -Z(2);
		Z(3) = -Z(3);
	}

	// std::cout << "\n XHat " << XHat << std::endl;
	// std::cout << "\n Z " << Z << std::endl;

	//process jacobian
	F(0,2) = 2.0*qwHat*vxHat*dt;
	F(0,3) = -2.0*qzHat*vxHat*dt;
	F(0,4) = (qwHat*qwHat - qzHat*qzHat)*dt;
	F(1,2) = 2.0*qzHat*vxHat*dt;
	F(1,3) = 2.0*qwHat*vxHat*dt;
	F(1,4) = 2.0*qwHat*qzHat*dt;
	F(2,3) = -0.5*wzHat*dt;
	F(2,5) = -0.5*qzHat*dt;
	F(3,2) = 0.5*wzHat*dt;
	F(3,5) = 0.5*qwHat*dt;
	F(4,6) = dt;
	F(5,7) = dt;

	// std::cout << "\n F " << F << std::endl;

	Eigen::MatrixXf S = P.block(0,0,6,6) + R;
	Eigen::JacobiSVD<Eigen::MatrixXf> svdS(S, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::MatrixXf SI = svdS.solve(Eigen::MatrixXf::Identity(6,6));

	// std::cout << "\n S " << S << std::endl;

	Eigen::MatrixXf K = P.block(0,0,8,6)*SI;
	// std::cout << "\n K " << K << std::endl;
	// std::cout << "\n K \n" << K <<std::endl;

	// std::cout << "\n z \n" << z <<std::endl;
	// std::cout << "\n 88888 xHat ls \n" << xHat <<std::endl;
	// std::cout << "\n z-xHat \n" << (z-xHat.segment(0,7)) <<std::endl;
	// std::cout << "\n K*(z-xHat) \n" << (K*(z-xHat.segment(0,7))).segment(0,7) <<std::endl;

	// std::cout << std::endl << xHat.segment(0,7) << std::endl;
	// std::cout << std::endl << P << std::endl;
	XHat += (K*(Z-XHat.segment(0,6)));
	// std::cout << "\n XHat " << XHat << std::endl;
	// std::cout << std::endl << "----------------" << std::endl;
	// xHat.segment(3,4) /= xHat.segment(3,4).norm();
	// P = (Eigen::Matrix<float,6,6>::Identity() - K*H)*P;
	P -= (K*H*P);
	// std::cout << "\n P " << P << std::endl;

	// build and publish odom message for body
	float normqHat = sqrtf(XHat(2)*XHat(2)+XHat(3)*XHat(3));
	// std::cout << "\n normqHat " << normqHat << std::endl;
	nav_msgs::Odometry bodyOdomMsg;
	bodyOdomMsg.header.stamp = t;
	bodyOdomMsg.header.frame_id = "world";
	bodyOdomMsg.child_frame_id = "body_mocap";
	bodyOdomMsg.pose.pose.position.x = XHat(0);
	bodyOdomMsg.pose.pose.position.y = XHat(1);
	bodyOdomMsg.pose.pose.position.z = 0.0;
	bodyOdomMsg.pose.pose.orientation.w = XHat(2)/normqHat;
	bodyOdomMsg.pose.pose.orientation.x = 0.0;
	bodyOdomMsg.pose.pose.orientation.y = 0.0;
	bodyOdomMsg.pose.pose.orientation.z = XHat(3)/normqHat;
	bodyOdomMsg.twist.twist.linear.x = XHat(4);
	bodyOdomMsg.twist.twist.linear.y = 0.0;
	bodyOdomMsg.twist.twist.linear.z = 0.0;
	bodyOdomMsg.twist.twist.angular.x = 0.0;
	bodyOdomMsg.twist.twist.angular.y = 0.0;
	bodyOdomMsg.twist.twist.angular.z = XHat(5);
	bodyOdomPub.publish(bodyOdomMsg);
}
