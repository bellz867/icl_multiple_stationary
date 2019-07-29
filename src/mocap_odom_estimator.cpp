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

	XHat = Eigen::VectorXf::Zero(4);//px,py,qw,qz,vx,wz,ax,alz
	P = Eigen::MatrixXf::Zero(4,4);
	Q = Eigen::MatrixXf::Zero(4,4);
	R = Eigen::MatrixXf::Zero(4,4);
	F = Eigen::MatrixXf::Identity(4,4);
	H = Eigen::MatrixXf::Identity(4,4);
	HT = H.transpose();
	Hb = Eigen::MatrixXf::Zero(2,8);
	Hb.block(0,4,2,2) = Eigen::MatrixXf::Identity(2,2);

	P(0,0) = 1.0;//px
	P(1,1) = 1.0;//py
	P(2,2) = 0.1;//qw
	P(3,3) = 0.1;//qz

	Q(0,0) = 2.5;//px
	Q(1,1) = 2.5;//py
	Q(2,2) = 0.7;//qw
	Q(3,3) = 0.7;//qz

	R(0,0) = 200.0;//px
	R(1,1) = 200.0;//py
	R(2,2) = 2.0*M_PI;//qw
	R(3,3) = 2.0*M_PI;//qz

	px = 0.0;
	py = 0.0;
	pxLast = 0.0;
	pyLast = 0.0;
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
	float vxHat = vx;
	float wzHat = wz;

	Eigen::VectorXf DXHat = Eigen::VectorXf::Zero(4);
	DXHat(0) = (qwHat*vxHat*qwHat - qzHat*vxHat*qzHat)*dt;
	DXHat(1) = 2.0*qzHat*vxHat*qwHat*dt;
	DXHat(2) = -0.5*qzHat*wzHat*dt;
	DXHat(3) = 0.5*qwHat*wzHat*dt;

	//process jacobian
	F(0,2) = 2.0*qwHat*vxHat*dt;
	F(0,3) = -2.0*qzHat*vxHat*dt;
	F(1,2) = 2.0*qzHat*vxHat*dt;
	F(1,3) = 2.0*qwHat*vxHat*dt;
	F(2,3) = -0.5*wzHat*dt;
	F(3,2) = 0.5*wzHat*dt;

	XHat += DXHat;
	P = (F*P*F.transpose() + Q);

	// std::cout << "\n DXHat " << DXHat << std::endl;
	// std::cout << "\n P " << P << std::endl;

	Eigen::VectorXf Z = Eigen::VectorXf::Zero(4);
	Z(0) = px;
	Z(1) = py;
	Z(2) = qw;
	Z(3) = qz;

	float xDiff = fabsf(px-pxLast);
	float yDiff = fabsf(py-pyLast);
	bool measGood = true;
	if ((xDiff < 0.01) && (yDiff < 0.01))
	{
		measGood = false;
	}
	else
	{
		pxLast = px;
		pyLast = py;
	}

	if (sqrtf((qwHat+Z(2))*(qwHat+Z(2))+(qzHat+Z(3))*(qzHat+Z(3))) < sqrtf((qwHat-Z(2))*(qwHat-Z(2))+(qzHat-Z(3))*(qzHat-Z(3))))
	{
		Z(2) = -Z(2);
		Z(3) = -Z(3);
	}
	velMutex.unlock();

	float distance = float((Z-XHat).norm());

	// std::cout << "\n XHat " << XHat << std::endl;
	// std::cout << "\n Z " << Z << std::endl;

	if (measGood)
	{
		// std::cout << "\n F " << F << std::endl;
		Eigen::MatrixXf S = P + (1.0+25*distance)*R;
		Eigen::JacobiSVD<Eigen::MatrixXf> svdS(S, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::MatrixXf SI = svdS.solve(Eigen::MatrixXf::Identity(4,4));

		// std::cout << "\n S " << S << std::endl;
		Eigen::MatrixXf K = P*SI;
		XHat += (K*(Z-XHat));
		P -= (K*P);
	}

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
	bodyOdomMsg.twist.twist.linear.x = vxHat;
	bodyOdomMsg.twist.twist.linear.y = 0.0;
	bodyOdomMsg.twist.twist.linear.z = 0.0;
	bodyOdomMsg.twist.twist.angular.x = 0.0;
	bodyOdomMsg.twist.twist.angular.y = 0.0;
	bodyOdomMsg.twist.twist.angular.z = wzHat;
	bodyOdomPub.publish(bodyOdomMsg);
}
