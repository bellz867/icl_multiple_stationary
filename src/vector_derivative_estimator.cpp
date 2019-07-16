#include <vector_derivative_estimator.h>

VectorDerivativeEstimator::VectorDerivativeEstimator()
{
}

void VectorDerivativeEstimator::initialize(int stateSizeInit)
{
	stateSize = stateSizeInit;
	xHat = Eigen::VectorXf::Zero(2*stateSize);
	firstUpdate = true;
	P = Eigen::MatrixXf::Zero(2*stateSize,2*stateSize);
	Q = Eigen::MatrixXf::Zero(2*stateSize,2*stateSize);

	// feature variance
	float rr = 0.00001;
	R = rr*Eigen::MatrixXf::Identity(stateSize,stateSize);//measurment covariance
	P.block(0,0,stateSize,stateSize) = R;//covariance
	Q.block(0,0,stateSize,stateSize) =  0.1*Eigen::MatrixXf::Identity(stateSize,stateSize);//process covariance

	// flow variance
	P.block(stateSize,stateSize,stateSize,stateSize) = Eigen::MatrixXf::Identity(stateSize,stateSize);//covariance
	Q.block(stateSize,stateSize,stateSize,stateSize) = Eigen::MatrixXf::Identity(stateSize,stateSize);//process covariance

	//process jacobian
	F = Eigen::MatrixXf::Identity(2*stateSize,2*stateSize);

	//measruement jacobian
	H = Eigen::MatrixXf::Zero(stateSize,2*stateSize);
	H.block(0,0,stateSize,stateSize) = Eigen::MatrixXf::Identity(stateSize,stateSize);
	HT = H.transpose();
}

Eigen::VectorXf VectorDerivativeEstimator::update(Eigen::VectorXf newMeasure, ros::Time newTime)
{
	ros::Time t = newTime;
	Eigen::VectorXf z = newMeasure;

	if (firstUpdate)
	{
		xHat.segment(0,stateSize) = z;
		tLast = t;
		firstUpdate = false;
	}

	float dt = (t-tLast).toSec();
	tLast = t;

	//predict
	F.block(0,stateSize,stateSize,stateSize) = dt*Eigen::MatrixXf::Identity(stateSize,stateSize);
	// std::cout << std::endl << "+++++++++++++++++" << std::endl;
	// std::cout << "\n xHat \n" << xHat <<std::endl;
	// std::cout << "\n xDot(xHat) \n" << xDot(xHat) <<std::endl;
	xHat += (xDot(xHat)*dt);
	// xHat.segment(3,4) /= xHat.segment(3,4).norm();
	// // P += ((F*P + P*F.transpose() + Q)*dt);
	P = (F*P*F.transpose() + Q);
	//
	// std::cout << "\n F \n" << F <<std::endl;
	// std::cout << "\n P \n" << P <<std::endl;

	// Eigen::Matrix2f argK = H*P*HT+R;
	// std::cout << "\n argK \n" << argK <<std::endl;
	// Eigen::JacobiSVD<Eigen::MatrixXf> svdargK(argK, Eigen::ComputeThinU | Eigen::ComputeThinV);
	// Eigen::Matrix2f argKI = svdargK.solve(Eigen::Matrix2f::Identity());
	// Eigen::Matrix3f argK = H*P*HT + R;
	Eigen::MatrixXf argK = P.block(0,0,stateSize,stateSize) + R;
	Eigen::JacobiSVD<Eigen::MatrixXf> svdargK(argK, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::MatrixXf argKI = svdargK.solve(Eigen::MatrixXf::Identity(stateSize,stateSize));
	// std::cout << "\n argKI \n" << argKI <<std::endl;
	// Eigen::Matrix<float,6,3> K = P*HT*argKI;
	Eigen::MatrixXf K = P.block(0,0,2*stateSize,stateSize)*argKI;
	// std::cout << "\n K \n" << K <<std::endl;

	// std::cout << "\n z \n" << z <<std::endl;
	// std::cout << "\n 88888 xHat ls \n" << xHat <<std::endl;
	// std::cout << "\n z-xHat \n" << (z-xHat.segment(0,7)) <<std::endl;
	// std::cout << "\n K*(z-xHat) \n" << (K*(z-xHat.segment(0,7))).segment(0,7) <<std::endl;

	// std::cout << std::endl << xHat.segment(0,7) << std::endl;
	// std::cout << std::endl << P << std::endl;
	xHat += (K*(z-xHat.segment(0,stateSize)));

	// std::cout << std::endl << "----------------" << std::endl;
	// xHat.segment(3,4) /= xHat.segment(3,4).norm();
	// P = (Eigen::Matrix<float,6,6>::Identity() - K*H)*P;
	P -= (K*H*P);

	return xHat;
}

Eigen::VectorXf VectorDerivativeEstimator::xDot(Eigen::VectorXf x)
{
	Eigen::VectorXf xDot = Eigen::VectorXf::Zero(2*stateSize);
	xDot.segment(0,stateSize) = x.segment(stateSize,stateSize);
	return xDot;
}
