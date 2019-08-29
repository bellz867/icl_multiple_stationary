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
	Eigen::MatrixXf S = P.block(0,0,stateSize,stateSize) + R;
	Eigen::JacobiSVD<Eigen::MatrixXf> svdargK(S, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::MatrixXf SI = svdargK.solve(Eigen::MatrixXf::Identity(stateSize,stateSize));
	// std::cout << "\n SI \n" << SI <<std::endl;


	//check to see if the value is an inlier before saving
	Eigen::VectorXf zDx = z-xHat.segment(0,stateSize);
	// float chi2 = 3.84; //chi^2 for 95%
	float chi2 = 6.63; //chi^2 for 99%
	float chiTestVal = zDx.transpose()*SI*zDx;
	if (chiTestVal > chi2)
	{
		// std::cout << "\n chiTest fail " << chiTestVal << " zDx " << zDx.transpose() << std::endl;
		return xHat;
	}
	else
	{
		// std::cout << "\n chiTest pass " << chiTestVal << " zDx " << zDx.transpose() << std::endl;
	}

	// Eigen::Matrix<float,6,3> K = P*HT*argKI;
	Eigen::MatrixXf K = P.block(0,0,2*stateSize,stateSize)*SI;
	xHat += (K*zDx);
	P -= (K*H*P);

	return xHat;
}

Eigen::VectorXf VectorDerivativeEstimator::update(Eigen::VectorXf newMeasure, ros::Time newTime, Eigen::VectorXf newMeasureExpected)
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
	Eigen::MatrixXf S = P.block(0,0,stateSize,stateSize) + R;
	Eigen::JacobiSVD<Eigen::MatrixXf> svdargK(S, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::MatrixXf SI = svdargK.solve(Eigen::MatrixXf::Identity(stateSize,stateSize));
	// std::cout << "\n SI \n" << SI <<std::endl;


	//check to see if the value is an inlier before saving
	Eigen::VectorXf zDz = z-newMeasureExpected;
	// float chi2 = 3.84; //chi^2 for 95%
	float chi2 = 6.63; //chi^2 for 99%
	float chiTestVal = zDz.transpose()*SI*zDz;
	if (chiTestVal > chi2)
	{
		// std::cout << "\n chiTest fail " << chiTestVal << " zDz " << zDz.transpose() << std::endl;
		z = newMeasureExpected;
	}
	else
	{
		// std::cout << "\n chiTest pass " << chiTestVal << " zDz " << zDz.transpose() << std::endl;
	}

	Eigen::VectorXf zDx = z-xHat.segment(0,stateSize);

	// Eigen::Matrix<float,6,3> K = P*HT*argKI;
	Eigen::MatrixXf K = P.block(0,0,2*stateSize,stateSize)*SI;
	xHat += (K*zDx);
	P -= (K*H*P);

	return xHat;
}

Eigen::VectorXf VectorDerivativeEstimator::xDot(Eigen::VectorXf x)
{
	Eigen::VectorXf xDot = Eigen::VectorXf::Zero(2*stateSize);
	xDot.segment(0,stateSize) = x.segment(stateSize,stateSize);
	return xDot;
}
