#include <mutex>
#include <deque>

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <helper_functions.h>
#include <icl_multiple_stationary/PoseDelta.h>

struct OdomEstimator
{
	ros::NodeHandle nh;
  ros::Publisher camOdomPub,bodyOdomPub,camPosePub;
  ros::Subscriber velSub,poseDeltaSub,initialPoseSub;
  std::string bodyName;
	std::string cameraName;
	ros::Time tVelLast;//last time
	std::mutex poseDeltaMutex;
	std::deque<icl_multiple_stationary::PoseDelta::ConstPtr> poseDeltas;
	bool useMocap,gotInitialPose;

	bool firstVel;
	Eigen::Vector3f pcwHat;
	Eigen::Vector4f qcwHat;
	Eigen::Vector3f vcHat;
	Eigen::Vector3f wcHat;
	Eigen::Vector3f pbwHat;
	Eigen::Vector4f qbwHat;
	Eigen::Vector3f vbHat;
	Eigen::Vector3f wbHat;
	Eigen::Vector3f pcb;
	Eigen::Vector4f qcb;
	Eigen::Vector3f pbInit;
	Eigen::Vector4f qbInit;

	float pTau;
	float qTau;
	float vTau;
	float wTau;

  OdomEstimator();

	void mocapPoseCB(const nav_msgs::Odometry::ConstPtr& msg);

	void velCB(const nav_msgs::Odometry::ConstPtr& msg);

	void poseDeltaCB(const icl_multiple_stationary::PoseDelta::ConstPtr& msg);
};
