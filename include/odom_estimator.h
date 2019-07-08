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
  ros::Subscriber velSub,poseDeltaSub;
  std::string bodyName;
	std::string cameraName;
	ros::Time tVelLast;//last time
	std::mutex poseDeltaMutex;
	std::deque<icl_multiple_stationary::PoseDelta::ConstPtr> poseDeltas;

	bool firstVel;
	Eigen::Vector3f pcwHat;
	Eigen::Vector4f qcwHat;
	Eigen::Vector3f vcHat;
	Eigen::Vector3f wcHat;
	Eigen::Vector3f pcb;
	Eigen::Vector4f qcb;

	float pTau;
	float qTau;
	float vTau;
	float wTau;

  OdomEstimator();

	void velCB(const nav_msgs::Odometry::ConstPtr& msg);

	void poseDeltaCB(const icl_multiple_stationary::PoseDelta::ConstPtr& msg);
};
