#include <mutex>

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <helper_functions.h>

struct OdomEstimator
{
	ros::NodeHandle nh;
  ros::Publisher odomPub,camPosePub;
  ros::Subscriber velSub;
  std::string bodyName;
	std::string cameraName;
	ros::Time tVelLast;//last time

	bool firstVel;
	Eigen::Vector3f pcwHat;
	Eigen::Vector4f qcwHat;
	Eigen::Vector3f vcHat;
	Eigen::Vector3f wcHat;
	Eigen::Vector3f pfi;
	Eigen::Vector4f qfi;

	float piTau;
	float qiTau;
	float viTau;
	float wiTau;

  OdomEstimator();

	void velCB(const nav_msgs::Odometry::ConstPtr& msg);
};
