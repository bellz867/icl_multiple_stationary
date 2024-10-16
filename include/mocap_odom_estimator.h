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

struct MocapOdomEstimator
{
	ros::NodeHandle nh;
  ros::Publisher bodyOdomPub,camOdomPub;
  ros::Subscriber velSub,poseSub;
  std::string bodyName;
	ros::Time tLast;//last time

	bool firstVel;
	bool firstPose;
	Eigen::MatrixXf P,Q,R,F,H,HT,Hb;
	Eigen::VectorXf XHat;
	std::mutex poseMutex,velMutex;
	Eigen::Vector3f p,pLast;
	Eigen::Vector4f q;
	float vx,wz;

	Eigen::Vector3f pcb;
	Eigen::Vector4f qcb;

  MocapOdomEstimator();

	void velCB(const nav_msgs::Odometry::ConstPtr& msg);

	void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
};
