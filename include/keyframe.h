#include <iostream>
#include <fstream>
#include <ctime>
#include <string>
#include <vector>
#include <deque>
#include <mutex>
// #include <queue>

#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <icl_multiple_stationary/Output.h>
#include <icl_multiple_stationary/Wall.h>
#include <icl_multiple_stationary/PoseDelta.h>
#include <icl_multiple_stationary/Key.h>

#include <helper_functions.h>
#include <patch_estimator.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

struct Keyframe
{
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
  image_transport::Publisher imageOutputPub,keyPub;
	image_transport::Subscriber imageSub;
	// cv::Mat kgray,pgray;
	cv::Mat kgray;
  ros::Publisher featurePub,wallPub,poseDeltaPub,keyInfoPub,odomPub,pointCloudPub;
  ros::Subscriber outputSub,odomSub;
  std::string cameraName;//body name and camera name
	// cv::Mat tlmask,trmask,blmask,brmask;
	std::vector<cv::Mat> masks;
	int imageWidth,imageHeight;
	int partitionCols,partitionRows;

	ros::Time tLast;//last time
	nav_msgs::Odometry keyOdom,imageOdom;
	Eigen::Vector4f qpw;

	std::deque<nav_msgs::Odometry> odomSync;
	std::mutex odomMutex,pubMutex,patchMutex;
	bool firstGray;

	// std::queue<ros::Time> plotPointsTimes;//past points estimate
	// std::queue<std::vector<cv::Point2f>> plotPoints;//past points estimate
	PatchEstimator* newPatch;
	std::vector<PatchEstimator*> patchs;
	int patchIndMax;
	// std::vector<int> activePatchs;
	int patchRadius;
	float fx,fy,cx,cy;
	float fq,fp,ft,fn,fd;
	float zmin,zmax;
	float tau;

	std::string expName;
	bool saveExp;
	int keyInd;
	bool keyframeShutdown;
	float maxdt;
	bool firstOutput;
	float keyHeight;
	float keyToCeiling;
	int minDistance;
	int blockSize;
	float qualityLevel;
	int minFeaturesBad;
	int minFeaturesDanger;
	bool tooFewFeatures;
	bool foundKeyImage;
	bool firstOdom;
	int numberFeaturesToFindPerPart;
	int initialNumberFeatures;
	bool keyframeInDanger;
	int numberFeaturesPerPartRow;
	int numberFeaturesPerPartCol;
	int partitionSide;
	int patchSizeBase;
	int checkSizeBase;

  Keyframe();

	Keyframe(int keyIndInit, cv::Mat& camMat, std::vector<cv::Mat>& masksInit, int imageWidth, int imageHeight);

	bool findFeatures(cv::Mat& gray, ros::Time t, nav_msgs::Odometry odom);

	void publishFeatures(ros::Time t, uint8_t needToShutdown,  nav_msgs::Odometry odom);

	void match(cv::Mat& gray, float dt, Eigen::Vector3f vc, Eigen::Vector3f wc, ros::Time t, Eigen::Vector4f qcp);

	void odomCB(const nav_msgs::Odometry::ConstPtr& msg);

	void imageCB(const sensor_msgs::Image::ConstPtr& msg);

	void outputCB(const icl_multiple_stationary::Output::ConstPtr& msg);

	void shutdown();

	void startup();
};
