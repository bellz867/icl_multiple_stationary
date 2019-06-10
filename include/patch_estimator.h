#ifndef PATCHESTIMATOR_H
#define PATCHESTIMATOR_H

#include <vector>
#include <deque>
#include <mutex>

#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <depth_estimator.h>
#include <helper_functions.h>
#include <data_save.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

struct PatchEstimator
{
	ros::NodeHandle nh;
	std::string cameraName;
	image_transport::ImageTransport it;
	image_transport::Subscriber imageSub;
	image_transport::Publisher imagePub;
	cv::Mat kimage,pimage;
	int keyInd,patchInd;
  ros::Subscriber odomSub,roiSub;
	ros::Publisher wallPub,poseDeltaPub,roiPub,odomPub,pointCloudPub,odomDelayedPub;
	Eigen::Vector3f tkcHat;
	Eigen::Vector3f nkHat;
	Eigen::Vector4f qkcHat;
	Eigen::Vector3f pkcHat;
	Eigen::Vector3f pckHat;
	Eigen::Vector4f qckHat;
	float dkcHat,tau,dkHat;
	float fx,fy,cx,cy,zmin,zmax;
	DepthEstimator* newDepthEstimator;
	std::vector<DepthEstimator*> depthEstimators;
	std::deque<nav_msgs::Odometry> odomSync;
	std::deque<nav_msgs::Odometry> markerOdomSync;
	std::mutex odomMutex,roiMutex,pubMutex,markerOdomMutex,featureMutex;
	ros::Time tLast;
	float pTau,qTau,tTau,nTau,dTau;
	nav_msgs::Odometry keyOdom,imageOdom;
	cv::Mat camMat;
	Eigen::Matrix3f camMatf,camMatIf;
	bool firstOdomImageCB;
	bool dkEstimated;
	int imageWidth,imageHeight;
	int minFeaturesDanger;
	int minFeaturesBad;
	bool patchShutdown;
	bool saveExp;
	std::string expName;
	std::vector<DataSave*> data;
	ros::Time tStart;
	Eigen::Matrix<float,2,3> TfLast;

	~PatchEstimator();

	PatchEstimator();

	PatchEstimator(int imageWidth, int imageHeight, int minFeaturesDanger, int minFeaturesBad, int keyInd, int patchInd, cv::Mat& image,
		             nav_msgs::Odometry imageOdom, std::vector<cv::Point2f> pts, float fxInit, float fyInit, float cxInit, float cyInit,
								 float zminInit, float zmaxInit, ros::Time t, float fq, float fp, float ft, float fn, float fd,
								 std::string cameraNameInit, float tauInit, bool saveExpInit, std::string expNameInit);

	void markerOdomCB(const nav_msgs::Odometry::ConstPtr& msg);

	void odomCB(const nav_msgs::Odometry::ConstPtr& msg);

	void imageCB(const sensor_msgs::Image::ConstPtr& msg);

	void match(cv::Mat& image, float dt, Eigen::Vector3f vc, Eigen::Vector3f wc, ros::Time t);

	void update(std::vector<cv::Point2f>& pPts, std::vector<cv::Point2f>& kPts, std::vector<cv::Point2f>& cPts, Eigen::Vector3f vc,
		          Eigen::Vector3f wc, ros::Time t, float dt);
};

#endif
