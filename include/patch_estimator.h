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

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/eigen.hpp>


#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <depth_estimator.h>
#include <helper_functions.h>
#include <data_save.h>
#include <vector_derivative_estimator.h>

#include <icl_multiple_stationary/PoseDelta.h>
#include <icl_multiple_stationary/Roi.h>
#include <icl_multiple_stationary/Wall.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

struct PatchEstimator
{
	ros::NodeHandle nh;
	std::string cameraName;
	image_transport::ImageTransport it;
	image_transport::Subscriber imageSub;
	image_transport::Publisher imagePub;//,imagePub2;
	// image_transport::Publisher imagePub2;
	cv::Mat kimage,pimage;
	int keyInd,patchInd,partitionInd,partitionRows,partitionCols,minDistance,currentAvgPartition,numberFeaturesPerPartCol,numberFeaturesPerPartRow;
  ros::Subscriber odomSub,roiSub,chessboardSub;
	ros::Publisher poseDeltaPub,roiPub,wallPub,pointCloudPub,chessboardPub;
	// ros::Publisher wallPub,poseDeltaPub,roiPub,odomPub,pointCloudPub,odomDelayedPub;
	Eigen::Vector3f tkcHat;
	Eigen::Vector3f nkHat;
	Eigen::Vector4f qkcHat;
	Eigen::Vector3f pkcHat;
	Eigen::Vector3f pckHat;
	Eigen::Vector4f qckHat;
	Eigen::Vector3f pkp;
	Eigen::Vector4f qkp;
	Eigen::Vector3f ppw;
	Eigen::Vector4f qpw;
	bool firstkp;
	VectorDerivativeEstimator tkcDotEstimator,qkcDotEstimator;
	float dkcHat,tau,dkHat;
	float fx,fy,cx,cy,zmin,zmax;
	DepthEstimator* newDepthEstimator;
	std::vector<DepthEstimator*> depthEstimators;
	std::deque<nav_msgs::Odometry> odomSync;
	std::deque<nav_msgs::Odometry> markerOdomSync;
	std::mutex odomCBMutex,odomMutex,roiMutex,pubMutex,markerOdomMutex,featureMutex,findPointsMutex,chessboardMutex,destroyLock;
	ros::Time tLast;
	float pTau,qTau,tTau,nTau,dTau,GTau;
	nav_msgs::Odometry keyOdom,imageOdom;
	cv::Mat camMat,camMatD;
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
	Eigen::Matrix<float,3,3> GfLast,GkfLast;
	cv::Mat Gkcum;
	int patchSizeBase,checkSizeBase;
	Eigen::Vector3f pcb;
	Eigen::Vector4f qcb;
	bool firstImage;
	bool allPtsKnown;
	bool landmarkView;
	int numLandmarkCheck;
	Eigen::Vector3f wcbHat;

	VectorDerivativeEstimator qDotEstimator;

	~PatchEstimator();

	PatchEstimator();

	PatchEstimator(int imageWidthInit, int imageHeightInit, int partitionRowsInit, int partitionColsInit, int minDistanceInit, int minFeaturesDangerInit,
								 int minFeaturesBadInit, int keyIndInit, int patchIndInit, int partitionIndInit, float fxInit, float fyInit,
								 float cxInit, float cyInit, float zminInit, float zmaxInit, float fq, float fp, float ft, float fn, float fd,
								 float fG, std::string cameraNameInit, float tauInit, bool saveExpInit, std::string expNameInit,
							   int patchSizeBaseInit,int checkSizeBaseInit,Eigen::Vector3f pcbInit, Eigen::Vector4f qcbInit,
								 int numberFeaturesPerPartColInit, int numberFeaturesPerPartRowInit);

	bool initialize(cv::Mat& image, nav_msgs::Odometry imageOdom, ros::Time t);

	void markerOdomCB(const nav_msgs::Odometry::ConstPtr& msg);

	void odomCB(const nav_msgs::Odometry::ConstPtr& msg);

	void roiCB(const icl_multiple_stationary::Roi::ConstPtr& msg);

	void imageCB(const sensor_msgs::Image::ConstPtr& msg);

	void chessboardCB(const std_msgs::Time::ConstPtr& msg);

	void match(cv::Mat& image, float dt, Eigen::Vector3f vc, Eigen::Vector3f wc, ros::Time t, Eigen::Vector3f ppcHat, Eigen::Vector4f qpcHat, Eigen::Vector3f pkc, Eigen::Vector4f qkc);

	void findPoints(cv::Mat& image, std::vector<cv::Point2f>& kPts, std::vector<cv::Point2f>& pPts, std::vector<cv::Point2f>& cPts, cv::Mat& G);

	void update(cv::Mat& image, std::vector<cv::Point2f>& kPts, std::vector<cv::Point2f>& cPts, Eigen::Vector3f vc,
		          Eigen::Vector3f wc, ros::Time t, float dt, Eigen::Vector3f pkc, Eigen::Vector4f qkc);
};

#endif
