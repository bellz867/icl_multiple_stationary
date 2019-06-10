#ifndef IMAGERECEIVER_H
#define IMAGERECEIVER_H

#include <ctime>
#include <vector>
#include <mutex>

#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

#include <keyframe.h>

struct ImageReceiver
{
  //ros
  ros::NodeHandle nh;
  ros::Subscriber camInfoSub,odomSub;
  image_transport::ImageTransport it;
  image_transport::Subscriber imageSub,keyframeSub;
  image_transport::Publisher undistortPub;

  //camera parameters
  cv::Mat camMat;//camera intrinsic matrix
  cv::Mat distCoeffs;//camera distortion coefficients
  cv::Mat map1,map2;//undistort maps
  bool gotCamParam;//indicate the camera intrinsic parameters are received
  std::string cameraName;//body name and camera name

  float fq,fp,ft,fn,fd;
  float zmin,zmax;

  bool saveExp;
  std::string expName;

  Keyframe* newKeyframe;
  std::vector<Keyframe*> keyframes;
  int keyInd;
  bool firstKey;

  bool firstOdom;
  bool firstHeight;
  std::deque<nav_msgs::Odometry> odomSync;
  std::mutex odomMutex;
  nav_msgs::Odometry lastKeyOdom,imageOdom;

  // cv::Mat tlmask,trmask,blmask,brmask;
  int imageWidth;
  int imageHeight;
  std::vector<cv::Mat> masks;
  int partitionRows;
  int partitionCols;
  float image_roi_percent;
  int numberFeaturesToFindPerPart;
  int minFeaturesDanger;
  int minFeaturesBad;

  ImageReceiver();

  ~ImageReceiver();

  void odomCB(const nav_msgs::Odometry::ConstPtr& msg);

  void imageCB(const sensor_msgs::Image::ConstPtr& msg);

  void keyframeCB(const sensor_msgs::Image::ConstPtr& msg);

  void camInfoCB(const sensor_msgs::CameraInfo::ConstPtr& camInfoMsg);
};

#endif
