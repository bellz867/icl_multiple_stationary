#ifndef WALLMAPPER_H
#define WALLMAPPER_H

#include <iostream>
#include <fstream>
#include <ctime>
#include <vector>
#include <queue>
#include <mutex>

#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

#include <icl_multiple_stationary/Wall.h>

#include <keyframe_planes.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

struct WallMapper
{
    //ros
    ros::NodeHandle nh;
    ros::Subscriber wallSub,odomSub;
    ros::Publisher pointCloudPub,pointCloudTruePub;
    image_transport::ImageTransport it;
    // image_transport::Publisher wallPub;
    int width;// width in decimeters
    int height;// height in decimeters
    float minarea;
    float maxarea;
    float minheight;
    float maxheight;

    cv::Point2i camCenter;
    cv::Point2i camEnd;
    bool firstOdom;

    cv::Point2i center;
    std::string cameraName;

    PointCloudRGB cloud_true;

    Eigen::Vector3f pcw;
    Eigen::Vector4f qcw;

    std::mutex odomMutex,wallMutex;

    // std::vector<std::vector<cv::Point2f>> wallPts;
    // std::vector<std::vector<geometry_msgs::Point32>> keyClouds;
    // std::vector<std::vector<uint8_t>> keyColors;
    // std::vector<uint16_t> wallKeys;
    // std::vector<std::vector<std::vector<geometry_msgs::Point32>>> keyClouds;
    // std::vector<std::vector<std::vector<uint8_t>>> keyColors;
    // std::vector<std::vector<uint16_t>> wallKeys;
    std::vector<KeyframePlanes*> keyframePlanes;

    WallMapper();

    //wall sub callback
    void wallCB(const icl_multiple_stationary::Wall::ConstPtr& msg);

    //odom callback
    void odomCB(const nav_msgs::Odometry::ConstPtr& msg);
};

#endif
