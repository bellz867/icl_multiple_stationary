#include <ros/ros.h>

#include <mocap_odom_estimator.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mocap_odom_main_node");

    MocapOdomEstimator mocapOdomEstimator;

    //ros::AsyncSpinner spinner(4);
    //spinner.start();
    //ros::waitForShutdown();

    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    // ros::spin();
    return 0;
}
