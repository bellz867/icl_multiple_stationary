#include <ros/ros.h>

#include <image_receiver.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "estimator_node");

    ImageReceiver imageReceiver;

    // ros::AsyncSpinner spinner(4);
    // spinner.start();
    // ros::waitForShutdown();

    ros::MultiThreadedSpinner spinner(8);
    spinner.spin();

    // ros::spin();
    return 0;
}
