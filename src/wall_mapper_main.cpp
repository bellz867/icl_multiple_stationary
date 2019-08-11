#include <ros/ros.h>

#include <wall_mapper.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "icl_wall_mapper_node");

    WallMapper wallMapper;

    //ros::AsyncSpinner spinner(4);
    //spinner.start();
    //ros::waitForShutdown();

    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    // ros::spin();
    return 0;
}
