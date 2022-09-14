#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "main_node");
    ros::NodeHandle nh("~");

    ros::spin();
    return 0;
}