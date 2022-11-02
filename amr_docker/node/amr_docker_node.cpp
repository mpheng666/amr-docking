#include "amr_docker/amr_docker.hpp"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "amr_docker_node");
    ros::NodeHandle nh("~");

    amr_docker::AMRDocker amr_docker(nh);
    amr_docker.start();

    ros::spin();
    return 0;
}