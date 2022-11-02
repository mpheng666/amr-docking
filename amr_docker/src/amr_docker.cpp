#include "amr_docker/amr_docker.hpp"

namespace amr_docker
{
    AMRDocker::AMRDocker(ros::NodeHandle& nh)
    {

    }

    void AMRDocker::start()
    {
        loadParams();
        while (ros::ok()) {
            ros::Rate(ros::Duration(0.05)).sleep();
            ros::spinOnce();
        }
    }

    void AMRDocker::loadParams()
    {

    }
}