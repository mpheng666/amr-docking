#ifndef AMR_DOCKING_AMR_DOCKER_HPP_
#define AMR_DOCKING_AMR_DOCKER_HPP_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

namespace amr_docker
{
    class AMRDocker {
        public:
            AMRDocker(ros::NodeHandle& nh);
            void start();

        private:
            ros::Subscriber target_pose_input_sub_;
            ros::Publisher twist_output_pub_;

            void loadParams();
    };
}

#endif