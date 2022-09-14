#ifndef _AMR_DOCKING_TARGET_LOCALIZATION_HPP_
#define _AMR_DOCKING_TARGET_LOCALIZATION_HPP_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace target_localization
{
    using VectorPC = std::vector<sensor_msgs::PointCloud2>;

    class TargetLocalization
    {
        public:
            TargetLocalization();
            ~TargetLocalization();
            void start();
            VectprPC getClusters(sensor_msgs::PointCloud2);
            void runRANSAC();
            void getTargetLocation();

        private:
            void loadParams();
            
    };
}