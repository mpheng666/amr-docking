#ifndef _AMR_DOCKING_TARGET_LOCALIZATION_HPP_
#define _AMR_DOCKING_TARGET_LOCALIZATION_HPP_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

namespace target_localization
{
    using VectorPC = std::vector<sensor_msgs::PointCloud2>;

    class TargetLocalization
    {
        public:
            TargetLocalization();
            ~TargetLocalization();
            void start();
            VectorPC getClusters(sensor_msgs::PointCloud2);
            void runRANSAC();
            void getTargetLocation();

        private:
            void loadParams();

            ros::Subscriber scan_sub_;
            ros::Publisher filterd_cloud_pub_;
            ros::Publisher clustered_cloud_pub_;

            void scanCb(const sensor_msgs::LaserScan::ConstPtr& msg);
            
    };
}

#endif