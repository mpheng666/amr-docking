#ifndef AMR_DOCKING_TARGET_RVISUALIZATION_HPP_
#define AMR_DOCKING_TARGET_RVISUALIZATION_HPP_

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include "pcl_ros/point_cloud.h"

namespace target_localization {
    class TargetRvisualization {
    public:
        TargetRvisualization(ros::NodeHandle& nh,
                             const std::string& target_frame);

    private:
        ros::NodeHandle nh_p_;
        ros::Publisher clusters_marker_pub_;
        ros::Publisher centers_marker_pub_;
        ros::Publisher dock_target_marker_pub_;
        ros::Publisher perimeter_marker_pub_;

        ros::Publisher pc_pub_;
        ros::Publisher viz_pub_;

        std::string target_frame_{"base_link"};

        void loadParams();
        void visualizePoints(const std::vector<geometry_msgs::Point>& points,
                             const std_msgs::ColorRGBA& colour,
                             const double point_size);
        void visualizePointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg);
        void visualizePose(const geometry_msgs::Pose::ConstPtr& msg);
        void visualizeRectangleLineStrip(
        const std::vector<geometry_msgs::Point>& points,
        const std_msgs::ColorRGBA& colour,
        const int32_t id);
    };
} // namespace target_localization

#endif