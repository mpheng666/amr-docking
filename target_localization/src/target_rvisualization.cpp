#include "target_localization/target_rvisualization.hpp"

namespace target_localization {
    TargetRvisualization::TargetRvisualization(ros::NodeHandle& nh,
                                               const std::string& target_frame)
        : nh_p_(nh)
        , target_frame_(target_frame)
        , clusters_marker_pub_(nh_p_.advertise<visualization_msgs::Marker>(
          "clusters_centroid_point", 10))
        , centers_marker_pub_(
          nh_p_.advertise<visualization_msgs::Marker>("targets_centre", 10))
        , perimeter_marker_pub_(nh_p_.advertise<visualization_msgs::Marker>(
          "dock_target_perimeter", 10))
        , dock_target_marker_pub_(
          nh_p_.advertise<visualization_msgs::Marker>("dock_target", 10))
    {
    }

    void TargetRvisualization::loadParams() {}

    void TargetRvisualization::visualizePoints(
    const std::vector<geometry_msgs::Point>& points,
    const std_msgs::ColorRGBA& colour,
    const double point_size)
    {
        visualization_msgs::Marker viz_marker_msg;

        viz_marker_msg.header.frame_id = target_frame_;
        viz_marker_msg.header.stamp = ros::Time::now();
        viz_marker_msg.id = 99;
        viz_marker_msg.type = visualization_msgs::Marker::POINTS;
        viz_marker_msg.action = visualization_msgs::Marker::ADD;
        viz_marker_msg.scale.x = point_size;
        viz_marker_msg.scale.y = point_size;
        viz_marker_msg.color = colour;
        viz_marker_msg.pose.orientation.w = 1.0;

        viz_marker_msg.points = points;

        viz_pub_.publish(viz_marker_msg);
    }

    void TargetRvisualization::visualizePointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
    {
    }

    void TargetRvisualization::visualizePose(
    const geometry_msgs::Pose::ConstPtr& msg)
    {
    }

    void TargetRvisualization::visualizeRectangleLineStrip(
    const std::vector<geometry_msgs::Point>& points,
    const std_msgs::ColorRGBA& colour,
    const int32_t id)
    {
    }
} // namespace target_localization