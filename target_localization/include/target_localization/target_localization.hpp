#ifndef _AMR_DOCKING_TARGET_LOCALIZATION_HPP_
#define _AMR_DOCKING_TARGET_LOCALIZATION_HPP_

#include <pcl/ModelCoefficients.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include "pcl_ros/point_cloud.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
namespace target_localization {
    using gPoint = geometry_msgs::Point;

    class TargetLocalization {
    public:
        TargetLocalization(ros::NodeHandle& nh);
        ~TargetLocalization();
        void start();

    private:
        ros::NodeHandle nh_p_;
        ros::Subscriber cloud_sub_;
        ros::Publisher clustered_cloud_pub_;
        ros::Publisher clusters_marker_pub_;
        ros::Publisher centers_marker_pub_;
        ros::Publisher dock_target_marker_pub_;
        ros::Publisher perimeter_marker_pub_;

        static constexpr double loop_rate_{20.0};

        double four_poles_width_{0.8};
        double four_poles_length_{0.48};
        double four_poles_diagonal_;
        double match_tolerance_{0.075};
        double cluster_tolerance_{0.05};
        double min_cluster_size_{5};
        double max_cluster_size_{30};

        void loadParams();

        /**
         * @brief Cloud callback, main processing is running here
         *
         * @param msg
         */
        void CloudCb(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg);

        /**
         * @brief Get the Matched Clusters based on the parameters From Cloud
         * object
         *
         * @param cloud
         * @return std::vector<pcl::PointIndices>
         */
        std::vector<pcl::PointIndices> getMatchedClustersFromCloud(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);

        /**
         * @brief Get the Centroids From Matched Clusters object
         *
         * @param cloud
         * @param cluster_indices
         * @return std::vector<gPoint>
         */
        std::vector<gPoint> getCentroidsFromMatchedClusters(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
        const std::vector<pcl::PointIndices>& cluster_indices);

        /**
         * @brief Get the Valid Centroids With Middle From Centroids object
         *
         * @param centroids
         * @param target_diagonal_len
         * @param target_diagonal_len_tolerance
         * @return std::vector<std::pair<gPoint, std::pair<gPoint, gPoint>>>
         */
        std::vector<std::pair<gPoint, std::pair<gPoint, gPoint>>>
        getValidCentroidsWithMiddleFromCentroids(
        const std::vector<gPoint>& centroids,
        const double target_diagonal_len,
        const double target_diagonal_len_tolerance);

        /**
         * @brief Get the Targets From Valid Centroids With Middle object
         *
         * @param valid_centroids_with_middle
         * @param target_diagonal_len_tolerance
         * @return std::vector<std::pair<gPoint, std::array<gPoint, 4>>>
         */
        std::vector<std::pair<gPoint, std::array<gPoint, 4>>>
        getTargetsFromValidCentroidsWithMiddle(
        const std::vector<std::pair<gPoint, std::pair<gPoint, gPoint>>>&
        valid_centroids_with_middle,
        const double target_diagonal_len_tolerance);

        /**
         * @brief Get the Target Pose From Targets object
         *
         * @param targets
         * @param target_width
         * @param target_length
         * @param tolerance
         * @return geometry_msgs::Pose
         */
        geometry_msgs::Pose getTargetPoseFromTargets(
        const std::vector<std::pair<gPoint, std::array<gPoint, 4>>>& targets,
        const double target_width,
        const double target_length,
        const double tolerance);
    };
} // namespace target_localization

#endif