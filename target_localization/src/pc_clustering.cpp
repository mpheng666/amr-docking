// #ifndef _AMR_DOCKING_CLUSTERING_HPP_
// #define _AMR_DOCKING_CLUSTERING_HPP_

#include <geometry_msgs/Point.h>
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
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

namespace pc_clustering_ns {
class PCClustering {
public:
    PCClustering(ros::NodeHandle& nh)
        : nh_p_(nh),
          cloud_sub_(nh_p_.subscribe(
                  "filtered_docking_cloud", 100, &PCClustering::CloudCb, this)),
          clustered_cloud_pub_(nh_p_.advertise<sensor_msgs::PointCloud2>(
                  "clustered_cloud", 100)),
          clusters_marker_pub_(nh_p_.advertise<visualization_msgs::Marker>(
                  "clusters_pos", 10)),
          centers_marker_pub_(nh_p_.advertise<visualization_msgs::Marker>(
                  "rec_centres", 10)),
          dock_target_marker_pub_(nh_p_.advertise<visualization_msgs::Marker>(
                  "dock_target", 10)) {}

    ~PCClustering() {}

    void start() {
        auto r = ros::Rate(20.0);

        while (ros::ok()) {
            ros::spinOnce();
            r.sleep();
        }
    }

private:
    ros::NodeHandle nh_p_;
    ros::Subscriber cloud_sub_;
    ros::Publisher clustered_cloud_pub_;
    ros::Publisher clusters_marker_pub_;
    ros::Publisher centers_marker_pub_;
    ros::Publisher dock_target_marker_pub_;

    double cage_diagonal_ = sqrt(0.8 * 0.8 + 0.48 * 0.48);
    double tolerance_{0.05};

    void CloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
                new pcl::PointCloud<pcl::PointXYZ>);
        sensor_msgs::PointCloud2 output;

        pcl::fromROSMsg(*msg, *cloud);

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
                new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        // Store searched clusters
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.05);  // 2cm
        ec.setMinClusterSize(5);
        ec.setMaxClusterSize(30);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
                new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<pcl::PointXYZ> cloud_centroids;

        int j = 0;
        for (auto it = cluster_indices.begin(); it != cluster_indices.end();
             ++it) {
            pcl::CentroidPoint<pcl::PointXYZ> centroid_cluster;
            for (const auto& idx : it->indices) {
                cloud_cluster->push_back((*cloud)[idx]);  //*
                centroid_cluster.add((*cloud)[idx]);
            }
            pcl::PointXYZ centroid_pos;
            centroid_cluster.get(centroid_pos);
            // ROS_INFO_STREAM("cloud centroid: " << centroid_pos.x << "\n");
            cloud_centroids.emplace_back(centroid_pos);

            cloud_cluster->width = cloud_cluster->size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            // ROS_INFO_STREAM("PointCloud representing the Cluster: " <<
            // cloud_cluster->size () << " data points." << 'n');
            j++;
        }

        visualization_msgs::Marker cluster_pos_marker_msg;
        cluster_pos_marker_msg.header = msg->header;
        cluster_pos_marker_msg.ns = "cluster";
        cluster_pos_marker_msg.id = 0;
        cluster_pos_marker_msg.type = visualization_msgs::Marker::POINTS;
        cluster_pos_marker_msg.action = visualization_msgs::Marker::ADD;
        cluster_pos_marker_msg.scale.x = 0.05;
        cluster_pos_marker_msg.scale.y = 0.05;
        cluster_pos_marker_msg.color.b = 1.0f;
        cluster_pos_marker_msg.color.a = 1.0;
        cluster_pos_marker_msg.pose.orientation.w = 1.0;

        for (const auto& centroid : cloud_centroids) {
            geometry_msgs::Point point;
            point.x = centroid.x;
            point.y = centroid.y;
            point.z = centroid.z;
            cluster_pos_marker_msg.points.push_back(point);
        }

        auto valid_r = findTarget(cloud_centroids, cage_diagonal_, tolerance_);

        clusters_marker_pub_.publish(cluster_pos_marker_msg);

        pcl::toROSMsg(*cloud_cluster, output);
        output.header.frame_id = msg->header.frame_id;
        clustered_cloud_pub_.publish(output);
    }

    geometry_msgs::Point findTarget(const std::vector<pcl::PointXYZ>& points,
                                    const double& diagonal_len,
                                    const double& tolerance) {
        geometry_msgs::Point valid_centre;

        visualization_msgs::Marker centre_marker_msg;
        centre_marker_msg.header.frame_id = "base_link";
        centre_marker_msg.ns = "centre";
        centre_marker_msg.id = 1;
        centre_marker_msg.type = visualization_msgs::Marker::POINTS;
        centre_marker_msg.action = visualization_msgs::Marker::ADD;
        centre_marker_msg.scale.x = 0.1;
        centre_marker_msg.scale.y = 0.1;
        centre_marker_msg.color.r = 1.0f;
        centre_marker_msg.color.a = 1.0;
        centre_marker_msg.pose.orientation.w = 1.0;

        visualization_msgs::Marker target_marker_msg;
        target_marker_msg.header.frame_id = "base_link";
        target_marker_msg.ns = "target";
        target_marker_msg.id = 2;
        target_marker_msg.type = visualization_msgs::Marker::CUBE;
        target_marker_msg.action = visualization_msgs::Marker::ADD;
        target_marker_msg.scale.x = 0.5;
        target_marker_msg.scale.y = 0.5;
        target_marker_msg.scale.z = 0.05;
        target_marker_msg.color.r = 1.0f;
        target_marker_msg.color.g = 1.0f;
        target_marker_msg.color.a = 0.5;
        target_marker_msg.pose.orientation.w = 1.0;
        target_marker_msg.lifetime = ros::Duration(0.5);

        std::map<geometry_msgs::Point, std::pair<int, int>> centres_and_idxs;
        for (auto i = 0; i < size(points); ++i) {
            for (auto j = i + 1; j < size(points); ++j) {
                double delta_x = points.at(i).x - points.at(j).x;
                double delta_y = points.at(i).y - points.at(j).y;
                double distance = sqrt(delta_x * delta_x + delta_y * delta_y);
                if (distance <= diagonal_len + tolerance &&
                    distance >= diagonal_len - tolerance) {
                    geometry_msgs::Point line_centre;
                    std::pair<int, int> two_points = std::make_pair(i, j);
                    line_centre.x = (points.at(i).x + points.at(j).x) / 2;
                    line_centre.y = (points.at(i).y + points.at(j).y) / 2;
                    // ROS_INFO_STREAM("centre x: " << line_centre.x << " centre
                    // y: " << line_centre.y <<'\n');
                    centre_marker_msg.points.push_back(line_centre);
                    // centres_and_idxs.insert({line_centre, two_points});
                }
            }
        }

        centers_marker_pub_.publish(centre_marker_msg);

        if (target_marker_msg.points.empty() &&
            centre_marker_msg.points.size() >= 2) {
            for (auto i = 0; i < size(centre_marker_msg.points); ++i) {
                for (auto j = i; j < size(centre_marker_msg.points); ++j) {
                    auto delta_x = centre_marker_msg.points.at(i).x -
                                   centre_marker_msg.points.at(j).x;
                    auto delta_y = centre_marker_msg.points.at(i).y -
                                   centre_marker_msg.points.at(j).y;
                    auto distance = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
                    if (distance <= tolerance) {
                        valid_centre = centre_marker_msg.points.at(i);
                        target_marker_msg.pose.position = valid_centre;
                        dock_target_marker_pub_.publish(target_marker_msg);
                    }
                }
            }
        }

        return valid_centre;
    }
};
}  // namespace pc_clustering_ns

// #endif

int main(int argc, char** argv) {
    ros::init(argc, argv, "clustering_node");
    ros::NodeHandle nh("~");
    pc_clustering_ns::PCClustering pc_c(nh);
    pc_c.start();

    ros::spin();
    return 0;
}
