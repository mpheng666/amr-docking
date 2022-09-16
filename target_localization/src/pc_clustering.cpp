// #ifndef _AMR_DOCKING_CLUSTERING_HPP_
// #define _AMR_DOCKING_CLUSTERING_HPP_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

namespace pc_clustering_ns
{
    class PCClustering
    {
        public:
            PCClustering(ros::NodeHandle& nh)
            :
            nh_p_(nh),
            cloud_sub_(nh_p_.subscribe("cloud_merged", 100, &PCClustering::CloudCb, this)),
            clustered_cloud_pub_(nh_p_.advertise<sensor_msgs::PointCloud2>("clustered_cloud", 100))
            {

            }

            ~PCClustering()
            {

            }

            void start()
            {
                auto r = ros::Rate(20.0);

                while(ros::ok())
                {

                    ros::spinOnce();
                    r.sleep();
                }
            }

        private:
            ros::NodeHandle nh_p_;
            ros::Subscriber cloud_sub_;
            ros::Publisher clustered_cloud_pub_;

            void CloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                sensor_msgs::PointCloud2 output;

                pcl::fromROSMsg(*msg, *cloud);

                // Creating the KdTree object for the search method of the extraction
                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
                tree->setInputCloud(cloud);
                
                // Store seartched clusters
                std::vector<pcl::PointIndices> cluster_indices;
                pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
                ec.setClusterTolerance (0.05); // 2cm
                ec.setMinClusterSize (5);
                ec.setMaxClusterSize (30);
                ec.setSearchMethod (tree);
                ec.setInputCloud (cloud);
                ec.extract (cluster_indices);
                
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

                int j = 0;
                for (auto it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
                {
                    for (const auto& idx : it->indices)
                    {
                        cloud_cluster->push_back ((*cloud)[idx]); //*
                    }
                    cloud_cluster->width = cloud_cluster->size ();
                    cloud_cluster->height = 1;
                    cloud_cluster->is_dense = true;
                
                    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
                    j++;
                }

                
                pcl::toROSMsg(*cloud_cluster, output);
                output.header.frame_id = msg->header.frame_id;
                clustered_cloud_pub_.publish(output);
            }
        
    };
}

// #endif

int main(int argc, char** argv)
{
    ros::init(argc, argv, "clustering_node");
    ros::NodeHandle nh("~");
    pc_clustering_ns::PCClustering pc_c(nh);
    pc_c.start();

    ros::spin();
    return 0;
}
