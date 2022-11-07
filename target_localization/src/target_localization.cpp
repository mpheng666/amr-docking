#include "target_localization/target_localization.hpp"

namespace target_localization
{
    TargetLocalization::TargetLocalization(ros::NodeHandle& nh)
        : nh_p_(nh)
        , cloud_sub_(nh_p_.subscribe<pcl::PointCloud<pcl::PointXYZ>>(
          "filtered_docking_cloud", 100, &TargetLocalization::CloudCb, this))
        , clusters_marker_pub_(nh_p_.advertise<visualization_msgs::Marker>(
          "clusters_centroid_point", 10))
        , centers_marker_pub_(
          nh_p_.advertise<visualization_msgs::Marker>("targets_centre", 10))
        , perimeter_marker_pub_(nh_p_.advertise<visualization_msgs::Marker>(
          "dock_target_perimeter", 10))
        , dock_target_marker_pub_(
          nh_p_.advertise<visualization_msgs::Marker>("dock_target", 10))
        , dock_target_pose_pub_(
          nh_p_.advertise<geometry_msgs::PoseStamped>("dock_target_pose", 10))
    {
}

TargetLocalization::~TargetLocalization()
{
}

void TargetLocalization::start()
{
  loadParams();
  auto rate = ros::Rate(loop_rate_);
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}

void TargetLocalization::loadParams()
{
  if (!nh_p_.param("four_poles_width", four_poles_width_, four_poles_width_))
  {
    ROS_WARN_STREAM("four_poles_width is not set! Use default " << four_poles_width_);
  }
  if (!nh_p_.param("four_poles_length", four_poles_length_, four_poles_length_))
  {
    ROS_WARN_STREAM("four_poles_length is not set! Use default " << four_poles_length_);
  }
  four_poles_diagonal_ = sqrt(four_poles_width_ * four_poles_width_ + four_poles_length_ * four_poles_length_);
  if (!nh_p_.param("match_tolerance", match_tolerance_, match_tolerance_))
  {
    ROS_WARN_STREAM("match_tolerance is not set! Use default " << match_tolerance_);
  }
  if (!nh_p_.param("cluster_tolerance", cluster_tolerance_, cluster_tolerance_))
  {
    ROS_WARN_STREAM("cluster_tolerance is not set! Use default " << cluster_tolerance_);
  }
  if (!nh_p_.param("min_cluster_size", min_cluster_size_, min_cluster_size_))
  {
    ROS_WARN_STREAM("min_cluster_size_ is not set! Use default " << min_cluster_size_);
  }
  if (!nh_p_.param("max_cluster_size", max_cluster_size_, max_cluster_size_))
  {
    ROS_WARN_STREAM("max_cluster_size_ is not set! Use default " << max_cluster_size_);
  }
}

void TargetLocalization::CloudCb(
const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
{
  if (msg->size())
  {
      auto matched_clusters = getMatchedClustersFromCloud(msg);

      auto centroids = getCentroidsFromMatchedClusters(msg, matched_clusters);

      auto valid_centroids_with_middle =
      getValidCentroidsWithMiddleFromCentroids(centroids, four_poles_diagonal_,
                                               match_tolerance_);

      auto targets = getTargetsFromValidCentroidsWithMiddle(
      valid_centroids_with_middle, match_tolerance_);

      auto target = getTargetPoseFromTargets(
      targets, four_poles_width_, four_poles_length_, match_tolerance_);
  }
}

std::vector<pcl::PointIndices>
TargetLocalization::getMatchedClustersFromCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
  std::vector<pcl::PointIndices> cluster_indices;
  if (cloud->size())
  {
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    // Store searched clusters
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
  }
  return cluster_indices;
}

std::vector<geometry_msgs::Point>
TargetLocalization::getCentroidsFromMatchedClusters(
const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
const std::vector<pcl::PointIndices>& cluster_indices)
{
  std::vector<geometry_msgs::Point> centroids_points;
  if (cluster_indices.size())
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

    visualization_msgs::Marker cluster_pos_marker_msg;
    cluster_pos_marker_msg.header.frame_id = "base_link";
    cluster_pos_marker_msg.header.stamp = ros::Time::now();
    cluster_pos_marker_msg.ns = "cluster";
    cluster_pos_marker_msg.id = 0;
    cluster_pos_marker_msg.type = visualization_msgs::Marker::POINTS;
    cluster_pos_marker_msg.action = visualization_msgs::Marker::ADD;
    cluster_pos_marker_msg.scale.x = 0.05;
    cluster_pos_marker_msg.scale.y = 0.05;
    cluster_pos_marker_msg.color.b = 1.0f;
    cluster_pos_marker_msg.color.a = 1.0;
    cluster_pos_marker_msg.pose.orientation.w = 1.0;

    for (const auto& cluster_index : cluster_indices)
    {
      // Calculate clustered centroids
      pcl::CentroidPoint<pcl::PointXYZ> centroid_cluster;
      for (const auto& idx : cluster_index.indices)
      {
        cloud_cluster->push_back((*cloud)[idx]);  //*
        centroid_cluster.add((*cloud)[idx]);
      }
      pcl::PointXYZ centroid_pos;
      centroid_cluster.get(centroid_pos);

      // Reformat centroid_pos to ROS geometry_msg::Point
      geometry_msgs::Point centroid_point;
      centroid_point.x = centroid_pos.x;
      centroid_point.y = centroid_pos.y;
      centroid_point.z = centroid_pos.z;

      // Store centroids into cloud_centroids
      centroids_points.emplace_back(centroid_point);
      cluster_pos_marker_msg.points.push_back(centroid_point);
    }
    clusters_marker_pub_.publish(cluster_pos_marker_msg);
  }
  return centroids_points;
}

std::vector<std::pair<gPoint, std::pair<gPoint, gPoint>>> TargetLocalization::getValidCentroidsWithMiddleFromCentroids(
    const std::vector<gPoint>& centroids, const double target_diagonal_len, const double target_diagonal_len_tolerance)
{
  std::vector<std::pair<gPoint, std::pair<gPoint, gPoint>>> valid_centroids_with_middle;
  if (centroids.size())
  {
    for (auto i = 0; i < size(centroids); ++i)
    {
      for (auto j = i + 1; j < size(centroids); ++j)
      {
        double delta_x = centroids.at(i).x - centroids.at(j).x;
        double delta_y = centroids.at(i).y - centroids.at(j).y;
        double distance = sqrt(delta_x * delta_x + delta_y * delta_y);

        if (distance <= target_diagonal_len + target_diagonal_len_tolerance &&
            distance >= target_diagonal_len - target_diagonal_len_tolerance)
        {
          gPoint line_middle;
          line_middle.x = (centroids.at(i).x + centroids.at(j).x) / 2;
          line_middle.y = (centroids.at(i).y + centroids.at(j).y) / 2;

          valid_centroids_with_middle.emplace_back(
              std::make_pair(line_middle, std::make_pair(centroids.at(i), centroids.at(j))));
        }
      }
    }
  }
  return valid_centroids_with_middle;
}

std::vector<std::pair<gPoint, std::array<gPoint, 4>>> TargetLocalization::getTargetsFromValidCentroidsWithMiddle(
    const std::vector<std::pair<gPoint, std::pair<gPoint, gPoint>>>& valid_centroids_with_middle,
    const double target_diagonal_len_tolerance)
{
  std::vector<std::pair<gPoint, std::array<gPoint, 4>>> targets;
  if (valid_centroids_with_middle.size())
  {
    visualization_msgs::Marker middle_marker_msg;
    middle_marker_msg.header.frame_id = "base_link";
    middle_marker_msg.ns = "centre";
    middle_marker_msg.id = 1;
    middle_marker_msg.type = visualization_msgs::Marker::POINTS;
    middle_marker_msg.action = visualization_msgs::Marker::ADD;
    middle_marker_msg.scale.x = 0.1;
    middle_marker_msg.scale.y = 0.1;
    middle_marker_msg.color.r = 1.0f;
    middle_marker_msg.color.a = 1.0;
    middle_marker_msg.pose.orientation.w = 1.0;

    // Find two intersect middle points
    for (auto i = 0; i < valid_centroids_with_middle.size(); ++i)
    {
      for (auto j = i + 1; j < valid_centroids_with_middle.size(); ++j)
      {
        const double delta_x = valid_centroids_with_middle.at(i).first.x - valid_centroids_with_middle.at(j).first.x;
        const double delta_y = valid_centroids_with_middle.at(i).first.y - valid_centroids_with_middle.at(j).first.y;
        const auto distance = sqrt(pow(delta_x, 2) + pow(delta_y, 2));

        if (distance <= target_diagonal_len_tolerance)
        {
          std::array<gPoint, 4> target_corners;
          target_corners.at(0) = valid_centroids_with_middle.at(i).second.first;
          target_corners.at(1) = valid_centroids_with_middle.at(j).second.first;
          target_corners.at(2) = valid_centroids_with_middle.at(i).second.second;
          target_corners.at(3) = valid_centroids_with_middle.at(j).second.second;

          middle_marker_msg.points.push_back(valid_centroids_with_middle.at(i).first);

          targets.emplace_back(std::make_pair(valid_centroids_with_middle.at(i).first, target_corners));
        }
      }
    }
    centers_marker_pub_.publish(middle_marker_msg);
  }

  return targets;
}

geometry_msgs::Pose TargetLocalization::getTargetPoseFromTargets(
    const std::vector<std::pair<gPoint, std::array<gPoint, 4>>>& targets, const double target_width,
    const double target_length, const double tolerance)
{
  geometry_msgs::Pose target_pose;
  if (targets.size())
  {
    visualization_msgs::Marker perimeter_marker_msg;
    perimeter_marker_msg.header.frame_id = "base_link";
    perimeter_marker_msg.ns = "perimeter";
    perimeter_marker_msg.id = 3;
    perimeter_marker_msg.type = visualization_msgs::Marker::LINE_STRIP;
    perimeter_marker_msg.action = visualization_msgs::Marker::ADD;
    perimeter_marker_msg.scale.x = 0.01;
    perimeter_marker_msg.scale.y = 0.5;
    perimeter_marker_msg.scale.z = 0.05;
    perimeter_marker_msg.color.r = 1.0f;
    perimeter_marker_msg.color.g = 1.0f;
    perimeter_marker_msg.color.a = 0.5;
    perimeter_marker_msg.pose.orientation.w = 1.0;
    perimeter_marker_msg.lifetime = ros::Duration(0.5);

    visualization_msgs::Marker target_viz_marker_msg;
    target_viz_marker_msg.header.frame_id = "base_link";
    target_viz_marker_msg.ns = "target";
    target_viz_marker_msg.id = 2;
    target_viz_marker_msg.type = visualization_msgs::Marker::ARROW;
    target_viz_marker_msg.action = visualization_msgs::Marker::ADD;
    target_viz_marker_msg.scale.x = 0.5;
    target_viz_marker_msg.scale.y = 0.05;
    target_viz_marker_msg.scale.z = 0.05;
    target_viz_marker_msg.color.r = 1.0f;
    target_viz_marker_msg.color.g = 1.0f;
    target_viz_marker_msg.color.a = 1.0;
    target_viz_marker_msg.lifetime = ros::Duration(0.5);

    geometry_msgs::PoseStamped dock_target_pose_msg;
    dock_target_pose_msg.header.frame_id = "base_link";

    // target_viz_marker_msg.points.push_back(valid_centroids_with_middle.at(i).first);

    // Use first target in targets, TODO: find nearest?
    for (const auto& point : targets.at(0).second)
    {
      perimeter_marker_msg.points.push_back(point);
    }
    // Extra point to close up the rectangle
    perimeter_marker_msg.points.push_back(targets.at(0).second.at(0));

    // Find the orientation of the target with respect to base_link
    for (auto i = 0; i < targets.at(0).second.size() - 1; ++i)
    {
      const double delta_x = targets.at(0).second.at(i).x - targets.at(0).second.at(i + 1).x;
      const double delta_y = targets.at(0).second.at(i).y - targets.at(0).second.at(i + 1).y;
      const auto distance = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
      // Check if it's the length side (same orientation)
      if (distance <= target_length + tolerance && distance >= target_length - tolerance)
      {
        // Add pi to change direction
        const double orientation_euler = atan(delta_y/ delta_x);
        tf2::Quaternion target_orientation_quaternion;
        target_orientation_quaternion.setRPY(0, 0, orientation_euler);
        target_orientation_quaternion.normalize();

        target_pose.position = targets.at(0).first;
        target_pose.orientation = tf2::toMsg(target_orientation_quaternion);

        dock_target_pose_msg.pose = target_pose;

        break;
      }
    }

    // Publish pose for docking
    target_viz_marker_msg.pose = target_pose;

    perimeter_marker_pub_.publish(perimeter_marker_msg);
    dock_target_marker_pub_.publish(target_viz_marker_msg);
    dock_target_pose_pub_.publish(dock_target_pose_msg);
  }

  return target_pose;
}

}  // namespace target_localization