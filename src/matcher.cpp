#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "laser_geometry/laser_geometry.hpp"

#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;

class PatternMatcher : public rclcpp::Node
{
public:
  PatternMatcher() : Node("pattern_matcher")
  {
    // Parameters
    this->declare_parameter("scan_topic", "scan");
    this->declare_parameter("pattern_topic", "pattern");
    this->declare_parameter("clustered_points_topic", "clustered_points");
    this->declare_parameter("pattern_filepath", "");

    scan_topic_ = this->get_parameter("scan_topic").as_string();
    pattern_topic_ = this->get_parameter("pattern_topic").as_string();
    clustered_points_topic_ = this->get_parameter("clustered_points_topic").as_string();
    
    std::string pattern_filepath = this->get_parameter("pattern_filepath").as_string();
    if (pattern_filepath.empty()) {
      pattern_filepath_ = ament_index_cpp::get_package_share_directory("pattern_matcher") + "/pcd/pattern.pcd";
    } else {
      pattern_filepath_ = pattern_filepath;
    }

    // ROS 2 Publishers and Subscribers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, 1,
      std::bind(&PatternMatcher::scan_cb, this, std::placeholders::_1));
    
    pattern_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      pattern_topic_, 100);
    
    clustered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      clustered_points_topic_, 100);

    // Accuracy display publishers
    accuracy_text_pub_ = this->create_publisher<std_msgs::msg::String>(
      "pattern_accuracy", 10);
    
    accuracy_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "accuracy_marker", 10);

    // TF2 setup
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Read pattern cloud from file
    pattern_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pattern_filepath_, *pattern_) == -1)
    {
      RCLCPP_ERROR(this->get_logger(), "Could not read pattern file");
      return;
    }

    // Calculate pattern centroid
    pcl::computeCentroid(*pattern_, pattern_centroid_);

    // PCL Specific
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    vg_.setLeafSize(0.01f, 0.01f, 0.01f);
    tree_ = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();

    ec_.setClusterTolerance(0.1);
    ec_.setMinClusterSize(20);
    ec_.setMaxClusterSize(500);

    icp_.setInputSource(pattern_);
    icp_.setMaxCorrespondenceDistance(0.04);

    // Convert pattern cloud from PCL Cloud to ROS PointCloud2
    pcl::toROSMsg(*pattern_, pattern_msg_);
    pattern_msg_.header.frame_id = "front_of_station";

    // Timer for main loop
    timer_ = this->create_wall_timer(
      166ms, std::bind(&PatternMatcher::timer_callback, this));
  }

private:
  void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    scan_msg_ = *msg;
    scan_msg_came_ = true;
  }

  void timer_callback()
  {
    if (!scan_msg_came_) return;
    scan_msg_came_ = false;

    // Convert sensor_msgs::LaserScan to PCL Cloud
    sensor_msgs::msg::PointCloud2 pointCloud;
    projector_.transformLaserScanToPointCloud(scan_msg_.header.frame_id, scan_msg_, pointCloud, *tf_buffer_);
    pcl::fromROSMsg(pointCloud, cloud_);

    // Distance filter (1 meter)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_distance(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point : cloud_)
    {
      float distance = std::sqrt(point.x * point.x + point.y * point.y);
      if (distance <= 1.0)  // 1 meter threshold
      {
        cloud_filtered_distance->push_back(point);
      }
    }

    // Apply voxel filter
    vg_.setInputCloud(cloud_filtered_distance);
    vg_.filter(*cloud_filtered_);

    // Set filtered cloud as input for KD Tree
    tree_->setInputCloud(cloud_filtered_);

    // Extract clusters
    std::vector<pcl::PointIndices> cluster_indices;
    ec_.setSearchMethod(tree_);
    ec_.setInputCloud(cloud_filtered_);
    ec_.extract(cluster_indices);
    
    pcl::PointCloud<pcl::PointXYZRGB> cloud_reconstructed;
    std::vector<std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointXYZ>> clusters;

    best_fitness_ = 1.0;

    /**
     * For Every Cluster:
     * 1. Give different color
     * 2. Run ICP on it
     * 3. Check if ICP result is the best
     */
    for(auto cluster_indice : cluster_indices)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);

      // Create two PCL clouds from cluster_indice
      // Give different color for every cluster
      for (const auto& indice : cluster_indice.indices)
      {
        pcl::PointXYZ point;
        pcl::PointXYZRGB point_rgb;

        point.x = point_rgb.x = (*cloud_filtered_)[indice].x;
        point.y = point_rgb.y = (*cloud_filtered_)[indice].y;
        point.z = point_rgb.z = (*cloud_filtered_)[indice].z;

        point_rgb.r = *(colors_ + (color_index_*3));
        point_rgb.g = *(colors_ + (color_index_*3) + 1);
        point_rgb.b = *(colors_ + (color_index_*3) + 2);

        cloud_cluster->push_back(point);
        cloud_cluster_rgb->push_back(point_rgb);
      }

      color_index_ += 1;
      if(color_index_ >= color_count_) color_index_ = 0;

      cloud_cluster_rgb->width = cloud_cluster_rgb->size();
      cloud_cluster_rgb->height = 1;
      cloud_cluster_rgb->is_dense = true;
      
      cloud_reconstructed += *cloud_cluster_rgb;

      sensor_msgs::msg::PointCloud2 cloud_reconstructed_msg;
      pcl::toROSMsg(cloud_reconstructed, cloud_reconstructed_msg);
      cloud_reconstructed_msg.header.stamp = this->now();
      cloud_reconstructed_msg.header.frame_id = scan_msg_.header.frame_id;
      clustered_pub_->publish(cloud_reconstructed_msg);
      
      // Compute cluster centroid
      pcl::PointXYZ centroid;
      pcl::computeCentroid(*cloud_cluster, centroid);

      // Calculate initial alignment
      Eigen::Affine3f affine_transform;
      pcl::getTransformation(centroid.x - pattern_centroid_.x, centroid.y - pattern_centroid_.y, 0, 0, 0, 0, affine_transform);
      Eigen::Matrix4f initial_aligment = affine_transform.matrix();

      // Run ICP
      icp_.setInputTarget(cloud_cluster);
      icp_.align(*icp_result_, initial_aligment);

      // Check if ICP converged
      if(!icp_.hasConverged()) continue;            

      // Check if this convergence is better
      if(best_fitness_ < icp_.getFitnessScore()) continue;

      // Store best fitness and transformation values
      best_fitness_ = icp_.getFitnessScore();
      pcl::getTranslationAndEulerAngles(Eigen::Affine3f(icp_.getFinalTransformation()), tx_, ty_, tz_, roll_, pitch_, yaw_);
    }

    // Check if a good match were found
    if(best_fitness_ >= 1.0) return;

    // Pattern Match Transformation
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = scan_msg_.header.frame_id;
    transform.child_frame_id = "front_of_station";
    transform.transform.translation.x = tx_;
    transform.transform.translation.y = ty_;
    transform.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    // Publish TF and Pattern PointCloud2
    tf_broadcaster_->sendTransform(transform);
    pattern_msg_.header.stamp = this->now();
    pattern_pub_->publish(pattern_msg_);

    publish_accuracy_info(tx_, ty_, yaw_, best_fitness_);
  }

  void navigation_feedback_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Navigation feedback: Distance remaining = %.2f", feedback->distance_remaining);
  }

  void publish_accuracy_info(float tx, float ty, float yaw, double fitness)
  {
    // Publish text accuracy info
    auto accuracy_msg = std_msgs::msg::String();
    accuracy_msg.data = "Pattern Match\nFitness: " + std::to_string(fitness).substr(0, 6) + 
                       "\nPos: (" + std::to_string(tx).substr(0, 5) + ", " + 
                       std::to_string(ty).substr(0, 5) + ")\nYaw: " + 
                       std::to_string(yaw * 180.0 / M_PI).substr(0, 5) + "°";
    accuracy_text_pub_->publish(accuracy_msg);

    // Publish 3D marker for accuracy visualization
    auto marker_msg = visualization_msgs::msg::Marker();
    marker_msg.header.frame_id = scan_msg_.header.frame_id;
    marker_msg.header.stamp = this->now();
    marker_msg.ns = "pattern_accuracy";
    marker_msg.id = 0;
    marker_msg.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker_msg.action = visualization_msgs::msg::Marker::ADD;
    
    // Position the text above the pattern
    marker_msg.pose.position.x = tx;
    marker_msg.pose.position.y = ty;
    marker_msg.pose.position.z = 0.5;  // 50cm above the pattern
    marker_msg.pose.orientation.w = 1.0;
    
    // Text content
    marker_msg.text = "Fitness: " + std::to_string(fitness).substr(0, 6);
    
    // Text properties
    marker_msg.scale.z = 0.1;  // Text size
    marker_msg.color.r = 1.0;
    marker_msg.color.g = 1.0;
    marker_msg.color.b = 1.0;
    marker_msg.color.a = 0.8;
    
    // Color based on fitness (green = good, red = bad)
    if (fitness < 0.1) {
      marker_msg.color.r = 0.0;
      marker_msg.color.g = 1.0;
      marker_msg.color.b = 0.0;
    } else if (fitness < 0.3) {
      marker_msg.color.r = 1.0;
      marker_msg.color.g = 1.0;
      marker_msg.color.b = 0.0;
    } else {
      marker_msg.color.r = 1.0;
      marker_msg.color.g = 0.0;
      marker_msg.color.b = 0.0;
    }
    
    accuracy_marker_pub_->publish(marker_msg);
  }

  // Parameters
  std::string scan_topic_;
  std::string pattern_topic_;
  std::string clustered_points_topic_;
  std::string pattern_filepath_;

  // ROS 2 specific members
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pattern_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clustered_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // PCL specific members
  pcl::PointCloud<pcl::PointXYZ>::Ptr pattern_;
  pcl::PointXYZ pattern_centroid_;
  pcl::VoxelGrid<pcl::PointXYZ> vg_;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec_;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
  pcl::PointCloud<pcl::PointXYZ> cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_{new pcl::PointCloud<pcl::PointXYZ>};
  pcl::PointCloud<pcl::PointXYZ>::Ptr icp_result_{new pcl::PointCloud<pcl::PointXYZ>};
  sensor_msgs::msg::PointCloud2 pattern_msg_;

  // State variables
  bool scan_msg_came_{false};
  sensor_msgs::msg::LaserScan scan_msg_;
  laser_geometry::LaserProjection projector_;
  double best_fitness_{1.0};
  float tx_{0.0}, ty_{0.0}, tz_{0.0}, roll_{0.0}, pitch_{0.0}, yaw_{0.0};

  // Colors
  int color_index_{0};
  int color_count_{6};
  uint8_t colors_[18] = {255, 0, 0,    // Red
                         0, 255, 0,    // Green
                         0, 0, 255,    // Blue
                         255, 255, 0,  // Yellow
                         255, 0, 255,  // Magenta
                         0, 255, 255}; // Cyan

  // Accuracy display publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr accuracy_text_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr accuracy_marker_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PatternMatcher>());
  rclcpp::shutdown();
  return 0;
}
