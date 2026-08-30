#include <mutex>
#include <memory>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace fast_lio_localization
{

class FastLIOLocalization : public rclcpp::Node
{
public:
  explicit FastLIOLocalization(const rclcpp::NodeOptions & options)
  : Node("fast_lio_localization", options)
  {
    // Declare Parameters
    this->declare_parameter("map_voxel_size", 0.1);
    this->declare_parameter("scan_voxel_size", 0.05);
    this->declare_parameter("freq_localization", 0.5);
    this->declare_parameter("localization_threshold", 0.8);
    this->declare_parameter("fov", 6.28319);
    this->declare_parameter("fov_far", 300.0);
    this->declare_parameter("max_jump_distance", 1.5);
    this->declare_parameter("pcd_map_path", "");

    // Publishers
    pub_pc_in_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cur_scan_in_map", 10);
    pub_submap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/submap", 10);
    pub_map_to_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/map_to_odom", 10);

    // Load Global Map
    initialize_global_map();

    // Subscribers
    sub_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered", 10, std::bind(&FastLIOLocalization::cb_save_cur_scan, this, std::placeholders::_1));
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry", 10, std::bind(&FastLIOLocalization::cb_save_cur_odom, this, std::placeholders::_1));
    sub_initial_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10, std::bind(&FastLIOLocalization::cb_initialize_pose, this, std::placeholders::_1));

    // Timers
    double freq = this->get_parameter("freq_localization").as_double();
    timer_localization_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / freq),
        std::bind(&FastLIOLocalization::localisation_timer_callback, this));
  }

private:
  pcl::PointCloud<pcl::PointXYZI>::Ptr global_map_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cur_scan_;
  nav_msgs::msg::Odometry::SharedPtr cur_odom_;
  
  Eigen::Matrix4f T_map_to_odom_ = Eigen::Matrix4f::Identity();
  bool initialized_ = false;
  bool force_initial_pose_ = true;
  std::mutex data_mutex_;

  // Publishers and Subscribers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc_in_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_submap_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_map_to_odom_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_scan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initial_pose_;
  rclcpp::TimerBase::SharedPtr timer_localization_;

  void initialize_global_map()
  {
    std::string map_path = this->get_parameter("pcd_map_path").as_string();
    global_map_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(map_path, *global_map_) == -1) 
    {
      RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD map file at: %s", map_path.c_str());
      return;
    }

    double map_voxel_size = this->get_parameter("map_voxel_size").as_double();
    global_map_ = voxel_down_sample(global_map_, map_voxel_size);
    RCLCPP_INFO(this->get_logger(), "Global map received and downsampled to %lu points.", global_map_->points.size());
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_down_sample(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, double voxel_size)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
    if (!cloud || cloud->empty()) return cloud_filtered;

    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(voxel_size, voxel_size, voxel_size);
    sor.filter(*cloud_filtered);
    return cloud_filtered;
  }

  Eigen::Matrix4f pose_to_mat(const geometry_msgs::msg::Pose& pose)
  {
    Eigen::Quaternionf q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
    mat.block<3, 3>(0, 0) = q.toRotationMatrix();
    mat.block<3, 1>(0, 3) = Eigen::Vector3f(pose.position.x, pose.position.y, pose.position.z);
    return mat;
  }

  void cb_save_cur_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    cur_odom_ = msg;
  }

  void cb_save_cur_scan(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    cur_scan_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cur_scan_);
    
    // Republish just for debugging like python script did
    pub_pc_in_map_->publish(*msg);
  }

  void cb_initialize_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    Eigen::Matrix4f initial_pose = pose_to_mat(msg->pose.pose);
    initialized_ = true;
    force_initial_pose_ = true;
    RCLCPP_INFO(this->get_logger(), "Initial pose received.");

    if (cur_scan_ != nullptr) 
    {
      global_localization(initial_pose);
    } else 
    {
      RCLCPP_WARN(this->get_logger(), "Scan is null during initialization.");
    }
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr crop_global_map_in_FOV(const Eigen::Matrix4f& pose_estimation)
  {
    if (cur_odom_ == nullptr) 
    {
      return pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    }

    Eigen::Matrix4f T_odom_to_base = pose_to_mat(cur_odom_->pose.pose);
    Eigen::Matrix4f T_map_to_base = pose_estimation * T_odom_to_base;
    Eigen::Matrix4f T_base_to_map = T_map_to_base.inverse();

    // Transform map to base_link to easily check FOV
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_in_base(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*global_map_, *map_in_base, T_base_to_map);

    double fov = this->get_parameter("fov").as_double();
    double fov_far = this->get_parameter("fov_far").as_double();

    pcl::PointCloud<pcl::PointXYZI>::Ptr cropped_map(new pcl::PointCloud<pcl::PointXYZI>());
    
    // Note: Extract points from original map (global_map_) using indices found in base_link frame
    for (size_t i = 0; i < map_in_base->points.size(); ++i) 
    {
      const auto& pt = map_in_base->points[i];
      double angle = std::atan2(pt.y, pt.x);
      
      bool in_fov = false;
      if (fov > 3.14) 
      {
        if (pt.x < fov_far && std::abs(angle) < fov / 2.0) 
          in_fov = true;
      } 
      else 
      {
        if (pt.x > 0 && pt.x < fov_far && std::abs(angle) < fov / 2.0) 
          in_fov = true;
      }

      if (in_fov) 
      {
          cropped_map->points.push_back(global_map_->points[i]);
      }
    }

    // Publish submap for RViz
    sensor_msgs::msg::PointCloud2 submap_msg;
    pcl::toROSMsg(*cropped_map, submap_msg);
    submap_msg.header = cur_odom_->header;
    submap_msg.header.frame_id = "map";
    pub_submap_->publish(submap_msg);

    return cropped_map;
  }

  std::pair<Eigen::Matrix4f, double> registration_at_scale(
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan,
    pcl::PointCloud<pcl::PointXYZI>::Ptr map,
    Eigen::Matrix4f initial, double scale)
  {
    double scan_vs = this->get_parameter("scan_voxel_size").as_double() * scale;
    double map_vs = this->get_parameter("map_voxel_size").as_double() * scale;
    double max_corr_dist = 1.0 * scale;

    auto scan_ds = voxel_down_sample(scan, scan_vs);
    auto map_ds = voxel_down_sample(map, map_vs);

    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    icp.setInputSource(scan_ds);
    icp.setInputTarget(map_ds);
    icp.setMaxCorrespondenceDistance(max_corr_dist);
    icp.setMaximumIterations(20);

    pcl::PointCloud<pcl::PointXYZI> final_cloud;
    icp.align(final_cloud, initial);

    Eigen::Matrix4f result = icp.getFinalTransformation();
    
    // Calculate overlapping ratio (fitness equivalent in Open3D)
    double fitness = 0.0;
    if (icp.hasConverged() && !final_cloud.empty()) 
    {
      pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
      kdtree.setInputCloud(map_ds);
      
      int inliers = 0;
      std::vector<int> idx(1);
      std::vector<float> sq_dist(1);
      
      for (const auto& pt : final_cloud.points) 
      {
        if (kdtree.nearestKSearch(pt, 1, idx, sq_dist) > 0) 
        {
          if (sq_dist[0] < (max_corr_dist * max_corr_dist)) 
          {
            inliers++;
          }
        }
      }
      fitness = static_cast<double>(inliers) / final_cloud.points.size();
    }

    return {result, fitness};
  }

  void global_localization(const Eigen::Matrix4f& pose_estimation)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_tobe_mapped(new pcl::PointCloud<pcl::PointXYZI>(*cur_scan_));
    auto global_map_in_FOV = crop_global_map_in_FOV(pose_estimation);

    if (global_map_in_FOV->empty()) 
      return;

    auto [trans_coarse, fit_coarse] = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, pose_estimation, 2.0);
    auto [trans_fine, fit_fine] = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, trans_coarse, 1.0);

    double threshold = this->get_parameter("localization_threshold").as_double();
    double max_jump_distance = this->get_parameter("max_jump_distance").as_double(); // <--- Read parameter

    if (fit_fine > threshold) 
    {
      Eigen::Matrix4f delta_T = T_map_to_odom_.inverse() * trans_fine;
      float jump_dist = delta_T.block<3, 1>(0, 3).norm();

      static bool first_success = true;
      
      if (first_success || jump_dist < max_jump_distance)
      {
        T_map_to_odom_ = trans_fine;
        publish_odom(trans_fine);
        first_success = false;
        
        if (force_initial_pose_) 
        {
          RCLCPP_INFO(this->get_logger(), "Initial pose successfully locked in via ICP!");
          force_initial_pose_ = false; // Reset flag so future periodic checks use jump protection
        } 
        else if (jump_dist > 0.5) 
        {
          RCLCPP_WARN(this->get_logger(), "ICP gently corrected FAST-LIO drift by %.2f meters.", jump_dist);
        }
      } 
      else 
      {
        RCLCPP_WARN(this->get_logger(), "Ignored bad ICP match! It tried to jump %.2f meters (limit: %.2f).", jump_dist, max_jump_distance);
      }
    } 
    else 
    {
      RCLCPP_WARN(this->get_logger(), "Fitness score %f less than threshold %f", fit_fine, threshold);
    }
  }
  
  void publish_odom(const Eigen::Matrix4f& transform)
  {
    Eigen::Matrix3f R = transform.block<3, 3>(0, 0);
    Eigen::Quaternionf q(R);
    Eigen::Vector3f t = transform.block<3, 1>(0, 3);

    // 1. Create a unique pointer instead of a stack object
    auto msg = std::make_unique<nav_msgs::msg::Odometry>();
    
    msg->header.stamp = this->get_clock()->now();
    msg->header.frame_id = "map";
    msg->pose.pose.position.x = t.x();
    msg->pose.pose.position.y = t.y();
    msg->pose.pose.position.z = t.z();
    msg->pose.pose.orientation.w = q.w();
    msg->pose.pose.orientation.x = q.x();
    msg->pose.pose.orientation.y = q.y();
    msg->pose.pose.orientation.z = q.z();

    // 2. Move the memory pointer directly into the publisher
    pub_map_to_odom_->publish(std::move(msg));
  }

  void localisation_timer_callback()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!initialized_) 
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for initial pose...");
      return;
    }

    if (cur_scan_ != nullptr) 
    {
      global_localization(T_map_to_odom_);
    }
  }
};

} // namespace fast_lio_localization

// Register node as a ROS 2 component
RCLCPP_COMPONENTS_REGISTER_NODE(fast_lio_localization::FastLIOLocalization)