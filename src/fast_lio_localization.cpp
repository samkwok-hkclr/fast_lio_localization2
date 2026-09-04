#include <mutex>
#include <memory>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <nav2_util/lifecycle_node.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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

class FastLIOLocalization : public nav2_util::LifecycleNode
{
public:
  explicit FastLIOLocalization(const rclcpp::NodeOptions & options)
  : nav2_util::LifecycleNode("fast_lio_localization", "", options)
  {
    this->declare_parameter("map_voxel_size", 0.1);
    this->declare_parameter("scan_voxel_size", 0.05);
    this->declare_parameter("freq_localization", 0.5);
    this->declare_parameter("localization_threshold", 0.8);
    this->declare_parameter("fov", 6.28319);
    this->declare_parameter("fov_far", 300.0);
    this->declare_parameter("max_jump_distance", 1.5);
    this->declare_parameter("pcd_map_path", "");
    this->declare_parameter("odom_topic", "odom");

    RCLCPP_INFO(this->get_logger(), "Fast LIO Localization node constructed. (Unconfigured State)");
  }

  // --- Lifecycle Callbacks ---

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(this->get_logger(), "Configuring node...");

    std::string odom_topic = this->get_parameter("odom_topic").as_string();

    pub_pc_in_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cur_scan_in_map", 10);
    pub_submap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/submap", 10);
    pub_map_to_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/map_to_odom", 50);

    initialize_global_map();

    sub_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cloud_registered", 10, std::bind(&FastLIOLocalization::cb_save_cur_scan, this, std::placeholders::_1));
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 50, std::bind(&FastLIOLocalization::cb_save_cur_odom, this, std::placeholders::_1));
    sub_initial_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10, std::bind(&FastLIOLocalization::cb_initialize_pose, this, std::placeholders::_1));

    double freq = this->get_parameter("freq_localization").as_double();
    timer_localization_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / freq),
      std::bind(&FastLIOLocalization::localization_timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Node configured successfully.");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(this->get_logger(), "Activating node...");
    
    pub_pc_in_map_->on_activate();
    pub_submap_->on_activate();
    pub_map_to_odom_->on_activate();

    createBond();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(this->get_logger(), "Deactivating node...");
    
    pub_pc_in_map_->on_deactivate();
    pub_submap_->on_deactivate();
    pub_map_to_odom_->on_deactivate();

    destroyBond();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(this->get_logger(), "Cleaning up node...");
    
    pub_pc_in_map_.reset();
    pub_submap_.reset();
    pub_map_to_odom_.reset();
    sub_scan_.reset();
    sub_odom_.reset();
    sub_initial_pose_.reset();
    timer_localization_.reset();
    
    global_map_.reset();
    cur_scan_.reset();
    cur_odom_.reset();
    
    tf_map_to_odom_ = tf2::Transform::getIdentity();
    initialized_ = false;
    force_initial_pose_ = true;

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down node...");
    return nav2_util::CallbackReturn::SUCCESS;
  }

private:
  pcl::PointCloud<pcl::PointXYZI>::Ptr global_map_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cur_scan_;
  nav_msgs::msg::Odometry::SharedPtr cur_odom_;
  
  tf2::Transform tf_map_to_odom_ = tf2::Transform::getIdentity();
  bool initialized_ = false;
  bool force_initial_pose_ = true;
  std::mutex data_mutex_;

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc_in_map_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_submap_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub_map_to_odom_;
  
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_scan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initial_pose_;
  rclcpp::TimerBase::SharedPtr timer_localization_;

  // --- Helper Methods ---

  // Convert tf2::Transform to Eigen::Matrix4f for PCL operations
  Eigen::Matrix4f tf_to_eigen(const tf2::Transform& tf)
  {
    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
    tf2::Matrix3x3 rot = tf.getBasis();
    tf2::Vector3 trans = tf.getOrigin();
    for (int i = 0; i < 3; i++) {
      mat(i, 3) = trans[i];
      for (int j = 0; j < 3; j++) {
        mat(i, j) = rot[i][j];
      }
    }
    return mat;
  }

  // Convert Eigen::Matrix4f to tf2::Transform from PCL results
  tf2::Transform eigen_to_tf(const Eigen::Matrix4f& mat)
  {
    tf2::Matrix3x3 rot(
      mat(0, 0), mat(0, 1), mat(0, 2),
      mat(1, 0), mat(1, 1), mat(1, 2),
      mat(2, 0), mat(2, 1), mat(2, 2)
    );
    tf2::Vector3 trans(mat(0, 3), mat(1, 3), mat(2, 3));
    return tf2::Transform(rot, trans);
  }

  // Convert geometry_msgs Pose to tf2::Transform
  tf2::Transform pose_to_tf(const geometry_msgs::msg::Pose& pose)
  {
    tf2::Transform t;
    tf2::fromMsg(pose, t);
    return t;
  }

  bool is_active()
  {
    return this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  }

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
    
    if (!cloud || cloud->empty()) 
      return cloud_filtered;

    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(voxel_size, voxel_size, voxel_size);
    sor.filter(*cloud_filtered);
    return cloud_filtered;
  }

  void cb_save_cur_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!is_active()) return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    cur_odom_ = msg;
  }

  void cb_save_cur_scan(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!is_active()) return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    cur_scan_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cur_scan_);
    
    pub_pc_in_map_->publish(*msg);
  }

  void cb_initialize_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    if (!is_active()) return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    tf2::Transform initial_pose = pose_to_tf(msg->pose.pose);
    initialized_ = true;
    force_initial_pose_ = true;
    RCLCPP_INFO(this->get_logger(), "Initial pose received.");

    if (cur_scan_ != nullptr) 
    {
      global_localization(initial_pose);
    } 
    else 
    {
      RCLCPP_WARN(this->get_logger(), "Scan is null during initialization.");
    }
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr crop_global_map_in_FOV(const tf2::Transform& pose_estimation)
  {
    if (cur_odom_ == nullptr) 
    {
      return pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    }

    tf2::Transform tf_odom_to_base = pose_to_tf(cur_odom_->pose.pose);
    tf2::Transform tf_map_to_base = pose_estimation * tf_odom_to_base;
    tf2::Transform tf_base_to_map = tf_map_to_base.inverse();

    pcl::PointCloud<pcl::PointXYZI>::Ptr map_in_base(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*global_map_, *map_in_base, tf_to_eigen(tf_base_to_map));

    double fov = this->get_parameter("fov").as_double();
    double fov_far = this->get_parameter("fov_far").as_double();

    pcl::PointCloud<pcl::PointXYZI>::Ptr cropped_map(new pcl::PointCloud<pcl::PointXYZI>());
    
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

    sensor_msgs::msg::PointCloud2 submap_msg;
    pcl::toROSMsg(*cropped_map, submap_msg);
    submap_msg.header = cur_odom_->header;
    submap_msg.header.frame_id = "map";
    pub_submap_->publish(submap_msg);

    return cropped_map;
  }

  std::pair<tf2::Transform, double> registration_at_scale(
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan,
    pcl::PointCloud<pcl::PointXYZI>::Ptr map,
    const tf2::Transform& initial, double scale)
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
    // PCL requires Eigen::Matrix4f for alignment initial guess
    icp.align(final_cloud, tf_to_eigen(initial));

    Eigen::Matrix4f result = icp.getFinalTransformation();
    
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

    return {eigen_to_tf(result), fitness};
  }

  void global_localization(const tf2::Transform& pose_estimation)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_tobe_mapped(new pcl::PointCloud<pcl::PointXYZI>(*cur_scan_));
    auto global_map_in_FOV = crop_global_map_in_FOV(pose_estimation);

    if (global_map_in_FOV->empty()) 
      return;

    auto [trans_coarse, fit_coarse] = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, pose_estimation, 2.0);
    auto [trans_fine, fit_fine] = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, trans_coarse, 1.0);

    double threshold = this->get_parameter("localization_threshold").as_double();
    double max_jump_distance = this->get_parameter("max_jump_distance").as_double();

    if (fit_fine > threshold) 
    {
      tf2::Transform delta_T = tf_map_to_odom_.inverse() * trans_fine;
      float jump_dist = delta_T.getOrigin().length();

      static bool first_success = true;
      
      if (first_success || jump_dist < max_jump_distance)
      {
        tf_map_to_odom_ = trans_fine;
        publish_odom(trans_fine);
        first_success = false;
        
        if (force_initial_pose_) 
        {
          RCLCPP_INFO(this->get_logger(), "Initial pose successfully locked in via ICP!");
          force_initial_pose_ = false;
        } 
        else if (jump_dist > 1.0) 
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
  
  void publish_odom(const tf2::Transform& transform)
  {
    auto msg = std::make_unique<nav_msgs::msg::Odometry>();
    
    msg->header.stamp = this->get_clock()->now();
    msg->header.frame_id = "map";
    
    // Set position
    msg->pose.pose.position.x = transform.getOrigin().x();
    msg->pose.pose.position.y = transform.getOrigin().y();
    msg->pose.pose.position.z = transform.getOrigin().z();
    
    // Set orientation
    tf2::Quaternion q = transform.getRotation();
    msg->pose.pose.orientation.x = q.x();
    msg->pose.pose.orientation.y = q.y();
    msg->pose.pose.orientation.z = q.z();
    msg->pose.pose.orientation.w = q.w();

    pub_map_to_odom_->publish(std::move(msg));
  }

  void localization_timer_callback()
  {
    if (!is_active()) return;
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (!initialized_) 
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "Waiting for initial pose...");
      return;
    }

    if (cur_scan_ != nullptr) 
    {
      global_localization(tf_map_to_odom_);
    }
  }
};

} // namespace fast_lio_localization

// Register node as a ROS 2 component
RCLCPP_COMPONENTS_REGISTER_NODE(fast_lio_localization::FastLIOLocalization)