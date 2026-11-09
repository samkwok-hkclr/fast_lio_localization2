#include <mutex>
#include <memory>
#include <cmath>
#include <optional>

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
    declare_parameter("map_voxel_size", 0.1);
    declare_parameter("scan_voxel_size", 0.05);
    declare_parameter("freq_localization", 0.5);
    declare_parameter("localization_threshold", 0.8);
    declare_parameter("fov", 6.28319);
    declare_parameter("fov_far", 300.0);
    declare_parameter("max_jump_distance", 1.5);
    declare_parameter("pcd_map_path", "");
    declare_parameter("odom_topic", "odom");
  }

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Configuring node...");

    const double freq = get_parameter("freq_localization").as_double();
    if (freq <= 0.0 || freq > 10.0) 
    {
      RCLCPP_ERROR(get_logger(), "freq_localization %.2f out of range (0, 10]", freq);
      return nav2_util::CallbackReturn::FAILURE;
    }

    if (!initialize_global_map()) 
    {
      RCLCPP_ERROR(get_logger(), "Failed to initialize global map");
      return nav2_util::CallbackReturn::FAILURE;
    }
    if (!global_map_->empty()) 
    {
      map_kdtree_.setInputCloud(global_map_);
    }

    pub_pc_in_map_   = create_publisher<sensor_msgs::msg::PointCloud2>("/cur_scan_in_map", 10);
    pub_submap_      = create_publisher<sensor_msgs::msg::PointCloud2>("/submap", 10);
    pub_map_to_odom_ = create_publisher<nav_msgs::msg::Odometry>("/map_to_odom", 50);

    const auto odom_topic = get_parameter("odom_topic").as_string();
    sub_scan_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cloud_registered", 10,
      std::bind(&FastLIOLocalization::save_cur_scan_cb, this, std::placeholders::_1));
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 50,
      std::bind(&FastLIOLocalization::save_cur_odom_cb, this, std::placeholders::_1));
    sub_initial_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10,
      std::bind(&FastLIOLocalization::initialize_pose_cb, this, std::placeholders::_1));

    timer_localization_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / freq),
      std::bind(&FastLIOLocalization::localization_timer_cb, this));

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      initialized_        = false;
      force_initial_pose_ = true;
      first_success_      = true;
      tf_map_to_odom_     = tf2::Transform::getIdentity();
      cur_scan_.reset();
      cur_odom_.reset();
    }

    RCLCPP_INFO(get_logger(),
      "Configured: map=%zu pts, voxel=%.2f, freq=%.2f Hz, threshold=%.2f",
      global_map_->size(),
      get_parameter("map_voxel_size").as_double(),
      freq,
      get_parameter("localization_threshold").as_double());
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    pub_pc_in_map_->on_activate();
    pub_submap_->on_activate();
    pub_map_to_odom_->on_activate();
    createBond();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    pub_pc_in_map_->on_deactivate();
    pub_submap_->on_deactivate();
    pub_map_to_odom_->on_deactivate();
    destroyBond();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    pub_pc_in_map_.reset();
    pub_submap_.reset();
    pub_map_to_odom_.reset();
    sub_scan_.reset();
    sub_odom_.reset();
    sub_initial_pose_.reset();
    timer_localization_.reset();

    std::lock_guard<std::mutex> lock(data_mutex_);
    global_map_.reset();
    cur_scan_.reset();
    cur_odom_.reset();
    tf_map_to_odom_     = tf2::Transform::getIdentity();
    initialized_        = false;
    force_initial_pose_ = true;
    first_success_      = true;
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_INFO(get_logger(), "Shutting down from %s", state.label().c_str());
    if (state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) 
    {
      on_deactivate(state);
    }
    on_cleanup(state);
    return nav2_util::CallbackReturn::SUCCESS;
  }

private:
  std::mutex data_mutex_;
  pcl::KdTreeFLANN<pcl::PointXYZI>    map_kdtree_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr global_map_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cur_scan_;
  nav_msgs::msg::Odometry::SharedPtr   cur_odom_;
  tf2::Transform tf_map_to_odom_ = tf2::Transform::getIdentity();
  bool initialized_        = false;
  bool force_initial_pose_ = true;
  bool first_success_      = true;

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc_in_map_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_submap_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr      pub_map_to_odom_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr                sub_scan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                      sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initial_pose_;
  rclcpp::TimerBase::SharedPtr                                                  timer_localization_;

  // ---------- math helpers ----------
  static Eigen::Matrix4f tf_to_eigen(const tf2::Transform& tf)
  {
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    const tf2::Matrix3x3 r = tf.getBasis();
    const tf2::Vector3   t = tf.getOrigin();
    for (int i = 0; i < 3; ++i) 
    {
      m(i, 3) = static_cast<float>(t[i]);
      for (int j = 0; j < 3; ++j) 
      { 
        m(i, j) = static_cast<float>(r[i][j]);
      }
    }
    return m;
  }
  static tf2::Transform eigen_to_tf(const Eigen::Matrix4f& m)
  {
    tf2::Matrix3x3 r(m(0,0),m(0,1),m(0,2),
                     m(1,0),m(1,1),m(1,2),
                     m(2,0),m(2,1),m(2,2));
    return tf2::Transform(r, tf2::Vector3(m(0,3), m(1,3), m(2,3)));
  }
  static tf2::Transform pose_to_tf(const geometry_msgs::msg::Pose& pose)
  {
    tf2::Transform t; tf2::fromMsg(pose, t); return t;
  }

  bool is_active()
  {
    return get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  }

  // ---------- map loading ----------
  bool initialize_global_map()
  {
    const std::string map_path = get_parameter("pcd_map_path").as_string();
    if (map_path.empty()) 
    {
      RCLCPP_ERROR(get_logger(), "pcd_map_path is empty");
      return false;
    }

    auto raw = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(map_path, *raw) == -1) 
    {
      RCLCPP_ERROR(get_logger(), "Cannot load PCD: %s", map_path.c_str());
      return false;
    }
    if (raw->empty()) 
    {
      RCLCPP_ERROR(get_logger(), "PCD file is empty: %s", map_path.c_str());
      return false;
    }

    global_map_ = voxel_down_sample(raw, get_parameter("map_voxel_size").as_double());
    RCLCPP_INFO(get_logger(), "Map: %zu -> %zu pts", raw->size(), global_map_->size());
    return !global_map_->empty();
  }

  static pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_down_sample(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, double voxel)
  {
    auto out = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    if (!cloud || cloud->empty() || voxel <= 0.0) 
      return out;
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(static_cast<float>(voxel),
                   static_cast<float>(voxel),
                   static_cast<float>(voxel));
    vg.filter(*out);
    return out;
  }

  // ---------- callbacks ----------
  void save_cur_odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!is_active()) 
      return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    cur_odom_ = msg;
  }

  void save_cur_scan_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!is_active()) 
      return;

    // ROS→PCL conversion is slow; do it outside the lock
    auto new_scan = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::fromROSMsg(*msg, *new_scan);

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      cur_scan_ = std::move(new_scan);
    }

    // NOTE: publishes the raw scan in the body frame; rename topic later
    if (pub_pc_in_map_->is_activated()) 
      pub_pc_in_map_->publish(*msg);
  }

  void initialize_pose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    if (!is_active()) 
      return;

    tf2::Transform new_tf;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!cur_odom_) 
      {
        RCLCPP_WARN(get_logger(), "No odometry yet; cannot anchor initial pose.");
        return;
      }
      // User provides map→base; convert to map→odom
      tf2::Transform tf_map_to_base;
      tf2::fromMsg(msg->pose.pose, tf_map_to_base);
      tf2::Transform tf_odom_to_base;
      tf2::fromMsg(cur_odom_->pose.pose, tf_odom_to_base);

      new_tf              = tf_map_to_base * tf_odom_to_base.inverse();
      tf_map_to_odom_     = new_tf;
      initialized_        = true;
      force_initial_pose_ = true;
      // Don't let ICP teleport on the next tick; we want only refinement.
      first_success_      = false;
    }
    RCLCPP_INFO(get_logger(), "Initial pose received; refining on next tick.");
    publish_odom(new_tf);
  }

  // ---------- crop + ICP (pure: no member mutation) ----------
  pcl::PointCloud<pcl::PointXYZI>::Ptr crop_global_map_in_FOV(
    const tf2::Transform& tf_map_to_base,
    const std_msgs::msg::Header& header)
  {
    auto cropped = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    if (!global_map_ || global_map_->empty()) 
      return cropped;

    const double fov     = get_parameter("fov").as_double();
    const double fov_far = get_parameter("fov_far").as_double();

    pcl::PointXYZI center;
    center.x = static_cast<float>(tf_map_to_base.getOrigin().x());
    center.y = static_cast<float>(tf_map_to_base.getOrigin().y());
    center.z = static_cast<float>(tf_map_to_base.getOrigin().z());

    std::vector<int>   k_idx;
    std::vector<float> k_sq;
    map_kdtree_.radiusSearch(center, fov_far * 1.5, k_idx, k_sq);

    cropped->reserve(k_idx.size());

    if (fov >= 2.0 * M_PI - 1e-6) 
    {
      for (int i : k_idx) 
      {
        cropped->points.push_back(global_map_->points[i]);
      }
    } 
    else 
    {
      const tf2::Transform tf_base_to_map = tf_map_to_base.inverse();
      const tf2::Matrix3x3 rot   = tf_base_to_map.getBasis();
      const tf2::Vector3   trans = tf_base_to_map.getOrigin();
      const double         half  = fov / 2.0;

      for (int i : k_idx) 
      {
        const auto& p_map = global_map_->points[i];
        const tf2::Vector3 p_base = rot * tf2::Vector3(p_map.x, p_map.y, p_map.z) + trans;
        if (p_base.x() > 0.0 && p_base.x() < fov_far &&
            std::abs(std::atan2(p_base.y(), p_base.x())) < half)
        {
          cropped->points.push_back(p_map);
        }
      }
    }
    cropped->width  = cropped->points.size();
    cropped->height = 1;

    if (pub_submap_->is_activated()) 
    {
      sensor_msgs::msg::PointCloud2 submsg;
      pcl::toROSMsg(*cropped, submsg);
      submsg.header            = header;
      submsg.header.frame_id   = "map";
      pub_submap_->publish(submsg);
    }
    return cropped;
  }

  std::pair<tf2::Transform, double> registration_at_scale(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& map,
    const tf2::Transform& initial,
    double scale)
  {
    const double scan_vs = get_parameter("scan_voxel_size").as_double() * scale;
    const double map_vs  = get_parameter("map_voxel_size").as_double()  * scale;
    const double max_corr = 1.0 * scale;

    auto scan_ds = voxel_down_sample(scan, scan_vs);
    auto map_ds  = voxel_down_sample(map,  map_vs);
    if (scan_ds->empty() || map_ds->empty()) 
      return {initial, 0.0};

    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    icp.setInputSource(scan_ds);
    icp.setInputTarget(map_ds);
    icp.setMaxCorrespondenceDistance(max_corr);
    icp.setMaximumIterations(20);
    icp.setTransformationEpsilon(1e-6);

    pcl::PointCloud<pcl::PointXYZI> aligned;
    icp.align(aligned, tf_to_eigen(initial));
    if (!icp.hasConverged() || aligned.empty()) 
      return {initial, 0.0};

    pcl::KdTreeFLANN<pcl::PointXYZI> kd;
    kd.setInputCloud(map_ds);
    const float max_sq = static_cast<float>(max_corr * max_corr);
    std::vector<int>   nn_i(1);
    std::vector<float> nn_d(1);
    int inliers = 0;
    for (const auto& pt : aligned.points) 
    {
      if (kd.nearestKSearch(pt, 1, nn_i, nn_d) > 0 && nn_d[0] < max_sq) 
        ++inliers;
    }
    const double fitness = static_cast<double>(inliers) / aligned.points.size();
    return {eigen_to_tf(icp.getFinalTransformation()), fitness};
  }

  std::optional<tf2::Transform> run_icp(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan,
    const nav_msgs::msg::Odometry& odom,
    const tf2::Transform& prior_map_to_odom)
  {
    if (!scan || scan->empty()) 
      return std::nullopt;

    const tf2::Transform tf_odom_to_base = pose_to_tf(odom.pose.pose);
    const tf2::Transform tf_map_to_base  = prior_map_to_odom * tf_odom_to_base;

    auto cropped = crop_global_map_in_FOV(tf_map_to_base, odom.header);
    if (cropped->empty()) 
      return std::nullopt;

    auto [coarse_tf, coarse_fit] = registration_at_scale(scan, cropped, prior_map_to_odom, 2.0);
    if (coarse_fit < 0.3) 
      return std::nullopt;

    auto [fine_tf, fine_fit] = registration_at_scale(scan, cropped, coarse_tf, 1.0);

    const double threshold = get_parameter("localization_threshold").as_double();
    if (fine_fit < threshold) 
    {
      RCLCPP_WARN(get_logger(), "ICP fitness %.3f < %.3f; discarding", fine_fit, threshold);
      return std::nullopt;
    }
    return fine_tf;
  }

  // ---------- periodic ----------
  void localization_timer_cb()
  {
    if (!is_active()) 
      return;

    nav_msgs::msg::Odometry::SharedPtr odom;
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan;
    tf2::Transform prior_tf;
    bool first_success, force_initial;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!initialized_) 
      {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "Waiting for initial pose...");
        return;
      }
      if (!cur_scan_ || !cur_odom_) 
        return;

      odom          = cur_odom_;      // shared_ptr copy — no deep copy needed
      scan          = cur_scan_;
      prior_tf      = tf_map_to_odom_;
      first_success = first_success_;
      force_initial = force_initial_pose_;
    }

    auto new_tf = run_icp(scan, *odom, prior_tf);
    if (!new_tf) 
      return;

    const double jump_dist = (prior_tf.inverse() * *new_tf).getOrigin().length();
    const double max_jump  = get_parameter("max_jump_distance").as_double();
    if (!first_success && jump_dist >= max_jump) 
    {
      RCLCPP_WARN(get_logger(), "Ignoring ICP jump %.2f m (limit %.2f m)", jump_dist, max_jump);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      tf_map_to_odom_ = *new_tf;
      first_success_  = false;
      if (force_initial) 
      {
        RCLCPP_INFO(get_logger(), "Initial pose locked via ICP.");
        force_initial_pose_ = false;
      } 
      else if (jump_dist > 1.0) 
      {
        RCLCPP_INFO(get_logger(), "ICP corrected drift by %.2f m", jump_dist);
      }
    }
    publish_odom(*new_tf);
  }

  void publish_odom(const tf2::Transform& tf)
  {
    auto msg = std::make_unique<nav_msgs::msg::Odometry>();
    msg->header.stamp         = now();
    msg->header.frame_id      = "map";
    msg->child_frame_id       = "odom";
    msg->pose.pose.position.x = tf.getOrigin().x();
    msg->pose.pose.position.y = tf.getOrigin().y();
    msg->pose.pose.position.z = tf.getOrigin().z();
    const tf2::Quaternion q = tf.getRotation();
    msg->pose.pose.orientation.x = q.x();
    msg->pose.pose.orientation.y = q.y();
    msg->pose.pose.orientation.z = q.z();
    msg->pose.pose.orientation.w = q.w();
    pub_map_to_odom_->publish(std::move(msg));
  }
};

}  // namespace fast_lio_localization

RCLCPP_COMPONENTS_REGISTER_NODE(fast_lio_localization::FastLIOLocalization)