#include <mutex>
#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <nav2_util/lifecycle_node.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>

namespace fast_lio_localization
{

class TransformFusion : public nav2_util::LifecycleNode
{
public:
  explicit TransformFusion(const rclcpp::NodeOptions & options)
  : nav2_util::LifecycleNode("transform_fusion", "", options)
  {
    // Parameters are declared in the constructor
    this->declare_parameter("odom_frame_id", "odom");
    this->declare_parameter("base_frame_id", "base_footprint");
    this->declare_parameter("lidar_frame_id", "livox_frame"); 
    this->declare_parameter("camera_init_frame_id", "camera_init");
    this->declare_parameter("body_frame_id", "body");
    this->declare_parameter("odom_topic", "odom");

    RCLCPP_INFO(this->get_logger(), "TransformFusion node constructed. (Unconfigured State)");
  }

  // --- Lifecycle Callbacks ---

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(this->get_logger(), "Configuring node...");

    odom_frame_id_         = this->get_parameter("odom_frame_id").as_string();
    base_frame_id_         = this->get_parameter("base_frame_id").as_string();
    lidar_frame_id_        = this->get_parameter("lidar_frame_id").as_string();
    camera_init_frame_id_  = this->get_parameter("camera_init_frame_id").as_string();
    body_frame_id_         = this->get_parameter("body_frame_id").as_string();
    std::string odom_topic = this->get_parameter("odom_topic").as_string();

    // TF Setup
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    tf_broadcaster_        = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    // Lifecycle Publisher
    pub_localization_ = this->create_publisher<nav_msgs::msg::Odometry>("/localization", 10);

    // Subscribers
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 50,
      std::bind(&TransformFusion::cb_save_cur_odom, this, std::placeholders::_1));

    sub_map_to_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/map_to_odom", 10,
      std::bind(&TransformFusion::cb_save_map_to_odom, this, std::placeholders::_1));

    // Timer 50 Hz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&TransformFusion::transform_fusion_callback, this));

    RCLCPP_INFO(this->get_logger(),
      "TransformFusion configured.\n"
      "  camera_init : %s\n"
      "  odom        : %s\n"
      "  base        : %s\n"
      "  lidar/body  : %s",
      camera_init_frame_id_.c_str(),
      odom_frame_id_.c_str(),
      base_frame_id_.c_str(),
      lidar_frame_id_.c_str());

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(this->get_logger(), "Activating node...");
    
    createBond();

    // Lifecycle publishers must be explicitly activated
    pub_localization_->on_activate();
    
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(this->get_logger(), "Deactivating node...");
    
    // Lifecycle publishers must be explicitly deactivated
    pub_localization_->on_deactivate();

    destroyBond();
    
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(this->get_logger(), "Cleaning up node...");

    // Release resources
    tf_buffer_.reset();
    tf_listener_.reset();
    tf_broadcaster_.reset();
    tf_static_broadcaster_.reset();

    pub_localization_.reset();
    sub_odom_.reset();
    sub_map_to_odom_.reset();
    timer_.reset();

    // Reset message caches
    std::lock_guard<std::mutex> lock(data_mutex_);
    curr_camera_init_body_.reset();
    curr_map_to_camera_init_.reset();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down node...");
    return nav2_util::CallbackReturn::SUCCESS;
  }

private:
  std::mutex data_mutex_;

  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string lidar_frame_id_;
  std::string camera_init_frame_id_;
  std::string body_frame_id_;

  nav_msgs::msg::Odometry::SharedPtr curr_camera_init_body_;    // camera_init -> body
  nav_msgs::msg::Odometry::SharedPtr curr_map_to_camera_init_;  // map -> camera_init

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

  // Lifecycle publisher
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub_localization_;
  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_map_to_odom_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Helper check to skip callbacks when node isn't active
  bool is_active()
  {
    return this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  }

  void cb_save_cur_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!is_active()) return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    curr_camera_init_body_ = msg;
  }

  void cb_save_map_to_odom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    if (!is_active()) return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    curr_map_to_camera_init_ = std::const_pointer_cast<nav_msgs::msg::Odometry>(msg);
  }

  void transform_fusion_callback()
  {
    if (!is_active()) return;
    
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (!curr_camera_init_body_) 
      return;

    if (!curr_map_to_camera_init_)
      return;

    auto get_flat_transform = [&] (tf2::Transform g) -> tf2::Transform {
      double roll, pitch, yaw;
      tf2::Matrix3x3(g.getRotation()).getRPY(roll, pitch, yaw);

      tf2::Transform flat_transform;
      flat_transform.setOrigin(tf2::Vector3(g.getOrigin().x(), g.getOrigin().y(), 0.0));

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      flat_transform.setRotation(q);

      return flat_transform;
    };

    // ============================================================
    // 1. map -> camera_init  (KEEP original TF - lidar frame)
    // ============================================================
    tf2::Transform g_map_camera_init;
    g_map_camera_init.setIdentity();

    tf2::fromMsg(curr_map_to_camera_init_->pose.pose, g_map_camera_init);

    geometry_msgs::msg::TransformStamped tf_map_camera;
    tf_map_camera.header.stamp    = this->get_clock()->now();
    tf_map_camera.header.frame_id = "map";
    tf_map_camera.child_frame_id  = camera_init_frame_id_;
    tf_map_camera.transform       = tf2::toMsg(g_map_camera_init);
    tf_broadcaster_->sendTransform(tf_map_camera);

    // ============================================================
    // 2. Lookup extrinsic base ← lidar
    // ============================================================
    geometry_msgs::msg::TransformStamped tf_base_lidar_msg;
    try 
    {
      tf_base_lidar_msg = tf_buffer_->lookupTransform(base_frame_id_, lidar_frame_id_, tf2::TimePointZero);
    } 
    catch (const tf2::TransformException & ex) 
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Waiting for extrinsic %s -> %s : %s",
        base_frame_id_.c_str(), lidar_frame_id_.c_str(), ex.what());
      return;
    }

    tf2::Transform g_base_lidar;
    tf2::fromMsg(tf_base_lidar_msg.transform, g_base_lidar);

    // ============================================================
    // 3. map -> odom  (transformed to base frame)
    //    g_map_odom = g_map_camera_init * g_lidar_base
    // ============================================================
    tf2::Transform g_map_odom = g_map_camera_init * g_base_lidar.inverse();

    geometry_msgs::msg::TransformStamped tf_map_odom;
    tf_map_odom.header.stamp    = tf_map_camera.header.stamp;
    tf_map_odom.header.frame_id = "map";
    tf_map_odom.child_frame_id  = odom_frame_id_;
    tf_map_odom.transform       = tf2::toMsg(get_flat_transform(g_map_odom));
    
    tf_broadcaster_->sendTransform(tf_map_odom);

    // ============================================================
    // 4. camera_init -> body  (from FAST-LIO)
    // ============================================================
    tf2::Transform g_camera_init_body;          // actually camera_init -> body
    tf2::fromMsg(curr_camera_init_body_->pose.pose, g_camera_init_body);

    // ============================================================
    // 5. odom -> base_link
    // ============================================================
    tf2::Transform g_odom_base = g_camera_init_body;

    geometry_msgs::msg::TransformStamped tf_odom_base;
    tf_odom_base.header.stamp    = curr_camera_init_body_->header.stamp;
    tf_odom_base.header.frame_id = odom_frame_id_;
    tf_odom_base.child_frame_id  = base_frame_id_;
    tf_odom_base.transform       = tf2::toMsg(get_flat_transform(g_odom_base));
    tf_broadcaster_->sendTransform(tf_odom_base);

    // ============================================================
    // 6. map -> base_link (fused) = g_map_odom * g_odom_base
    // ============================================================
    tf2::Transform g_map_base = g_map_odom * g_odom_base;

    nav_msgs::msg::Odometry loc;
    loc.header.stamp    = curr_camera_init_body_->header.stamp;
    loc.header.frame_id = "map";
    loc.child_frame_id  = base_frame_id_;
    tf2::toMsg(g_map_base, loc.pose.pose);
    loc.twist = curr_camera_init_body_->twist;
    
    pub_localization_->publish(loc);
  }
};

}  // namespace fast_lio_localization

RCLCPP_COMPONENTS_REGISTER_NODE(fast_lio_localization::TransformFusion)