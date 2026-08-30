#include <mutex>
#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace fast_lio_localization
{

class TransformFusion : public rclcpp::Node
{
public:
  explicit TransformFusion(const rclcpp::NodeOptions & options)
  : Node("transform_fusion", options)
  {
    // Publishers and Broadcasters
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    pub_localization_ = this->create_publisher<nav_msgs::msg::Odometry>("/localization", 1);

    // Subscribers
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry", 1, std::bind(&TransformFusion::cb_save_cur_odom, this, std::placeholders::_1));
    
    sub_map_to_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/map_to_odom", 1, std::bind(&TransformFusion::cb_save_map_to_odom, this, std::placeholders::_1));

    // Timer (50 Hz = 20 ms)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&TransformFusion::transform_fusion_callback, this));
        
    RCLCPP_INFO(this->get_logger(), "TransformFusion node initialized.");
  }

private:
  nav_msgs::msg::Odometry::SharedPtr cur_odom_to_baselink_;
  nav_msgs::msg::Odometry::SharedPtr cur_map_to_odom_;
  std::mutex data_mutex_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_localization_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_map_to_odom_;
  rclcpp::TimerBase::SharedPtr timer_;

  void cb_save_cur_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    cur_odom_to_baselink_ = msg;
  }

  void cb_save_map_to_odom(const nav_msgs::msg::Odometry::ConstSharedPtr msg) // Use ConstSharedPtr
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    cur_map_to_odom_ = std::const_pointer_cast<nav_msgs::msg::Odometry>(msg); 
  }

  void transform_fusion_callback()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (cur_odom_to_baselink_ == nullptr) 
    {
      return;
    }

    // 1. Get map -> odom transform
    tf2::Transform t_map_to_odom;
    t_map_to_odom.setIdentity();
    
    if (cur_map_to_odom_ != nullptr) 
    {
      tf2::fromMsg(cur_map_to_odom_->pose.pose, t_map_to_odom);
    }

    // 2. Broadcast TF (map -> camera_init)
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "camera_init";
    tf_msg.transform = tf2::toMsg(t_map_to_odom);
    tf_broadcaster_->sendTransform(tf_msg);

    // 3. Calculate map -> base_link (T_map_to_odom * T_odom_to_base_link)
    tf2::Transform t_odom_to_baselink;
    tf2::fromMsg(cur_odom_to_baselink_->pose.pose, t_odom_to_baselink);
    
    tf2::Transform t_map_to_baselink = t_map_to_odom * t_odom_to_baselink;

    // 4. Publish Fused Odometry
    nav_msgs::msg::Odometry localization_msg;
    localization_msg.header.stamp = this->get_clock()->now();
    localization_msg.header.frame_id = "map";
    localization_msg.child_frame_id = "body";
    
    tf2::toMsg(t_map_to_baselink, localization_msg.pose.pose);
    localization_msg.twist = cur_odom_to_baselink_->twist;

    pub_localization_->publish(localization_msg);
  }
};

} // namespace fast_lio_localization

// Register node as a ROS 2 component
RCLCPP_COMPONENTS_REGISTER_NODE(fast_lio_localization::TransformFusion)