#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class JoyArmCartesianNode : public rclcpp::Node
{
public:
  JoyArmCartesianNode() : Node("joy_arm_cartesian_node")
    , tf_buffer_(get_clock())
    , tf_listener_(tf_buffer_)
  {
    deadzone_x_ = declare_parameter<double>("deadzone_x");
    deadzone_y_ = declare_parameter<double>("deadzone_y");
    deadzone_z_ = declare_parameter<double>("deadzone_z");
    input_max_x_ = declare_parameter<double>("input_max_x");
    input_max_y_ = declare_parameter<double>("input_max_y");
    input_max_z_ = declare_parameter<double>("input_max_z");
    max_linear_x_ = declare_parameter<double>("max_linear_x");
    max_linear_y_ = declare_parameter<double>("max_linear_y");
    max_linear_z_ = declare_parameter<double>("max_linear_z");
    servo_command_frame_ = declare_parameter<std::string>("servo_command_frame", "tool0");
    target_pose_frame_ = declare_parameter<std::string>("target_pose_frame", "base_mount");
    target_pose_topic_ = declare_parameter<std::string>("target_pose_topic", "/target_pose");

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy_arm_cartesian",
      10,
      [this](sensor_msgs::msg::Joy::SharedPtr msg)
      {
        joyArmCartesianCallback(msg);
      }
    );
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(target_pose_topic_, 10);
    gripper_pub_ = create_publisher<std_msgs::msg::Float32>("/arm_gripper", 10);
    update_timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      [this]()
      {
        updateTargetPose();
      }
    );
    last_update_time_ = now();
  }

private:
  int axis_x_{0};
  int axis_y_{1};
  int axis_z_{4};
  int axis_gripper_open_{5};
  int axis_gripper_close_{2};
  double deadzone_x_{};
  double deadzone_y_{};
  double deadzone_z_{};
  double input_max_x_{};
  double input_max_y_{};
  double input_max_z_{};
  double max_linear_x_{};
  double max_linear_y_{};
  double max_linear_z_{};
  std::string servo_command_frame_{};
  std::string target_pose_frame_{};
  std::string target_pose_topic_{};
  bool target_pose_initialized_{false};
  geometry_msgs::msg::PoseStamped target_pose_{};
  std::vector<float> latest_axes_{};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gripper_pub_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  double scaleAxis(double input, double input_max, double deadzone, double max_linear) const
  {
    if(input_max == 0.0)
    {
      return 0.0;
    }

    double normalized = std::clamp(input / input_max, -1.0, 1.0);

    if(std::fabs(normalized) < deadzone)
    {
      return 0.0;
    }

    return normalized * max_linear;
  }

  void joyArmCartesianCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    latest_axes_ = msg->axes;
  }

  bool updateCurrentToolPose(geometry_msgs::msg::PoseStamped & current_pose)
  {
    try
    {
      const auto transform = tf_buffer_.lookupTransform(
        target_pose_frame_, servo_command_frame_, tf2::TimePointZero);

      current_pose.header.stamp = transform.header.stamp;
      current_pose.header.frame_id = target_pose_frame_;
      current_pose.pose.position.x = transform.transform.translation.x;
      current_pose.pose.position.y = transform.transform.translation.y;
      current_pose.pose.position.z = transform.transform.translation.z;
      current_pose.pose.orientation = transform.transform.rotation;
      return true;
    }
    catch(const tf2::TransformException & ex)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "failed to look up %s in %s: %s",
        servo_command_frame_.c_str(),
        target_pose_frame_.c_str(),
        ex.what());
      return false;
    }
  }

  void updateTargetPose()
  {
    auto current_time = now();
    double dt = (current_time - last_update_time_).seconds();
    last_update_time_ = current_time;

    if(dt <= 0.0)
    {
      return;
    }

    dt = std::min(dt, 0.1);

    geometry_msgs::msg::PoseStamped current_tool_pose{};
    const bool have_current_tool_pose = updateCurrentToolPose(current_tool_pose);

    if(!target_pose_initialized_)
    {
      if(!have_current_tool_pose)
      {
        return;
      }

      target_pose_ = current_tool_pose;
      target_pose_initialized_ = true;
    }

    if(have_current_tool_pose)
    {
      target_pose_.pose.orientation = current_tool_pose.pose.orientation;
    }

    std_msgs::msg::Float32 gripper{};
    const size_t max_axis_idx = static_cast<size_t>(
      std::max({axis_x_, axis_y_, axis_z_, axis_gripper_open_, axis_gripper_close_}));

    if(latest_axes_.size() > max_axis_idx)
    {
      const double dx = -scaleAxis(latest_axes_[axis_x_], input_max_x_, deadzone_x_, max_linear_x_) * dt;
      const double dy = scaleAxis(latest_axes_[axis_y_], input_max_y_, deadzone_y_, max_linear_y_) * dt;
      const double dz = scaleAxis(latest_axes_[axis_z_], input_max_z_, deadzone_z_, max_linear_z_) * dt;

      const auto & orientation = target_pose_.pose.orientation;
      tf2::Quaternion rotation(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w);
      tf2::Matrix3x3 basis(rotation);
      const tf2::Vector3 delta = basis * tf2::Vector3(dx, dy, dz);

      target_pose_.pose.position.x += delta.x();
      target_pose_.pose.position.y += delta.y();
      target_pose_.pose.position.z += delta.z();

      const double gripper_open = scaleAxis(
        (1.0 - latest_axes_[axis_gripper_open_]) / 2.0,
        input_max_x_,
        deadzone_x_,
        max_linear_x_);

      const double gripper_close = scaleAxis(
        (1.0 - latest_axes_[axis_gripper_close_]) / 2.0,
        input_max_x_,
        deadzone_x_,
        max_linear_x_);

      gripper.data = gripper_open - gripper_close;
    }

    target_pose_.header.stamp = current_time;
    target_pose_.header.frame_id = target_pose_frame_;
    pose_pub_->publish(target_pose_);
    gripper_pub_->publish(gripper);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyArmCartesianNode>());
  rclcpp::shutdown();
  return 0;
}
