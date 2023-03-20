#ifndef GODOT_ROBOT_INTERFACE_H
#define GODOT_ROBOT_INTERFACE_H

#include "godot_cpp/variant/vector2.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/timer.hpp"
#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/core/binder_common.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/int64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "geometry_msgs/msg/vector3.hpp"
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float64.hpp>

class RobotInterface : public godot::RefCounted {
  GDCLASS(RobotInterface, godot::RefCounted);

public:
  RobotInterface();
  
  int get_ammo();
  void set_target_velocity(godot::Vector2 linear, real_t angular);
  
  void set_camera_angle(real_t pitch, real_t yaw);
  real_t get_camera_pitch();
  real_t get_camera_yaw();
  
  void set_control(bool enable);

  void set_fire_command(bool enable);
  bool get_fire_command();

  void set_arm_lift_command(real_t command);
  real_t get_arm_lift_command();

  void set_arm_grabber_command(real_t command);
  real_t get_arm_grabber_command();

  void set_camera_lift_command(real_t command);
  real_t get_camera_lift_command();

  void set_aps_enabled(bool enable);
  bool get_aps_enabled();

protected:
  static void _bind_methods();
  
  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Node::SharedPtr node_;
  
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr ammo_sub_;
  int ammo_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr target_vel_pub_;
  geometry_msgs::msg::Twist target_vel_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr camera_angle_pub_;
  geometry_msgs::msg::Vector3 camera_angle_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr arm_lift_cmd_pub_;
  real_t arm_lift_cmd_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr arm_grabber_cmd_pub_;
  real_t arm_grabber_cmd_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr camera_lift_cmd_pub_;
  real_t camera_lift_cmd_;

  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_fire_command_cli_;
  bool fire_command_;

  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_aps_enable_cli_;
  bool aps_enabled_;

  bool control_enabled_;
};

#endif