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
// #include <lifecu

class RobotInterface : public godot::RefCounted {
  GDCLASS(RobotInterface, godot::RefCounted);

public:
  RobotInterface();
  
  int get_ammo();
  void set_target_velocity(godot::Vector2 linear, real_t angular);
  void set_camera_angle(real_t pitch, real_t yaw);

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
};

#endif