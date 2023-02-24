#ifndef GODOT_ROBOT_INTERFACE_H
#define GODOT_ROBOT_INTERFACE_H

#include "rclcpp/client.hpp"
#include "std_msgs/msg/detail/int64__struct.hpp"
#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/core/binder_common.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/int64.hpp>
// #include <lifecu

class RobotInterface : public godot::RefCounted {
  GDCLASS(RobotInterface, godot::RefCounted);

public:
  RobotInterface();
  
  int get_ammo();

protected:
  static void _bind_methods();
  rclcpp::Node::SharedPtr node_;
  int ammo_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr ammo_sub_;
  // rclcpp::Client<lifecycle_msgs::msg>
};

#endif