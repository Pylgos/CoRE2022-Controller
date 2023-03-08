#include "robot_interface.h"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/detail/vector3__struct.hpp"
#include "godot_cpp/classes/engine.hpp"
#include "godot_cpp/core/class_db.hpp"
#include "godot_cpp/core/object.hpp"
#include "godot_cpp/variant/utility_functions.hpp"
#include "rclcpp/create_subscription.hpp"
#include "rclcpp/qos.hpp"
#include "ros_node.h"
#include "std_msgs/msg/detail/int64__struct.hpp"
#include <functional>

using namespace godot;
using namespace std;
using namespace chrono_literals;
using namespace placeholders;

using geometry_msgs::msg::Twist;


RobotInterface::RobotInterface() : ammo_{0}, control_enabled_{true} {
  node_ = RosNode::get_singleton()->get_impl();
  timer_ = node_->create_wall_timer(33ms, bind(&RobotInterface::timer_callback, this));
  target_vel_pub_ = node_->create_publisher<Twist>("target_vel", rclcpp::SystemDefaultsQoS());
  camera_angle_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3>("camera_angle", rclcpp::SystemDefaultsQoS());

  ammo_sub_ = node_->create_subscription<std_msgs::msg::Int64>("ammo", rclcpp::SystemDefaultsQoS().reliable().transient_local(), [this](std_msgs::msg::Int64 msg){
    auto prev_ammo = ammo_;
    ammo_ = msg.data;
    if (ammo_ != prev_ammo) {
      emit_signal("ammo_changed");
    }
  });
}

void RobotInterface::set_control(bool enable) {
  control_enabled_ = enable;
}

void RobotInterface::set_target_velocity(godot::Vector2 linear, real_t angular) {
  target_vel_.linear.x = linear.x;
  target_vel_.linear.y = linear.y;
  target_vel_.angular.z = angular;
}

void RobotInterface::set_camera_angle(real_t pitch, real_t yaw) {
  camera_angle_.y = pitch;
  camera_angle_.z = yaw;
  emit_signal("camera_angle_changed");
}

real_t RobotInterface::get_camera_pitch() {
  return camera_angle_.y;
}

real_t RobotInterface::get_camera_yaw() {
  return camera_angle_.z;
}

void RobotInterface::_bind_methods() {
  ClassDB::bind_method(D_METHOD("get_ammo"), &RobotInterface::get_ammo);
  ClassDB::bind_method(D_METHOD("set_target_velocity"), &RobotInterface::set_target_velocity);
  ClassDB::bind_method(D_METHOD("set_camera_angle"), &RobotInterface::set_camera_angle);
  ClassDB::bind_method(D_METHOD("set_control"), &RobotInterface::set_control);
  ClassDB::bind_method(D_METHOD("get_camera_pitch"), &RobotInterface::get_camera_pitch);
  ClassDB::bind_method(D_METHOD("get_camera_yaw"), &RobotInterface::get_camera_yaw);

  ADD_SIGNAL(MethodInfo("ammo_changed"));
  ADD_SIGNAL(MethodInfo("camera_angle_changed"));
}

int RobotInterface::get_ammo() {
  return ammo_;
}

void RobotInterface::timer_callback() {
  if (!control_enabled_) return;

  target_vel_pub_->publish(target_vel_);
  camera_angle_pub_->publish(camera_angle_);
}
