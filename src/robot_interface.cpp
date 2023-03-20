#include "robot_interface.h"
#include "godot_cpp/classes/engine.hpp"
#include "godot_cpp/core/class_db.hpp"
#include "godot_cpp/core/object.hpp"
#include "godot_cpp/variant/utility_functions.hpp"
#include "rclcpp/create_subscription.hpp"
#include "rclcpp/qos.hpp"
#include "ros_node.h"
#include "std_msgs/msg/detail/float64__struct.hpp"
#include <functional>

using namespace godot;
using namespace std;
using namespace chrono_literals;
using namespace placeholders;

using geometry_msgs::msg::Twist;
using std_srvs::srv::SetBool;
using std_msgs::msg::Float64;


RobotInterface::RobotInterface()
    : ammo_{0},
      arm_lift_cmd_{0},
      arm_grabber_cmd_{0},
      camera_lift_cmd_{0},
      fire_command_{false},
      aps_enabled_{false},
      control_enabled_{true} {

  node_ = RosNode::get_singleton()->get_impl();
  timer_ = node_->create_wall_timer(33ms, bind(&RobotInterface::timer_callback, this));
  // target_vel_pub_ = node_->create_publisher<Twist>("target_vel", rclcpp::SystemDefaultsQoS());
  // camera_angle_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3>("camera_angle", rclcpp::SystemDefaultsQoS());
  // arm_lift_cmd_pub_ = node_->create_publisher<Float64>("arm_lift_cmd", rclcpp::SystemDefaultsQoS());
  // arm_grabber_cmd_pub_ = node_->create_publisher<Float64>("arm_grabber_cmd", rclcpp::SystemDefaultsQoS());
  // camera_lift_cmd_pub_ = node_->create_publisher<Float64>("camera_lift_cmd", rclcpp::SystemDefaultsQoS());

  ammo_sub_ = node_->create_subscription<std_msgs::msg::Int64>("ammo", rclcpp::SystemDefaultsQoS().reliable().transient_local(), [this](std_msgs::msg::Int64 msg){
    auto prev_ammo = ammo_;
    ammo_ = msg.data;
    if (ammo_ != prev_ammo) {
      emit_signal("ammo_changed");
    }
  });

  // set_fire_command_cli_ = node_->create_client<SetBool>("set_fire_cmd");
  // set_fire_command(false);

  // set_aps_enable_cli_ = node_->create_client<SetBool>("enable_aps");
  // set_aps_enabled(false);
}

void RobotInterface::set_control(bool enable) {
  // control_enabled_ = enable;
}

void RobotInterface::set_target_velocity(godot::Vector2 linear, real_t angular) {
//   target_vel_.linear.x = linear.x;
//   target_vel_.linear.y = linear.y;
//   target_vel_.angular.z = angular;
}

void RobotInterface::set_camera_angle(real_t pitch, real_t yaw) {
  // camera_angle_.y = pitch;
  // camera_angle_.z = Math::wrapf(yaw, -M_PI, M_PI);
  // emit_signal("camera_angle_changed");
}

real_t RobotInterface::get_camera_pitch() {
  return camera_angle_.y;
}

real_t RobotInterface::get_camera_yaw() {
  return camera_angle_.z;
}

void RobotInterface::set_arm_grabber_command(real_t command) {
  arm_grabber_cmd_ = command;
}

void RobotInterface::set_arm_lift_command(real_t command) {
  arm_lift_cmd_ = command;
}

void RobotInterface::set_camera_lift_command(real_t command) {
  camera_lift_cmd_ = command;
}

real_t RobotInterface::get_arm_grabber_command() {
  return arm_grabber_cmd_;
}

real_t RobotInterface::get_arm_lift_command() {
  return arm_lift_cmd_;
}

real_t RobotInterface::get_camera_lift_command() {
  return camera_lift_cmd_;
}

void RobotInterface::set_fire_command(bool enable) {
  // auto req = make_shared<SetBool::Request>();
  // fire_command_ = enable;
  // req->data = fire_command_;
  // set_fire_command_cli_->async_send_request(req, [](rclcpp::Client<SetBool>::SharedFuture fut) {
  //   if (!fut.valid() || !fut.get()->success) {
  //     UtilityFunctions::printerr("set_fire_command request failed");
  //   }
  // });
  // emit_signal("fire_command_changed");
}

void RobotInterface::set_aps_enabled(bool enable) {
  // auto req = make_shared<SetBool::Request>();
  // aps_enabled_ = enable;
  // req->data = aps_enabled_;
  // set_aps_enable_cli_->async_send_request(req, [](rclcpp::Client<SetBool>::SharedFuture fut) {
  //   if (!fut.valid() || !fut.get()->success) {
  //     UtilityFunctions::printerr("set_aps_enable request failed");
  //   }
  // });
  // emit_signal("aps_state_changed");
}

bool RobotInterface::get_aps_enabled() {
  return aps_enabled_;
}

bool RobotInterface::get_fire_command() {
  return fire_command_;
}

void RobotInterface::_bind_methods() {
  ClassDB::bind_method(D_METHOD("get_ammo"), &RobotInterface::get_ammo);
  ClassDB::bind_method(D_METHOD("set_target_velocity"), &RobotInterface::set_target_velocity);
  ClassDB::bind_method(D_METHOD("set_control"), &RobotInterface::set_control);
  
  ClassDB::bind_method(D_METHOD("set_camera_angle"), &RobotInterface::set_camera_angle);
  ClassDB::bind_method(D_METHOD("get_camera_pitch"), &RobotInterface::get_camera_pitch);
  ClassDB::bind_method(D_METHOD("get_camera_yaw"), &RobotInterface::get_camera_yaw);
  
  ClassDB::bind_method(D_METHOD("set_fire_command"), &RobotInterface::set_fire_command);
  ClassDB::bind_method(D_METHOD("get_fire_command"), &RobotInterface::get_fire_command);
  
  ClassDB::bind_method(D_METHOD("get_arm_grabber_command"), &RobotInterface::get_arm_grabber_command);
  ClassDB::bind_method(D_METHOD("set_arm_grabber_command"), &RobotInterface::set_arm_grabber_command);
  
  ClassDB::bind_method(D_METHOD("get_arm_lift_command"), &RobotInterface::get_arm_lift_command);
  ClassDB::bind_method(D_METHOD("set_arm_lift_command"), &RobotInterface::set_arm_lift_command);

  ClassDB::bind_method(D_METHOD("get_camera_lift_command"), &RobotInterface::get_camera_lift_command);
  ClassDB::bind_method(D_METHOD("set_camera_lift_command"), &RobotInterface::set_camera_lift_command);

  ClassDB::bind_method(D_METHOD("get_aps_enabled"), &RobotInterface::get_aps_enabled);
  ClassDB::bind_method(D_METHOD("set_aps_enabled"), &RobotInterface::set_aps_enabled);

  ADD_SIGNAL(MethodInfo("ammo_changed"));
  ADD_SIGNAL(MethodInfo("camera_angle_changed"));
  ADD_SIGNAL(MethodInfo("fire_command_changed"));
  ADD_SIGNAL(MethodInfo("aps_state_changed"));
}

int RobotInterface::get_ammo() {
  return ammo_;
}

void RobotInterface::timer_callback() {
  if (!control_enabled_) return;

  // target_vel_pub_->publish(target_vel_);
  // camera_angle_pub_->publish(camera_angle_);
  // arm_grabber_cmd_pub_->publish(Float64().set__data(arm_grabber_cmd_));
  // arm_lift_cmd_pub_->publish(Float64().set__data(arm_lift_cmd_));
  // camera_lift_cmd_pub_->publish(Float64().set__data(camera_lift_cmd_));
}
