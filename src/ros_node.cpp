#include "ros_node.h"
#include "godot_cpp/core/class_db.hpp"
#include "rclcpp/executors.hpp"
#include <godot_cpp/classes/engine.hpp>

using namespace godot;

RosNode::RosNode() {
  if (singleton_ == nullptr) {
    singleton_ = this;
  }
}

void RosNode::_bind_methods() {
  ClassDB::bind_method(D_METHOD("init"), &RosNode::init);
  ClassDB::bind_method(D_METHOD("spin_some"), &RosNode::spin_some);
}

RosNode *RosNode::get_singleton() {
  return singleton_;
}

void RosNode::init(String node_name) {
  node_ = std::make_shared<rclcpp::Node>(node_name.ascii().get_data());
}

void RosNode::spin_some() {
  rclcpp::spin_some(node_);
}

RosNode* RosNode::singleton_ = nullptr;
