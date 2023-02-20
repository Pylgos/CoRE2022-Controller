#include "ros_node.h"
#include <memory>
#include <godot_cpp/classes/engine.hpp>


using namespace godot;


RosNode::RosNode() {
  rclcpp::NodeOptions opts;
  node_ = std::make_shared<rclcpp::Node>("godot_ros", opts);
}

RosNode::~RosNode() {
}

void RosNode::_bind_methods() {

}
