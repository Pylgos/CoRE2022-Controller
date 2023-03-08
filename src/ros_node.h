#ifndef GODOT_ROSNODE_H
#define GODOT_ROSNODE_H

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/core/binder_common.hpp>
#include <rclcpp/node.hpp>

class RosNode : public godot::RefCounted {
  GDCLASS(RosNode, godot::RefCounted);

public:
  RosNode();
  static RosNode *get_singleton();
  rclcpp::Node::SharedPtr get_impl() { return node_; }

  void init(godot::String node_name);
  void spin_some();

  static bool ok();

protected:
  static RosNode* singleton_;
  static void _bind_methods();
  rclcpp::Node::SharedPtr node_;
};

#endif