#ifndef GODOT_ROS_NODE_H
#define GODOT_ROS_NODe_H

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/core/binder_common.hpp>
#include <rclcpp/node.hpp>


class RosNode : public godot::RefCounted {
	GDCLASS(RosNode, godot::RefCounted);

public:
	RosNode();
	~RosNode();

protected:
	static void _bind_methods();
  rclcpp::Node::SharedPtr node_;

private:

};


#endif