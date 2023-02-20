#ifndef GODOT_LASERSCAN_VISUALIZER_H
#define GODOT_LASERSCAN_VISUALIZER_H

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/immediate_mesh.hpp>
#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/core/binder_common.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>


class LaserScanVisualizer : public godot::MeshInstance3D {
	GDCLASS(LaserScanVisualizer, godot::MeshInstance3D);

public:
	LaserScanVisualizer();

  void _notification(int p_what);

protected:
	static void _bind_methods();
  godot::Ref<godot::ImmediateMesh> mesh_;
	rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
	sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
	void update_mesh();
	void draw_octahedron(godot::Vector3 pos, real_t size, godot::Color color);
	void setup_subscription(std::string topic_name);
	void set_topic_name(godot::String topic_name);
	godot::String get_topic_name();

	// void set_material(godot::Ref<godot::Material> material);
	// godot::Ref<godot::Material> get_material();

private:
	godot::String topic_name_;
	godot::Ref<godot::StandardMaterial3D> material_; 

};


#endif