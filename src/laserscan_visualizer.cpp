#include "laserscan_visualizer.h"
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/node.hpp>
#include <rclcpp/executors.hpp>
#include <random>


using namespace godot;
using sensor_msgs::msg::LaserScan;


static std::string random_string(std::string::size_type length)
{
    static auto& chrs = "0123456789"
        "abcdefghijklmnopqrstuvwxyz"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ";

    thread_local static std::mt19937 rg{std::random_device{}()};
    thread_local static std::uniform_int_distribution<std::string::size_type> pick(0, sizeof(chrs) - 2);

    std::string s;

    s.reserve(length);

    while(length--)
        s += chrs[pick(rg)];

    return s;
}


LaserScanVisualizer::LaserScanVisualizer() {
  node_ = std::make_shared<rclcpp::Node>("laser_scan_visualizer_" + random_string(5));
  set_topic_name("scan");
}

void LaserScanVisualizer::setup_subscription(std::string topic_name) {
  sub_ = node_->create_subscription<LaserScan>(topic_name, rclcpp::SensorDataQoS(), [this](LaserScan::ConstSharedPtr scan){
    scan_ = scan;
  });
}

void LaserScanVisualizer::set_topic_name(String topic_name) {
  topic_name_ = topic_name;
  setup_subscription(topic_name_.ascii().get_data());
}

String LaserScanVisualizer::get_topic_name() {
  return topic_name_;
} 


void LaserScanVisualizer::_bind_methods() {
  ClassDB::bind_method(D_METHOD("_notification"), &LaserScanVisualizer::_notification);

  ClassDB::bind_method(D_METHOD("set_topic_name"), &LaserScanVisualizer::set_topic_name);
  ClassDB::bind_method(D_METHOD("get_topic_name"), &LaserScanVisualizer::get_topic_name);
  ADD_PROPERTY(PropertyInfo(Variant::STRING, "topic_name"), "set_topic_name", "get_topic_name");

  // ClassDB::bind_method(D_METHOD("set_material"), &LaserScanVisualizer::set_material);
  // ClassDB::bind_method(D_METHOD("get_material"), &LaserScanVisualizer::get_material);
  // ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "material", PROPERTY_HINT_RESOURCE_TYPE, "BaseMaterial3D,ShaderMaterial"), "set_material", "get_material");
}

// void LaserScanVisualizer::set_material(godot::Ref<godot::Material> material) { material_ = material; }
// godot::Ref<godot::Material> LaserScanVisualizer::get_material() { return material_; }


void LaserScanVisualizer::draw_octahedron(Vector3 pos, real_t size, Color color) {
  auto half_size = size / 2;
  auto top = pos + Vector3(0, half_size, 0);
  auto btm = pos + Vector3(0, -half_size, 0);
  auto fr = pos + Vector3(half_size, 0, -half_size);
  auto fl = pos + Vector3(-half_size, 0, -half_size);
  auto rr = pos + Vector3(half_size, 0, half_size);
  auto rl = pos + Vector3(-half_size, 0, half_size);

  mesh_->surface_set_color(color);
  mesh_->surface_add_vertex(top);
  mesh_->surface_add_vertex(fr);
  mesh_->surface_add_vertex(fl);

  mesh_->surface_add_vertex(top);
  mesh_->surface_add_vertex(fl);
  mesh_->surface_add_vertex(rl);

  mesh_->surface_add_vertex(top);
  mesh_->surface_add_vertex(rl);
  mesh_->surface_add_vertex(rr);

  mesh_->surface_add_vertex(top);
  mesh_->surface_add_vertex(rr);
  mesh_->surface_add_vertex(fr);

  mesh_->surface_add_vertex(btm);
  mesh_->surface_add_vertex(fl);
  mesh_->surface_add_vertex(fr);

  mesh_->surface_add_vertex(btm);
  mesh_->surface_add_vertex(rl);
  mesh_->surface_add_vertex(fl);

  mesh_->surface_add_vertex(btm);
  mesh_->surface_add_vertex(rr);
  mesh_->surface_add_vertex(rl);

  mesh_->surface_add_vertex(btm);
  mesh_->surface_add_vertex(fr);
  mesh_->surface_add_vertex(rr);
}


void LaserScanVisualizer::update_mesh() {
  if (scan_ == nullptr) return;

  mesh_->clear_surfaces();
  mesh_->surface_begin(ImmediateMesh::PrimitiveType::PRIMITIVE_TRIANGLES);
  for (size_t i = 0; i < scan_->ranges.size(); i++) {
    auto angle = scan_->angle_min + scan_->angle_increment * i;
    auto range = scan_->ranges[i];
    auto x = range * cos(angle);
    auto y = range * sin(angle);

    if (std::isnan(x) || std::isnan(y) || std::isinf(x) || std::isinf(y)) continue;

    draw_octahedron(Vector3(x, y, 0), 0.05, Color(1, 0, 0));
  }
  mesh_->surface_end();
}

void LaserScanVisualizer::_notification(int p_what) {
  switch (p_what) {
    case NOTIFICATION_READY: {
      mesh_.instantiate();
      material_.instantiate();
      material_->set_shading_mode(BaseMaterial3D::SHADING_MODE_UNSHADED);
      material_->set_flag(BaseMaterial3D::Flags::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
      set_material_override(material_);
      set_mesh(mesh_);
      set_physics_process_internal(true);
    } break;

    case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
      update_mesh();
      rclcpp::spin_some(node_);
    } break;
  }
}
