/* godot-cpp integration testing project.
 *
 * This is free and unencumbered software released into the public domain.
 */

#include "register_types.h"

#include <cstdlib>
#include <gdextension_interface.h>

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/core/memory.hpp>
#include <godot_cpp/godot.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/window.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <random>

#include "laserscan_visualizer.h"
#include "rclcpp/init_options.hpp"
#include "rclcpp/utilities.hpp"
#include "ros_node.h"
#include "src/lifecycle_manager.h"
#include "src/robot_interface.h"


using namespace godot;


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


void initialize_ros_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}

	const char* dummy_args[1] = { "dummy" };
	rclcpp::init(1, dummy_args, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
	
	ClassDB::register_class<RosNode>();
	ClassDB::register_class<LaserScanVisualizer>();
	ClassDB::register_class<LifecycleTransitionFuture>();
	ClassDB::register_class<LifecycleManager>();
	ClassDB::register_class<RobotInterface>();

	Engine::get_singleton()->register_singleton("GlobalRosNode", memnew(RosNode));
	std::string global_node_name = "ui_node_";
	global_node_name += random_string(5);
	RosNode::get_singleton()->init(global_node_name.c_str());
	
	Engine::get_singleton()->register_singleton("Robot", memnew(RobotInterface));
}

void uninitialize_ros_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}

	Engine::get_singleton()->unregister_singleton("Robot");
	Engine::get_singleton()->unregister_singleton("GlobalRosNode");
	rclcpp::shutdown();
}

extern "C" {
// Initialization.
GDExtensionBool GDE_EXPORT ros_library_init(const GDExtensionInterface *p_interface, GDExtensionClassLibraryPtr p_library, GDExtensionInitialization *r_initialization) {
	godot::GDExtensionBinding::InitObject init_obj(p_interface, p_library, r_initialization);

	init_obj.register_initializer(initialize_ros_module);
	init_obj.register_terminator(uninitialize_ros_module);
	init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);

	return init_obj.init();
}
}
