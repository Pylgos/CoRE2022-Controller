#include "ros_executor.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/global_constants.hpp>
#include <godot_cpp/classes/label.hpp>
#include <godot_cpp/variant/utility_functions.hpp>


void RosExecutor::_bind_methods() {
	// ClassDB::bind_method(D_METHOD("get_id"), &ExampleRef::get_id);
}

RosExecutor::RosExecutor() {
	UtilityFunctions::print("RosExecutor created");
}

RosExecutor::~RosExecutor() {
	UtilityFunctions::print("RosExecutor destroyed");
}