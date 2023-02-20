#include "test_node.h"
#include <memory>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/variant/utility_functions.hpp>


using namespace godot;


TestNode::TestNode() {
}

TestNode::~TestNode() {
}

void TestNode::_bind_methods() {
  ClassDB::bind_method(D_METHOD("_notification"), &TestNode::_notification);
}

void TestNode::_notification(real_t delta) {
  UtilityFunctions::print("yay");
}
