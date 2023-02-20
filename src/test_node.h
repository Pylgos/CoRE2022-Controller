#ifndef GODOT_TEST_NODE_H
#define GODOT_TEST_NODe_H

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/core/binder_common.hpp>


class TestNode : public godot::Node {
	GDCLASS(TestNode, godot::Node);

public:
	TestNode();
	~TestNode();

  void _notification(real_t delta);

protected:
	static void _bind_methods();

private:

};


#endif