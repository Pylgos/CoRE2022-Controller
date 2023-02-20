/* godot-cpp integration testing project.
 *
 * This is free and unencumbered software released into the public domain.
 */

#ifndef GODOT_ROS_EXECUTOR_H
#define GODOT_ROS_EXECUTOR_H

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/core/binder_common.hpp>
#include <rclcpp/executor.hpp>

using namespace godot;

class RosExecutor : public RefCounted {
	GDCLASS(RosExecutor, RefCounted);

private:

protected:
	static void _bind_methods();

public:
	RosExecutor();
	~RosExecutor();

};


#endif // _CLASS_H
