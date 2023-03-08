#ifndef GODOT_LIFECYCLE_MANAGER_H
#define GODOT_LIFECYCLE_MANAGER_H

#include "lifecycle_msgs/msg/detail/transition_event__struct.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/timer.hpp"
#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/core/binder_common.hpp>
#include <lifecycle_msgs/msg/transition_event.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>


class LifecycleTransitionFuture : public godot::RefCounted {
  GDCLASS(LifecycleTransitionFuture, godot::RefCounted);

public:
  LifecycleTransitionFuture();
  bool is_success();
  bool is_success_;

protected:
  static void _bind_methods();
};


class LifecycleManager : public godot::RefCounted {
  GDCLASS(LifecycleManager, godot::RefCounted);

public:
  enum State {
    UNKNOWN,
    NODE_NOT_FOUND,
    UNCONFIGURED,
    INACTIVE,
    ACTIVE,
    FINALIZED,
  };

  enum Transition {
    CONFIGURE,
    ACTIVATE,
    DEACTIVATE,
    CLEANUP,
  };

  LifecycleManager();
  void init(godot::String target_node_name);
  State get_state();
  godot::Ref<LifecycleTransitionFuture> make_transition(Transition t);
  void request_state();

  static int get_transition_id(Transition t);
  static std::string get_transition_label(Transition t);

protected:
  void update_state(lifecycle_msgs::msg::State s);
  void timer_callback();

  static void _bind_methods();
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_cli_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state_cli_;
  rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr
      transition_event_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string target_node_name_;

  State state_;
};

VARIANT_ENUM_CAST(LifecycleManager::State);
VARIANT_ENUM_CAST(LifecycleManager::Transition);



#endif