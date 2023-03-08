#include "lifecycle_manager.h"
#include "godot_cpp/classes/engine.hpp"
#include "godot_cpp/core/class_db.hpp"
#include "godot_cpp/core/error_macros.hpp"
#include "godot_cpp/core/object.hpp"
#include "godot_cpp/variant/utility_functions.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/create_subscription.hpp"
#include "rclcpp/qos.hpp"
#include "robot_interface.h"
#include "ros_node.h"
#include <cassert>
#include <functional>
#include <memory>
#include <godot_cpp/core/error_macros.hpp>

using namespace std;
using namespace chrono_literals;
using namespace godot;
using lifecycle_msgs::srv::ChangeState;
using lifecycle_msgs::srv::GetState;
using lifecycle_msgs::msg::TransitionEvent;
using StateMsg = lifecycle_msgs::msg::State;
using TransitionMsg = lifecycle_msgs::msg::Transition;

LifecycleManager::LifecycleManager() : state_{UNKNOWN} {
  node_ = RosNode::get_singleton()->get_impl();
}

void LifecycleManager::init(String target_node_name) {
  string name = target_node_name.ascii().get_data();
  target_node_name_ = name;
  change_state_cli_ = node_->create_client<ChangeState>(name + "/change_state");
  get_state_cli_ = node_->create_client<GetState>(name + "/get_state");
  timer_ = node_->create_wall_timer(1s, bind(&LifecycleManager::timer_callback, this));
  
  transition_event_sub_ = node_->create_subscription<TransitionEvent>(name + "/transition_event", rclcpp::SystemDefaultsQoS().reliable(), [this](TransitionEvent::ConstSharedPtr msg){
    update_state(msg->goal_state);
  });

  request_state();
}

LifecycleManager::State LifecycleManager::get_state() { return state_; }

void LifecycleManager::update_state(StateMsg state) {
  auto prev_state = state_;
  switch (state.id) {
  case StateMsg::PRIMARY_STATE_ACTIVE: {
    state_ = ACTIVE;
  } break;
  case StateMsg::PRIMARY_STATE_INACTIVE: {
    state_ = INACTIVE;
  } break;
  case StateMsg::PRIMARY_STATE_UNCONFIGURED: {
    state_ = UNCONFIGURED;
  } break;
  case StateMsg::PRIMARY_STATE_FINALIZED: {
    state_ = FINALIZED;
  } break;
  case StateMsg::PRIMARY_STATE_UNKNOWN: {
    state_ = UNKNOWN;
  } break;
  default: {
    UtilityFunctions::push_error("Invalid state ", String(state.label.c_str()));
  } break;
  }
  if (prev_state != state_) {
    emit_signal("state_changed");
  }
}

void LifecycleManager::request_state() {
  auto req = std::make_shared<GetState::Request>();
  get_state_cli_->async_send_request(
      req, [this](rclcpp::Client<GetState>::SharedFuture fut) {
        if (!fut.valid()) {
          UtilityFunctions::push_error("Request failed");
          return;
        }
        update_state(fut.get()->current_state);
      });
}

int LifecycleManager::get_transition_id(Transition t) {
  switch (t) {
    case CONFIGURE: return TransitionMsg::TRANSITION_CONFIGURE;
    case ACTIVATE: return TransitionMsg::TRANSITION_ACTIVATE;
    case DEACTIVATE: return TransitionMsg::TRANSITION_DEACTIVATE;
    case CLEANUP: return TransitionMsg::TRANSITION_CLEANUP;
    default: ERR_FAIL_V_MSG(0, "Invalid transition");
  }
}

string LifecycleManager::get_transition_label(Transition t) {
  switch (t) {
    case CONFIGURE: return "configure";
    case ACTIVATE: return "activate";
    case DEACTIVATE: return "deactivate";
    case CLEANUP: return "cleanup";
    default: ERR_FAIL_V_MSG("invalid", "Invalid transition");
  }
}


static void transition_callback(Ref<LifecycleTransitionFuture> godot_fut, rclcpp::Client<ChangeState>::SharedFuture ros_fut) {
  if (!ros_fut.valid()) {
    godot_fut->is_success_ = false;
  } else if (ros_fut.get()->success) {
    godot_fut->is_success_ = true;
  } else {
    godot_fut->is_success_ = false;
  }
  
  godot_fut->emit_signal("completed");
}


Ref<LifecycleTransitionFuture> LifecycleManager::make_transition(Transition t) {
  Ref<LifecycleTransitionFuture> result = memnew(LifecycleTransitionFuture);
  auto req = std::make_shared<ChangeState::Request>();
  req->transition.id = get_transition_id(t);
  req->transition.label = get_transition_label(t);
  function<void(rclcpp::Client<ChangeState>::SharedFuture)> f = bind(transition_callback, result, placeholders::_1);
  change_state_cli_->async_send_request(req, f);
  return result;
}

void LifecycleManager::_bind_methods() {
  ClassDB::bind_method(D_METHOD("init"), &LifecycleManager::init);
  ClassDB::bind_method(D_METHOD("get_state"), &LifecycleManager::get_state);
  ClassDB::bind_method(D_METHOD("make_transition"), &LifecycleManager::make_transition);
  ADD_SIGNAL(MethodInfo("state_changed"));

  BIND_ENUM_CONSTANT(UNKNOWN);
  BIND_ENUM_CONSTANT(NODE_NOT_FOUND);
  BIND_ENUM_CONSTANT(UNCONFIGURED);
  BIND_ENUM_CONSTANT(INACTIVE);
  BIND_ENUM_CONSTANT(ACTIVE);
  BIND_ENUM_CONSTANT(FINALIZED);

  BIND_ENUM_CONSTANT(CONFIGURE);
  BIND_ENUM_CONSTANT(ACTIVATE);
  BIND_ENUM_CONSTANT(DEACTIVATE);
  BIND_ENUM_CONSTANT(CLEANUP);
}


void LifecycleManager::timer_callback() {
  auto node_names = node_->get_node_names();
  bool node_found = find(node_names.begin(), node_names.end(), target_node_name_) != node_names.end();
  if (!node_found) {
    if (state_ != NODE_NOT_FOUND) {
      state_ = NODE_NOT_FOUND;
      emit_signal("state_changed");
    }
  } else {
    request_state();
  }
}


LifecycleTransitionFuture::LifecycleTransitionFuture() : is_success_{false} {

}

bool LifecycleTransitionFuture::is_success() {
  return is_success_;
}

void LifecycleTransitionFuture::_bind_methods() {
  ClassDB::bind_method(D_METHOD("is_success"), &LifecycleTransitionFuture::is_success);
  ADD_SIGNAL(MethodInfo("completed"));
}
