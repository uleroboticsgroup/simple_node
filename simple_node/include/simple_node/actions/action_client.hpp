
#ifndef ACTION_CLIENT_HPP
#define ACTION_CLIENT_HPP

#include <chrono>
#include <future>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/client_goal_handle.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <action_msgs/msg/goal_status.hpp>

namespace simple_node {
namespace actions {

template <typename ActionT>
class ActionClient : public rclcpp_action::Client<ActionT> {

  using Goal = typename ActionT::Goal;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  using Result = typename GoalHandle::Result::SharedPtr;

public:
  ActionClient(rclcpp::Node *node, std::string action_name)
      : rclcpp_action::Client<ActionT>(
            node->get_node_base_interface(), node->get_node_graph_interface(),
            node->get_node_logging_interface(), action_name) {

    this->status = action_msgs::msg::GoalStatus::STATUS_UNKNOWN;

    this->goal_handle = nullptr;
    this->goal_thread = nullptr;
    this->result = nullptr;
  }

  ~ActionClient() { delete this->goal_thread; }

  void wait_for_result() { this->goal_thread->join(); }

  Result get_result() { return this->result; }

  void send_goal(Goal goal) {
    this->set_status(action_msgs::msg::GoalStatus::STATUS_UNKNOWN);
    this->goal_thread = new std::thread(&ActionClient::_send_goal, this, goal);
  }

  bool cancel_goal() {
    if (this->goal_handle != nullptr) {
      return this->_cancel_goal();
    } else {
      return false;
    }
  }

  // **************
  // STATUS
  // **************
  int8_t get_status() {
    const std::lock_guard<std::mutex> lock(this->status_mtx);
    return this->status;
  }

  bool is_accepted() {
    return this->get_status() == action_msgs::msg::GoalStatus::STATUS_ACCEPTED;
  }

  bool is_executing() {
    return this->get_status() == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
  }

  bool is_canceling() {
    return this->get_status() == action_msgs::msg::GoalStatus::STATUS_CANCELING;
  }

  bool is_succeeded() {
    return this->get_status() == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED;
  }

  bool is_canceled() {
    return this->get_status() == action_msgs::msg::GoalStatus::STATUS_CANCELED;
  }

  bool is_aborted() {
    return this->get_status() == action_msgs::msg::GoalStatus::STATUS_ABORTED;
  }

  bool is_working() {
    return (this->is_executing() || this->is_canceling() ||
            this->is_accepted());
  }

  bool is_terminated() {
    return (this->is_succeeded() || this->is_canceled() || this->is_aborted());
  }

private:
  int8_t status;
  std::mutex status_mtx;
  std::thread *goal_thread;
  Result result;
  std::shared_ptr<GoalHandle> goal_handle;

  void set_status(int8_t status) {
    const std::lock_guard<std::mutex> lock(this->status_mtx);
    this->status = status;
  }

  void _send_goal(Goal goal) {

    this->result = nullptr;

    auto send_goal_future = this->async_send_goal(goal);

    // wait for acceptance
    while (send_goal_future.wait_for(std::chrono::seconds(1)) !=
           std::future_status::ready) {
    }

    // check acceptance
    this->goal_handle = send_goal_future.get();
    if (!this->goal_handle) {

      // change status
      if (this->is_canceled()) {
        return;
      }
      this->set_status(action_msgs::msg::GoalStatus::STATUS_ABORTED);
      return;
    }

    // change status
    if (this->is_canceled()) {
      return;
    }
    this->set_status(action_msgs::msg::GoalStatus::STATUS_ACCEPTED);

    // get result
    auto get_result_future = this->async_get_result(this->goal_handle);

    // change status
    if (this->is_canceled()) {
      return;
    }
    this->set_status(action_msgs::msg::GoalStatus::STATUS_EXECUTING);

    // wait for result
    while (get_result_future.wait_for(std::chrono::seconds(1)) !=
           std::future_status::ready) {
    }

    // change status
    if (this->is_canceled()) {
      return;
    }

    this->set_status((uint8_t)get_result_future.get().code);
    this->result = get_result_future.get().result;
  }

  bool _cancel_goal() {
    int8_t old_status = this->get_status();

    auto cancel_goal_future = this->async_cancel_goal(this->goal_handle);
    this->set_status(action_msgs::msg::GoalStatus::STATUS_CANCELING);

    while (cancel_goal_future.wait_for(std::chrono::seconds(1)) !=
           std::future_status::ready) {
    }

    auto result = cancel_goal_future.get();
    if (result->goals_canceling.size() == 0) {
      this->set_status(old_status);
      return false;
    }

    this->set_status(action_msgs::msg::GoalStatus::STATUS_CANCELED);

    return true;
  }
};
} // namespace actions
} // namespace simple_node
#endif
