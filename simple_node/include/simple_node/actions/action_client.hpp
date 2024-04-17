// Copyright (C) 2023  Miguel Ángel González Santamarta

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef ACTION_CLIENT_HPP
#define ACTION_CLIENT_HPP

#include <condition_variable>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/client_goal_handle.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::placeholders;

namespace simple_node {
namespace actions {

template <typename ActionT>
class ActionClient : public rclcpp_action::Client<ActionT> {

  using SendGoalOptions =
      typename rclcpp_action::Client<ActionT>::SendGoalOptions;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;

  using Goal = typename ActionT::Goal;
  using Result = typename GoalHandle::Result::SharedPtr;

  using Feedback = typename ActionT::Feedback;
  using FeedbackCallback = std::function<void(
      typename GoalHandle::SharedPtr, const std::shared_ptr<const Feedback>)>;

public:
  ActionClient(rclcpp::Node *node, std::string action_name,
               FeedbackCallback feedback_cb = nullptr)
      : rclcpp_action::Client<ActionT>(
            node->get_node_base_interface(), node->get_node_graph_interface(),
            node->get_node_logging_interface(), action_name) {

    this->result = nullptr;
    this->status = rclcpp_action::ResultCode::UNKNOWN;

    this->goal_handle = nullptr;
    this->feedback_cb = feedback_cb;
  }

  ~ActionClient() {}

  void wait_for_result() {
    std::unique_lock<std::mutex> lock(this->action_done_mutex);
    this->action_done_cond.wait(lock);
  }

  Result get_result() { return this->result; }

  void send_goal(Goal goal, FeedbackCallback feedback_cb = nullptr) {

    this->goal_handle = nullptr;
    this->result = nullptr;
    this->set_status(rclcpp_action::ResultCode::UNKNOWN);

    FeedbackCallback _feedback_cb = this->feedback_cb;

    if (feedback_cb != nullptr) {
      _feedback_cb = feedback_cb;
    }

    auto send_goal_options = SendGoalOptions();

    send_goal_options.goal_response_callback =
        std::bind(&ActionClient::goal_response_callback, this, _1);

    send_goal_options.result_callback =
        std::bind(&ActionClient::result_callback, this, _1);

    send_goal_options.feedback_callback = _feedback_cb;

    this->async_send_goal(goal, send_goal_options);
  }

  void cancel_goal() {
    std::lock_guard<std::mutex> lock(this->goal_handle_mutex);
    if (this->goal_handle) {
      this->async_cancel_goal(this->goal_handle,
                              std::bind(&ActionClient::cancel_done, this));

      std::unique_lock<std::mutex> lock(this->action_done_mutex);
      this->action_done_cond.wait(lock);
    }
  }

  // **************
  // STATUS
  // **************
  rclcpp_action::ResultCode get_status() {
    const std::lock_guard<std::mutex> lock(this->status_mtx);
    return this->status;
  }

  bool is_succeeded() {
    return this->get_status() == rclcpp_action::ResultCode::SUCCEEDED;
  }

  bool is_canceled() {
    return this->get_status() == rclcpp_action::ResultCode::CANCELED;
  }

  bool is_aborted() {
    return this->get_status() == rclcpp_action::ResultCode::ABORTED;
  }

  bool is_working() {
    std::lock_guard<std::mutex> lock(this->goal_handle_mutex);
    return this->goal_handle != nullptr;
  }

  bool is_terminated() {
    return (this->is_succeeded() || this->is_canceled() || this->is_aborted());
  }

private:
  std::condition_variable action_done_cond;
  std::mutex action_done_mutex;

  std::condition_variable cancel_done_cond;
  std::mutex cancel_done_mutex;

  Result result;
  rclcpp_action::ResultCode status;
  std::mutex status_mtx;

  std::shared_ptr<GoalHandle> goal_handle;
  std::mutex goal_handle_mutex;

  FeedbackCallback feedback_cb;

  void set_status(rclcpp_action::ResultCode status) {
    const std::lock_guard<std::mutex> lock(this->status_mtx);
    this->status = status;
  }

  void
  goal_response_callback(const typename GoalHandle::SharedPtr &goal_handle) {
    std::lock_guard<std::mutex> lock(this->goal_handle_mutex);
    this->goal_handle = goal_handle;
  }

  void result_callback(const typename GoalHandle::WrappedResult &result) {
    this->result = result.result;
    this->set_status(result.code);

    std::lock_guard<std::mutex> lock(this->goal_handle_mutex);
    this->goal_handle = nullptr;

    this->action_done_cond.notify_one();
  }

  void cancel_done() { this->action_done_cond.notify_all(); }
};

} // namespace actions
} // namespace simple_node
#endif
