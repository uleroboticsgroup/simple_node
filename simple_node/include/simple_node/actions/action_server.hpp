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

#ifndef ACTION_SERVER_HPP
#define ACTION_SERVER_HPP

#include <chrono>
#include <string>
#include <thread>

#include "rclcpp/callback_group.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include <rcl_action/action_server.h>

using namespace std::placeholders;

namespace simple_node {
namespace actions {

template <typename ActionT>
class ActionServer : public rclcpp_action::Server<ActionT> {

  using Goal = typename ActionT::Goal;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;

  using UserExecuteCallback = std::function<void(std::shared_ptr<GoalHandle>)>;
  using UserAcceptedCallback = std::function<void(std::shared_ptr<GoalHandle>)>;
  using UserCancelCallback = std::function<void()>;

public:
  ActionServer(rclcpp::Node *node, std::string action_name,
               UserExecuteCallback execute_callback,
               UserCancelCallback cancel_callback)

      : rclcpp_action::Server<ActionT>(
            node->get_node_base_interface(), node->get_node_clock_interface(),
            node->get_node_logging_interface(), action_name,
            rcl_action_server_get_default_options(),
            std::bind(&ActionServer::handle_goal, this, _1, _2),
            std::bind(&ActionServer::handle_cancel, this, _1),
            std::bind(&ActionServer::handle_accepted, this, _1)) {

    this->execute_callback = execute_callback;
    this->cancel_callback = cancel_callback;
    this->goal_handle = nullptr;
  }

  ActionServer(rclcpp::Node *node, std::string action_name,
               UserExecuteCallback execute_callback)

      : rclcpp_action::Server<ActionT>(
            node->get_node_base_interface(), node->get_node_clock_interface(),
            node->get_node_logging_interface(), action_name,
            rcl_action_server_get_default_options(),
            std::bind(&ActionServer::handle_goal, this, _1, _2),
            std::bind(&ActionServer::handle_cancel, this, _1),
            std::bind(&ActionServer::handle_accepted, this, _1)) {

    this->execute_callback = execute_callback;
    this->cancel_callback = nullptr;
    this->goal_handle = nullptr;
  }

  bool is_working() { return this->goal_handle != nullptr; }

protected:
  std::shared_ptr<GoalHandle> goal_handle;

  void execute_goal_handle(const std::shared_ptr<GoalHandle> goal_handle) {
    std::thread{std::bind(&ActionServer::handle_execute, this, _1), goal_handle}
        .detach();
  }

private:
  UserExecuteCallback execute_callback;
  UserCancelCallback cancel_callback;
  std::unique_ptr<std::thread> cancel_thread;
  std::mutex handle_accepted_mtx;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                          std::shared_ptr<const Goal> goal) {
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {

    const std::lock_guard<std::mutex> lock(this->handle_accepted_mtx);

    if (this->goal_handle != nullptr && this->goal_handle->is_active()) {
      this->goal_handle->abort(std::make_shared<typename ActionT::Result>());
    }

    this->goal_handle = goal_handle;

    this->execute_goal_handle(goal_handle);
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) {
    (void)goal_handle;

    this->cancel_thread = std::make_unique<std::thread>(this->cancel_callback);

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_execute(const std::shared_ptr<GoalHandle> goal_handle) {
    this->cancel_thread = nullptr;
    this->execute_callback(this->goal_handle);
    this->goal_handle = nullptr;

    if (this->cancel_thread != nullptr) {
      this->cancel_thread->join();
    }
  }
};
} // namespace actions
} // namespace simple_node
#endif
