
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
               UserAcceptedCallback accepted_callback,
               UserCancelCallback cancel_callback)

      : rclcpp_action::Server<ActionT>(
            node->get_node_base_interface(), node->get_node_clock_interface(),
            node->get_node_logging_interface(), action_name,
            rcl_action_server_get_default_options(),
            std::bind(&ActionServer::handle_goal, this, _1, _2),
            std::bind(&ActionServer::handle_cancel, this, _1),
            accepted_callback) {

    this->execute_callback = execute_callback;
    this->cancel_callback = cancel_callback;
    this->server_canceled = false;
    this->goal_handle = nullptr;
  }

  ActionServer(rclcpp::Node *node, std::string action_name,
               UserExecuteCallback execute_callback,
               UserAcceptedCallback accepted_callback)

      : rclcpp_action::Server<ActionT>(
            node->get_node_base_interface(), node->get_node_clock_interface(),
            node->get_node_logging_interface(), action_name,
            rcl_action_server_get_default_options(),
            std::bind(&ActionServer::handle_goal, this, _1, _2),
            std::bind(&ActionServer::handle_cancel, this, _1),
            accepted_callback) {

    this->execute_callback = execute_callback;
    this->cancel_callback = nullptr;
    this->server_canceled = false;
    this->goal_handle = nullptr;
  }

  bool is_working() { return this->goal_handle != nullptr; }

  bool is_canceled() { return this->server_canceled; }

  void wait_for_canceling() {
    if (this->server_canceled && this->goal_handle != nullptr) {
      while (!this->goal_handle->is_canceling()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    }
  }

protected:
  std::shared_ptr<GoalHandle> goal_handle;

  void execute_goal_handle(const std::shared_ptr<GoalHandle> goal_handle) {

    std::thread{std::bind(&ActionServer::handle_execute, this, _1), goal_handle}
        .detach();
  }

private:
  UserExecuteCallback execute_callback;
  UserCancelCallback cancel_callback;
  bool server_canceled;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                          std::shared_ptr<const Goal> goal) {

    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_execute(const std::shared_ptr<GoalHandle> goal_handle) {

    this->goal_handle = goal_handle;
    this->server_canceled = false;

    using namespace std::placeholders;
    std::thread{std::bind(&ActionServer::execute, this, _1), goal_handle}
        .detach();

    while (this->goal_handle != nullptr) {
      if (this->goal_handle->is_canceling()) {

        if (this->cancel_callback != nullptr) {
          this->cancel_callback();
        }
        this->server_canceled = true;
        break;
      }
    }
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle) {
    this->execute_callback(goal_handle);
    this->goal_handle = nullptr;
  }
};
} // namespace actions
} // namespace simple_node
#endif
