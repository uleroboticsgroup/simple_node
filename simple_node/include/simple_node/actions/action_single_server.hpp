
#ifndef ACTION_SINGLE_SERVER_HPP
#define ACTION_SINGLE_SERVER_HPP

#include <chrono>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "simple_node/actions/action_server.hpp"

namespace simple_node {
namespace actions {

template <typename ActionT>
class ActionSingleServer : public ActionServer<ActionT> {

  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
  using ExecuteCallback = std::function<void(std::shared_ptr<GoalHandle>)>;
  using CancelCallback = std::function<void()>;

public:
  ActionSingleServer(rclcpp::Node *node, std::string action_name,
                     ExecuteCallback execute_callback,
                     CancelCallback cancel_callback)

      : ActionServer<ActionT>(
            node, action_name, execute_callback,
            std::bind(&ActionSingleServer::handle_accepted, this, _1),
            cancel_callback) {}

  ActionSingleServer(rclcpp::Node *node, std::string action_name,
                     ExecuteCallback execute_callback)

      : ActionServer<ActionT>(
            node, action_name, execute_callback,
            std::bind(&ActionSingleServer::handle_accepted, this, _1)) {}

private:
  std::mutex handle_accepted_mtx;

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {

    const std::lock_guard<std::mutex> lock(this->handle_accepted_mtx);

    if (this->goal_handle != nullptr && this->goal_handle->is_active()) {
      this->goal_handle->abort(std::make_shared<typename ActionT::Result>());
    }

    this->goal_handle = goal_handle;

    this->execute_goal_handle(goal_handle);
  }
};
} // namespace actions
} // namespace simple_node
#endif
