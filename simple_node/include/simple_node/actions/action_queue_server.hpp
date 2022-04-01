
#ifndef ACTION_QUEUE_SERVER_HPP
#define ACTION_QUEUE_SERVER_HPP

#include <queue>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "simple_node/actions/action_server.hpp"

namespace simple_node {
namespace actions {

template <typename ActionT>
class ActionQueueServer : public ActionServer<ActionT> {

  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
  using ExecuteCallback = std::function<void(std::shared_ptr<GoalHandle>)>;
  using CancelCallback = std::function<void()>;

public:
  ActionQueueServer(rclcpp::Node *node, std::string action_name,
                    ExecuteCallback execute_callback,
                    CancelCallback cancel_callback)
      : ActionServer<ActionT>(
            node, action_name,
            std::bind(&ActionQueueServer::queue_handle_execute, this, _1),
            std::bind(&ActionQueueServer::queue_handle_accepted, this, _1),
            cancel_callback) {

    this->queue_execute_callback = execute_callback;
  }

  ActionQueueServer(rclcpp::Node *node, std::string action_name,
                    ExecuteCallback execute_callback)
      : ActionServer<ActionT>(
            node, action_name,
            std::bind(&ActionQueueServer::queue_handle_execute, this, _1),
            std::bind(&ActionQueueServer::queue_handle_accepted, this, _1)) {

    this->queue_execute_callback = execute_callback;
  }

private:
  std::mutex queue_mtx;
  std::queue<std::shared_ptr<GoalHandle>> goal_queue;
  ExecuteCallback queue_execute_callback;

  void queue_handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {

    const std::lock_guard<std::mutex> lock(this->queue_mtx);

    if (this->goal_handle != nullptr) {
      this->goal_queue.push(goal_handle);
    } else {
      this->goal_handle = goal_handle;
      this->execute_goal_handle(goal_handle);
    }
  }

  void queue_handle_execute(const std::shared_ptr<GoalHandle> goal_handle) {

    this->queue_execute_callback(goal_handle);

    const std::lock_guard<std::mutex> lock(this->queue_mtx);

    if (!this->goal_queue.empty()) {
      this->goal_handle = this->goal_queue.front();
      this->goal_queue.pop();

      this->execute_goal_handle(this->goal_handle);

    } else {
      this->goal_handle = nullptr;
    }
  }
};
} // namespace actions
} // namespace simple_node
#endif
