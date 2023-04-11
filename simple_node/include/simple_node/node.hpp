
#ifndef SIMPLE_NODE_HPP
#define SIMPLE_NODE_HPP

#include <memory>
#include <string>
#include <thread>

#include "rclcpp/executor.hpp"
#include "rclcpp/rclcpp.hpp"

#include "simple_node/actions/action_client.hpp"
#include "simple_node/actions/action_queue_server.hpp"
#include "simple_node/actions/action_single_server.hpp"

namespace simple_node {

class Node : public rclcpp::Node {

public:
  Node(std::string name);
  Node(std::string name, rclcpp::Executor *executor);
  Node(std::string name, std::string _namespace);
  Node(std::string name, std::string _namespace, rclcpp::Executor *executor);
  ~Node();

  void join_spin();

  template <typename ActionT>
  typename std::shared_ptr<actions::ActionClient<ActionT>> create_action_client(
      std::string action_name,
      std::function<
          void(typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr,
               const std::shared_ptr<const typename ActionT::Feedback>)>
          feedback_cb = nullptr) {

    rclcpp::CallbackGroup::SharedPtr group = nullptr;

    std::shared_ptr<actions::ActionClient<ActionT>> action_client(
        new actions::ActionClient<ActionT>(this, action_name, feedback_cb),
        this->create_action_deleter<rclcpp_action::Client<ActionT>>(group));

    this->get_node_waitables_interface()->add_waitable(action_client, group);
    return action_client;
  }

  template <typename ActionT>
  typename std::shared_ptr<actions::ActionSingleServer<ActionT>>
  create_action_server(
      std::string action_name,
      std::function<
          void(std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>)>
          execute_callback) {

    rclcpp::CallbackGroup::SharedPtr group = nullptr;

    std::shared_ptr<actions::ActionSingleServer<ActionT>> action_server(
        new actions::ActionSingleServer<ActionT>(this, action_name,
                                                 execute_callback),
        this->create_action_deleter<rclcpp_action::Server<ActionT>>(group));

    this->get_node_waitables_interface()->add_waitable(action_server, group);
    return action_server;
  }

  template <typename ActionT>
  typename std::shared_ptr<actions::ActionQueueServer<ActionT>>
  create_action_queue_server(
      std::string action_name,
      std::function<
          void(std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>)>
          execute_callback) {

    rclcpp::CallbackGroup::SharedPtr group = nullptr;

    std::shared_ptr<actions::ActionQueueServer<ActionT>> action_server(
        new actions::ActionQueueServer<ActionT>(this, action_name,
                                                execute_callback),
        this->create_action_deleter<rclcpp_action::Server<ActionT>>(group));

    this->get_node_waitables_interface()->add_waitable(action_server, group);
    return action_server;
  }

private:
  rclcpp::Executor *executor;
  std::thread *spin_thread;

  void run_executor();

  template <typename TypeT>
  std::function<void(TypeT *ptr)>
  create_action_deleter(rclcpp::CallbackGroup::SharedPtr group) {

    rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr
        node_waitables_interface = this->get_node_waitables_interface();

    std::weak_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> weak_node =
        node_waitables_interface;
    std::weak_ptr<rclcpp::CallbackGroup> weak_group = group;
    bool group_is_null = (nullptr == group.get());

    auto deleter = [weak_node, weak_group, group_is_null](TypeT *ptr) {
      if (nullptr == ptr) {
        return;
      }
      auto shared_node = weak_node.lock();
      if (shared_node) {
        // API expects a shared pointer, give it one with a deleter that does
        // nothing.
        std::shared_ptr<TypeT> fake_shared_ptr(ptr, [](TypeT *) {});

        if (group_is_null) {
          // Was added to default group
          shared_node->remove_waitable(fake_shared_ptr, nullptr);
        } else {
          // Was added to a specific group
          auto shared_group = weak_group.lock();
          if (shared_group) {
            shared_node->remove_waitable(fake_shared_ptr, shared_group);
          }
        }
      }
      delete ptr;
    };

    return deleter;
  }
};

} // namespace simple_node

#endif
