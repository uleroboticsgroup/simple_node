
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

    std::shared_ptr<actions::ActionClient<ActionT>> action_client(
        new actions::ActionClient<ActionT>(this, action_name, feedback_cb),
        this->create_action_deleter<rclcpp_action::Client<ActionT>>());

    this->get_node_waitables_interface()->add_waitable(action_client,
                                                       this->group);
    return action_client;
  }

  template <typename ActionT>
  typename std::shared_ptr<actions::ActionSingleServer<ActionT>>
  create_action_server(
      std::string action_name,
      std::function<
          void(std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>)>
          execute_callback) {

    std::shared_ptr<actions::ActionSingleServer<ActionT>> action_server(
        new actions::ActionSingleServer<ActionT>(this, action_name,
                                                 execute_callback),
        this->create_action_deleter<rclcpp_action::Server<ActionT>>());

    this->get_node_waitables_interface()->add_waitable(action_server,
                                                       this->group);
    return action_server;
  }

  template <typename ActionT>
  typename std::shared_ptr<actions::ActionSingleServer<ActionT>>
  create_action_server(
      std::string action_name,
      std::function<
          void(std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>)>
          execute_callback,
      std::function<void()> cancel_callback) {

    std::shared_ptr<actions::ActionSingleServer<ActionT>> action_server(
        new actions::ActionSingleServer<ActionT>(
            this, action_name, execute_callback, cancel_callback),
        this->create_action_deleter<rclcpp_action::Server<ActionT>>());

    this->get_node_waitables_interface()->add_waitable(action_server,
                                                       this->group);
    return action_server;
  }

  template <typename ActionT>
  typename std::shared_ptr<actions::ActionQueueServer<ActionT>>
  create_action_queue_server(
      std::string action_name,
      std::function<
          void(std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>)>
          execute_callback) {

    std::shared_ptr<actions::ActionQueueServer<ActionT>> action_server(
        new actions::ActionQueueServer<ActionT>(this, action_name,
                                                execute_callback),
        this->create_action_deleter<rclcpp_action::Server<ActionT>>());

    this->get_node_waitables_interface()->add_waitable(action_server,
                                                       this->group);
    return action_server;
  }

  template <typename ActionT>
  typename std::shared_ptr<actions::ActionQueueServer<ActionT>>
  create_action_queue_server(
      std::string action_name,
      std::function<
          void(std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>)>
          execute_callback,
      std::function<void()> cancel_callback) {

    std::shared_ptr<actions::ActionQueueServer<ActionT>> action_server(
        new actions::ActionQueueServer<ActionT>(
            this, action_name, execute_callback, cancel_callback),
        this->create_action_deleter<rclcpp_action::Server<ActionT>>());

    this->get_node_waitables_interface()->add_waitable(action_server,
                                                       this->group);
    return action_server;
  }

  template <typename ServiceT>
  typename rclcpp::Client<ServiceT>::SharedPtr
  create_client(const std::string &service_name,
                const rmw_qos_profile_t &qos_profile) {
    return rclcpp::create_client<ServiceT>(
        node_base_, node_graph_, node_services_,
        rclcpp::extend_name_with_sub_namespace(service_name,
                                               this->get_sub_namespace()),
        qos_profile, this->group);
  }

  template <typename ServiceT, typename CallbackT>
  typename rclcpp::Service<ServiceT>::SharedPtr
  create_service(const std::string &service_name, CallbackT &&callback,
                 const rmw_qos_profile_t &qos_profile) {
    return rclcpp::create_service<ServiceT, CallbackT>(
        node_base_, node_services_,
        rclcpp::extend_name_with_sub_namespace(service_name,
                                               this->get_sub_namespace()),
        std::forward<CallbackT>(callback), qos_profile, this->group);
  }

private:
  rclcpp::CallbackGroup::SharedPtr group;
  rclcpp::Executor *executor;
  std::thread *spin_thread;

  void run_executor();

  template <typename TypeT>
  std::function<void(TypeT *ptr)> create_action_deleter() {

    rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr
        node_waitables_interface = this->get_node_waitables_interface();

    std::weak_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> weak_node =
        node_waitables_interface;
    std::weak_ptr<rclcpp::CallbackGroup> weak_group = this->group;
    bool group_is_null = (nullptr == this->group.get());

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
