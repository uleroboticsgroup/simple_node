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

#ifndef SIMPLE_NODE_HPP
#define SIMPLE_NODE_HPP

#include <memory>
#include <string>
#include <thread>

#include "rclcpp/executor.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"

#include "simple_node/actions/action_client.hpp"
#include "simple_node/actions/action_server.hpp"

#include "rclcpp/node_options.hpp"

namespace simple_node {

class Node : public rclcpp::Node {

public:
  Node(const std::string &name,
       const rclcpp::NodeOptions &options = rclcpp::NodeOptions(),
       rclcpp::Executor::SharedPtr executor = nullptr);
  Node(const std::string &name, const std::string &namespace_,
       const rclcpp::NodeOptions &options = rclcpp::NodeOptions(),
       rclcpp::Executor::SharedPtr executor = nullptr);

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
  typename std::shared_ptr<actions::ActionServer<ActionT>> create_action_server(
      std::string action_name,
      std::function<
          void(std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>)>
          execute_callback) {

    std::shared_ptr<actions::ActionServer<ActionT>> action_server(
        new actions::ActionServer<ActionT>(this, action_name, execute_callback),
        this->create_action_deleter<rclcpp_action::Server<ActionT>>());

    this->get_node_waitables_interface()->add_waitable(action_server,
                                                       this->group);
    return action_server;
  }

  template <typename ActionT>
  typename std::shared_ptr<actions::ActionServer<ActionT>> create_action_server(
      std::string action_name,
      std::function<
          void(std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>)>
          execute_callback,
      std::function<void()> cancel_callback) {

    std::shared_ptr<actions::ActionServer<ActionT>> action_server(
        new actions::ActionServer<ActionT>(this, action_name, execute_callback,
                                           cancel_callback),
        this->create_action_deleter<rclcpp_action::Server<ActionT>>());

    this->get_node_waitables_interface()->add_waitable(action_server,
                                                       this->group);
    return action_server;
  }

  template <typename ServiceT>
  typename rclcpp::Client<ServiceT>::SharedPtr create_client(
      const std::string &service_name,
      const rmw_qos_profile_t &qos_profile = rmw_qos_profile_services_default) {
    return rclcpp::create_client<ServiceT>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_services_interface(),
        rclcpp::extend_name_with_sub_namespace(service_name,
                                               this->get_sub_namespace()),
        qos_profile, this->group);
  }

  template <typename ServiceT, typename CallbackT>
  typename rclcpp::Service<ServiceT>::SharedPtr create_service(
      const std::string &service_name, CallbackT &&callback,
      const rmw_qos_profile_t &qos_profile = rmw_qos_profile_services_default) {
    return rclcpp::create_service<ServiceT, CallbackT>(
        this->get_node_base_interface(), this->get_node_services_interface(),
        rclcpp::extend_name_with_sub_namespace(service_name,
                                               this->get_sub_namespace()),
        std::forward<CallbackT>(callback), qos_profile, this->group);
  }

private:
  rclcpp::CallbackGroup::SharedPtr group;
  rclcpp::Executor::SharedPtr executor;
  std::unique_ptr<std::thread> spin_thread;

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
