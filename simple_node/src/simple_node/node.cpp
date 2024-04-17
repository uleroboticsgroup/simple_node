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

#include "simple_node/node.hpp"

using namespace simple_node;

Node::Node(const std::string &name, const rclcpp::NodeOptions &options,
           const rclcpp::Executor::SharedPtr executor)
    : Node(name, "", options, executor) {}

Node::Node(const std::string &name, const std::string &namespace_,
           const rclcpp::NodeOptions &options,
           rclcpp::Executor::SharedPtr executor)
    : rclcpp::Node(name, namespace_, options), executor(executor) {

  if (this->executor == nullptr) {
    this->executor =
        std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  }

  rclcpp::CallbackGroup::SharedPtr group =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  this->spin_thread = std::make_unique<std::thread>(&Node::run_executor, this);
}

void Node::join_spin() {
  this->spin_thread->join();
  RCLCPP_INFO(this->get_logger(), "Destroying node %s", this->get_name());
}

void Node::run_executor() {
  this->executor->add_node(this->get_node_base_interface());
  this->executor->spin();
}
