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

Node::Node(std::string name)
    : Node(name, "", new rclcpp::executors::MultiThreadedExecutor()) {}

Node::Node(std::string name, std::string _namespace)
    : Node(name, _namespace, new rclcpp::executors::MultiThreadedExecutor()) {}

Node::Node(std::string name, rclcpp::Executor *executor)
    : Node(name, "", executor) {}

Node::Node(std::string name, std::string _namespace, rclcpp::Executor *executor)
    : rclcpp::Node(name, _namespace) {

  rclcpp::CallbackGroup::SharedPtr group =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  this->executor = executor;
  this->spin_thread = new std::thread(&Node::run_executor, this);
}
Node::Node(std::string name, std::string _namespace, const rclcpp::NodeOptions &options):
    rclcpp::Node(name, _namespace, options) {}

Node::~Node() {
  delete this->executor;
  delete this->spin_thread;
}

void Node::join_spin() {
  this->spin_thread->join();
  RCLCPP_INFO(this->get_logger(), "Destroying node %s", this->get_name());
}

void Node::run_executor() {
  this->executor->add_node(this->get_node_base_interface());
  this->executor->spin();
}
