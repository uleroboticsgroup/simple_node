
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

  this->executor = executor;
  this->spin_thread = new std::thread(&Node::run_executor, this);
}

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
