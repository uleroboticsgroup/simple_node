# Simple Node

This is a new ROS 2 node to simplify the use of the spin and the executors. This way, service, actions and topics can be used as in ROS 1. Besides, this node also includes predefined action servers and one client to make its use more user-friendly to developers.


## Examples

### Node

```python
from simple_node import Node
import rclpy

class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")

def main(args=None):
    rclpy.init(args=args)

    node = MyNode()

    node.join_spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()

```

### Action Server

The following example shows how to create an action server that treats only one goal at the same time. If a new goal is received, the server aborts the current goal and starts treating the new one.

```python
class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")

        self.__action_server = self.create_action_server(ActionType,
                                                         "action_name",
                                                         self.__execute_cb)

    def __execute_server(self, goal_handle):
        result = ActionType.Result()

        succeed = self.do_something(goal_handle.request)

        if self.__action_server.is_canceled():
            self.__action_server.wait_for_canceling()
            goal_handle.canceled()

        else:
            if succeed:
                goal_handle.succeed()
            else:
                goal_handle.abort()

        return result
```

### Action Client

The following example shows how to create an action client.

```python
class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")

        self.__action_client = self.create_action_client(
            ActionType, "action_name")

        goal = ActionType.Goal()

        self.__action_client.wait_for_server()
        self.__action_client.send_goal(goal)
        self.__action_client.wait_for_result()

        self.__action_client.is_succeeded():
```
