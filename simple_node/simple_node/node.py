# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


""" ROS2 Node to simulate ROS1 Node """


import time
from threading import Thread
from typing import List, Callable, Type

import rclpy
from rclpy.client import Client
from rclpy.context import Context
from rclpy.node import Node as Node2
from rclpy.parameter import Parameter
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, Executor

from simple_node.actions.action_client import ActionClient
from simple_node.actions.action_server import ActionServer


class Node(Node2):
    """ Node Class """

    def __init__(
        self,
        node_name: str,
        context: Context = None,
        cli_args: List[str] = None,
        namespace: str = "",
        use_global_arguments: bool = True,
        enable_rosout: bool = True,
        start_parameter_services: bool = True,
        parameter_overrides: List[Parameter] = None,
        allow_undeclared_parameters: bool = False,
        automatically_declare_parameters_from_overrides: bool = False,
        executor: Executor = None
    ) -> None:

        super().__init__(
            node_name,
            context=context,
            cli_args=cli_args,
            namespace=namespace,
            use_global_arguments=use_global_arguments,
            enable_rosout=enable_rosout,
            start_parameter_services=start_parameter_services,
            parameter_overrides=parameter_overrides,
            allow_undeclared_parameters=allow_undeclared_parameters,
            automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides
        )

        if not executor:
            self._executor = MultiThreadedExecutor()
        else:
            self._executor = executor

        self._spin_thread = Thread(target=self._run_executor, daemon=True)
        self._spin_thread.start()

        self._wake_thread = Thread(target=self._wake_node, daemon=True)
        self._wake_thread.start()

    def _run_executor(self) -> None:
        """ run an executer with self (node) """

        self._executor.add_node(self)
        try:
            self._executor.spin()
        finally:
            self._executor.shutdown()

    def _wake_node(self) -> None:
        while rclpy.ok():
            self._executor.wake()
            time.sleep(1)

    def join_spin(self) -> None:
        """ wait for spin thread """

        try:
            self._spin_thread.join()
            self._wake_thread.join()
        finally:
            self.get_logger().info("Destroying node " + self.get_name())
            self.destroy_node()

    def create_client(
        self,
        srv_type: Type,
        srv_name: str
    ) -> Client:
        return super().create_client(srv_type, srv_name, callback_group=ReentrantCallbackGroup())

    def create_action_client(
        self,
        action_type: Type,
        action_name: str,
        feedback_cb: Callable = None
    ) -> ActionClient:
        """ create action client from node

        Args:
            action_type ([type]): action type (msg action)
            action_name (str): action name

        Returns:
            ActionClient: client created
        """

        return ActionClient(self, action_type, action_name, feedback_cb)

    def create_action_server(
        self,
        action_type: Type,
        action_name: str,
        execute_callback: Callable,
        cancel_callback: Callable = None
    ) -> ActionServer:
        """ create action server from node

        Args:
            action_type ([type]): action type (msg action)
            action_name (str): action name
            execute_callback ([type]): execute function
            cancel_callback ([type], optional): cancel function. Defaults to None.

        Returns:
            ActionSingleServer: server created
        """

        return ActionServer(
            self,
            action_type,
            action_name,
            execute_callback,
            cancel_callback=cancel_callback
        )
