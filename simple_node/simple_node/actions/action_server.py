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


""" Custom action server that add goals to a queue """

from typing import Callable
from threading import Lock, Thread

from rclpy.node import Node
from rclpy.action import ActionServer as ActionServer2
from rclpy.action import CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup


class ActionServer(ActionServer2):
    """ Action Server Class """

    def __init__(
        self,
        node: Node,
        action_type,
        action_name: str,
        execute_callback: Callable,
        cancel_callback: Callable = None
    ) -> None:

        self.__goal_lock = Lock()
        self.__user_execute_callback = execute_callback
        self.__user_cancel_callback = cancel_callback
        self.__cancel_thread = None
        self._goal_handle = None
        self.node = node

        super().__init__(
            node, action_type, action_name,
            execute_callback=self.__execute_callback,
            goal_callback=self.__goal_callback,
            handle_accepted_callback=self.__handle_accepted_callback,
            cancel_callback=self.__cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

    def is_working(self) -> bool:
        return self._goal_handle is not None

    def __goal_callback(self, goal_request) -> int:
        """ goal callback """

        return GoalResponse.ACCEPT

    def __handle_accepted_callback(self, goal_handle: ServerGoalHandle) -> None:
        """
            handle accepted calback for a single goal server
            only one goal can be treated
            if other goal is send, old goal is aborted and replaced with the new one
        """

        with self.__goal_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self._goal_handle.abort()

            self._goal_handle = goal_handle
            self._goal_handle.execute()

    def __cancel_callback(self, goal_handle: ServerGoalHandle) -> int:
        """ cancel calback """

        if self.__user_cancel_callback is not None:
            self.__cancel_thread = Thread(target=self.__user_cancel_callback)
            self.__cancel_thread.start()

        return CancelResponse.ACCEPT

    def __execute_callback(self, goal_handle: ServerGoalHandle):
        """
            execute callback
        """

        self.__cancel_thread = None
        results = self.__user_execute_callback(self._goal_handle)

        if self.__cancel_thread is not None:
            self.__cancel_thread.join()

        self._goal_handle = None
        return results
