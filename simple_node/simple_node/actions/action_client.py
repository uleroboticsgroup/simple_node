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


from threading import Event, RLock
from typing import Callable, Type, Any

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient as ActionClient2
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup


class ActionClient(ActionClient2):

    def __init__(
        self,
        node: Node,
        action_type: Type,
        action_name: str,
        feedback_cb: Callable = None
    ) -> None:

        self._action_done_event = Event()
        self._cancel_done_event = Event()

        self._result = None
        self._status = GoalStatus.STATUS_UNKNOWN
        self._status_lock = RLock()

        self._goal_handle = None
        self._goal_handle_lock = RLock()

        self.feedback_cb = feedback_cb

        super().__init__(
            node,
            action_type,
            action_name,
            callback_group=ReentrantCallbackGroup()
        )

    def get_status(self) -> int:
        with self._status_lock:
            return self._status

    def _set_status(self, status: int) -> None:
        with self._status_lock:
            self._status = status

    def is_succeeded(self) -> bool:
        return self.get_status() == GoalStatus.STATUS_SUCCEEDED

    def is_canceled(self) -> bool:
        return self.get_status() == GoalStatus.STATUS_CANCELED

    def is_aborted(self) -> bool:
        return self.get_status() == GoalStatus.STATUS_ABORTED

    def is_working(self) -> bool:
        with self._goal_handle_lock:
            return self._goal_handle is not None

    def is_terminated(self) -> bool:
        return (self.is_succeeded() or self.is_canceled() or self.is_aborted())

    def wait_for_result(self) -> None:
        self._action_done_event.clear()
        self._action_done_event.wait()

    def get_result(self) -> Any:
        return self._result

    def send_goal(self, goal, feedback_cb: Callable = None) -> None:

        with self._goal_handle_lock:
            self._goal_handle = None
        self._result = None
        self._set_status(GoalStatus.STATUS_UNKNOWN)

        _feedback_cb = self.feedback_cb
        if not feedback_cb is None:
            _feedback_cb = feedback_cb

        send_goal_future = self.send_goal_async(
            goal, feedback_callback=_feedback_cb)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future) -> None:
        with self._goal_handle_lock:
            self._goal_handle = future.result()
            get_result_future = self._goal_handle.get_result_async()
            get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future) -> None:
        self._result = future.result().result
        self._set_status(future.result().status)
        self._action_done_event.set()

    def cancel_goal(self) -> None:
        with self._goal_handle_lock:
            if self._goal_handle is not None:

                cancel_goal_future = self._cancel_goal_async(
                    self._goal_handle)
                cancel_goal_future.add_done_callback(self._cancel_done)

                self._cancel_done_event.clear()
                self._cancel_done_event.wait()

    def _cancel_done(self, future) -> None:
        self._cancel_done_event.set()
