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


from typing import Callable
import time
from threading import Thread, Lock
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient as ActionClient2
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup


class ActionClient(ActionClient2):

    def __init__(self, node: Node, action_type, action_name: str, feedback_cb: Callable = None):
        self._status = GoalStatus.STATUS_UNKNOWN
        self.__status_lock = Lock()
        self.__goal_handle = None
        self.__goal_thread = None
        self.__result = None
        self.feedback_cb = feedback_cb
        super().__init__(node, action_type, action_name,
                         callback_group=ReentrantCallbackGroup())

    def get_status(self) -> int:
        with self.__status_lock:
            return self._status

    def _set_status(self, status: int):
        with self.__status_lock:
            self._status = status

    def is_accepted(self) -> bool:
        return self.get_status() == GoalStatus.STATUS_ACCEPTED

    def is_executing(self) -> bool:
        return self.get_status() == GoalStatus.STATUS_EXECUTING

    def is_canceling(self) -> bool:
        return self.get_status() == GoalStatus.STATUS_CANCELING

    def is_succeeded(self) -> bool:
        return self.get_status() == GoalStatus.STATUS_SUCCEEDED

    def is_canceled(self) -> bool:
        return self.get_status() == GoalStatus.STATUS_CANCELED

    def is_aborted(self) -> bool:
        return self.get_status() == GoalStatus.STATUS_ABORTED

    def is_working(self) -> bool:
        return (self.is_executing() or self.is_canceling() or self.is_accepted())

    def is_terminated(self) -> bool:
        return (self.is_succeeded() or self.is_canceled() or self.is_aborted())

    def wait_for_result(self):
        self.__goal_thread.join()

    def get_result(self):
        return self.__result

    def __send_goal(self, goal, feedback_cb: Callable = None):

        self.__result = None

        send_goal_future = self.send_goal_async(
            goal, feedback_callback=feedback_cb)

        # wait for acceptance
        while not send_goal_future.done():
            time.sleep(0.01)

        # check acceptance
        self.__goal_handle = send_goal_future.result()
        if not self.__goal_handle.accepted:

            # change status
            if self.is_canceled():
                return
            self._set_status(GoalStatus.STATUS_ABORTED)
            return

        # change status
        if self.is_canceled():
            return
        self._set_status(GoalStatus.STATUS_ACCEPTED)

        # get result
        get_result_future = self.__goal_handle.get_result_async()

        # change status
        if self.is_canceled():
            return
        self._set_status(GoalStatus.STATUS_EXECUTING)

        # wait for result
        while not get_result_future.done():
            time.sleep(0.01)

        # change status
        if self.is_canceled():
            return

        self._set_status(get_result_future.result().status)
        self.__result = get_result_future.result().result

    def send_goal(self, goal, feedback_cb: Callable = None):

        _feedback_cb = self.feedback_cb

        if not feedback_cb is None:
            _feedback_cb = feedback_cb

        self.__goal_thread = Thread(
            target=self.__send_goal, args=(goal, _feedback_cb))
        self._set_status(GoalStatus.STATUS_UNKNOWN)
        self.__goal_thread.start()

    def __cancel_goal(self) -> bool:

        old_status = self.get_status()

        cancel_goal_future = self._cancel_goal_async(self.__goal_handle)
        self._set_status(GoalStatus.STATUS_CANCELING)

        while not cancel_goal_future.done():
            time.sleep(0.01)

        result = cancel_goal_future.result()
        if len(result.goals_canceling) == 0:
            self._set_status(old_status)
            return False

        self._set_status(GoalStatus.STATUS_CANCELED)

        return True

    def cancel_goal(self) -> bool:
        if self.__goal_handle:
            return self.__cancel_goal()

        else:
            return False
