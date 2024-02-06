#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://github.com/splintered-reality/py_trees_ros_tutorials/raw/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Action servers and clients
"""

##############################################################################
# Imports
##############################################################################

import action_msgs.msg as action_msgs  # GoalStatus
import argparse
import py_trees_ros.exceptions
from imc_ros_msgs.action import FollowSingleReference
from .imcpy_single_ref import FollowSingleRef
import rclpy.action
import threading
import asyncio
import time
import sys

from typing import Any, Callable

##############################################################################
# Action Server
##############################################################################


class GenericServer(object):
    """
    Generic action server that can be subclassed to quickly create action
    servers of varying types in a mock simulation.

    Dynamic Reconfigure:
        * **~duration** (:obj:`float`)

          * reconfigure the duration to be used for the next goal execution

    Args:
        node_name (:obj:`str`): name to use for the node (e.g. docking_controller)
        action_name (:obj:`str`): name of the action server (e.g. dock)
        action_type (:obj:`any`): type of the action server (e.g. py_trees_ros_interfaces.Dock
        custom_execute_callback (:obj:`func`): callback to be executed inside the execute loop, no args
        generate_cancelled_result (action_type.Result): custom result method
        generate_preempted_result (action_type.Result): custom result method
        generate_success_result (action_type.Result): custom result method
        goal_recieved_callback(:obj:`func`): callback to be executed immediately upon receiving a goal
        duration (:obj:`float`): forcibly override the dyn reconf time for a goal to complete (seconds)

    Use the ``dashboard`` to dynamically reconfigure the parameters.

    There are some shortcomings in this class that could be addressed to make it more robust:

     - check for matching goal id's when disrupting execution
     - execute multiple requests in parallel / pre-emption (not even tried)
    """
    def __init__(self,
                 node_name,
                 action_name,
                 action_type,  # e.g. py_trees_ros_interfaces.action.Dock
                 generate_feedback_message=None,
                 generate_cancelled_result=None,
                 generate_preempted_result=None,
                 generate_success_result=None,
                 goal_received_callback=lambda request: None,
                 duration=None):
        self.node = rclpy.create_node(
            node_name,
            parameter_overrides=[
                rclpy.parameter.Parameter(
                    'duration',
                    rclpy.parameter.Parameter.Type.DOUBLE,
                    5.0  # seconds
                ),
            ],
            automatically_declare_parameters_from_overrides=True
        )
        # override
        if duration is not None:
            self.node.set_parameters([
                rclpy.parameter.Parameter(
                    'duration',
                    rclpy.parameter.Parameter.Type.DOUBLE,
                    duration  # seconds
                ),
            ])
        # Needs a member variable to capture the dynamic parameter
        # value when accepting the goal and pass that to the execution
        # of the goal. Not necessary to initialise here, but done
        # for completeness
        self.duration = self.node.get_parameter("duration").value

        self.goal_handle = None
        self.goal_lock = threading.Lock()
        self.frequency = 3.0  # hz
        self.percent_completed = 0

        self.action_type = action_type
        if generate_feedback_message is None:
            self.generate_feedback_message = lambda: self.action_type.Feedback()
        else:
            self.generate_feedback_message = generate_feedback_message
        if generate_cancelled_result is None:
            self.generate_cancelled_result = lambda: self.action_type.Result(message="goal cancelled")
        else:
            self.generate_cancelled_result = generate_cancelled_result
        if generate_preempted_result is None:
            self.generate_preempted_result = lambda: self.action_type.Result(message="goal pre-empted")
        else:
            self.generate_preempted_result = generate_preempted_result
        if generate_success_result is None:
            self.generate_success_result = lambda: self.action_type.Result(message="goal executed with success")
        else:
            self.generate_success_result = generate_success_result
        self.goal_received_callback = goal_received_callback
        self.goal_handle = None

        self.action_server = rclpy.action.ActionServer(
            node=self.node,
            action_type=self.action_type,
            action_name=action_name,
            callback_group=rclpy.callback_groups.ReentrantCallbackGroup(),  # needed?
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_goal_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            result_timeout=10
        )

    def goal_callback(self, goal_request):
        """
        Args:
            goal_request: of <action_type>.GoalRequest with members
                goal_id (unique_identifier.msgs.UUID) and those specified in the action
        """
        self.node.get_logger().info("received a goal")
        self.goal_received_callback(goal_request)
        self.percent_completed = 0
        self.duration = self.node.get_parameter("duration").value
        return rclpy.action.server.GoalResponse.ACCEPT

    def cancel_callback(
            self,
            goal_handle: rclpy.action.server.ServerGoalHandle
         ) -> rclpy.action.CancelResponse:
        """
        Cancel any currently executing goal

        Args:
            cancel_request (:class:`~rclpy.action.server.ServerGoalHandle`):
                handle with information about the
                goal that is requested to be cancelled
        """
        self.node.get_logger().info("cancel requested: [{goal_id}]".format(
            goal_id=goal_handle.goal_id))
        return rclpy.action.CancelResponse.ACCEPT

    def execute_goal_callback(
            self,
            goal_handle: rclpy.action.server.ServerGoalHandle
         ):
        """
        Check for pre-emption, but otherwise just spin around gradually incrementing
        a hypothetical 'percent' done.

        Args:
            goal_handle (:class:`~rclpy.action.server.ServerGoalHandle`): the goal handle of the executing action
        """
        raise NotImplementedError

    def handle_accepted_callback(self, goal_handle):
        self.node.get_logger().info("handle accepted")
        with self.goal_lock:
            self.goal_handle = goal_handle
            goal_handle.execute()

    def abort(self):
        """
        This method is typically only used when the system is shutting down and
        there is an executing goal that needs to be abruptly terminated.
        """
        with self.goal_lock:
            if self.goal_handle and self.goal_handle.is_active:
                self.node.get_logger().info("aborting...")
                self.goal_handle.abort()

    def shutdown(self):
        """
        Cleanup
        """
        self.action_server.destroy()
        self.node.destroy_node()


##############################################################################
# Action Client
##############################################################################


class GenericClient(object):
    """
    Generic action client that can be used to test the mock action servers.

    Args:
        node_name: name to use when creating the node for this process
        action_name: the action namespace under which topics and services exist (e.g. move_base)
        action_type: the action type (e.g. move_base_msgs.msg.MoveBaseAction)
        generate_feedback_message: format the feedback message
    """
    def __init__(self,
                 node_name: str,
                 action_name: str,
                 action_type: Any,
                 generate_feedback_message: Callable[[Any], str]=None
                 ):
        self.action_type = action_type
        self.action_name = action_name
        self.node_name = node_name
        if generate_feedback_message is None:
            self.generate_feedback_message = lambda msg: str(msg)
        else:
            self.generate_feedback_message = generate_feedback_message

        self.status_strings = {
                action_msgs.GoalStatus.STATUS_UNKNOWN : "STATUS_UNKNOWN",  # noqa
                action_msgs.GoalStatus.STATUS_ACCEPTED : "STATUS_ACCEPTED",  # noqa
                action_msgs.GoalStatus.STATUS_EXECUTING: "STATUS_EXECUTING",  # noqa
                action_msgs.GoalStatus.STATUS_CANCELING: "STATUS_CANCELING",  # noqa
                action_msgs.GoalStatus.STATUS_SUCCEEDED: "STATUS_SUCCEEDED",  # noqa
                action_msgs.GoalStatus.STATUS_CANCELED : "STATUS_CANCELED",  # noqa
                action_msgs.GoalStatus.STATUS_ABORTED  : "STATUS_ABORTED"  # noqa
            }

        ####################
        # ROS Setup
        ####################
        self.node = rclpy.create_node(self.node_name)
        self.action_client = rclpy.action.ActionClient(
            node=self.node,
            action_type=self.action_type,
            action_name=self.action_name
        )
        self._timer = None
        self._goal_handle = None
        self._send_goal_future = None
        self._get_result_future = None

    def setup(self):
        """
        Middleware communications setup. This uses a hard coded default
        of 2.0 seconds to wait for discovery of the action server. If it
        should fail, it raises a timed out error.

        Raises:
            :class:`py_trees_ros.exceptions.TimedOutError`
        """
        self.node.get_logger().info(
            'waiting for server [{}]'.format(
                self.action_name
            )
        )
        result = self.action_client.wait_for_server(timeout_sec=2.0)
        if not result:
            message = "timed out waiting for the server [{}]".format(
                    self.action_name
                )
            self.node.get_logger().error(message)
            raise py_trees_ros.exceptions.TimedOutError(message)
        else:
            self.node.get_logger().info("...connected")

    def feedback_callback(self, msg: Any):
        """
        Prints the feedback on the node's logger.

        Args:
            msg: the feedback message, particular to the action type definition
        """
        self.node.get_logger().info('feedback: {0}'.format(self.generate_feedback_message(msg)))

    def send_cancel_request(self):
        """
        Start the cancel request, chain it to :func:`cancel_response_callback`.
        """
        self.node.get_logger().info('Cancelling goal')

        if self._goal_handle is not None:
            future = self._goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_response_callback)

        if self._timer is not None:
            self._timer.cancel()

    def cancel_response_callback(self, future: rclpy.task.Future):
        """
        Cancel response callback

        Args:
            future: details returning from the server about the cancel request
        """
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.node.get_logger().info('Goal successfully cancelled')
        else:
            self.node.get_logger().info('Goal failed to cancel')

    def send_goal(self) -> rclpy.task.Future:
        """
        Send the goal and get a future back, but don't do any
        spinning here to await the future result. Chain it to
        :func:`goal_response_callback`.

        Returns:
            rclpy.task.Future: the future awaits...
        """
        self.node.get_logger().info('sending goal...')
        self._send_goal_future = self.action_client.send_goal_async(
                self.action_type.Goal(),
                feedback_callback=self.feedback_callback,
                # A random uuid is always generated
                # goal_uuid=unique_identifier_msgs.UUID(
                #     uuid=list(uuid.uuid4().bytes)
                # )
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return self._send_goal_future

    def goal_response_callback(self, future: rclpy.task.Future):
        """
        Handle goal response, proceed to listen for the result if accepted.

        Args:
            future: details returning from the server about the goal request
        """
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.node.get_logger().info('...goal rejected :( \n%r' % (future.exception()))
            return
        self.node.get_logger().info("...goal accepted :)\n%s" % future.result())

        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future: rclpy.task.Future):
        """
        Finally, at the end of the pipeline, what was the result!?

        Args:
            future: details returning from the server about the goal result
        """
        message = future.result().result.message
        status = future.result().status
        status_string = self.status_strings[status]

        if status == action_msgs.GoalStatus.STATUS_SUCCEEDED:  # noqa
            self.node.get_logger().info("Result")
            self.node.get_logger().info("  status: {}".format(status_string))
            self.node.get_logger().info("  message: {}".format(message))
        else:
            self.node.get_logger().info('Goal failed with status: {0}'.format(status_string))

    def spin(self, cancel: bool=False):
        """
        Common spin method for clients.

        Args:
            cancel: send a cancel request shortly after sending the goal request
        """
        self.send_goal()
        if cancel:
            self._timer = self.node.create_timer(1.0, self.send_cancel_request)
        rclpy.spin(self.node)

    def shutdown(self):
        """
        Shutdown, cleanup.
        """
        self.action_client.destroy()
        self.node.destroy_node()


##############################################################################
# Action Clients
##############################################################################

def command_line_argument_parser():
    parser = argparse.ArgumentParser(
        description="action client",
        epilog="And his noodly appendage reached forth to tickle the blessed...\n"
    )
    parser.add_argument(
        '-c', '--cancel',
        action='store_true',
        default=False,
        help='send a cancel request a short period after sending the goal request')
    return parser.parse_args(sys.argv[1:])





class FollowSingleReferenceClient(GenericClient):
    """
    A mock rotation controller client.
    """
    def __init__(self):
        super().__init__(
            node_name="follow_single_reference_client",
            action_name="follow_single_reference",
            action_type=FollowSingleReference
        )


def follow_single_reference_client(args=None):
    """
    Spin up a rotation client and manage it from init to shutdown.
    """
    rclpy.init(args=args)
    action_client = FollowSingleReferenceClient()
    try:
        action_client.setup()
        action_client.spin()
    except (py_trees_ros.exceptions.TimedOutError, KeyboardInterrupt):
        pass
    action_client.shutdown()