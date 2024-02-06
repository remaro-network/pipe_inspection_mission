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
Mocks a simple action server that rotates the robot 360 degrees.
"""


##############################################################################
# Imports
##############################################################################

import argparse
import math
import time
import threading
from .actions import GenericServer
import py_trees_ros_interfaces.action as py_trees_actions
from imc_ros_msgs.action import FollowSingleReference
from .imcpy_single_ref import FollowSingleRef
import rclpy
import sys
import imcpy 

##############################################################################
# Class
##############################################################################


class FollowSingleReferenceServer(GenericServer):
    """
    Simple server that controls a full rotation of the robot.

    Node Name:
        * **rotation_controller**

    Action Servers:
        * **/rotate** (:class:`py_trees_ros_interfaces.action.Dock`)

          * motion primitives - rotation server

    Args:
        rotation_rate (:obj:`float`): rate of rotation (rad/s)
    """
    def __init__(self, rotation_rate: float=1.57):
        super().__init__(node_name="follow_reference_server",
                         action_name="FollowSingleReference",
                         action_type=FollowSingleReference,
                         generate_feedback_message=self.generate_feedback_message,
                         duration=2.0 * math.pi / rotation_rate
                         )
        self.node.get_logger().info("------------------CREATED SERVER---------------------")

    def generate_feedback_message(self, msg):
        """
        Create a feedback message that populates the percent completed.

        Returns:
            :class:`py_trees_actions.Rotate.Feedback`: the populated feedback message
        """
        # TODO: send some feedback message
        feedback_msg = FollowSingleReference.Feedback()
        feedback_msg.state = msg

        # msg.percentage_completed = self.percent_completed
        # msg.angle_rotated = 2*math.pi*self.percent_completed/100.0
        return feedback_msg
    
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
        # goal.details (e.g. pose) = don't care
        self.node.get_logger().info("------------------executing a goal---------------------")
        dune_actor = FollowSingleRef(lat = goal_handle._goal_request.lat, lon = goal_handle._goal_request.lon, depth = goal_handle._goal_request.z, speed= goal_handle._goal_request.speed, target_name='lauv-simulator-1')
        def initialize_imcpy_task():
            self.node.get_logger().info("Initializing async functions")
            dune_actor.run_async_function()
            # dune_actor.run()
            event.set()

        # Create an event for synchronization
        event = threading.Event()
        # Create a new thread and initialize the class instance within it
        self.node.get_logger().info("Initialize thread")
        thread = threading.Thread(target = initialize_imcpy_task)
        thread.daemon = True  # Set the thread as a daemon
        thread.start()
        self.node.get_logger().info("Start thread")
        # DO STUFF MEANWHILE
        continue_server = True
        while continue_server:
            with self.goal_lock:
                time.sleep(1)
                self.node.get_logger().info("Running tree stuff")
                state = dune_actor.state
                near = dune_actor.near
                if goal_handle.is_active:
                    if goal_handle.is_cancel_requested:
                        result = self.generate_cancelled_result()
                        message = "goal cancelled"
                        self.node.get_logger().info(message)
                        goal_handle.canceled()
                        continue_server = False
                    elif goal_handle.goal_id != self.goal_handle.goal_id:
                        result = self.generate_preempted_result()
                        message = "goal pre-empted"
                        self.node.get_logger().info(message)
                        goal_handle.abort()
                        continue_server = False
                    elif near:
                        result = self.generate_success_result()
                        message = "goal executed with success"
                        event.wait()
                        self.node.get_logger().info("Event finished")
                        time.sleep(10)
                        del dune_actor
                        self.node.get_logger().info("dune actor deleted")
                        goal_handle.succeed()
                        continue_server = False
                    else:
                        # self.node.get_logger().info("Mission state: {}".format(str(state)))
                        goal_handle.publish_feedback(
                            self.generate_feedback_message(str(state))
                            )
                else:  # ! active
                    self.node.get_logger().info("goal is no longer active, aborting")
                    result = self.action_type.Result()
                    return result

        # Wait for the thread to finish before continuing in the main thread
        event.wait()
        self.node.get_logger().info("Event finished")
        # if dune_actor._task_mc.cancelled():
        #     dune_actor.ros_node.get_logger().info('mc task cancelled')
        # else:
        #     dune_actor.ros_node.get_logger().info('mc task NOT cancelled')
        #     # dune_actor._task_mc.cancel()
        


        # del dune_actor
        # self.node.get_logger().info("dune actor deleted")
        
        return result
            



def main():
    """
    Entry point for the mock rotation controller node.
    """
    parser = argparse.ArgumentParser(description='Follow a single reference point')
    command_line_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    parser.parse_args(command_line_args)
    rclpy.init()  # picks up sys.argv automagically internally
    follow = FollowSingleReferenceServer()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(follow.node)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        follow.abort()
        # caveat: often broken, with multiple spin_once or shutdown, error is the
        # mysterious:
        #   The following exception was never retrieved: PyCapsule_GetPointer
        #   called with invalid PyCapsule object
        executor.shutdown()  # finishes all remaining work and exits
        rclpy.try_shutdown()
