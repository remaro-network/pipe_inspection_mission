#!/usr/bin/env python3.9
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://github.com/splintered-reality/py_trees_ros_tutorials/raw/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
About
^^^^^

In this, the first of the tutorials, we start out with a behaviour that
collects battery data from a subscriber and stores the result on the
blackboard for other behaviours to utilise.

Data gathering up front via subscribers is a useful convention for
a number of reasons:

* Freeze incoming data for remaining behaviours in the tree tick so that decision making is consistent across the entire tree
* Avoid redundantly invoking multiple subscribers to the same topic when not necessary
* Python access to the blackboard is easier than ROS middleware handling

Typically data gatherers will be assembled underneath a parallel at or near
the very root of the tree so they may always trigger their update() method
and be processed before any decision making behaviours elsewhere in the tree.

Tree
^^^^

.. code-block:: bash

   $ py-trees-render -b py_trees_ros_tutorials.one_data_gathering.tutorial_create_root

.. graphviz:: dot/tutorial-one-data-gathering.dot
   :align: center

.. literalinclude:: ../py_trees_ros_tutorials/one_data_gathering.py
   :language: python
   :linenos:
   :lines: 121-153
   :caption: one_data_gathering.py#tutorial_create_root

Along with the data gathering side, you'll also notice the dummy branch for
priority jobs (complete with idle behaviour that is always
:attr:`~py_trees.common.Status.RUNNING`). This configuration is typical
of the :term:`data gathering` pattern.

Behaviours
^^^^^^^^^^

The tree makes use of the :class:`py_trees_ros.battery.ToBlackboard` behaviour.

This behaviour will cause the entire tree will tick over with
:attr:`~py_trees.common.Status.SUCCESS` so long as there is data incoming.
If there is no data incoming, it will simply
:term:`block` and prevent the rest of the tree from acting.


Running
^^^^^^^

.. code-block:: bash

    # Launch the tutorial
    $ ros2 launch py_trees_ros_tutorials tutorial_one_data_gathering_launch.py
    # In a different shell, introspect the entire blackboard
    $ py-trees-blackboard-watcher
    # Or selectively get the battery percentage
    $ py-trees-blackboard-watcher --list
    $ py-trees-blackboard-watcher /battery.percentage

.. image:: images/tutorial-one-data-gathering.gif
"""

##############################################################################
# Imports
##############################################################################

import launch
import launch_ros
import operator
import py_trees
import py_trees_ros.trees
import py_trees.console as console
import py_trees_ros_interfaces.action as py_trees_actions  # noqa
import imc_ros_msgs.action as imc_ros_actions
import rclpy
import sys

# from .behaviours.follow_one_reference import FollowOneReference

import geometry_msgs.msg as geometry_msgs



##############################################################################
# Subscriber tree
##############################################################################


def tutorial_create_root() -> py_trees.behaviour.Behaviour:
    """
    Create a basic tree and start a 'Topics2BB' work sequence that
    will become responsible for data gathering behaviours.

    Returns:
        the root of the tree
    """
    root = py_trees.composites.Parallel(
        name="Tutorial One",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )

    topics2bb = py_trees.composites.Sequence(name="Topics2BB", memory=True)
    estimatedState2bb = py_trees_ros.subscribers.ToBlackboard(
        name="EstimatedState2BB",
        topic_name="base_link",
        topic_type=geometry_msgs.PoseStamped,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
    )
    priorities = py_trees.composites.Selector(name="Tasks", memory=False)
    idle = py_trees.behaviours.Running(name="Idle")
    tasks = py_trees.composites.Selector(name="Tasks", memory=False)

    # Worker Tasks
    scan = py_trees.composites.Sequence(name="MovetoNext", memory=True)

    goal1 = imc_ros_actions.FollowSingleReference.Goal()
    goal1.lat = 41.18541112
    goal1.lon = -8.70588612
    goal1.z = 2.
    goal1.speed = 1.6

    goal2 = imc_ros_actions.FollowSingleReference.Goal()
    goal2.lat = 41.18469722
    goal2.lon = -8.70514722
    goal2.z = 2.
    goal2.speed = 2.

    goal3 = imc_ros_actions.FollowSingleReference.Goal()
    goal3.lat = 41.18413888
    goal3.lon = -8.70608888
    goal3.z = 2.
    goal3.speed = 1.6

    goal4 = imc_ros_actions.FollowSingleReference.Goal()
    goal4.lat = 41.18485
    goal4.lon = -8.70683055
    goal4.z = 2.
    goal4.speed = 1.6

    goal5 = imc_ros_actions.FollowSingleReference.Goal()
    goal5.lat = 41.18541112
    goal5.lon = -8.70588612
    goal5.z = 2.
    goal5.speed = 1.6

    goal1_action = py_trees_ros.actions.ActionClient(
        name="FollowSingleReference_1",
        action_type=imc_ros_actions.FollowSingleReference,
        action_name="FollowSingleReference",
        action_goal = goal1,  # noqa
        generate_feedback_message=lambda msg: "{}".format(msg.feedback.state),
        wait_for_server_timeout_sec=100
    )

    goal2_action = py_trees_ros.actions.ActionClient(
        name="FollowSingleReference_2",
        action_type=imc_ros_actions.FollowSingleReference,
        action_name="FollowSingleReference",
        action_goal = goal2,  # noqa
        generate_feedback_message=lambda msg: "{}".format(msg.feedback.state),
        wait_for_server_timeout_sec=100
    ) 
    goal3_action = py_trees_ros.actions.ActionClient(
        name="FollowSingleReference_3",
        action_type=imc_ros_actions.FollowSingleReference,
        action_name="FollowSingleReference",
        action_goal = goal3,  # noqa
        generate_feedback_message=lambda msg: "{:d}".format(msg.feedback.state),
        wait_for_server_timeout_sec=100
    ) 
    goal4_action = py_trees_ros.actions.ActionClient(
        name="FollowSingleReference_4",
        action_type=imc_ros_actions.FollowSingleReference,
        action_name="FollowSingleReference",
        action_goal = goal4,  # noqa
        generate_feedback_message=lambda msg: "{:d}".format(msg.feedback.state),
        wait_for_server_timeout_sec=100
    ) 
    goal5_action = py_trees_ros.actions.ActionClient(
        name="FollowSingleReference_5",
        action_type=imc_ros_actions.FollowSingleReference,
        action_name="FollowSingleReference",
        action_goal = goal5,  # noqa
        generate_feedback_message=lambda msg: "{:d}".format(msg.feedback.state),
        wait_for_server_timeout_sec=100
    ) 
    root.add_child(topics2bb)
    topics2bb.add_child(estimatedState2bb)
    root.add_child(tasks)
    tasks.add_children([scan, idle])
    scan.add_children([goal1_action,goal2_action, goal3_action,goal4_action,goal5_action])

    return root


def main():
    """
    Entry point for the demo script.
    """
    rclpy.init(args=None)
    root = tutorial_create_root()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    try:
        tree.setup(node_name="behaviorTree_creator", timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)

    tree.tick_tock(1000)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()
