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
# from .behaviours.follow_one_reference import FollowOneReference
import rclpy
import sys
from imc_ros_msgs.msg import FollowRefState, VehicleState

from .behaviours.follow_one_reference import FollowOneReference

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
        name="Square mission",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )

    topics2bb = py_trees.composites.Parallel(name="Topics2BB", policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        ))
    followrefstate2BB = py_trees_ros.subscribers.ToBlackboard(
        name="FollowReferenceState",
        topic_name="from_imc/followref_state",
        topic_type=FollowRefState,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variables = {'FollowRefState': None} # copy all data
    )
    vehiclestate2BB = py_trees_ros.subscribers.ToBlackboard(
        name="VehicleState",
        topic_name="from_imc/vehicle_state",
        topic_type=VehicleState,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variables = {'VehicleState': 'op_mode'} # copy op_mode
    )

    priorities = py_trees.composites.Selector(name="Tasks", memory=False)
    idle = py_trees.behaviours.Running(name="Idle")
    tasks = py_trees.composites.Selector(name="Tasks", memory=False)

    # Worker Tasks
    scan = py_trees.composites.Sequence(name="MovetoNext", memory=True)

    goal1_behaviour = FollowOneReference(
        name = "FollowSingleReference_1",
        lat = 41.18541112,
        lon = -8.70588612,
        z = 2.,
        speed = 5.,
        radius=-1.        
    )

    goal2_behaviour = FollowOneReference(
        name = "FollowSingleReference_2",
        lat = 41.18469722,
        lon = -8.70514722,
        z = 2.,
        speed = 5.,
        radius=-1.        
    )

    goal3_behaviour = FollowOneReference(
        name = "FollowSingleReference_3",
        lat = 41.18413888,
        lon = -8.70608888,
        z = 2.,
        speed = 5.,
        radius=-1.        
    )

    goal4_behaviour = FollowOneReference(
        name = "FollowSingleReference_4",
        lat = 41.18485,
        lon = -8.70683055,
        z = 2.,
        speed = 5.,
        radius=-1.        
    )
    goal5_behaviour = FollowOneReference(
        name = "FollowSingleReference_5",
        lat = 41.18541112,
        lon = -8.70588612,
        z = 2.,
        speed = 5.,
        radius=-1.        
    )

    root.add_child(topics2bb)
    topics2bb.add_children([followrefstate2BB, vehiclestate2BB])
    root.add_child(tasks)
    tasks.add_children([scan, idle])
    scan.add_children([goal1_behaviour, goal2_behaviour, goal3_behaviour, goal4_behaviour, goal5_behaviour])

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
        tree.setup(timeout=15.0)
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

if __name__ == '__main__':
    main()