
##############################################################################
# Imports
##############################################################################

import py_trees
import time
import py_trees_ros
import rcl_interfaces.msg as rcl_msgs
import rcl_interfaces.srv as rcl_srvs
import rclpy
import imcpy
from numpy import pi as pi
import geometry_msgs.msg as geometry_msgs
from imc_ros_msgs.msg import FollowReference, Reference, DesiredSpeed, DesiredZ, PlanControl


##############################################################################
# Behaviours
##############################################################################

class FollowOneReference(py_trees.behaviour.Behaviour):
    """
    This behaviour simply shoots a command off to the AUV to follow
    a single point returns :attr:`~py_trees.common.Status.RUNNING`.
    Note that this behaviour will never return with
    :attr:`~py_trees.common.Status.SUCCESS` but will send a clearing
    command if it is cancelled or interrupted by a higher
    priority behaviour.

    Publishers:
        * **/led_strip/command** (:class:`std_msgs.msg.String`)

          * colourised string command for the led strip ['red', 'green', 'blue']

    Args:
        name: name of the behaviour
        topic_name : name of the battery state topic
        colour: colour to flash ['red', 'green', blue']
    """
    def __init__(
            self,
            name: str,
            lat: float,
            lon: float,
            z: float,
            speed: float,
            radius: float
    ):
        super(FollowOneReference, self).__init__(name=name)

        self.name = name
        self.r = Reference()
        self.r.lat = lat*pi/180
        self.r.lon = lon*pi/180
        self.radius = radius
        self.r.z.value = z
        self.r.z.z_units = 1
        self.r.speed.value = speed
        self.r.flags = imcpy.Reference.FlagsBits.LOCATION | imcpy.Reference.FlagsBits.SPEED | imcpy.Reference.FlagsBits.Z | imcpy.Reference.FlagsBits.MANDONE

        self.last_reference = self.r
        self.last_reference.flags = self.r.flags | imcpy.Reference.FlagsBits.MANDONE
        
        
        

    def setup(self, **kwargs):
        """
        Setup the publisher which will stream commands to the mock robot.

        Args:
            **kwargs (:obj:`dict`): look for the 'node' object being passed down from the tree

        Raises:
            :class:`KeyError`: if a ros2 node isn't passed under the key 'node' in kwargs
        """
        self.logger.info("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        
        # First we need to send FollowReference only once
        self.fr_publisher = self.node.create_publisher(
            msg_type = FollowReference,
            topic ='to_imc/follow_reference',
            qos_profile=py_trees_ros.utilities.qos_profile_latched()
        )

        self.reference_pub = self.node.create_publisher(
            msg_type = Reference,
            topic = 'to_imc/reference',
            qos_profile=py_trees_ros.utilities.qos_profile_latched()
        )

        fr = FollowReference()
        fr.control_src = 0xFFFF  # Controllable from all IMC adresses
        fr.control_ent = 0xFF  # Controllable from all entities
        fr.timeout = 10.0  # Maneuver stops when time since last Reference message exceeds this value
        fr.loiter_radius = -1.  # Default loiter radius when waypoint is reached
        fr.altitude_interval = 0.
        self.fr_publisher.publish(fr)
        self.reference_pub.publish(self.r)
        self.feedback_message = "FollowReference sent"

        self.stopped = False
        self.reached = False
        self.waiting = False
        self.end = False

    def update(self) -> py_trees.common.Status:
        """
        Annoy the led strip to keep firing every time it ticks over (the led strip will clear itself
        if no command is forthcoming within a certain period of time).
        This behaviour will only finish if it is terminated or priority interrupted from above.

        Returns:
            Always returns :attr:`~py_trees.common.Status.RUNNING`
        """
        if not self.end:
            bb=py_trees.blackboard.Blackboard()
            key,val = bb.key_with_attributes('/FollowRefState')
            fr_msg=bb.storage.get('/FollowRefState')
            vs_msg=bb.storage.get('/VehicleState')
            if fr_msg is not None or fr_msg == 0 and not self.stopped:
                self.logger.info("msg {}".format(fr_msg))
                if fr_msg.state == imcpy.FollowRefState.StateEnum.GOTO:
                    # In goto maneuver
                    self.feedback_message = "Goto"
                    if fr_msg.proximity & imcpy.FollowRefState.ProximityBits.XY_NEAR:
                        # Near XY - send next reference
                        self.feedback_message = "------ Near XY"
                        self.logger.info("NEAR")
                        self.reached = True
                        # return py_trees.common.Status.SUCCESS
                    self.reference_pub.publish(self.r)
                    # return py_trees.common.Status.RUNNING

                elif fr_msg.state in (imcpy.FollowRefState.StateEnum.LOITER, imcpy.FollowRefState.StateEnum.HOVER):
                    # Loitering/hovering/waiting - send next reference
                    self.feedback_message = "Point reached: Loiter/Hover"
                    self.logger.info("LOITER")
                    self.reached = True
                    # return py_trees.common.Status.RUNNING

                elif fr_msg.state == imcpy.FollowRefState.StateEnum.WAIT:
                    # Loitering/hovering/waiting - send next reference
                    self.feedback_message = "Waiting"
                    self.reference_pub.publish(self.r)
                    if self.reached:
                        self.waiting = True
                    # return py_trees.common.Status.RUNNING
                
                elif fr_msg.state == imcpy.FollowRefState.StateEnum.ELEVATOR:
                    # Moving in z-direction after reaching reference cylinder
                    self.feedback_message = "Elevator"
                    self.logger.info("ELEVATORR")
                    self.reference_pub.publish(self.r)
                    # return py_trees.common.Status.RUNNING

                elif fr_msg.state == imcpy.FollowRefState.StateEnum.TIMEOUT:
                    # Controlling system timed out
                    self.feedback_message = "TIMEOUT"
                    return py_trees.common.Status.FAILURE
                elif fr_msg.state ==0:
                    self.feedback_message = "Waiting for state msg"
                    fr = FollowReference()
                    fr.control_src = 0xFFFF  # Controllable from all IMC adresses
                    fr.control_ent = 0xFF  # Controllable from all entities
                    fr.timeout = 10.0  # Maneuver stops when time since last Reference message exceeds this value
                    fr.loiter_radius = -1.  # Default loiter radius when waypoint is reached
                    fr.altitude_interval = 0.
                    self.fr_publisher.publish(fr)
                    
            if fr_msg is None and not self.stopped:
                self.feedback_message = "Waiting for state msg"
                fr = FollowReference()
                fr.control_src = 0xFFFF  # Controllable from all IMC adresses
                fr.control_ent = 0xFF  # Controllable from all entities
                fr.timeout = 10.0  # Maneuver stops when time since last Reference message exceeds this value
                fr.loiter_radius = -1.  # Default loiter radius when waypoint is reached
                fr.altitude_interval = 0.
                self.fr_publisher.publish(fr)
                return py_trees.common.Status.RUNNING
            
            if vs_msg == 0 and self.stopped:
                # key,val = bb.key_with_attributes('/FollowRefState')
                # msg=bb.storage.get('/FollowRefState')
                # bb.set(key, None)
                # key,val = bb.key_with_attributes('/FollowRefState')
                # msg=bb.storage.get('/FollowRefState')
                # msg = None
                # key = None
                frstate_previous = py_trees.blackboard.Blackboard().get('/FollowRefState')
                frstate_previous.state = 0
                py_trees.blackboard.Blackboard().set('/FollowRefState', frstate_previous)

                self.feedback_message = "Plan stopped"
                self.logger.info("send success ---------------------")
                return py_trees.common.Status.SUCCESS
            elif self.reached and not self.stopped:
                end_pub = self.node.create_publisher(
                    msg_type = PlanControl,
                    topic = 'to_imc/plan_control',
                    qos_profile=py_trees_ros.utilities.qos_profile_latched()
                )

                end_r = PlanControl()
                end_r.type = 0 # imcpy.PlanControl.TypeEnum.REQUEST
                end_r.op = 1 # imcpy.PlanControl.OperationEnum.STOP
            
                end_pub.publish(end_r)
                self.stopped = True
                
                self.feedback_message = "Stopping plan"
                return py_trees.common.Status.RUNNING
            else:
                return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS


        

    def terminate(self, new_status: py_trees.common.Status):
        """
        Shoot off a clearing command to the led strip.

        Args:
            new_status: the behaviour is transitioning to this new status
        """
        self.logger.debug("SENDING STOP PLAN")
        self.end = True
        # if not self.stopped:
            # end_pub = self.node.create_publisher(
            #     msg_type = PlanControl,
            #     topic = 'to_imc/plan_control',
            #     qos_profile=py_trees_ros.utilities.qos_profile_latched()
            # )

            # end_r = PlanControl()
            # end_r.type = 0 # imcpy.PlanControl.TypeEnum.REQUEST
            # end_r.op = 1 # imcpy.PlanControl.OperationEnum.STOP
        
            # end_pub.publish(end_r)
            # self.stopped = True

        self.feedback_message = "cleared"
