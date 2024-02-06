import rclpy
import logging, sys
import time
import socket

import asyncio
import inspect
from contextlib import suppress

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from imc_ros_msgs.msg import EstimatedState
logger = logging.getLogger('examples.FollowRef')
from imcpy.network.udp import IMCProtocolUDP, IMCSenderUDP


import imcpy
from imcpy.actors.dynamic import DynamicActor
from imcpy.decorators import Periodic, RunOnce, Subscribe
import numpy as np
from imc_ros_msgs.action import FollowSingleReference

# Acknowledgements: Laszlo u da best


class FollowSingleRef(DynamicActor):
    

    def __init__(self, goal_handle = None, lat = 41.1854111111111, lon = -8.705886111111111, depth = 2., speed = 1.6, node_name='imcpy_single_ref', target_name='lauv-simulator-1'):
        super().__init__() 
        self.name = node_name+str(int(time.time()*1000)) 
        self.ros_node = Node(self.name)
        self.ros_node.get_logger().info('TTTTTTTTTTTTTTTTTTTTTTHIS IS WHEN I INNNNNIIITTT INIT')
        self.goal_handle = None
        if goal_handle:
            self.goal_handle = goal_handle
        
        self.pose_publisher_                = self.ros_node.create_publisher(PoseStamped,       'base_link', 10)
        self.EE_publisher_                  = self.ros_node.create_publisher(EstimatedState,    'estimated_state', 10)
        self.state = None
        self.last_ref = False

        self.lat = lat
        self.lon = lon 
        self.depth = depth
        self.speed = speed
        # IMC STUFF
        self.target_name = target_name
        # This list contains the target systems to maintain communications with
        self.heartbeat.append(target_name)       

        self.near = False
        self._run_task = None

    def run_async_function(self):
        self.setup_event_loop()
        try:
            self._loop.run_forever()
        except KeyboardInterrupt:
            pending = asyncio.all_tasks(self._loop)
            for task in pending:
                task.cancel()
                # Now we should await task to execute it's cancellation.
                # Cancelled task raises asyncio.CancelledError that we can suppress when canceling
                with suppress(asyncio.CancelledError):
                    self._loop.run_until_complete(task)
        finally:
            self._loop.close()
                    
    def setup_event_loop(self):
        """
        Setup of event loop and decorated functions
        """
        # Add event loop to instance
        self.ros_node.get_logger().info('SETUP EVENT LOOP')
        if not self._loop:
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)

        decorated = [(name, method) for name, method in inspect.getmembers(self) if hasattr(method, '_decorators')]
        for name, method in decorated:
            for decorator in method._decorators:
                decorator.add_event(self._loop, self, method)

                if type(decorator) is Subscribe:
                    # Collect subscribed message types for each function
                    for msg_type in decorator.subs:
                        try:
                            self._subs[msg_type].append(method)
                        except (KeyError, AttributeError):
                            self._subs[msg_type] = [method]

        # Sort subscriptions by position in inheritance hierarchy (parent classes are called first)
        cls_hier = [x.__qualname__ for x in inspect.getmro(type(self))]
        for msg_type, methods in self._subs.items():
            methods.sort(key=lambda x: -cls_hier.index(x.__qualname__.split('.')[0]))

        # Subscriptions has been collected from all decorators
        # Add asyncio datagram endpoints to event loop
        self.start_subscriptions()
    
    def start_subscriptions(self):
        """
        Add asyncio datagram endpoint for all subscriptions
        """
        try:
            self.ros_node.get_logger().info('******************STARTING SUBSCRIPTIONS****************')
            # Add datagram endpoint for multicast announce
            multicast_listener = self._loop.create_datagram_endpoint(lambda: IMCProtocolUDP(self, is_multicast=True),
                                                                    family=socket.AF_INET)

            # Add datagram endpoint for UDP IMC messages
            imc_listener = self._loop.create_datagram_endpoint(lambda: IMCProtocolUDP(self,
                                                                                    is_multicast=False,
                                                                                    static_port=self.static_port),
                                                            family=socket.AF_INET)
            self.multicast_listener = multicast_listener
            self.imc_listener = imc_listener
            if sys.version_info < (3, 4, 4):
                with suppress(asyncio.CancelledError):
                    self._task_mc = self._loop.create_task(multicast_listener)
                    self._task_imc = self._loop.create_task(imc_listener)
            else:
                with suppress(asyncio.CancelledError):
                    self._task_mc = asyncio.ensure_future(multicast_listener)
                    self._task_imc = asyncio.ensure_future(imc_listener)
        except Exception as e:
            self.ros_node.get_logger().info('CAUGHT EXCEPTION EE')
            self.ros_node.get_logger().info(e)

    def send_reference(self, node_id, final=False):
        """
        After the FollowReferenceManeuver is started, references must be sent continously
        """
        
        node = self.resolve_node_id(node_id)
        try:

            node = self.resolve_node_id(node_id)
            r = imcpy.Reference()
            r.lat = self.lat*np.pi/180  # Target waypoint
            r.lon = self.lon*np.pi/180  # Target waypoint
            r.radius = -1

            # Assign z
            dz = imcpy.DesiredZ()
            dz.value = self.depth
            dz.z_units = imcpy.ZUnits.DEPTH
            r.z = dz

            # Assign the speed
            ds = imcpy.DesiredSpeed()
            ds.value = self.speed
            ds.speed_units = imcpy.SpeedUnits.METERS_PS
            r.speed = ds

            # Bitwise flags (see IMC spec for explanation)
            flags = imcpy.Reference.FlagsBits.LOCATION | imcpy.Reference.FlagsBits.SPEED | imcpy.Reference.FlagsBits.Z
            flags = flags | imcpy.Reference.FlagsBits.MANDONE if final else flags
            r.flags = flags
            self.ros_node.get_logger().info('Sending reference')
            self.last_ref = r
            self.send(node, r)
            return False
        except KeyError:
            pass

    def stop_Plan(self):
        # follow ref finished
        # self.send_reference(node_id=self.target_name,final=True)
        # Stop plan
        pc = imcpy.PlanControl()
        pc.type = imcpy.PlanControl.TypeEnum.REQUEST
        pc.op = imcpy.PlanControl.OperationEnum.STOP
        pc.plan_id = self.name

        # Send the IMC message to the node
        node = self.resolve_node_id(self.target_name)
        self.send(node, pc)
        self.stop()
        # try:
        #     self._task_mc.cancel()
        # except asyncio.CancelledError:
        #     pass  # Handle the CancelledError if needed
        # except Exception as e:
        #     # Handle other exceptions that may occur during cancellation
        #     print(f"An error occurred during taskmc cancellation: {e}")
        # try:
        #     self._task_imc.cancel()
        # except asyncio.CancelledError:
        #     pass  # Handle the CancelledError if needed
        # except Exception as e:
        #     # Handle other exceptions that may occur during cancellation
        #     print(f"An error occurred during taskimc cancellation: {e}")
        
        
        self.ros_node.destroy_node()
        self.remove_node(node)
        self.near = True
        self.ros_node.get_logger().info('***********Stop FollowRef command*************')
        # if self._task_mc.cancelled():
        #     self.ros_node.get_logger().info('mc task cancelled')
        # else:
        #     self.ros_node.get_logger().info('mc task NOT cancelled')
        
        # if self._task_imc.cancelled():
        #     self.ros_node.get_logger().info('imc task cancelled')
        # else:
        #     self.ros_node.get_logger().info('imc task NOT cancelled')
        return True

    def mystop(self):
        """
        Cancels all running tasks and stops the event loop.
        :return:
        """
        loop = self._loop

        self.ros_node.get_logger().info('Stop called by user. Cancelling all running tasks.')
        for task in asyncio.all_tasks(loop):
            task.cancel()
            with suppress(asyncio.CancelledError):
                    self._loop.run_until_complete(task)

        async def exit_event_loop():
            self.ros_node.get_logger().info('Tasks cancelled. Stopping event loop.')
            loop.stop()

        asyncio.ensure_future(exit_event_loop())


        pending = asyncio.all_tasks(self._loop)
        for task in pending:
            task.cancel()
            # Now we should await task to execute it's cancellation.
            # Cancelled task raises asyncio.CancelledError that we can suppress when canceling
            with suppress(asyncio.CancelledError):
                self._loop.run_until_complete(task)
        
    @Periodic(10)
    def send_FollowReference(self):
        """
        If target is connected, start the FollowReferenceManeuver
        """
        if not self.state:
            try:
                # This function resolves the map of connected nodes
                node = self.resolve_node_id(self.target_name)

                # Create FollowReference msg
                fr = imcpy.FollowReference()
                fr.control_src = 0xFFFF  # Controllable from all IMC adresses
                fr.control_ent = 0xFF  # Controllable from all entities
                fr.timeout = 10.0  # Maneuver stops when time since last Reference message exceeds this value
                fr.loiter_radius = -1  # Default loiter radius when waypoint is reached
                fr.altitude_interval = 0
                
                # Add to PlanManeuver message
                pman = imcpy.PlanManeuver()
                pman.data = fr
                pman.maneuver_id = self.name

                # Add to PlanSpecification
                spec = imcpy.PlanSpecification()
                spec.plan_id = self.name
                spec.maneuvers.append(pman)
                spec.start_man_id = self.name
                spec.description = 'A test plan sent from imcpy'

                # Start plan
                pc = imcpy.PlanControl()
                pc.type = imcpy.PlanControl.TypeEnum.REQUEST
                pc.op = imcpy.PlanControl.OperationEnum.START
                pc.plan_id = self.name
                pc.arg = spec

                # Send the IMC message to the node
                self.send(node, pc)
                self.ros_node.get_logger().info('***********Started FollowRef command*************')
                self.send_reference(node_id=self.target_name)
                return False

            except KeyError as e:
                # Target system is not connected
                self.ros_node.get_logger().info('Could not deliver GOTO.')
                return False


    @Subscribe(imcpy.FollowRefState)
    def recv_plandbstate(self, msg: imcpy.FollowRefState):
        self.ros_node.get_logger().info('YOYOYOYOYOO1111111')
        if self.goal_handle:
            feedback_msg = FollowSingleReference.Feedback()
            feedback_msg.state = 0
            self.goal_handle.publish_feedback(feedback_msg)

        self.state = msg.state

        if msg.state == imcpy.FollowRefState.StateEnum.GOTO:
            # In goto maneuver
            self.ros_node.get_logger().info('Goto')
            if msg.proximity & imcpy.FollowRefState.ProximityBits.XY_NEAR:
                # Near XY - send next reference
                self.ros_node.get_logger().info('-------------- Near XY')
                self.last_ref = False
                self.stop_Plan()
        elif msg.state in (imcpy.FollowRefState.StateEnum.LOITER, imcpy.FollowRefState.StateEnum.HOVER):
            # Loitering/hovering/waiting - send next reference
            self.ros_node.get_logger().info('Point reached: Loiter/Hover')
            time.sleep(1)
            self.stop_Plan()
        elif msg.state == imcpy.FollowRefState.StateEnum.WAIT:
            # Loitering/hovering/waiting - send next reference
            self.ros_node.get_logger().info('Waiting')
            time.sleep(1)
            self.send_reference(node_id=self.target_name)
        
        elif msg.state == imcpy.FollowRefState.StateEnum.ELEVATOR:
            # Moving in z-direction after reaching reference cylinder
            self.ros_node.get_logger().info('Elevator')
        elif msg.state == imcpy.FollowRefState.StateEnum.TIMEOUT:
            # Controlling system timed out
            self.ros_node.get_logger().info('Timeout')
        return False

    @Subscribe(imcpy.EstimatedState)
    def recv_estate(self, msg: imcpy.EstimatedState):
        self.imc_EE_to_ros(msg)
        return False 
    
    @Periodic(5.0)
    def periodic_ref(self):
        if self.last_ref:
            try:
                self.send(self.target_name, self.last_ref)
            except KeyError:
                pass
        return False
  
    def imc_EE_to_ros(self, imc_msg: imcpy.EstimatedState):
        # Publish on PoseStamped (for UNavSim)
        msg = PoseStamped()
        msg.header.stamp = self.ros_node.get_clock().now().to_msg()
        msg.header.frame_id = 'dune_map'
        msg.pose.position.x = imc_msg.x
        msg.pose.position.y = imc_msg.y
        msg.pose.position.z = imc_msg.z
        msg.pose.orientation.x = imc_msg.phi
        msg.pose.orientation.y = imc_msg.theta
        msg.pose.orientation.z = imc_msg.psi
        self.pose_publisher_.publish(msg)
        # Publish on Estimated State (for ROS2)
        msg = EstimatedState()
        msg.x = imc_msg.x
        msg.y = imc_msg.y
        msg.z = imc_msg.z
        msg.phi = imc_msg.phi
        msg.theta = imc_msg.theta
        msg.psi = imc_msg.psi
        msg.u = imc_msg.u
        msg.v = imc_msg.v
        msg.w = imc_msg.w
        msg.p = imc_msg.p
        msg.q = imc_msg.q
        msg.r = imc_msg.r
        msg.depth = imc_msg.depth
        msg.alt = imc_msg.alt
        msg.lat = imc_msg.lat
        msg.lon = imc_msg.lon
        msg.height = imc_msg.height
        self.EE_publisher_.publish(msg)

        self.ros_node.get_logger().debug('Published Estimated State {}.'.format(imc_msg.x))
        return False

def main():
    print('Hi from ros2imc.')
    rclpy.init()
    logging.basicConfig(stream=sys.stdout, level=logging.INFO)
    imcpysingleref = FollowSingleRef(node_name='imcpy_single_ref', target_name='lauv-simulator-1')


    imcpysingleref.ros_node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
