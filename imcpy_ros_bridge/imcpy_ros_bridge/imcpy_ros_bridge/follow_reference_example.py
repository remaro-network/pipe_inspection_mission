import logging, math, sys

import imcpy
from imcpy.actors import DynamicActor
from imcpy.decorators import Subscribe, Periodic
import numpy as np

logger = logging.getLogger('examples.FollowRef')


class FollowSingleRef(DynamicActor):
    def __init__(self, lat = 41.1854111111111, lon = -8.705886111111111, depth = 2., speed = 1.6, target_name='lauv-simulator-1'):
        super().__init__()
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
        # This command starts the asyncio event loop
        self.run()

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
            logger.info('Sending reference')
            self.last_ref = r
            self.send(node, r)
            return False
        except KeyError:
            pass

    def stop_Plan(self):
        # Stop plan
        pc = imcpy.PlanControl()
        pc.type = imcpy.PlanControl.TypeEnum.REQUEST
        pc.op = imcpy.PlanControl.OperationEnum.STOP
        pc.plan_id = 'FollowReference'

        # Send the IMC message to the node
        node = self.resolve_node_id(self.target_name)
        self.send(node, pc)
        self.stop()
        logger.info('***********Stop FollowRef command*************')
        return True
    
    def is_from_target(self, msg):
        """
        Check that incoming message is from the target system
        """
        try:
            node = self.resolve_node_id(msg)
            return node.sys_name == self.target
        except KeyError:
            return False

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
                fr.loiter_radius = 0  # Default loiter radius when waypoint is reached
                fr.altitude_interval = 0
                
                # Add to PlanManeuver message
                pman = imcpy.PlanManeuver()
                pman.data = fr
                pman.maneuver_id = 'FollowReferenceManeuver'

                # Add to PlanSpecification
                spec = imcpy.PlanSpecification()
                spec.plan_id = 'FollowReference'
                spec.maneuvers.append(pman)
                spec.start_man_id = 'FollowReferenceManeuver'
                spec.description = 'A test plan sent from imcpy'

                # Start plan
                pc = imcpy.PlanControl()
                pc.type = imcpy.PlanControl.TypeEnum.REQUEST
                pc.op = imcpy.PlanControl.OperationEnum.START
                pc.plan_id = 'FollowReference'
                pc.arg = spec

                # Send the IMC message to the node
                self.send(node, pc)
                logger.info('***********Started FollowRef command*************')
                return False

            except KeyError as e:
                # Target system is not connected
                logger.info('Could not deliver GOTO.')
                return False

    @Subscribe(imcpy.FollowRefState)
    def recv_plandbstate(self, msg: imcpy.FollowRefState):

        self.state = msg.state
        if msg.state == imcpy.FollowRefState.StateEnum.GOTO:
            # In goto maneuver
            logger.info('Goto')
            if msg.proximity & imcpy.FollowRefState.ProximityBits.XY_NEAR:
                # Near XY - send next reference
                logger.info('-------------- Near XY')
                self.last_ref = False
                self.stop_Plan()
        elif msg.state in (imcpy.FollowRefState.StateEnum.LOITER, imcpy.FollowRefState.StateEnum.HOVER, imcpy.FollowRefState.StateEnum.WAIT):
            # Loitering/hovering/waiting - send next reference
            logger.info('Waiting')
            self.send_reference(node_id=self.target_name)
        elif msg.state == imcpy.FollowRefState.StateEnum.ELEVATOR:
            # Moving in z-direction after reaching reference cylinder
            logger.info('Elevator')
        elif msg.state == imcpy.FollowRefState.StateEnum.TIMEOUT:
            # Controlling system timed out
            logger.info('Timeout')
        return False

    @Subscribe(imcpy.EstimatedState)
    def recv_estate(self, msg: imcpy.EstimatedState):
        return False 
    
    @Periodic(5.0)
    def periodic_ref(self):
        if self.last_ref:
            try:
                self.send(self.target_name, self.last_ref)
            except KeyError:
                pass
        return False

def main():
    print('Hi from ros2imc.')
    logging.basicConfig(stream=sys.stdout, level=logging.INFO)
    ros2imc = FollowSingleRef(target_name='lauv-simulator-1')

if __name__ == '__main__':
    main()
    

