import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import time
from imc_ros_msgs.action import FollowSingleReference
from .ros2imc import FollowSingleRef


class FollowSingleReferenceActionServer(Node):

    def __init__(self):
        super().__init__('FollowSingleReference_action_server')
        self._action_server = ActionServer(
            self, #ros2 node to add to action
            FollowSingleReference, # type of action
            'FollowSingleReference', # action name
            self.execute_callback) #callback for executing goals

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = FollowSingleReference.Feedback()
        feedback_msg.state = 0

        dune_actor = FollowSingleRef(goal_handle=goal_handle, node_name='actionnode', lat = goal_handle._goal_request.lat, lon = goal_handle._goal_request.lon, depth = goal_handle._goal_request.z, speed= goal_handle._goal_request.speed, target_name='lauv-simulator-1')

        self.get_logger().info('Feedback: {0}'.format(feedback_msg.state))
        goal_handle.publish_feedback(feedback_msg)
        time.sleep(1)

        goal_handle.succeed()
        result = FollowSingleReference.Result()
        result.reference_reached = True
        return result


def main(args=None):
    rclpy.init(args=args)

    FollowSingleReference_action_server = FollowSingleReferenceActionServer()

    rclpy.spin(FollowSingleReference_action_server)


if __name__ == '__main__':
    main()