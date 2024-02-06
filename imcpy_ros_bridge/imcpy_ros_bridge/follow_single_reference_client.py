import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from imc_ros_msgs.action import FollowSingleReference


class FollowSingleReferenceActionClient(Node):

    def __init__(self):
        super().__init__('FollowSingleReference_action_client')
        self._action_client = ActionClient(self, FollowSingleReference, 'FollowSingleReference')

    def send_goal(self, lat, lon, z, speed):
        goal_msg = FollowSingleReference.Goal()
        goal_msg.lat = lat 
        goal_msg.lon = lon
        goal_msg.z = z
        goal_msg.speed = speed

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.reference_reached))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.state))


def main(args=None):
    rclpy.init(args=args)

    action_client = FollowSingleReferenceActionClient()

    action_client.send_goal(41.18469722, -8.70514722, 2., 1.6,)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()