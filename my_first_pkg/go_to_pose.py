import rclpy

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from std_srvs.srv import Trigger


class InitialPoseNode(BasicNavigator):

    def __init__(self):
        super().__init__()

        self.service = self.create_service(
            Trigger,
            '/set_initial_pose',
            self.set_initial_pose_callback
        )

        self.get_logger().info('Initial pose service ready.')


    def set_initial_pose_callback(self, request, response):

        start_point = PoseStamped()

        start_point.header.frame_id = 'map'
        start_point.header.stamp = self.get_clock().now().to_msg()

        start_point.pose.position.x = 0.0
        start_point.pose.position.y = 0.0
        start_point.pose.position.z = 0.0

        start_point.pose.orientation.x = 0.0
        start_point.pose.orientation.y = 0.0
        start_point.pose.orientation.z = 0.0
        start_point.pose.orientation.w = 1.0

        self.setInitialPose(start_point)

        self.get_logger().info(
            'Initial pose set: x=0.0, y=0.0, yaw=0.0'
        )

        response.success = True
        response.message = 'Initial pose set.'

        return response


def main(args=None):

    rclpy.init(args=args)

    node = InitialPoseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()