import threading
import time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from std_msgs.msg import String


class Core(Node):

    def __init__(self):
        super().__init__(
            'core',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )

        self.target_marker_pub = self.create_publisher(
            Int32,
            '/target_marker_id',
            10
        )
        self.manipulator_pub = self.create_publisher(
            String,
            '/manipulator_control',
            10
        )

        self.scenario_sub = self.create_subscription(
            String,
            '/scenario_selection',
            self.scenario_callback,
            10
        )
        self.is_parking_completed_sub = self.create_subscription(
            Bool,
            '/is_parking_completed',
            self.parking_callback,
            10
        )
        self.is_manipulator_completed_sub = self.create_subscription(
            Bool,
            '/is_manipulator_completed',
            self.manipulator_callback,
            10
        )

        self.selected_scenario = None
        self.scenario_running = False
        self.parking_event = threading.Event()
        self.manipulator_event = threading.Event()

    def scenario_callback(self, msg: String):
        if not self.scenario_running:
            self.selected_scenario = msg.data
            self.get_logger().info(f'Selected scenario: {self.selected_scenario}')
            thread = threading.Thread(target=self.run_scenario)
            thread.start()
        else:
            self.get_logger().info('A scenario is already running.')

    def parking_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Parking completed')
            self.parking_event.set()

    def manipulator_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Manipulator task completed.')
            self.manipulator_event.set()

    def list_to_pose_stamped(self, pose_list: list) -> PoseStamped:
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = pose_list[0]
        pose_msg.pose.position.y = pose_list[1]
        pose_msg.pose.position.z = pose_list[2]
        pose_msg.pose.orientation.x = pose_list[3]
        pose_msg.pose.orientation.y = pose_list[4]
        pose_msg.pose.orientation.z = pose_list[5]
        pose_msg.pose.orientation.w = pose_list[6]
        return pose_msg

    def run_scenario(self):
        self.scenario_running = True

        target_marker_id = 'scenario.' + self.selected_scenario + '.target_marker_id'
        object_name = 'scenario.' + self.selected_scenario + '.object_name'
        goal_marker_id = 'scenario.' + self.selected_scenario + '.goal_marker_id'
        goal_pose_list = 'scenario.' + self.selected_scenario + '.goal_pose'
        end_pose_list = 'scenario.' + self.selected_scenario + '.end_pose'

        target_marker_id = self.get_parameter(
            target_marker_id).get_parameter_value().integer_value
        object_name = self.get_parameter(
            object_name).get_parameter_value().string_value
        goal_marker_id = self.get_parameter(
            goal_marker_id).get_parameter_value().integer_value
        goal_pose_list = self.get_parameter(
            goal_pose_list).get_parameter_value().double_array_value
        end_pose_list = self.get_parameter(
            end_pose_list).get_parameter_value().double_array_value

        marker_msg = Int32()
        marker_msg.data = target_marker_id
        self.parking_event.clear()
        self.target_marker_pub.publish(marker_msg)

        if not self.parking_event.wait(timeout=90.0):
            self.get_logger().error('Parking did not complete within timeout.')
            self.scenario_running = False
            return

        pick_msg = String()
        pick_msg.data = f'pick_target:{object_name}'
        self.manipulator_event.clear()
        self.manipulator_pub.publish(pick_msg)

        if not self.manipulator_event.wait(timeout=30.0):
            self.get_logger().error('Manipulator pick_target did not complete within timeout.')
            self.scenario_running = False
            return
        self.get_logger().info('Manipulator pick_target completed.')

        self.get_logger().info('Sending navigation goal to target location.')
        nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.list_to_pose_stamped(goal_pose_list)

        if not nav_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Navigation action server not available!')
            self.scenario_running = False
            return

        send_goal_future = nav_action_client.send_goal_async(goal_msg)
        while not send_goal_future.done():
            time.sleep(0.1)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected!')
            self.scenario_running = False
            return

        self.get_logger().info('Navigation goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.1)

        result = result_future.result()
        if result.status != 4:
            self.get_logger().error('Navigation failed!')
            self.scenario_running = False
            return

        self.get_logger().info('Navigation reached target location.')

        goal_marker_msg = Int32()
        goal_marker_msg.data = goal_marker_id
        self.parking_event.clear()
        self.target_marker_pub.publish(goal_marker_msg)

        if not self.parking_event.wait(timeout=90.0):
            self.get_logger().error('Parking did not complete within timeout.')
            self.scenario_running = False
            return

        place_msg = String()
        place_msg.data = f'place_target:{object_name}'
        self.manipulator_event.clear()
        self.manipulator_pub.publish(place_msg)

        if not self.manipulator_event.wait(timeout=30.0):
            self.get_logger().error('Manipulator place_target did not complete within timeout.')
            self.scenario_running = False
            return
        self.get_logger().info('Manipulator place_target completed.')

        self.get_logger().info('Sending navigation goal to home.')

        home_goal = NavigateToPose.Goal()
        home_goal.pose = self.list_to_pose_stamped(end_pose_list)

        send_home_future = nav_action_client.send_goal_async(home_goal) 

        while not send_home_future.done():
            time.sleep(0.1)

        home_goal_handle = send_home_future.result()

        if not home_goal_handle.accepted:
            self.get_logger().error('Home navigation goal rejected!')
            self.scenario_running = False
            return

        home_result_future = home_goal_handle.get_result_async()

        while not home_result_future.done():
            time.sleep(0.1)

        home_result = home_result_future.result()

        if home_result.status != 4:
            self.get_logger().error('Navigation to home failed!')
            self.scenario_running = False
            return

        self.get_logger().info('Navigation to home completed.')
        self.get_logger().info('Scenario completed.')
        self.scenario_running = False


def main(args=None):
    rclpy.init(args=args)
    node = Core()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
