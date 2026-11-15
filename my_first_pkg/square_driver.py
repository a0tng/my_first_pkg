#!/usr/bin/env python3
"""4주차 강의 예제 — 서비스로 켜고 끄는 원 그리기 노드.

  ros2 run ire_week04_service_launch circle_driver
  ros2 service call /start_stop std_srvs/srv/SetBool "{data: true}"
  ros2 param set /circle_driver speed 1.5
"""
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from turtlesim.msg import Pose


class SquareDriver(Node):
    def __init__(self):
        super().__init__('square_driver')
        self.declare_parameter('speed', 1.0)      # 선속도 [m/s]
        self.declare_parameter('turn', 0.01)       # 각속도 [rad/s]
        self.declare_parameter('side_length', 2.0) # 변의 길이 [m]
        self.running = False
        self.state = 'straight'
        self.count = 0
        self.theta = 0.0
        self.base_theta = 0.0
        self.target_theta = 0.0
        self.turn_count = 0
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.on_pose, 10)
        self.srv = self.create_service(SetBool, 'start_stop', self.on_call)
        self.create_timer(0.02, self.on_timer)

    def on_call(self, req, res):
        self.running = req.data
        if req.data:
            self.state = 'straight'
            self.count = 0
            self.turn_count = 0
            self.base_theta = self.theta
            self.target_theta = self.theta

        res.success = True
        res.message = 'running' if req.data else 'stopped'
        self.get_logger().info(res.message)
        return res    

    def on_pose(self, msg):
        self.theta = msg.theta

    def on_timer(self):
        if not self.running:
            return
        msg = Twist()
        if self.state == 'straight':
            msg.linear.x = self.get_parameter('speed').value
            msg.angular.z = 0.0
            self.count += 1
            
            if self.count >= self.get_parameter('side_length').value / (self.get_parameter('speed').value * 0.02):  # 0.02초마다 호출되므로, 속도 * 시간 = 거리
                self.state = 'turn'
                self.count = 0
                self.turn_count += 1
                self.target_theta = (self.base_theta + self.turn_count * (math.pi / 2))  # 90도 회전
                self.target_theta = math.atan2(math.sin(self.target_theta), math.cos(self.target_theta))  # -pi ~ pi 범위로 조정

        elif self.state == 'turn':
            msg.linear.x = 0.0
            angle_error = math.atan2(math.sin(self.target_theta - self.theta), math.cos(self.target_theta - self.theta))
            if abs(angle_error) < 0.01:  # 목표 각도에 거의 도달했으면
                msg.angular.z = 0.0
                self.state = 'straight'
                self.count = 0
            else:
                if angle_error > 0:
                    msg.angular.z = self.get_parameter('turn').value
                else:
                    msg.angular.z = -self.get_parameter('turn').value
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SquareDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
