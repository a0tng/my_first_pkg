#!/usr/bin/env python3
"""11주차 — Pick & Place.

bringup(또는 gazebo) 상태에서:
  ros2 run ire_week11_moveit pick_demo

제공하는 것: 팔 관절 궤적 전송 + 그리퍼 열기/닫기 + 예시 시퀀스.
학생 몫: 지정 pick → place 시퀀스 설계

주의: POSES의 관절값은 예시다 — 실물에서 teleop으로 팔을 움직여
      /joint_states 를 echo 하면서 자기 값을 찾아 바꿔 넣을 것.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from builtin_interfaces.msg import Duration

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']

POSES = {                       # [joint1, joint2, joint3, joint4] (rad) — 예시!
    'home':     [0.0, -1.501, 0.8029, 0.6981],
    'ready1':    [0.0, 0.0873, 0.0, -0.1222],
    'approach1': [0.0,  0.8727, -0.1047, -0.7854],
    'lift1':     [0.0,  -0.7156, 0.6458, 0.0873],
    'ready2':    [1.0996, -0.7156, 0.6458, 0.0873],
    'approach2': [1.0996,  0.4189, 0.5585, -0.9774],
}


class PickDemo(Node):
    def __init__(self):
        super().__init__('pick_demo')
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.grip = ActionClient(self, GripperCommand,
                                 '/gripper_controller/gripper_cmd')

    def arm_to(self, name, sec=2.0):
        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = POSES[name]
        pt.time_from_start = Duration(sec=int(sec), nanosec=int((sec % 1) * 1e9))
        traj.points = [pt]
        self.arm_pub.publish(traj)
        self.get_logger().info(f'arm → {name}')

    def gripper(self, open_):
        goal = GripperCommand.Goal()
        goal.command.position = 0.015 if open_ else -0.01   # 예시값 — 물체 크기에 맞게
        goal.command.max_effort = 5.0
        self.grip.wait_for_server()
        self.grip.send_goal_async(goal)
        self.get_logger().info('gripper ' + ('open' if open_ else 'close'))


def main():
    rclpy.init()
    node = PickDemo()
    import time
    seq = [('home', None), (None, True), ('ready1', None), ('approach1', None), (None, False),
           ('lift1', None), ('ready2', None), ('approach2', None), (None, True), ('home', None)]
    for pose, grip in seq:                      # 예시 시퀀스 — 미션에선 직접 설계
        if pose: node.arm_to(pose)
        if grip is not None: node.gripper(grip)
        time.sleep(3.0)
    node.get_logger().info('sequence done')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
