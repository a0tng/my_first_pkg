#!/usr/bin/env python3
"""10주차 강의 예제 — 코드로 Nav2에 목표를 보낸다 (RViz 클릭의 코드 버전).

Nav2가 이미 떠 있는 상태에서:
  ros2 run ire_week10_nav_demo go_to_pose

  git add. git commit -m "update" git push

  ros2 topic echo /clicked_point
"""
import rclpy
import math
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def make_pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()

    p.pose.position.x = float(x)
    p.pose.position.y = float(y)

    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def main():
    rclpy.init()
    nav = BasicNavigator()                     # AMCL·planner 준비 대기

    start_point = make_pose(nav, 0.0, 0.0, 0.0)
    nav.setInitialPose(start_point)

    nav.waitUntilNav2Active() 

    goals = [
        (0.95,0.93),
        (0.93, -0.04)
    ]

    for i, (x, y) in enumerate(goals):

        if i < len(goals) - 1:
            next_x, next_y = goals[i + 1]
            yaw = math.atan2(next_y - y, next_x - x)
        else:
            prev_x, prev_y = goals[i - 1]
            yaw = math.atan2(y - prev_y, x - prev_x)
        
        goal = make_pose(nav, x, y, yaw)
        nav.goToPose(goal)

        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            if fb:
                print(f'남은 거리: {fb.distance_remaining:.2f} m')
    
        result = nav.getResult()
        print('결과:', 'SUCCEEDED' if result == TaskResult.SUCCEEDED else result)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
