#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class WallFollower:
    def __init__(self):
        rospy.init_node("wall_follower")

        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.rate = rospy.Rate(10)

        self.target_distance = 0.3  # 원하는 왼쪽 벽까지의 거리
        self.kp = 1.5  # 비례 상수
        self.max_angular_z = 0.5  # 최대 회전 속도
        self.min_safe_distance = 0.1  # 충돌 방지를 위한 최소 안전 거리

    def laser_callback(self, msg):
        # 왼쪽 벽까지의 거리 측정
        left_distances = msg.ranges[180:360]  # 전체 왼쪽에 대한 거리
        avg_left_distance = sum(left_distances) / len(left_distances)

        # 로봇의 제어 입력 계산
        error = self.target_distance - avg_left_distance
        angular_z = self.kp * error

        # 회전 속도 제한
        angular_z = max(-self.max_angular_z, min(self.max_angular_z, angular_z))

        # 안전 거리 체크
        if avg_left_distance < self.min_safe_distance:
            angular_z = -self.max_angular_z  # 벽에 가까워질 때는 반대 방향으로 회전

        # 로봇 제어 입력 발행
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.2  # 일정한 속도로 직진
        cmd_vel.angular.z = angular_z  # 비례 제어를 이용하여 회전
        self.cmd_pub.publish(cmd_vel)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == "__main__":
    try:
        wall_follower = WallFollower()
        wall_follower.run()
    except rospy.ROSInterruptException:
        pass
