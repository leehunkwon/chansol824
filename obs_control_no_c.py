#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
import math

class OBS_Control:
    def __init__(self):
        rospy.init_node("control_node")

        rospy.Subscriber("/dist90R", Float32, self.dist90R_CB)
        rospy.Subscriber("/dist75R", Float32, self.dist75R_CB)
        rospy.Subscriber("/dist90L", Float32, self.dist90L_CB)
        rospy.Subscriber("/dist75L", Float32, self.dist75L_CB)
        rospy.Subscriber("/obsR", Bool, self.obsR_CB)
        rospy.Subscriber("/obsL", Bool, self.obsL_CB)

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.cmd_msg = Twist()
        self.obsR = False
        self.obsL = False
        self.dist90R = 0.5
        self.dist90L = 0.5
        self.dist75R = 0.5
        self.dist75L = 0.5
        self.wall_dist = 0.2
        self.kp = 0.01
        self.pre_steer = 0

        self.rate = rospy.Rate(10)

    def dist90R_CB(self, data):
        self.dist90R = data.data

    def dist75R_CB(self, data):
        self.dist75R = data.data

    def dist90L_CB(self, data):
        self.dist90L = data.data

    def dist75L_CB(self, data):
        self.dist75L = data.data

    def obsR_CB(self, data):
        self.obsR = data.data

    def obsL_CB(self, data):
        self.obsL = data.data

    def control(self):
        if self.dist90L == 0.5:  # dist90L 값이 측정되지 않은 경우
            steer = -1.0  # 좌회전 명령
            speed = 0.2  # 속도 설정
        elif self.dist90R == 0.5:  # dist90R 값이 측정되지 않은 경우
            steer = 1.0  # 우회전 명령
            speed = 0.2  # 속도 설정
        else:
            # dist90L 값 기반 PID 제어
            error = self.dist90L - self.wall_dist  # 오차 계산
            steer = self.kp * error  # PID 제어에 따른 조향각 계산
            speed = 0.4  # 기본 속도 설정

            # 조향각 제한
            if steer > 1.0:
                steer = 1.0
            elif steer < -1.0:
                steer = -1.0

        # Twist 메시지에 설정
        self.cmd_msg.linear.x = speed  # 속도 설정
        self.cmd_msg.angular.z = steer  # 조향각 설정

        # 제어 메시지 발행
        self.pub.publish(self.cmd_msg)

    def run(self):
        while not rospy.is_shutdown():
            self.control()
            self.rate.sleep()

def main():
    OBS_driving = OBS_Control()
    OBS_driving.run()

if __name__ == "__main__":
    main()
