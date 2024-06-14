#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool

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
        self.dist90R = 1
        self.dist90L = 1
        self.dist75R = 0.75
        self.dist75L = 1
        self.wall_dist = 0.25
        self.kp = 0.4
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
        steer = 0.0  # 기본 조향값
        speed = 0.3  # 기본 속도

        # 센서값이 모두 측정되지 않은 경우
        if self.dist90L == 1 and self.dist75L == 1 and self.dist75R == 0.75:
            steer = 1.5  # 방향을 유지
            speed = 0.3  # 속도를 낮춤
            print("None")
        
        # 우측 센서값이 측정되지 않은 경우
        elif self.dist75R == 0.75 and not self.dist90L == 1 and not self.dist75L == 1:
            steer = -1.5  # 우회전 명령
            speed = 0.5  # 속도 설정
            print("turn RIGHT")

        # 좌측 센서값이 측정되지 않은 경우
        elif self.dist90L == 1 and self.dist75L == 1 and not self.dist75R == 0.75:
            steer = 1.5  # 좌회전 명령
            speed = 0.2  # 속도 설정
            print("turn LEFT")
        
        # 왼쪽 센서가 측정되는 경우 PID 제어
        elif 0.35 < self.dist90L < 1 and 0.35 < self.dist75L < 1.1:
            error = (self.dist75R + self.dist90L) / 2 + self.wall_dist # 오차 계산
            steer = self.kp * error  # PID 제어에 따른 조향각 계산
            speed = 0.45  # 기본 속도 설정
            print("Right wall follow", steer)

        # 조향각 제한
        if steer > 1.0:
            steer = 1.0
        elif steer < -1.0:
            steer = -1.0
        
        # 왼쪽 센서가 벽에 매우 가까운 경우
        elif 0 < self.dist90L < 0.38 and 0 < self.dist75L < 0.34:
            error = -(self.dist75L + self.dist90L) / 2 + self.wall_dist # 오차 계산
            steer = self.kp * error  # PID 제어에 따른 조향각 계산
            speed = 0.45  # 기본 속도 설정
            print("Left wall follow", steer)

        # 조향각 제한
        if steer > 1.0:
            steer = 1.0
        elif steer < -1.0:
            steer = -1.0

        # dist75L만 인식되는 경우 오른쪽으로 조향
        elif self.dist75L < 0.25 and self.dist90L == 1 and self.dist75R == 0.75:
            steer = -0.75  # 우회전 명령
            speed = 0.2  # 기본 속도 설정
            print("Only dist75L")

        # dist90L만 인식되는 경우 속도 줄임
        elif self.dist90L != 1 and self.dist75L == 1 and self.dist75R == 0.75:
            steer = -1.5  # 조향 없음
            speed = 0.3  # 속도 줄임
            print("Only dist90L")

        # dist90L와 dist75R만 인식되는 경우 왼쪽으로 조향
        elif self.dist90L != 1 and self.dist75L == 1 and self.dist75R != 0.75:
            steer = 1.5  # 좌회전 명령
            speed = 0.3  # 기본 속도 설정
            print("dist90L and dist75R")

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
