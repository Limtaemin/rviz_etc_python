#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
import csv
import numpy as np

class JoyDrive:
    def __init__(self):
        rospy.init_node('joy_drive', anonymous=True)
        self.sub_joy = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.sub_utm_x = rospy.Subscriber("/utm_x", Float64, self.utm_x_callback)
        self.sub_utm_y = rospy.Subscriber("/utm_y", Float64, self.utm_y_callback)
        
        self.utm_x = None  # 현재 UTM x값
        self.utm_y = None  # 현재 UTM y값
        self.utm_points = []  # UTM 좌표 저장 리스트
        self.rate = rospy.Rate(20)  # 20Hz 주기 설정
        self.spacing = 0.01  # 간격을 m로 설정

        self.manual = False
        print("4번(왼쪽)을 누르면 현재 utm값을 저장하고, 5번(오른쪽)을 누르면 직접 입력을 받아 csv를 생성합니다.")
        print("0.35보다 아래로 떨어지면 부동소수점 계산 오류때문에 값이 살짝 이상해집니다")
        print("현재 간격: {}m".format(self.spacing))

    def joy_callback(self, data):
        # 특정 버튼을 눌렀을 때 UTM 좌표 저장 (예: 버튼 4을 누르면 저장)
        if data.buttons[4] and self.utm_x is not None and self.utm_y is not None:
            if len(self.utm_points) < 2:
                self.utm_points.append([self.utm_x, self.utm_y])
                rospy.loginfo("UTM 좌표 저장됨: {}, {}".format(self.utm_x, self.utm_y))

            if len(self.utm_points) == 2:
                self.save_points_to_csv()

        # 버튼 5을 눌렀을 때 두 개의 UTM 좌표를 직접 입력하는 기능
        if data.buttons[5]:
            self.manual_input()

    def utm_x_callback(self, data):
        self.utm_x = data.data  # UTM x 값 저장

    def utm_y_callback(self, data):
        self.utm_y = data.data  # UTM y 값 저장

    def manual_input(self):
        # 두 개의 좌표를 직접 입력받아 저장
        try:
            utm_x1 = float(raw_input("첫 번째 UTM x 값을 입력하세요: "))
            utm_y1 = float(raw_input("첫 번째 UTM y 값을 입력하세요: "))
            utm_x2 = float(raw_input("두 번째 UTM x 값을 입력하세요: "))
            utm_y2 = float(raw_input("두 번째 UTM y 값을 입력하세요: "))

            self.utm_points = [[utm_x1, utm_y1], [utm_x2, utm_y2]]
            rospy.loginfo("직접 입력한 UTM 좌표 저장됨.")
            self.save_points_to_csv()

        except ValueError:
            rospy.logerr("잘못된 입력입니다. 숫자를 입력해주세요.")

    def save_points_to_csv(self):
        # 두 UTM 좌표를 이용하여 정확히 0.2m 간격으로 점을 생성
        utm1 = np.array(self.utm_points[0])
        utm2 = np.array(self.utm_points[1])
        
        # 두 좌표 사이의 x, y 변화량 계산
        delta_x = utm2[0] - utm1[0]
        delta_y = utm2[1] - utm1[1]
        
        # 두 좌표 사이의 전체 거리 계산
        total_distance = np.linalg.norm([delta_x, delta_y])
        
        # 0.2m 간격으로 나눌 점의 수 계산
        num_points = int(total_distance / self.spacing)
        
        # 방향 벡터 계산 (x, y 변화량을 총 거리로 나눈 값)
        direction_vector = np.array([delta_x, delta_y]) / total_distance
        
        # 각 점을 0.2m 간격으로 생성
        points = []
        for i in range(num_points + 1):
            new_point = utm1 + direction_vector * self.spacing * i
            points.append([round(new_point[0], 10), round(new_point[1], 10)])  # 소수점 10자리까지 저장
            
        # CSV 파일로 저장
        with open('utm_points.csv', 'w') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(['utm_x', 'utm_y'])  # 헤더 작성
            for point in points:
                csvwriter.writerow(point)

        rospy.loginfo("CSV 파일로 UTM 좌표 저장 완료. 간격: {}m".format(self.spacing))

    def run(self):
        rospy.spin()  # 노드를 계속 실행 상태로 유지

if __name__ == '__main__':
    joy_drive = JoyDrive()
    joy_drive.run()
