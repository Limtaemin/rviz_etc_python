#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PointStamped
import pandas as pd
import math

# ROS 노드 초기화
rospy.init_node('utm_lastyear_visualization_node')

# ROS 토픽 생성 (예: /utm_left_points)
pub = rospy.Publisher('/utm_last_points', PointStamped, queue_size=10)

# CSV 파일에서 UTM 좌표 읽기
df = pd.read_csv('작년차선.csv')

# 좌표 오프셋 값 설정 (예: X 좌표에 5미터를 더함)
x_offset = -332254.0  # 원하는 오프셋 값 (X 방향)
y_offset = -4128604.0  # Y 방향 오프셋도 설정할 수 있음
row_count = 0

# 이전 좌표 (처음에는 첫 번째 좌표로 초기화)
previous_x = df['utm_x'].iloc[0]
previous_y = df['utm_y'].iloc[0]

rate = rospy.Rate(10)  # 주기 설정 (10Hz)

while not rospy.is_shutdown():
    for index, row in df.iterrows():
        # 현재 좌표
        current_x = row['utm_x'] + x_offset
        current_y = row['utm_y'] + y_offset

        # X, Y 차이 계산
        x_diff = current_x - previous_x
        y_diff = current_y - previous_y

        # 절대 거리 계산 (유클리드 거리)
        distance = math.sqrt(x_diff**2 + y_diff**2)

        # X 차이, Y 차이, 절대 거리 출력
        print("index_num: {:1f},X 차이: {:.6f}, Y 차이: {:.6f}, 거리 차이: {:.6f}".format(row_count, x_diff, y_diff, distance))
        row_count += 1

        # 현재 좌표를 이전 좌표로 업데이트
        previous_x = current_x
        previous_y = current_y

        # ROS 메시지 생성 및 발행
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "map"  # 기준 프레임

        # UTM 좌표에 오프셋 적용 후 메시지에 설정
        point.point.x = current_x
        point.point.y = current_y
        point.point.z = 0  # 고도는 0으로 설정

        # 메시지 발행
        pub.publish(point)
        
        rate.sleep()
