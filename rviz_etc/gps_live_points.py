#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped
import utm
import math

# 전역 변수를 사용하여 최초의 UTM 좌표를 저장
first_utm_x = None
first_utm_y = None

# 임의의 회전 각도를 설정 (도 단위)
rotation_angle_deg = None

def gps_callback(msg):
    global first_utm_x, first_utm_y, rotation_angle_deg

    # 위도와 경도를 UTM 좌표로 변환
    utm_x, utm_y, _, _ = utm.from_latlon(msg.latitude, msg.longitude)

    # 첫 번째 UTM 좌표를 기준으로 평행 이동
    if first_utm_x is None and first_utm_y is None:
        first_utm_x = utm_x
        first_utm_y = utm_y

    # 평행 이동된 좌표 계산
    relative_x = utm_x - first_utm_x
    relative_y = utm_y - first_utm_y

    # 도 단위로 받은 회전 각도를 라디안으로 변환
    rotation_angle_rad = math.radians(rotation_angle_deg)

    # 임의의 각도로 회전
    rotated_x = relative_x * math.cos(rotation_angle_rad) - relative_y * math.sin(rotation_angle_rad)
    rotated_y = relative_x * math.sin(rotation_angle_rad) + relative_y * math.cos(rotation_angle_rad)

    # PointStamped 메시지 생성하여 발행
    point = PointStamped()
    point.header.stamp = rospy.Time.now()
    point.header.frame_id = "odom"  # rviz에서 사용할 프레임 ID
    point.point.x = rotated_x
    point.point.y = rotated_y
    point.point.z = msg.altitude  # 고도를 그대로 사용

    point_pub.publish(point)

if __name__ == '__main__':
    rospy.init_node('gps_to_utm')

    # 회전 각도를 도 단위로 파라미터에서 읽어옴 (디폴트는 90도)
    rotation_angle_deg = rospy.get_param('~rotation_angle', 95)

    # /ublox_gps_base/fix 토픽을 구독
    rospy.Subscriber('/ublox_gps_base/fix', NavSatFix, gps_callback)

    # 평행 이동된 UTM 좌표를 발행할 토픽 생성
    point_pub = rospy.Publisher('/utm_point', PointStamped, queue_size=10)

    rospy.spin()
