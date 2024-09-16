#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import tf
import math

def odom_callback(msg):
    # Orientation (quaternion) 값을 가져옴
    orientation_q = msg.pose.pose.orientation
    # 쿼터니언을 Euler 각도로 변환 (roll, pitch, yaw)
    quaternion = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    
    # yaw 값이 heading (헤딩) 값에 해당 (-pi ~ pi 범위)
    yaw = euler[2]

    # ENU -> NED 변환을 위해 yaw 값을 반전
    yaw = -yaw

    # yaw 값을 0 ~ 2*pi 범위로 변환 (시계방향 증가)
    if yaw < 0:
        yaw += 2 * math.pi

    # 라디안 값을 360도 범위로 변환
    heading = yaw * 180 / math.pi

    # Heading 값을 새로운 토픽으로 발행
    heading_pub.publish(heading)

if __name__ == '__main__':
    rospy.init_node('heading_publisher')

    # /odometry/filtered 토픽을 구독
    rospy.Subscriber('/odometry/filtered', Odometry, odom_callback)

    # heading 값을 발행할 토픽 생성
    heading_pub = rospy.Publisher('/heading', Float64, queue_size=10)

    rospy.spin()

