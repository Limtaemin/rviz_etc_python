import rospy
import tf
from sensor_msgs.msg import Imu
import math

# 전역 변수를 사용하여 시작 yaw offset을 저장
yaw_offset = None

def imu_callback(msg):
    global yaw_offset

    # Orientation 쿼터니언 값을 가져옴
    orientation_q = msg.orientation

    # 쿼터니언을 Euler 각도로 변환 (roll, pitch, yaw)
    quaternion = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)

    # yaw 값을 얻음
    yaw = euler[2]

    # 첫 번째 IMU 데이터에서의 yaw 값을 시작 offset으로 저장
    if yaw_offset is None:
        yaw_offset = yaw*180/math.pi
        rospy.loginfo(f"Yaw Offset (Starting Orientation): {yaw_offset} degree")

    # 이후의 처리...
    # 필요시 여기서 yaw 값을 다른 목적으로 사용할 수 있음

if __name__ == '__main__':
    rospy.init_node('imu_offset_calculator')

    # /imu 데이터 구독
    rospy.Subscriber('/handsfree/imu', Imu, imu_callback)

    rospy.spin()
