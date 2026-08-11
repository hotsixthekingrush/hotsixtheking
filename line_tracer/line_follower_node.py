import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower_node')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 튜닝 파라미터 (환경에 맞게 조정)
        self.linear_speed = 0.08      # 전진 속도 (m/s)
        self.kp = 0.005               # 비례 제어 게인
        self.threshold_value = 80     # 이진화 임계값 (0~255, 검은선 감도)

        self.get_logger().info('Line follower node started.')

    def image_callback(self, msg: Image):
        # ROS Image -> OpenCV 이미지 변환
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, _ = frame.shape

        # 이미지 하단 1/3만 잘라서 사용 (로봇 바로 앞바닥만 보기 위함)
        roi = frame[int(height * 0.66):height, 0:width]

        # 흑백 변환 + 이진화 (검은 선 -> 흰색 픽셀로)
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(
            gray, self.threshold_value, 255, cv2.THRESH_BINARY_INV)

        # 선의 중심(centroid) 계산
        M = cv2.moments(binary)

        twist = Twist()

        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            roi_width = roi.shape[1]
            error = cx - (roi_width // 2)   # 화면 중앙과의 픽셀 오차

            twist.linear.x = self.linear_speed
            twist.angular.z = -float(error) * self.kp

            self.get_logger().info(
                f'line detected: cx={cx}, error={error}, angular.z={twist.angular.z:.3f}')
        else:
            # 선을 못 찾으면 정지 (안전을 위해 회전 탐색 대신 정지로 시작)
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().warn('line not detected — stopping')

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
