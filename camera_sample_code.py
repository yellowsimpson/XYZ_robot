import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        timer_period = 0.033  # 30FPS를 위한 타이머 주기
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # 비디오 캡처 객체 생성 (파일 또는 카메라)
        # self.cap = cv2.VideoCapture(0)  # 웹캠 사용시
        self.cap = cv2.VideoCapture('your_video.mp4')  # 비디오 파일 사용시

        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()

        if ret:
            # 프레임이 끝났다면 다시 시작
            if frame is None:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = self.cap.read()

            # OpenCV 이미지를 ROS 메시지로 변환
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing video frame')

    def __del__(self):
        self.cap.release()


def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()

    try:
        rclpy.spin(video_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        video_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
