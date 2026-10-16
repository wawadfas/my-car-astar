import os
import time

import cv2
import rclpy
import torch
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from ultralytics import YOLO


class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')

        self.declare_parameter('model_path', os.path.expanduser('~/my_ws/src/my_bot/models/yolov8n.pt'))
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('target_class', 'person')  # 비상정지를 걸 객체 (COCO 클래스 이름)
        self.declare_parameter('conf', 0.5)               # 이 신뢰도 이상만 검출로 인정
        self.declare_parameter('hold_sec', 1.0)           # 마지막 검출 후 이 시간 동안은 정지 유지 (깜빡임 방지)
        self.declare_parameter('show', True)              # 검출 화면 띄우기
        self.declare_parameter('threads', 2)              # YOLO CPU 스레드 수 (Gazebo와 CPU를 나눠 쓰므로 적게)

        self.target_class = self.get_parameter('target_class').value
        self.conf = self.get_parameter('conf').value
        self.hold_sec = self.get_parameter('hold_sec').value
        self.show = self.get_parameter('show').value
        torch.set_num_threads(self.get_parameter('threads').value)

        # cv2.VideoCapture(0) 대신: 로봇 카메라 토픽을 구독
        # depth=1: 처리가 밀려도 쌓인 옛 프레임 대신 항상 가장 최신 프레임만 검사
        self.bridge = CvBridge()
        self.model = YOLO(self.get_parameter('model_path').value)
        image_topic = self.get_parameter('image_topic').value
        latest_only = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, image_topic, self.image_callback, latest_only)

        self.pub_stop = self.create_publisher(Bool, '/emergency_stop', 10)
        self.last_seen = 0.0
        self.stopped = False

        self.get_logger().info(f"Watching {image_topic} for '{self.target_class}' (conf >= {self.conf})")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')  # cap.read() 대신

        results = self.model(frame, conf=self.conf, verbose=False)
        detected = False

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                name = self.model.names[int(box.cls[0])]
                if name == self.target_class:
                    detected = True
                color = (0, 0, 255) if name == self.target_class else (127, 0, 127)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                label_y = y1 - 10 if y1 > 20 else y1 + 20  # 박스가 화면 위에 붙으면 글자를 박스 안쪽에 표시
                cv2.putText(frame, f"{name} {conf:.2f}", (x1, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        now = time.monotonic()
        if detected:
            self.last_seen = now
        stop = (now - self.last_seen) < self.hold_sec

        if stop != self.stopped:
            self.stopped = stop
            if stop:
                self.get_logger().warn(f"'{self.target_class}' detected -> EMERGENCY STOP")
            else:
                self.get_logger().info(f"'{self.target_class}' gone -> resume")
        self.pub_stop.publish(Bool(data=stop))

        if self.show:
            if stop:
                # 박스 글자와 겹치지 않게 화면 아래쪽에 표시
                cv2.putText(frame, 'EMERGENCY STOP', (10, frame.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            cv2.imshow('person_detector', frame)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = PersonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
