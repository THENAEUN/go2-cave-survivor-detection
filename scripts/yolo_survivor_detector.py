#!/usr/bin/env python3

import os
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ultralytics import YOLO


# ============================================================
# 사용자 설정
# ============================================================

# Go2 RGB 카메라 토픽
IMAGE_TOPIC = "/camera/rgbd/image_raw"

# Go2 이동 명령 토픽
CMD_VEL_TOPIC = "/cmd_vel"

# 사람 탐지 신뢰도 기준
CONFIDENCE_THRESHOLD = 0.55

# 몇 프레임 연속 탐지해야 정지할지
DETECT_CONFIRM_FRAMES = 3

# YOLO 추론 최대 빈도
MAX_INFERENCE_FPS = 3.0

# YOLO 입력 이미지 크기
INFERENCE_IMAGE_SIZE = 640

# True:
# 한 번 사람을 탐지하면 탐지 노드를 종료할 때까지 정지 유지
LATCH_STOP = True

# 탐지 결과 창 표시 여부
SHOW_WINDOW = True


def find_model_path() -> str:
    """
    사용할 YOLO 모델을 찾는다.

    우선순위:
    1. YOLO_MODEL 환경변수
    2. 프로젝트 폴더의 yolo11n.pt
    3. 프로젝트 폴더의 yolov8n.pt
    4. scripts 폴더 내부 모델
    5. 기본 yolo11n.pt
    """

    project_dir = Path.home() / "cave_world_project"

    env_model = os.getenv("YOLO_MODEL")

    candidates = [
        Path(env_model).expanduser() if env_model else None,
        project_dir / "yolo11n.pt",
        project_dir / "yolov8n.pt",
        project_dir / "scripts" / "yolo11n.pt",
        project_dir / "scripts" / "yolov8n.pt",
    ]

    for candidate in candidates:
        if candidate is not None and candidate.is_file():
            return str(candidate)

    # 로컬 파일이 없으면 Ultralytics 기본 이름 사용
    return "yolo11n.pt"


class YoloSurvivorDetector(Node):
    def __init__(self):
        super().__init__("yolo_survivor_detector")

        self.bridge = CvBridge()

        self.model_path = find_model_path()

        self.get_logger().info(
            f"YOLO 모델을 불러옵니다: {self.model_path}"
        )

        self.model = YOLO(self.model_path)

        # RGB 이미지 구독
        self.image_subscriber = self.create_subscription(
            Image,
            IMAGE_TOPIC,
            self.image_callback,
            qos_profile_sensor_data,
        )

        # Go2 정지 명령 발행
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            CMD_VEL_TOPIC,
            10,
        )

        # 사람이 감지된 동안 정지 명령을 반복 발행한다.
        # teleop_twist_keyboard가 이동 명령을 보내더라도
        # 바로 다시 0 속도 명령을 보내 로봇을 정지시킨다.
        self.stop_timer = self.create_timer(
            0.05,
            self.stop_timer_callback,
        )

        self.detect_streak = 0
        self.stop_active = False
        self.last_inference_time = 0.0
        self.window_available = SHOW_WINDOW

        self.get_logger().info(
            f"이미지 토픽 대기 중: {IMAGE_TOPIC}"
        )
        self.get_logger().info(
            "사람이 3프레임 연속 탐지되면 Go2를 정지합니다."
        )

    def image_callback(self, msg: Image):
        current_time = time.monotonic()

        minimum_interval = 1.0 / MAX_INFERENCE_FPS

        if current_time - self.last_inference_time < minimum_interval:
            return

        self.last_inference_time = current_time

        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )
        except Exception as error:
            self.get_logger().error(
                f"ROS 이미지를 OpenCV 이미지로 변환하지 못했습니다: {error}"
            )
            return

        try:
            results = self.model.predict(
                source=frame,
                classes=[0],  # COCO person 클래스
                conf=CONFIDENCE_THRESHOLD,
                imgsz=INFERENCE_IMAGE_SIZE,
                verbose=False,
            )
        except Exception as error:
            self.get_logger().error(
                f"YOLO 추론 중 오류가 발생했습니다: {error}"
            )
            return

        result = results[0]
        person_count = 0
        highest_confidence = 0.0

        if result.boxes is not None and len(result.boxes) > 0:
            for box in result.boxes:
                class_id = int(box.cls[0].item())
                confidence = float(box.conf[0].item())

                if (
                    class_id == 0
                    and confidence >= CONFIDENCE_THRESHOLD
                ):
                    person_count += 1
                    highest_confidence = max(
                        highest_confidence,
                        confidence,
                    )

        person_found = person_count > 0

        if person_found:
            self.detect_streak += 1
        else:
            self.detect_streak = 0

            # LATCH_STOP이 False일 때만 사람이 사라지면 해제
            if not LATCH_STOP:
                self.stop_active = False

        if (
            not self.stop_active
            and self.detect_streak >= DETECT_CONFIRM_FRAMES
        ):
            self.stop_active = True

            # 즉시 정지 명령 발행
            self.publish_stop_command()

            self.get_logger().warn(
                "조난자 감지! Go2를 정지합니다. "
                f"탐지 인원={person_count}, "
                f"최대 신뢰도={highest_confidence:.2f}"
            )

        self.show_detection_result(
            result=result,
            person_count=person_count,
            highest_confidence=highest_confidence,
        )

    def stop_timer_callback(self):
        if self.stop_active:
            self.publish_stop_command()

    def publish_stop_command(self):
        stop_message = Twist()

        stop_message.linear.x = 0.0
        stop_message.linear.y = 0.0
        stop_message.linear.z = 0.0

        stop_message.angular.x = 0.0
        stop_message.angular.y = 0.0
        stop_message.angular.z = 0.0

        self.cmd_vel_publisher.publish(stop_message)

    def show_detection_result(
        self,
        result,
        person_count: int,
        highest_confidence: float,
    ):
        if not self.window_available:
            return

        try:
            annotated_frame = result.plot()

            if self.stop_active:
                status_text = "SURVIVOR DETECTED - ROBOT STOPPED"
            elif person_count > 0:
                status_text = (
                    f"Confirming detection "
                    f"{self.detect_streak}/{DETECT_CONFIRM_FRAMES}"
                )
            else:
                status_text = "SEARCHING"

            cv2.putText(
                annotated_frame,
                status_text,
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255) if self.stop_active else (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            cv2.putText(
                annotated_frame,
                (
                    f"Persons: {person_count}  "
                    f"Confidence: {highest_confidence:.2f}"
                ),
                (15, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            cv2.imshow(
                "Go2 Survivor Detection",
                annotated_frame,
            )

            pressed_key = cv2.waitKey(1) & 0xFF

            if pressed_key == ord("q"):
                self.get_logger().info(
                    "Q 키 입력으로 탐지 노드를 종료합니다."
                )
                rclpy.shutdown()

        except cv2.error as error:
            self.get_logger().warn(
                f"탐지 화면을 표시할 수 없습니다: {error}"
            )
            self.window_available = False

    def destroy_node(self):
        # 종료 직전에도 정지 명령을 한 번 전송
        self.publish_stop_command()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    detector_node = None

    try:
        detector_node = YoloSurvivorDetector()
        rclpy.spin(detector_node)

    except KeyboardInterrupt:
        pass

    except Exception as error:
        print(f"탐지 노드 실행 오류: {error}")

    finally:
        if detector_node is not None:
            detector_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
