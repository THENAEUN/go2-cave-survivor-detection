#!/usr/bin/env python3

import os
import time
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np
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

# RGB 및 Depth 토픽
IMAGE_TOPIC = "/camera/rgbd/image_raw"
DEPTH_TOPIC = "/camera/rgbd/depth/image_raw"

# Go2 이동 명령 토픽
CMD_VEL_TOPIC = "/cmd_vel"

# 사람 탐지 신뢰도 기준
CONFIDENCE_THRESHOLD = 0.55

# 연속 탐지 횟수
DETECT_CONFIRM_FRAMES = 3

# YOLO 추론 최대 빈도
MAX_INFERENCE_FPS = 3.0

# YOLO 입력 이미지 크기
INFERENCE_IMAGE_SIZE = 640

# 한 번 탐지되면 정지 상태 유지
LATCH_STOP = True

# 탐지 결과 창 표시
SHOW_WINDOW = True

# 유효한 Depth 범위
MIN_DEPTH_M = 0.10
MAX_DEPTH_M = 20.0

# 바운딩 박스 중심에서 거리 측정에 사용할 영역 비율
DEPTH_ROI_RATIO = 0.30

# 거리 계산에 필요한 최소 유효 픽셀 수
MIN_VALID_DEPTH_PIXELS = 10

# 조난자와 이 거리 이하가 되면 정지
STOP_DISTANCE_M = 1.0


def find_model_path() -> str:
    """사용할 YOLO 모델 파일을 찾는다."""

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

        # 가장 최근에 수신한 Depth 영상
        self.latest_depth_image: Optional[np.ndarray] = None

        # RGB 영상 구독
        self.image_subscriber = self.create_subscription(
            Image,
            IMAGE_TOPIC,
            self.image_callback,
            qos_profile_sensor_data,
        )

        # Depth 영상 구독
        self.depth_subscriber = self.create_subscription(
            Image,
            DEPTH_TOPIC,
            self.depth_callback,
            qos_profile_sensor_data,
        )

        # 정지 명령 발행
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            CMD_VEL_TOPIC,
            10,
        )

        # 정지 상태일 때 0 속도를 반복 발행
        self.stop_timer = self.create_timer(
            0.05,
            self.stop_timer_callback,
        )

        self.detect_streak = 0
        self.stop_active = False
        self.last_inference_time = 0.0
        self.last_distance_log_time = 0.0
        self.window_available = SHOW_WINDOW

        self.get_logger().info(
            f"RGB 토픽 대기 중: {IMAGE_TOPIC}"
        )
        self.get_logger().info(
            f"Depth 토픽 대기 중: {DEPTH_TOPIC}"
        )
        self.get_logger().info(
            f"사람이 {DETECT_CONFIRM_FRAMES}회 연속 탐지되고 "
            f"거리가 {STOP_DISTANCE_M:.2f} m 이하가 되면 "
            "Go2를 정지합니다."
        )

    def depth_callback(self, msg: Image):
        """32FC1 Depth 영상을 NumPy 배열로 저장한다."""

        try:
            depth_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="passthrough",
            )

            self.latest_depth_image = np.asarray(
                depth_image,
                dtype=np.float32,
            )

        except Exception as error:
            self.get_logger().error(
                f"Depth 영상을 변환하지 못했습니다: {error}"
            )

    def estimate_distance_m(
        self,
        bbox: Tuple[float, float, float, float],
        rgb_shape,
    ) -> Optional[float]:
        """
        사람 바운딩 박스 중심 영역의 Depth 중앙값을 계산한다.

        RGB 영상과 Depth 영상의 해상도가 다를 경우 좌표를 비례 변환한다.
        """

        if self.latest_depth_image is None:
            return None

        depth_image = self.latest_depth_image

        if depth_image.ndim != 2:
            return None

        rgb_height, rgb_width = rgb_shape[:2]
        depth_height, depth_width = depth_image.shape

        x1, y1, x2, y2 = bbox

        scale_x = depth_width / float(rgb_width)
        scale_y = depth_height / float(rgb_height)

        x1 = int(x1 * scale_x)
        x2 = int(x2 * scale_x)
        y1 = int(y1 * scale_y)
        y2 = int(y2 * scale_y)

        x1 = max(0, min(x1, depth_width - 1))
        x2 = max(0, min(x2, depth_width))
        y1 = max(0, min(y1, depth_height - 1))
        y2 = max(0, min(y2, depth_height))

        if x2 <= x1 or y2 <= y1:
            return None

        box_width = x2 - x1
        box_height = y2 - y1

        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2

        roi_width = max(
            4,
            int(box_width * DEPTH_ROI_RATIO),
        )
        roi_height = max(
            4,
            int(box_height * DEPTH_ROI_RATIO),
        )

        roi_x1 = max(0, center_x - roi_width // 2)
        roi_x2 = min(depth_width, center_x + roi_width // 2)

        roi_y1 = max(0, center_y - roi_height // 2)
        roi_y2 = min(depth_height, center_y + roi_height // 2)

        depth_roi = depth_image[
            roi_y1:roi_y2,
            roi_x1:roi_x2,
        ]

        if depth_roi.size == 0:
            return None

        valid_mask = (
            np.isfinite(depth_roi)
            & (depth_roi >= MIN_DEPTH_M)
            & (depth_roi <= MAX_DEPTH_M)
        )

        valid_depth_values = depth_roi[valid_mask]

        if valid_depth_values.size < MIN_VALID_DEPTH_PIXELS:
            return None

        return float(np.median(valid_depth_values))

    def image_callback(self, msg: Image):
        current_time = time.monotonic()
        minimum_interval = 1.0 / MAX_INFERENCE_FPS

        if (
            current_time - self.last_inference_time
            < minimum_interval
        ):
            return

        self.last_inference_time = current_time

        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )

        except Exception as error:
            self.get_logger().error(
                "ROS RGB 이미지를 OpenCV 이미지로 "
                f"변환하지 못했습니다: {error}"
            )
            return

        try:
            results = self.model.predict(
                source=frame,
                classes=[0],
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

        detections = []
        highest_confidence = 0.0

        if result.boxes is not None:
            for box in result.boxes:
                class_id = int(box.cls[0].item())
                confidence = float(box.conf[0].item())

                if (
                    class_id != 0
                    or confidence < CONFIDENCE_THRESHOLD
                ):
                    continue

                coordinates = box.xyxy[0].cpu().numpy()

                x1, y1, x2, y2 = [
                    float(value)
                    for value in coordinates
                ]

                distance_m = self.estimate_distance_m(
                    bbox=(x1, y1, x2, y2),
                    rgb_shape=frame.shape,
                )

                detections.append(
                    {
                        "bbox": (x1, y1, x2, y2),
                        "confidence": confidence,
                        "distance_m": distance_m,
                    }
                )

                highest_confidence = max(
                    highest_confidence,
                    confidence,
                )

        person_count = len(detections)
        person_found = person_count > 0

        valid_distances = [
            detection["distance_m"]
            for detection in detections
            if detection["distance_m"] is not None
        ]

        closest_distance_m = (
            min(valid_distances)
            if valid_distances
            else None
        )

        if person_found:
            self.detect_streak += 1
        else:
            self.detect_streak = 0

            if not LATCH_STOP:
                self.stop_active = False

        # 탐지 중 거리값을 약 1초마다 출력
        if (
            person_found
            and current_time - self.last_distance_log_time >= 1.0
        ):
            self.last_distance_log_time = current_time

            if closest_distance_m is not None:
                self.get_logger().info(
                    "조난자 탐지 중: "
                    f"인원={person_count}, "
                    f"가장 가까운 거리={closest_distance_m:.2f} m, "
                    f"연속 탐지={self.detect_streak}/"
                    f"{DETECT_CONFIRM_FRAMES}"
                )
            else:
                self.get_logger().info(
                    "조난자 탐지 중: "
                    f"인원={person_count}, "
                    "거리값 없음, "
                    f"연속 탐지={self.detect_streak}/"
                    f"{DETECT_CONFIRM_FRAMES}"
                )

        if (
            not self.stop_active
            and self.detect_streak >= DETECT_CONFIRM_FRAMES
            and closest_distance_m is not None
            and closest_distance_m <= STOP_DISTANCE_M
        ):
            self.stop_active = True
            self.publish_stop_command()

            if closest_distance_m is not None:
                distance_text = (
                    f", 추정 거리={closest_distance_m:.2f} m"
                )
            else:
                distance_text = ", 추정 거리=계산 불가"

            self.get_logger().warn(
                "조난자 감지! Go2를 정지합니다. "
                f"탐지 인원={person_count}, "
                f"최대 신뢰도={highest_confidence:.2f}"
                f"{distance_text}"
            )

        self.show_detection_result(
            result=result,
            detections=detections,
            person_count=person_count,
            highest_confidence=highest_confidence,
            closest_distance_m=closest_distance_m,
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
        detections,
        person_count: int,
        highest_confidence: float,
        closest_distance_m: Optional[float],
    ):
        if not self.window_available:
            return

        try:
            annotated_frame = result.plot()

            # 각 사람 바운딩 박스 위에 거리 표시
            for detection in detections:
                x1, y1, _, _ = detection["bbox"]
                distance_m = detection["distance_m"]

                if distance_m is None:
                    distance_text = "Depth: N/A"
                else:
                    distance_text = f"Distance: {distance_m:.2f} m"

                text_y = max(20, int(y1) - 10)

                cv2.putText(
                    annotated_frame,
                    distance_text,
                    (int(x1), text_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

            if self.stop_active:
                status_text = (
                    "SURVIVOR DETECTED - ROBOT STOPPED"
                )
            elif person_count > 0:
                status_text = (
                    f"Confirming detection "
                    f"{self.detect_streak}/"
                    f"{DETECT_CONFIRM_FRAMES}"
                )
            else:
                status_text = "SEARCHING"

            cv2.putText(
                annotated_frame,
                status_text,
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (
                    (0, 0, 255)
                    if self.stop_active
                    else (255, 255, 255)
                ),
                2,
                cv2.LINE_AA,
            )

            if closest_distance_m is None:
                distance_summary = "Closest distance: N/A"
            else:
                distance_summary = (
                    f"Closest distance: "
                    f"{closest_distance_m:.2f} m"
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

            cv2.putText(
                annotated_frame,
                distance_summary,
                (15, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
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
        if self.stop_active:
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
