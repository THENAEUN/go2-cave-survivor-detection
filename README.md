## RGB-D 기반 조난자 위치 추정

Go2에 장착된 RGB-D 카메라와 YOLO를 이용해 조난자를 탐지하고, 조난자의 위치를 Gazebo World 좌표로 변환합니다.

### 주요 기능

- RGB 영상 기반 YOLO 사람 탐지
- 사람 바운딩 박스 중심 위치 계산
- RGB 좌표와 Depth 영상 좌표 대응
- 중앙 ROI 기반 대표 Depth 계산
- CameraInfo 기반 카메라 3차원 좌표 계산
- 카메라 좌표를 Go2 몸체 기준 좌표로 변환
- Go2 몸체 기준 좌표를 Gazebo World 좌표로 변환
- 조난자 좌표 ROS2 토픽 발행
- 조난자 1m 이내 접근 시 Go2 자동 정지

### 처리 흐름

```text
RGB 영상 입력
→ YOLO 사람 탐지
→ 바운딩 박스 중심 위치 계산
→ Depth 영상 좌표 대응
→ 중앙 ROI 30% 추출
→ 유효 Depth 중앙값 계산
→ 카메라 기준 3차원 좌표 계산
→ Go2 몸체 기준 좌표로 변환
→ Gazebo World 좌표로 변환
→ 조난자 좌표 발행 및 자동 정지
