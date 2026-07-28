# Go2 Cave Survivor Detection

Gazebo 동굴 환경에서 **Unitree Go2**, **RGB-D 카메라**, **YOLO**를 이용해 조난자를 탐지하고, 조난자의 위치를 Gazebo World 좌표로 변환하는 ROS2 프로젝트입니다.

RGB 영상에서는 조난자의 화면 내 위치를 탐지하고, Depth 영상에서는 조난자까지의 깊이를 계산합니다. 이후 카메라와 로봇 사이의 위치 관계와 Gazebo에서 제공하는 로봇의 위치·방향을 반영하여 조난자의 최종 World 좌표를 계산합니다.

탐지된 조난자의 좌표는 ROS2 토픽으로 발행되며, 조난자가 카메라 기준 약 1m 이내로 접근하면 Go2가 자동으로 정지합니다.

---

## 주요 기능

- RGB 영상 기반 YOLO 사람 탐지
- 사람 클래스, 탐지 신뢰도 및 바운딩 박스 출력
- 바운딩 박스 중심 픽셀 계산
- RGB 바운딩 박스와 Depth 영상 좌표 대응
- 바운딩 박스 중앙 ROI 기반 대표 깊이 계산
- CameraInfo 기반 카메라 기준 3차원 위치 계산
- 카메라 기준 좌표를 Go2 몸체 기준 좌표로 변환
- Go2 몸체 기준 좌표를 Gazebo World 좌표로 변환
- World 기준 조난자 좌표 ROS2 토픽 발행
- 다수 인원 탐지 시 가장 가까운 조난자 선택
- 조난자 연속 탐지 및 1m 이내 자동 정지

---

## 시스템 처리 흐름

```text
RGB 영상 입력
→ YOLO 사람 탐지
→ 바운딩 박스 중심 위치 계산
→ RGB 좌표를 Depth 영상 좌표에 대응
→ 바운딩 박스 중앙 ROI 30% 추출
→ 유효한 Depth 값 필터링
→ Depth 중앙값을 대표 깊이로 사용
→ CameraInfo 기반 카메라 기준 3차원 위치 계산
→ Go2 몸체 중심 기준 좌표로 변환
→ Gazebo World 기준 좌표로 변환
→ 조난자 좌표 ROS2 토픽 발행
→ 1m 이내 접근 시 Go2 자동 정지
```

---

## 시스템 구성

```text
Gazebo Cave World
├── Unitree Go2
│   ├── RGB 카메라
│   ├── Depth 카메라
│   └── CameraInfo
├── YOLO 사람 탐지
├── TF 좌표 변환
├── Gazebo Ground Truth Odometry
└── Survivor Detection Node
    ├── 조난자 거리 계산
    ├── 조난자 World 좌표 계산
    ├── /survivor_position 발행
    └── /cmd_vel 정지 명령 발행
```

---

## RGB 영상 기반 사람 탐지

Go2의 RGB 카메라 영상을 YOLO 모델에 입력하여 사람을 탐지합니다.

YOLO는 다음 정보를 출력합니다.

- 탐지된 객체의 클래스
- 사람으로 판단한 신뢰도
- 사람을 둘러싼 바운딩 박스 좌표
- 한 프레임에서 탐지된 사람 수

바운딩 박스는 왼쪽 위 좌표와 오른쪽 아래 좌표로 표현됩니다.

```text
왼쪽 위:     (x1, y1)
오른쪽 아래: (x2, y2)
```

두 좌표를 이용해 사람 영역의 중심 픽셀을 계산합니다.

```text
바운딩 박스 중심 픽셀: (u, v)
```

이 중심 픽셀은 조난자가 RGB 영상에서 왼쪽, 오른쪽, 위쪽 또는 아래쪽 중 어느 방향에 보이는지를 나타냅니다.

RGB 영상만으로는 조난자가 카메라에서 얼마나 멀리 떨어져 있는지 알 수 없기 때문에 Depth 영상을 함께 사용합니다.

---

## RGB와 Depth 영상 좌표 대응

YOLO는 RGB 영상에서 사람을 탐지하지만, 사람까지의 거리값은 Depth 영상에서 가져옵니다.

RGB 영상과 Depth 영상의 해상도가 다르면 같은 물체도 서로 다른 픽셀 좌표로 표현됩니다.

예를 들어 다음과 같은 경우,

```text
RGB 영상:   640 × 480
Depth 영상: 320 × 240
```

RGB 영상의 중심 좌표 `(320, 240)`은 Depth 영상에서 `(160, 120)`에 대응합니다.

따라서 RGB 영상에서 얻은 바운딩 박스 좌표를 Depth 영상의 가로·세로 해상도 비율에 맞게 변환합니다.

```text
RGB 바운딩 박스 좌표
→ Depth 영상 해상도에 맞게 비례 조정
→ 같은 사람에 대응하는 Depth 영역 추출
```

현재 Gazebo RGB-D 카메라는 RGB 영상과 Depth 영상이 정렬되어 있다는 전제로 처리합니다.

---

## 중앙 ROI 기반 대표 깊이 계산

YOLO의 바운딩 박스는 사람의 형태를 정확히 분리한 영역이 아니라 사람 전체를 둘러싼 사각형입니다.

따라서 바운딩 박스 가장자리에는 다음과 같은 배경이 포함될 수 있습니다.

- 사람 뒤의 벽
- 동굴 바닥
- 주변 장애물
- 사람과 배경 사이의 경계 영역

이러한 배경 Depth의 영향을 줄이기 위해 바운딩 박스 중앙의 약 30% 영역을 관심 영역인 ROI로 사용합니다.

```text
전체 바운딩 박스
┌────────────────────┐
│                    │
│       ┌──────┐     │
│       │ ROI  │     │
│       └──────┘     │
│                    │
└────────────────────┘
```

ROI는 영상 전체의 중앙에 고정된 영역이 아닙니다. YOLO가 탐지한 사람 바운딩 박스의 중심을 따라 이동합니다.

따라서 사람이 영상의 왼쪽이나 오른쪽에 있어도 해당 사람의 바운딩 박스 중앙에서 ROI가 생성됩니다.

---

## 유효한 Depth 값 필터링

Depth 영상은 각 픽셀에 거리값이 저장된 영상입니다.

현재 Depth 영상은 `32FC1` 인코딩을 사용합니다.

```text
32F: 픽셀당 32비트 부동소수점
C1: 채널 1개
```

각 픽셀에는 색상값이 아니라 미터 단위의 깊이값 하나가 저장됩니다.

```text
Depth 픽셀값 0.85
→ 해당 방향의 물체 깊이 약 0.85m
```

Depth 영상에는 정상적인 거리값뿐 아니라 측정 실패값이나 센서 범위를 벗어난 값도 포함될 수 있습니다.

ROI 내부에서 다음 값은 제외합니다.

- NaN과 같은 측정 실패값
- 무한대 값
- 0.1m보다 가까운 값
- 20m보다 먼 값
- 유효 Depth 픽셀 수가 부족한 경우

현재 사용한 주요 기준은 다음과 같습니다.

```text
최소 Depth: 0.1m
최대 Depth: 20.0m
최소 유효 픽셀 수: 10개
```

---

## Depth 중앙값 사용

유효한 Depth 값들을 크기순으로 정렬한 뒤 중앙값을 조난자의 대표 깊이로 사용합니다.

예를 들어 ROI 내부의 Depth 값이 다음과 같다고 가정합니다.

```text
0.82, 0.84, 0.85, 0.86, 8.00
```

`8.00m`는 사람 뒤에 있는 벽이나 잘못 포함된 배경값일 수 있습니다.

평균은 큰 값의 영향을 받아 실제 사람 거리보다 크게 계산될 수 있지만, 중앙값은 `0.85m`로 사람 영역의 대표 깊이에 가까운 값을 유지합니다.

따라서 중앙값을 사용하여 소수의 배경값과 이상치가 거리 계산에 미치는 영향을 줄였습니다.

> 중앙값은 소수의 이상치에는 강하지만, ROI 대부분이 배경으로 구성된 경우에는 잘못된 깊이를 출력할 수 있습니다.

---

## CameraInfo 기반 카메라 3차원 위치 계산

다음 CameraInfo 토픽에서 카메라 내부 파라미터를 수신합니다.

```text
/camera/rgbd/depth/camera_info
```

사용하는 주요 값은 다음과 같습니다.

| 값 | 의미 |
|---|---|
| `fx` | 가로 방향 픽셀 단위 초점거리 |
| `fy` | 세로 방향 픽셀 단위 초점거리 |
| `cx` | 가로 방향 광학 중심 |
| `cy` | 세로 방향 광학 중심 |

현재 시뮬레이션에서 확인된 값은 다음과 같습니다.

```text
fx = 190.681
fy = 190.681
cx = 160.500
cy = 120.500
```

`cx`와 `cy`는 카메라가 정면으로 바라보는 지점이 영상의 어느 픽셀에 해당하는지를 나타냅니다.

현재 Depth 영상 해상도가 `320 × 240`이므로 광학 중심 `(160.5, 120.5)`는 영상 중앙 부근에 위치합니다.

사람의 중심 픽셀, ROI에서 계산한 대표 깊이, CameraInfo를 이용해 카메라 기준 조난자의 3차원 위치를 계산합니다.

카메라 기준 위치는 다음 정보를 나타냅니다.

- 카메라 기준 좌우 위치
- 카메라 기준 상하 위치
- 카메라 기준 전방 깊이

```text
카메라 기준 조난자 위치
→ 카메라 렌즈 중심에서 조난자가 어느 방향에 있고
   얼마나 떨어져 있는지를 나타내는 3차원 좌표
```

---

## 좌표계 변환

카메라 기준으로 계산한 조난자 위치는 다음 순서로 변환합니다.

```text
Camera Optical Frame
→ Go2 Body Frame
→ Gazebo World Frame
```

---

## 카메라 기준 좌표에서 Go2 몸체 기준 좌표로 변환

카메라는 Go2 몸체의 기준점과 동일한 위치에 설치되어 있지 않습니다.

현재 카메라는 Go2 몸체의 대표 기준점보다 대략 다음 위치에 설치되어 있습니다.

```text
전방 약 0.23m
상단 약 0.04m
```

또한 카메라와 로봇 몸체가 사용하는 좌표축 방향도 서로 다릅니다.

카메라 Optical Frame은 일반적으로 다음 방향을 사용합니다.

```text
X축: 영상 오른쪽
Y축: 영상 아래쪽
Z축: 카메라 전방
```

로봇 몸체 좌표는 일반적으로 다음 방향을 사용합니다.

```text
X축: 로봇 전방
Y축: 로봇 왼쪽
Z축: 로봇 위쪽
```

따라서 단순히 카메라에서 계산한 좌표에 로봇의 위치를 더할 수 없습니다.

ROS2 TF를 이용해 다음 정보를 반영합니다.

- 카메라와 Go2 몸체 사이의 위치 차이
- 카메라와 Go2 몸체 사이의 방향 차이
- 카메라 좌표축과 로봇 좌표축의 차이

이를 통해 조난자가 Go2 몸체를 기준으로 앞쪽, 옆쪽 또는 위쪽 중 어디에 있는지를 계산합니다.

```text
카메라 기준 조난자 위치
→ 카메라 설치 위치와 축 방향 반영
→ Go2 몸체 중심 기준 조난자 위치
```

---

## Go2 몸체 기준 좌표에서 World 기준 좌표로 변환

Go2 몸체 기준 좌표는 로봇과 함께 움직이는 좌표입니다.

로봇이 이동하면 좌표의 기준점도 함께 이동하고, 로봇이 회전하면 좌표축의 방향도 함께 회전합니다.

조난자의 위치를 동굴 전체에서 고정된 좌표로 표현하기 위해 Gazebo가 제공하는 로봇의 현재 위치와 방향을 사용합니다.

```text
/odom/ground_truth
```

이 토픽은 Gazebo World 좌표계를 기준으로 Go2 몸체가 현재 어디에 있고 어느 방향을 바라보고 있는지를 제공합니다.

```text
World 기준 로봇 위치
+ World 기준 로봇 방향
+ 로봇 몸체 기준 조난자 상대 위치
= World 기준 조난자 위치
```

최종 좌표 변환 순서는 다음과 같습니다.

```text
카메라 기준 조난자 위치
→ Go2 몸체 기준 조난자 위치
→ 동굴 전체 World 기준 조난자 위치
```

---

## Ground Truth Odometry 사용 이유

초기에는 원시 Odometry 토픽을 확인했지만, 현재 시뮬레이션 환경에서 실제 동굴 크기와 맞지 않는 비정상적으로 큰 위치값이 출력됐습니다.

```text
예시: 약 10^34 수준의 위치값
```

이 값은 수십 미터 규모의 동굴 환경과 일치하지 않기 때문에 좌표 변환에 사용할 수 없었습니다.

반면 `/odom/ground_truth`는 Gazebo가 내부적으로 알고 있는 로봇 모델의 실제 위치와 자세를 제공합니다.

현재 프로젝트는 좌표 변환 기능을 시뮬레이션에서 검증하는 단계이므로 안정적인 Ground Truth 값을 사용했습니다.

> Ground Truth는 실제 로봇에서는 사용할 수 없습니다. 실제 환경에서는 SLAM과 Localization을 이용해 지도 기준 로봇 위치를 계산해야 합니다.

---

## 사용 토픽

### 입력 토픽

| 토픽 | 메시지 역할 |
|---|---|
| `/camera/rgbd/image_raw` | YOLO 사람 탐지용 RGB 영상 |
| `/camera/rgbd/depth/image_raw` | 픽셀별 Depth 영상 |
| `/camera/rgbd/depth/camera_info` | 카메라 내부 파라미터 |
| `/odom/ground_truth` | World 기준 Go2 위치와 자세 |

### 출력 토픽

| 토픽 | 메시지 형식 | 역할 |
|---|---|---|
| `/survivor_position` | `geometry_msgs/msg/PointStamped` | World 기준 조난자 좌표 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Go2 이동 및 정지 명령 |

---

## 조난자 좌표 토픽 발행

최종적으로 계산한 조난자 좌표는 다음 ROS2 토픽으로 발행합니다.

```text
/survivor_position
```

메시지 형식은 다음과 같습니다.

```text
geometry_msgs/msg/PointStamped
```

출력 예시는 다음과 같습니다.

```yaml
header:
  stamp:
    sec: 1784939204
    nanosec: 384540549
  frame_id: world
point:
  x: 5.45
  y: 5.16
  z: 0.24
```

각 항목의 의미는 다음과 같습니다.

| 항목 | 의미 |
|---|---|
| `header.stamp` | 좌표가 계산되고 발행된 시간 |
| `header.frame_id` | 좌표의 기준 좌표계 |
| `point.x` | World 기준 X 위치 |
| `point.y` | World 기준 Y 위치 |
| `point.z` | World 기준 높이 |

`frame_id`를 `world`로 설정하여 해당 좌표가 카메라 기준이나 로봇 기준이 아니라 동굴 전체 World 기준임을 나타냅니다.

다른 ROS2 노드는 `/survivor_position`을 구독하여 다음 기능에 활용할 수 있습니다.

- 조난자 위치 저장
- RViz 좌표 시각화
- 구조 임무 관리
- 자율주행 목표 생성
- 조난자 안전 접근 지점 계산

---

## 다수 조난자 처리

한 프레임에서 여러 명의 사람이 탐지되면 각 사람에 대해 다음 정보를 계산합니다.

- YOLO 바운딩 박스
- 탐지 신뢰도
- 대표 Depth
- 카메라 기준 3차원 위치
- World 기준 3차원 위치

유효한 Depth가 계산된 사람 중 카메라에서 가장 가까운 사람을 선택합니다.

```text
다수 사람 탐지
→ 각 사람의 대표 Depth 계산
→ 가장 가까운 사람 선택
→ 선택된 사람의 World 좌표 발행
```

현재 `/survivor_position`에는 가장 가까운 조난자 한 명의 좌표를 발행합니다.

---

## Go2 자동 정지 조건

다음 조건을 모두 만족하면 Go2에 정지 명령을 발행합니다.

- 사람이 3회 이상 연속으로 탐지됨
- 가장 가까운 조난자의 대표 깊이가 1.0m 이하임

```text
사람 연속 탐지 3회 이상
+
대표 깊이 1.0m 이하
→ Go2 자동 정지
```

한 프레임의 일시적인 오탐으로 로봇이 정지하는 것을 줄이기 위해 연속 탐지 조건을 사용합니다.

정지 조건이 만족되면 `/cmd_vel` 토픽으로 선속도와 각속도가 모두 0인 명령을 발행합니다.

```text
linear.x  = 0
linear.y  = 0
linear.z  = 0
angular.x = 0
angular.y = 0
angular.z = 0
```

정지 이후에도 0 속도 명령을 반복 발행하여 정지 상태가 유지되도록 구성했습니다.

---

## 실행 환경

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Classic 11
- Python 3
- OpenCV
- NumPy
- Ultralytics YOLO
- Unitree Go2 ROS2 패키지
- `cv_bridge`
- `tf2_ros`

---

## 프로젝트 주요 파일

```text
cave_world_project/
├── scripts/
│   └── yolo_survivor_detector.py
├── src/
│   ├── gazebo_cave_world/
│   └── unitree-go2-ros2/
├── yolo11n.pt
└── README.md
```

주요 탐지 및 좌표 계산 코드는 다음 파일에 포함되어 있습니다.

```text
scripts/yolo_survivor_detector.py
```

---

## 실행 방법

### 1. Gazebo World 및 Go2 실행

```bash
cd ~/cave_world_project

source .venv/bin/activate
source /opt/ros/humble/setup.bash
source install/setup.bash
source /usr/share/gazebo/setup.bash

export ROS_DOMAIN_ID=30
export GAZEBO_MODEL_PATH="$HOME/cave_world_project/src/gazebo_cave_world/worlds/models:${GAZEBO_MODEL_PATH:-}"

ros2 launch go2_config gazebo.launch.py \
  world:=$HOME/cave_world_project/src/gazebo_cave_world/worlds/cave_world_rescue.world \
  world_init_x:=0.0 \
  world_init_y:=0.0 \
  world_init_z:=0.35 \
  world_init_heading:=0.0 \
  rviz:=false \
  gui:=false
```

### 2. Gazebo GUI 실행

새 터미널에서 다음 명령어를 실행합니다.

```bash
cd ~/cave_world_project

source .venv/bin/activate
source /opt/ros/humble/setup.bash
source install/setup.bash
source /usr/share/gazebo/setup.bash

export ROS_DOMAIN_ID=30
export GAZEBO_MODEL_PATH="$HOME/cave_world_project/src/gazebo_cave_world/worlds/models:${GAZEBO_MODEL_PATH:-}"

export XDG_RUNTIME_DIR="/tmp/runtime-$USER"
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"

export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1

gzclient --verbose
```

### 3. 카메라 영상 확인

새 터미널에서 다음 명령어를 실행합니다.

```bash
cd ~/cave_world_project

source .venv/bin/activate
source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=30
export XDG_RUNTIME_DIR="/tmp/runtime-$USER"

mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"

export QT_QPA_PLATFORM=xcb

ros2 run rqt_image_view rqt_image_view
```

`rqt_image_view`에서 다음 토픽을 선택합니다.

```text
/camera/rgbd/image_raw
```

### 4. 키보드 조작 실행

새 터미널에서 다음 명령어를 실행합니다.

```bash
cd ~/cave_world_project

source .venv/bin/activate
source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=30

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -r cmd_vel:=/cmd_vel
```

### 5. 조난자 탐지 및 좌표 계산 노드 실행

새 터미널에서 다음 명령어를 실행합니다.

```bash
cd ~/cave_world_project

source .venv/bin/activate
source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=30
export XDG_RUNTIME_DIR="/tmp/runtime-$USER"

mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"

export QT_QPA_PLATFORM=xcb

python3 -u scripts/yolo_survivor_detector.py
```

---

## 실행 출력 예시

### 초기화 출력

```text
YOLO 모델을 불러옵니다:
/home/naeun/cave_world_project/yolo11n.pt

RGB 토픽 대기 중:
/camera/rgbd/image_raw

Depth 토픽 대기 중:
/camera/rgbd/depth/image_raw

사람이 3회 연속 탐지되고 거리가 1.00m 이하가 되면 Go2를 정지합니다.
```

### CameraInfo 수신

```text
Depth CameraInfo 수신:
frame=rgbd_camera_optical_frame,
fx=190.681,
fy=190.681,
cx=160.500,
cy=120.500
```

### 실시간 탐지 출력

```text
조난자 탐지중:
인원=1,
가장 가까운 거리=0.95m,
연속 탐지=10/3,
로봇 world=(5.60, 3.84, 0.21),
조난자 world=(5.45, 5.16, 0.24)
```

각 출력값의 의미는 다음과 같습니다.

| 출력 | 의미 |
|---|---|
| `인원=1` | 현재 탐지된 사람 수 |
| `가장 가까운 거리=0.95m` | 가장 가까운 사람의 대표 Depth |
| `연속 탐지=10/3` | 최소 3회 조건에 대해 현재 10회 연속 탐지 |
| `로봇 world` | World 기준 Go2 몸체 위치 |
| `조난자 world` | World 기준 조난자 위치 |

### 자동 정지 출력

```text
조난자 감지! Go2를 정지합니다.
탐지 인원=1,
최대 신뢰도=0.85,
추정 거리=0.98m
```

`최대 신뢰도=0.85`는 모델이 해당 객체를 사람이라고 판단한 신뢰도입니다.

이 값은 시스템 전체 정확도가 85%라는 의미가 아닙니다.

---

## 조난자 좌표 확인

다음 명령어로 World 기준 조난자 좌표를 확인할 수 있습니다.

```bash
ros2 topic echo /survivor_position
```

한 번만 출력하려면 다음 명령어를 사용합니다.

```bash
ros2 topic echo /survivor_position --once
```

출력 예시:

```yaml
header:
  stamp:
    sec: 1784939204
    nanosec: 384540549
  frame_id: world
point:
  x: 5.278398051235347
  y: 5.355524358642068
  z: 0.16135673386713476
```

---

## 검증 결과

- YOLO 기반 조난자 1명 탐지 성공
- 최대 탐지 신뢰도 0.85 확인
- 조난자 접근에 따라 대표 Depth 감소 확인
- 바운딩 박스 중앙 ROI 기반 대표 깊이 계산 성공
- CameraInfo 기반 카메라 3차원 위치 계산 성공
- 카메라 기준 좌표에서 Go2 몸체 기준 좌표 변환 성공
- Go2 몸체 기준 좌표에서 Gazebo World 좌표 변환 성공
- `/survivor_position` 토픽 발행 성공
- 조난자 약 1m 이내에서 Go2 자동 정지 성공
- 정지 이후 로봇 World 위치가 거의 변하지 않는 것을 확인
- 정지 이후 조난자 World 좌표가 일정 범위에서 유지되는 것을 확인

실험에서 확인된 결과 예시는 다음과 같습니다.

```text
YOLO 최대 신뢰도:
0.85

정지 조건이 처음 만족된 거리:
약 0.98m

정지 후 Go2 World 위치:
약 (5.60, 3.84, 0.21)

정지 후 조난자 World 위치:
약 (5.45, 5.16, 0.24)
```

---

## 현재 한계

- 바운딩 박스 중앙 ROI를 사용하므로 가림이 발생한 사람에서는 배경 Depth가 포함될 수 있습니다.
- 사람이 영상 밖으로 일부 잘린 경우 바운딩 박스 중심이 실제 몸체 중심과 다를 수 있습니다.
- 조난자가 누워 있거나 기울어진 경우 중앙 ROI가 몸체가 아닌 바닥을 포함할 수 있습니다.
- ROI 대부분이 배경이면 중앙값도 잘못된 거리로 계산될 수 있습니다.
- RGB 영상과 Depth 영상이 정렬되어 있다는 전제로 좌표를 대응합니다.
- 실제 RGB 센서와 Depth 센서의 위치가 다르면 별도의 정렬 과정이 필요합니다.
- RGB 영상과 Depth 영상의 촬영 시점 차이로 오차가 발생할 수 있습니다.
- 조난자 위치는 바운딩 박스 중심과 ROI 대표 깊이를 결합한 근사 위치입니다.
- 현재 World 좌표 계산에는 Gazebo Ground Truth를 사용합니다.
- 실제 로봇에서는 Ground Truth를 사용할 수 없습니다.
- 정식 Precision, Recall, mAP 성능 평가는 진행하지 않았습니다.
- 현재는 가장 가까운 조난자 한 명의 좌표만 발행합니다.
- SLAM 기반 탐색과 Nav2 자율 접근은 아직 구현하지 않았습니다.

---

## 향후 개선 방향

- YOLO Segmentation을 이용해 사람 마스크 내부의 Depth만 사용
- 사람 바운딩 박스 하단 중심을 이용한 지면 위치 추정
- RGB 영상과 Depth 영상의 시간 동기화
- 여러 프레임의 조난자 좌표 평균 또는 중앙값 평활화
- Depth 값 군집화를 이용한 사람과 배경 분리
- 화면 가장자리에서 잘린 탐지 결과 제외
- SLAM 및 Localization 기반 Map 좌표 변환
- 조난자 위치에서 일정 거리 떨어진 안전 접근 지점 생성
- Nav2 기반 조난자 자율 접근
- Frontier Exploration 기반 동굴 자율 탐색
- 열화상 카메라와 RGB-D 센서 융합
- 다수 조난자 좌표 저장 및 관리
- 조난자 거리와 상태에 따른 구조 우선순위 결정
- Precision, Recall, mAP 및 거리 오차 성능 평가

---

## 향후 실제 로봇 적용 구조

현재 시뮬레이션에서는 Gazebo Ground Truth를 사용하지만, 실제 로봇에서는 다음 구조로 변경해야 합니다.

```text
RGB-D 카메라
→ 사람 탐지 및 상대 위치 계산
→ 카메라 기준 좌표
→ 로봇 몸체 기준 좌표
→ SLAM / Localization
→ Map 기준 조난자 좌표
→ Nav2 안전 접근 목표 생성
```

실제 환경에서는 다음 좌표 관계를 사용하게 됩니다.

```text
map
→ odom
→ base_link
→ camera
```

---

## 요약

본 프로젝트에서는 RGB 영상으로 조난자의 화면 내 위치를 탐지하고, Depth 영상으로 조난자까지의 대표 깊이를 계산했습니다.

CameraInfo를 이용해 카메라 기준 3차원 좌표를 계산한 뒤, 카메라와 Go2 몸체 사이의 위치·방향 관계를 반영하여 로봇 몸체 기준 좌표로 변환했습니다.

마지막으로 Gazebo가 제공하는 로봇의 현재 위치와 방향을 적용해 동굴 전체 World 기준의 조난자 좌표를 계산했습니다.

계산된 좌표는 `/survivor_position` 토픽으로 발행하며, 조난자가 카메라 기준 약 1m 이내로 탐지되면 Go2가 자동으로 정지합니다.

```text
RGB 사람 탐지
+ Depth 거리 계산
+ CameraInfo 3차원 복원
+ TF 카메라-로봇 좌표 변환
+ Gazebo 로봇 위치 적용
= World 기준 조난자 좌표
```
