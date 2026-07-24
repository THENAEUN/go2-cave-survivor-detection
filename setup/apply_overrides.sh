#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="${1:-$HOME/cave_world_project}"

UNITREE="$WORKSPACE/src/unitree-go2-ros2"
CAVE_WORLD="$WORKSPACE/src/gazebo_cave_world"

if [ ! -d "$UNITREE" ]; then
    echo "오류: Unitree Go2 저장소가 없습니다: $UNITREE"
    exit 1
fi

if [ ! -d "$CAVE_WORLD" ]; then
    echo "오류: Gazebo cave world 저장소가 없습니다: $CAVE_WORLD"
    exit 1
fi

echo "[1/7] RGB-D 카메라 파일 적용"
cp -v \
"$PROJECT_ROOT/overrides/unitree-go2-ros2/robots/descriptions/go2_description/xacro/rgbd_camera.xacro" \
"$UNITREE/robots/descriptions/go2_description/xacro/rgbd_camera.xacro"

echo "[2/7] Go2 robot.xacro 적용"
cp -v \
"$PROJECT_ROOT/overrides/unitree-go2-ros2/robots/descriptions/go2_description/xacro/robot.xacro" \
"$UNITREE/robots/descriptions/go2_description/xacro/robot.xacro"

echo "[3/7] Go2 gait 설정 적용"
cp -v \
"$PROJECT_ROOT/overrides/unitree-go2-ros2/robots/configs/go2_config/config/gait/gait.yaml" \
"$UNITREE/robots/configs/go2_config/config/gait/gait.yaml"

echo "[4/7] description.launch.py 적용"
cp -v \
"$PROJECT_ROOT/overrides/unitree-go2-ros2/champ/champ_description/launch/description.launch.py" \
"$UNITREE/champ/champ_description/launch/description.launch.py"

echo "[5/7] bringup.launch.py 적용"
cp -v \
"$PROJECT_ROOT/overrides/unitree-go2-ros2/champ/champ_bringup/launch/bringup.launch.py" \
"$UNITREE/champ/champ_bringup/launch/bringup.launch.py"

echo "[6/7] gazebo.launch.py 적용"
cp -v \
"$PROJECT_ROOT/overrides/unitree-go2-ros2/champ/champ_gazebo/launch/gazebo.launch.py" \
"$UNITREE/champ/champ_gazebo/launch/gazebo.launch.py"

echo "[7/7] 수정된 조난자 월드와 YOLO 코드 적용"
cp -v \
"$PROJECT_ROOT/worlds/cave_world_rescue.world" \
"$CAVE_WORLD/worlds/cave_world_rescue.world"

mkdir -p "$WORKSPACE/scripts"

cp -v \
"$PROJECT_ROOT/scripts/yolo_survivor_detector.py" \
"$WORKSPACE/scripts/yolo_survivor_detector.py"

chmod +x "$WORKSPACE/scripts/yolo_survivor_detector.py"

echo
echo "수정 파일 적용 완료"
echo "다음 명령을 실행하세요:"
echo "cd $WORKSPACE"
echo "colcon build --symlink-install"
