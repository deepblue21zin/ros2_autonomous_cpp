#!/bin/bash
# =============================================================
# ADAS 자율주행 Docker 환경 시작 스크립트
# 사용법: ./start.sh [옵션]
#   ./start.sh              : 컨테이너 시작 + 접속
#   ./start.sh build        : 이미지 빌드 후 시작
#   ./start.sh ros-build    : ROS2 colcon 빌드 (컨테이너 내부)
#   ./start.sh run          : track_launch.py 실행 (기존 차선 인식)
#   ./start.sh test         : test_mode로 실행
#   ./start.sh yolo         : YOLOv8n-seg 노드 실행
#   ./start.sh camera       : 카메라+차선인식 테스트 (Python, 주행 없음)
#   ./start.sh camera_cpp   : 카메라+차선인식 테스트 (C++, 주행 없음)
#   ./start.sh camera_cpp_bag : rosbag 재생 + 차선인식 테스트 (C++, rviz2)
#   ./start.sh ros_bag_motor : rosbag 재생 + 차선인식 + 모터 구동 (C++, rviz2)
#   ./start.sh yolo-drive   : YOLO 차선 주행 모드 (세그멘테이션 기반)
#   ./start.sh yolo_rosbag  : YOLO rosbag 테스트 (rviz2 시각화)
#   ./start.sh hybrid-motor : HSV+YOLO 하이브리드 주행 (추천 ⭐)
#   ./start.sh hybrid-rosbag: HSV+YOLO rosbag 테스트 (모터 OFF)
#   ./start.sh hybrid-rosbag-motor: HSV+YOLO rosbag + 모터 구동 확인
#   ./start.sh rviz         : rviz2 실행 (토픽 자동 설정)
#   ./start.sh restart      : 컨테이너 재시작 (장치 재마운트)
#   ./start.sh stop         : 컨테이너 정지
# =============================================================

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CONTAINER_NAME="adas_container"

# 색상 출력
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

info()  { echo -e "${GREEN}[INFO]${NC} $1"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; }

# --- 1. X11 권한 설정 ---
setup_x11() {
    info "X11 권한 설정..."
    xhost +local:docker > /dev/null 2>&1
}

# --- 2. 장치 확인 ---
check_devices() {
    info "연결된 장치 확인..."

    # Arduino
    if [ -e /dev/ttyACM0 ]; then
        info "  Arduino:  /dev/ttyACM0"
    else
        warn "  Arduino:  미연결 (/dev/ttyACM0 없음)"
    fi

    # LiDAR
    if [ -e /dev/ttyUSB0 ]; then
        info "  LiDAR:    /dev/ttyUSB0"
    else
        warn "  LiDAR:    미연결 (/dev/ttyUSB0 없음)"
    fi

    # Camera - USB 웹캠 자동 감지
    CAM_FOUND=false
    for dev in /dev/video4 /dev/video6 /dev/video0; do
        if [ -e "$dev" ]; then
            info "  Camera:   $dev"
            CAM_FOUND=true
            break
        fi
    done
    if [ "$CAM_FOUND" = false ]; then
        warn "  Camera:   미연결 (video 장치 없음)"
    fi

    echo ""
}

# --- 3. Docker 컨테이너 시작 ---
start_container() {
    cd "$SCRIPT_DIR"

    # 컨테이너 실행 상태 확인
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        info "컨테이너 이미 실행 중: ${CONTAINER_NAME}"
    else
        info "컨테이너 시작..."
        docker compose up -d
        sleep 2

        if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            info "컨테이너 시작 완료"
        else
            error "컨테이너 시작 실패"
            docker compose logs --tail=20
            exit 1
        fi
    fi
}

# --- 4. 컨테이너 내부 장치 확인 ---
verify_container_devices() {
    info "컨테이너 내부 장치 확인..."

    # 카메라
    CAM_LIST=$(docker exec "$CONTAINER_NAME" ls /dev/video* 2>/dev/null)
    if [ -n "$CAM_LIST" ]; then
        info "  카메라 장치: $(echo $CAM_LIST | tr '\n' ' ')"
    else
        warn "  카메라 장치 없음 - 컨테이너 재시작 필요: ./start.sh restart"
    fi

    # 시리얼
    SERIAL_LIST=$(docker exec "$CONTAINER_NAME" ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null)
    if [ -n "$SERIAL_LIST" ]; then
        info "  시리얼 장치: $(echo $SERIAL_LIST | tr '\n' ' ')"
    else
        warn "  시리얼 장치 없음"
    fi

    echo ""
}

# --- 5. 컨테이너 접속 ---
enter_container() {
    info "컨테이너 접속: ${CONTAINER_NAME}"
    echo "--------------------------------------------"
    echo "  ROS2 빌드: colcon build --symlink-install"
    echo "  시스템 실행: ros2 launch bringup track_launch.py"
    echo "  테스트 모드: ros2 launch bringup track_launch.py test_mode:=true"
    echo "--------------------------------------------"
    echo ""
    docker exec -it "$CONTAINER_NAME" bash
}

# --- Docker 내부 명령 실행 헬퍼 ---
docker_run() {
    docker exec -it "$CONTAINER_NAME" bash -c "source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash 2>/dev/null; $1"
}

# --- 카메라 자동 감지 + 컨테이너 확인 ---
detect_and_verify_camera() {
    info "카메라 장치 확인..."
    CAM_DEV=""
    # 고정 경로 우선 사용 (장치 번호 변경 방지)
    BY_ID="/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920-video-index0"
    if [ -e "$BY_ID" ]; then
        # 심볼릭 링크를 실제 /dev/videoN 경로로 변환 (Docker 내부에는 by-id 없음)
        CAM_DEV=$(readlink -f "$BY_ID")
        info "고정 경로 감지: $BY_ID → $CAM_DEV"
    else
        # fallback: YUYV/MJPEG 지원하는 장치 자동 감지
        for dev in /dev/video2 /dev/video0 /dev/video4 /dev/video6; do
            if [ -e "$dev" ]; then
                if v4l2-ctl -d "$dev" --list-formats 2>/dev/null | grep -qE "YUYV|MJPEG"; then
                    CAM_DEV="$dev"
                    break
                fi
            fi
        done
    fi

    if [ -z "$CAM_DEV" ]; then
        error "카메라를 찾을 수 없습니다!"
        echo "  연결된 video 장치:"
        ls /dev/video* 2>/dev/null || echo "    없음"
        exit 1
    fi

    CAM_NAME=$(v4l2-ctl -d "$CAM_DEV" --all 2>/dev/null | grep "Card type" | sed 's/.*: //' || echo "Unknown")
    info "카메라 감지: $CAM_DEV ($CAM_NAME)"

    # 컨테이너 내부 카메라 확인
    if docker exec "$CONTAINER_NAME" test -e "$CAM_DEV" 2>/dev/null; then
        info "컨테이너 내부 카메라 확인: $CAM_DEV OK"
    else
        warn "컨테이너에 $CAM_DEV 없음 - 재시작 필요"
        info "컨테이너 재시작 중..."
        cd "$SCRIPT_DIR"
        docker compose down
        sleep 1
        docker compose up -d
        sleep 2
        if docker exec "$CONTAINER_NAME" test -e "$CAM_DEV" 2>/dev/null; then
            info "재시작 후 카메라 확인: $CAM_DEV OK"
        else
            error "재시작 후에도 카메라 미인식. docker-compose.yml 장치 매핑 확인 필요"
            exit 1
        fi
    fi
}

# --- numpy 호환성 확인 (cv_bridge 충돌 방지) ---
fix_numpy() {
    docker exec "$CONTAINER_NAME" bash -c "python3 -c 'import numpy; v=numpy.__version__; exit(0 if v.startswith(\"1.\") else 1)'" 2>/dev/null
    if [ $? -ne 0 ]; then
        warn "numpy 2.x 감지 → 1.x로 다운그레이드..."
        docker exec "$CONTAINER_NAME" pip3 install -q 'numpy<2' 2>/dev/null
    fi
}

# --- 메인 ---
case "${1:-}" in
    build)
        setup_x11
        check_devices
        info "Docker 이미지 빌드..."
        cd "$SCRIPT_DIR"
        docker compose build
        docker compose down 2>/dev/null
        start_container
        verify_container_devices
        enter_container
        ;;
    ros-build)
        info "ROS2 colcon 빌드..."
        docker_run "cd /root/ros2_ws && colcon build --symlink-install"
        info "빌드 완료"
        ;;
    run)
        info "track_launch.py 실행..."
        docker_run "ros2 launch bringup track_launch.py"
        ;;
    test)
        info "test_mode로 실행..."
        docker_run "ros2 launch bringup track_launch.py test_mode:=true"
        ;;
    camera)
        setup_x11
        start_container

        fix_numpy

        info "듀얼 카메라 + 차선인식 + YOLO 테스트 [Python] 시작..."
        info "  Front (lane): /dev/video4, Upper (YOLO): /dev/video6"
        docker_run "ros2 launch bringup camera_test_launch.py use_cpp:=false"
        ;;
    camera_cpp)
        setup_x11
        start_container

        fix_numpy

        info "듀얼 카메라 + 차선인식 + YOLO 테스트 [C++] 시작..."
        info "  Front (lane): /dev/video4, Upper (YOLO): /dev/video6"
        docker_run "ros2 launch bringup camera_test_launch.py use_cpp:=true"
        ;;
    yolo)
        setup_x11
        start_container
        detect_and_verify_camera
        fix_numpy
        info "YOLOv8n-seg 노드 실행 (카메라 포함)..."
        docker_run "ros2 launch usb_cam_driver usb_cam_launch.py video_device:=$CAM_DEV & sleep 2 && ros2 run perception_pkg yolov8n_seg_node.py --ros-args -p model_path:=/root/ros2_ws/src/perception_pkg/models/yolo26n_main.pt"
        ;;
    camera_cpp_bag)
        setup_x11
        start_container

        info "rosbag + 차선 인식 테스트 [C++] + rviz2 시작..."
        docker_run "ros2 launch bringup camera_bag_test_launch.py"
        ;;
    ros_bag_motor)
        setup_x11
        start_container

        info "rosbag + 차선 인식 + 모터 구동 [C++] + rviz2 시작..."
        docker_run "ros2 launch bringup bag_motor_launch.py"
        ;;
    yolo-drive)
        start_container
        fix_numpy
        info "YOLO 차선 주행 모드 실행..."
        docker_run "ros2 launch bringup yolo_drive_launch.py"
        ;;
    yolo_rosbag)
        setup_x11
        start_container
        fix_numpy
        info "YOLO rosbag 테스트 (rviz2 시각화)..."
        docker_run "ros2 launch bringup yolo_bag_test_launch.py"
        ;;
    hybrid-motor)
        setup_x11
        start_container
        detect_and_verify_camera
        fix_numpy
        info "HSV + YOLO 하이브리드 주행 모드 실행..."
        info "  - HSV: 30Hz 빠른 차선 추종"
        info "  - YOLO: 10Hz 장애물 검출 및 회피"
        info "  - RViz2 시각화 자동 실행"
        docker_run "ros2 launch bringup hybrid_drive_launch.py front_video_device:=/dev/video4 upper_video_device:=/dev/video6 use_rviz:=true yolo_publish_overlay:=true"
        ;;
    hybrid-rosbag)
        setup_x11
        start_container
        fix_numpy
        info "HSV + YOLO 하이브리드 rosbag 테스트 (모터 OFF)..."
        docker_run "ros2 launch bringup hybrid_bag_test_launch.py use_motor:=false use_rviz:=true"
        ;;
    hybrid-rosbag-motor)
        setup_x11
        start_container
        fix_numpy
        info "HSV + YOLO 하이브리드 rosbag + 모터 구동 테스트..."
        docker_run "ros2 launch bringup hybrid_bag_test_launch.py use_motor:=true use_rviz:=true"
        ;;
    rviz)
        setup_x11
        RVIZ_CONFIG="/root/ros2_ws/install/bringup/share/bringup/config/adas_default.rviz"
        # symlink-install이면 src에서 직접 사용
        RVIZ_SRC="/root/ros2_ws/src/bringup/config/adas_default.rviz"
        info "rviz2 실행 (토픽 자동 설정)..."
        docker_run "if [ -f $RVIZ_SRC ]; then rviz2 -d $RVIZ_SRC; elif [ -f $RVIZ_CONFIG ]; then rviz2 -d $RVIZ_CONFIG; else echo 'rviz config 없음 - ros-build 먼저 실행'; fi"
        ;;
    restart)
        setup_x11
        check_devices
        info "컨테이너 재시작 (장치 재마운트)..."
        cd "$SCRIPT_DIR"
        docker compose down
        sleep 1
        docker compose up -d
        sleep 2
        verify_container_devices
        enter_container
        ;;
    stop)
        info "컨테이너 정지..."
        cd "$SCRIPT_DIR"
        docker compose down
        info "완료"
        ;;
    help)
        echo "사용법: ./start.sh [옵션]"
        echo ""
        echo "  (없음)       컨테이너 시작 + 접속"
        echo "  build        Docker 이미지 빌드 후 시작"
        echo "  ros-build    ROS2 colcon 빌드"
        echo "  run          track_launch.py 실행 (기존 차선 인식)"
        echo "  test         test_mode로 실행"
        echo "  camera       카메라+차선인식 테스트 (Python, 주행 없음)"
        echo "  camera_cpp   카메라+차선인식 테스트 (C++, 주행 없음)"
        echo "  camera_cpp_bag rosbag 재생 + 차선인식 테스트 (C++, rviz2)"
        echo "  ros_bag_motor rosbag 재생 + 차선인식 + 모터 구동 (C++, rviz2)"
        echo "  yolo         YOLOv8n-seg 노드 실행"
        echo "  yolo-drive   YOLO 차선 주행 모드 실행 (세그멘테이션 기반)"
        echo "  yolo_rosbag  YOLO rosbag 테스트 (rviz2 시각화)"
        echo "  hybrid-motor HSV+YOLO 하이브리드 주행 (추천 ⭐)"
        echo "  hybrid-rosbag HSV+YOLO rosbag 테스트 (모터 OFF)"
        echo "  hybrid-rosbag-motor HSV+YOLO rosbag + 모터 구동 테스트"
        echo "  rviz         rviz2 실행 (토픽 자동 설정)"
        echo "  restart      컨테이너 재시작"
        echo "  stop         컨테이너 정지"
        echo "  help         이 도움말"
        ;;
    *)
        setup_x11
        check_devices
        start_container
        verify_container_devices
        enter_container
        ;;
esac
