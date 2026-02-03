# YOLOv8n-seg 통합 감지 시스템 가이드

## 개요

YOLOv8n-seg 모델을 활용한 통합 감지 시스템으로, 기존 OpenCV 기반 인식을 보조합니다.

**목적:**
1. **신호등 감지:** YOLO로 신호등 바운딩 박스 감지 → HSV 색상 분석 → 빨간불 정지 / 초록불 주행
2. **차선 인식 보조:** 세그멘테이션 마스크로 장애물 영역을 제외하여 코너/횡단보도에서 차선 인식 안정성 향상
3. **장애물 감지:** 보행자, 차량 등 장애물 감지 + 세그멘테이션 마스크

## 파일 구조

```
src/perception_pkg/
├── perception_pkg/perception/object_detection/
│   ├── detector.py                  # 기존 YOLO 감지기 (traffic_sign_detector.pt)
│   ├── detector_yolov8n.py          # YOLOv8n-seg 감지 라이브러리 (신규)
│   └── yolo_speed_sign_pt.py        # 속도 표지판 감지기
├── scripts/
│   └── yolov8n_seg_node.py          # ROS2 노드 (신규)
└── models/
    ├── traffic_sign_detector.pt     # 기존 모델
    └── yolov8n-seg.pt               # YOLOv8n 세그멘테이션 모델
```

## ROS2 토픽

### 발행 (Publish)

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/perception/traffic_light_state` | `String` | 신호등 상태: red, yellow, green, unknown |
| `/perception/traffic_light_detected` | `Bool` | 신호등 감지 여부 |
| `/perception/yolo_obstacles` | `Float32MultiArray` | 장애물 정보 [x1,y1,x2,y2,conf, ...] |
| `/yolo/overlay` | `Image` | 디버그 오버레이 이미지 (rviz2용) |
| `/perception/drivable_mask` | `Image` | 주행 가능 영역 마스크 (mono8) |

### 구독 (Subscribe)

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/camera/front/image` | `Image` | 전방 카메라 이미지 (기본값) |

## 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `model_path` | 자동 탐색 | yolov8n-seg.pt 경로 |
| `camera_topic` | `/camera/front/image` | 카메라 토픽 |
| `conf_threshold` | 0.4 | YOLO 신뢰도 임계값 |
| `iou_threshold` | 0.45 | NMS IOU 임계값 |
| `imgsz` | 640 | 추론 이미지 크기 |
| `device` | 자동 | 추론 장치 (cpu, cuda:0) |
| `publish_overlay` | true | 오버레이 이미지 발행 여부 |
| `publish_drivable_mask` | true | 주행 가능 마스크 발행 여부 |
| `detect_obstacles` | true | 장애물 감지 사용 여부 |
| `traffic_light_min_ratio` | 0.05 | 신호등 색상 최소 비율 |
| `rate_hz` | 10.0 | 추론 주기 (Hz) |

## 실행 방법

### 1. 단독 실행

```bash
# Docker 안에서
ros2 run perception_pkg yolov8n_seg_node.py
```

### 2. 파라미터 지정 실행

```bash
ros2 run perception_pkg yolov8n_seg_node.py --ros-args \
  -p model_path:=/root/ros2_ws/src/perception_pkg/models/yolov8n-seg.pt \
  -p camera_topic:=/camera/front/image \
  -p conf_threshold:=0.3 \
  -p rate_hz:=5.0
```

### 3. track_launch.py와 함께 실행

별도 터미널에서 실행:
```bash
# 터미널 1: 기본 시스템
ros2 launch bringup track_launch.py

# 터미널 2: YOLO 노드
ros2 run perception_pkg yolov8n_seg_node.py
```

## rviz2에서 확인하는 방법

1. rviz2 왼쪽 하단 **Add** 클릭
2. **By topic** 탭 선택
3. 원하는 토픽 선택:
   - `/yolo/overlay` → **Image** → OK: YOLO 감지 결과 오버레이
   - `/perception/drivable_mask` → **Image** → OK: 주행 가능 영역 마스크
4. 왼쪽 패널 하단에 Image 패널이 추가됨

## 신호등 감지 원리

```
카메라 이미지
    ↓
YOLOv8n-seg 추론 (traffic light class = 9)
    ↓
바운딩 박스 추출
    ↓
ROI 내부 HSV 색상 분석
    ├── 빨간색: H[0-10, 170-180], S[80-255], V[80-255]
    ├── 초록색: H[45-90], S[80-255], V[80-255]
    └── 노란색: H[18-38], S[100-255], V[100-255]
    ↓
최대 비율 색상 = 신호 상태
    ↓
/perception/traffic_light_state 발행
```

## 차선 인식 보조 원리

```
카메라 이미지
    ↓
YOLOv8n-seg 추론
    ↓
장애물 클래스 (person, car, bus, truck) 세그멘테이션 마스크 추출
    ↓
주행 가능 영역 마스크 생성 (255=주행가능, 0=장애물)
    ↓
/perception/drivable_mask 발행
    ↓
OpenCV 차선 인식에서 마스크 적용:
  canny_masked = cv2.bitwise_and(canny_edge, canny_edge, mask=drivable)
```

## 감지 가능 클래스 (COCO)

| ID | 클래스 | 용도 |
|----|--------|------|
| 0 | person | 보행자 장애물 |
| 2 | car | 차량 장애물 |
| 5 | bus | 차량 장애물 |
| 7 | truck | 차량 장애물 |
| 9 | traffic light | 신호등 감지 → HSV 색상 분석 |

## 빌드

```bash
cd /root/ros2_ws
colcon build --symlink-install --packages-select perception_pkg
source install/setup.bash
```

## 의존성

- `ultralytics` (YOLOv8): Dockerfile에 추가됨
- `opencv-python`: 기존 설치
- `cv_bridge`: ROS2 이미지 변환
- `numpy`: 배열 처리
