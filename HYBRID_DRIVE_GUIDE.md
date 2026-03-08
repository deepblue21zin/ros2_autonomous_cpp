# HSV + YOLO Hybrid Drive 사용 가이드

## 📋 시스템 구조

```
┌──────────────────────────────────────────────────────────┐
│  Camera (30Hz)                                           │
│  ↓                                                       │
├──────────────────────────────────────────────────────────┤
│  HSV Lane Tracking (30Hz)    YOLO Segmentation (10Hz)   │
│  ├─ 빠른 차선 중심 검출         ├─ 주행 가능 영역           │
│  └─ /lane/center_offset       └─ /perception/drivable_mask│
│                                  (장애물 검출)             │
├──────────────────────────────────────────────────────────┤
│  Hybrid Lane Drive (20Hz)                                │
│  ┌────────────────────────────────────┐                  │
│  │  상태 기계:                         │                  │
│  │  • NORMAL: HSV 차선 중심 추종       │                  │
│  │  • AVOIDING: 장애물 회피 (차선 변경) │                  │
│  │  • RETURNING: 차선 중앙 복귀        │                  │
│  └────────────────────────────────────┘                  │
│  ↓                                                       │
│  /decision/cmd (AckermannDrive)                          │
├──────────────────────────────────────────────────────────┤
│  Arduino Bridge → 모터 제어                              │
└──────────────────────────────────────────────────────────┘
```

---

## 🚀 빠른 시작

### 1. Docker 컨테이너 접속
```bash
# Docker 컨테이너 실행 (호스트에서)
docker exec -it <container_name> bash

# 또는
docker-compose exec ros2_dev bash
```

### 2. start.sh 복사 (처음 한 번만)
```bash
# Docker 컨테이너 내부에서
cd /root/ros2_ws
cp src/start.sh .
chmod +x start.sh
```

### 3. 빌드
```bash
./start.sh build
source install/setup.bash
```

### 4. 실행
```bash
# 실제 주행 (추천 ⭐)
./start.sh hybrid-motor

# rosbag 테스트
./start.sh yolo-rosbag

# 또는 특정 rosbag 경로 지정
./start.sh yolo-rosbag /path/to/rosbag
```

---

## 📁 생성된 파일

```
ros2_autonomous_cpp/
├── src/
│   ├── decision/
│   │   ├── scripts/
│   │   │   ├── hybrid_lane_drive_node.py  ✨ 새로 생성
│   │   │   └── yolo_lane_drive_node.py
│   │   └── config/
│   │       └── hybrid_lane_drive.yaml     ✨ 새로 생성
│   │
│   ├── bringup/
│   │   └── launch/
│   │       ├── hybrid_drive_launch.py      ✨ 새로 생성
│   │       ├── hybrid_bag_test_launch.py   ✨ 새로 생성
│   │       ├── yolo_drive_launch.py
│   │       └── yolo_bag_test_launch.py
│   │
│   └── perception_pkg/
│       └── scripts/
│           ├── lane_tracking_node.py       (기존)
│           └── yolov8n_seg_node.py         (기존)
│
└── start.sh                                 ✨ 새로 생성
```

---

## 🎮 start.sh 사용법

### 주요 명령어

```bash
# 1. 실제 주행 (HSV + YOLO)
./start.sh hybrid-motor

# 2. rosbag 테스트 (HSV + YOLO + Motor)
./start.sh yolo-rosbag

# 3. rosbag 경로 지정
./start.sh yolo-rosbag /root/ros2_ws/rosbag2_2026_01_30-03_20_53

# 4. 전체 빌드
./start.sh build

# 5. 특정 패키지만 빌드
./start.sh build-pkg decision
./start.sh build-pkg bringup

# 6. 워크스페이스 정리
./start.sh clean

# 7. 도움말
./start.sh help
```

### Legacy 모드 (YOLO만 사용)
```bash
# YOLO 단독 주행
./start.sh yolo-drive

# YOLO 단독 rosbag 테스트
./start.sh yolo-bag-test
```

---

## ⚙️ 파라미터 조정

### YAML 파일 수정
```bash
# Docker 컨테이너 내부에서
nano /root/ros2_ws/src/decision/config/hybrid_lane_drive.yaml
```

### 주요 파라미터

| 파라미터 | 기본값 | 설명 | 조정 방법 |
|---------|--------|------|----------|
| `cruise_speed_mps` | 0.3 | 정상 주행 속도 (m/s) | 0.25 ~ 0.4 |
| `slow_speed_mps` | 0.2 | 회피 시 속도 (m/s) | 0.15 ~ 0.3 |
| `kp` | 2.5 | 비례 게인 (조향 강도) | 약하면 증가: 2.5 → 3.0 → 3.5 |
| `kd` | 0.6 | 미분 게인 (안정성) | 진동하면 증가: 0.6 → 0.8 → 1.0 |
| `max_steer_rad` | 0.45 | 최대 조향각 (rad) | 0.35 ~ 0.7 (25° ~ 40°) |
| `avoid_offset` | 0.4 | 회피 시 차선 변경 offset | 0.3 ~ 0.6 |
| `obstacle_detect_threshold` | 0.3 | 장애물 감지 임계값 | 민감: 0.2, 둔감: 0.4 |

### 실시간 파라미터 변경 (Launch 시)
```bash
# 속도와 게인 조정
ros2 launch bringup hybrid_drive_launch.py \
  cruise_speed:=0.35 \
  kp:=3.0 \
  kd:=0.8

# rosbag 테스트 시
ros2 launch bringup hybrid_bag_test_launch.py \
  bag_path:=/path/to/rosbag \
  cruise_speed:=0.3 \
  kp:=2.5
```

---

## 📊 모니터링

### 실시간 모니터링 명령어

```bash
# 터미널 1: 주행 상태
ros2 topic echo /decision/state

# 터미널 2: HSV 차선 offset
ros2 topic echo /lane/center_offset

# 터미널 3: 모터 명령
ros2 topic echo /decision/cmd

# 터미널 4: 노드 정보
ros2 node list
ros2 node info /hybrid_lane_drive
```

### 로그 분석

**정상 동작 로그:**
```
[NORMAL] lane=-0.10, target=-0.10, steer=+0.25rad, speed=0.30m/s
[NORMAL] lane=-0.05, target=-0.05, steer=+0.12rad, speed=0.30m/s
장애물 감지 → AVOIDING_RIGHT
[AVOIDING_RIGHT] lane=-0.10, target=+0.30, steer=-0.45rad, speed=0.20m/s
[AVOIDING_RIGHT] lane=+0.15, target=+0.55, steer=-0.45rad, speed=0.20m/s
장애물 통과 → RETURNING
[RETURNING] lane=+0.20, target=+0.20, steer=-0.45rad, speed=0.30m/s
[RETURNING] lane=+0.10, target=+0.10, steer=-0.25rad, speed=0.30m/s
차선 복귀 완료 → NORMAL
```

**문제 발생 로그:**
```
❌ STOP: no lane offset
   → HSV 차선 검출 실패, /lane/center_offset 토픽 확인

❌ STOP: lane timeout (2.5s)
   → HSV 노드가 죽었거나 카메라 문제

❌ [ERROR] 마스크 변환 실패
   → YOLO 노드 문제, /perception/drivable_mask 확인
```

---

## 🔍 문제 해결

### Q1: "no lane offset" 에러
**원인:** HSV 차선 검출 실패
```bash
# 해결:
ros2 topic hz /lane/center_offset  # 토픽 확인
ros2 node info /lane_tracking       # 노드 상태 확인

# lane_tracking_node가 없으면 재시작
```

### Q2: 조향이 너무 약함
**원인:** kp 게인이 낮음
```yaml
# hybrid_lane_drive.yaml 수정
kp: 3.0  # 2.5 → 3.0
```

### Q3: 조향이 진동함
**원인:** kp가 너무 크거나 kd가 작음
```yaml
kp: 2.0  # 2.5 → 2.0 (줄이기)
kd: 0.8  # 0.6 → 0.8 (높이기)
```

### Q4: 장애물 회피가 약함
**원인:** avoid_offset이 작음
```yaml
avoid_offset: 0.5  # 0.4 → 0.5
```

### Q5: 장애물 감지가 너무 민감함
**원인:** obstacle_detect_threshold가 낮음
```yaml
obstacle_detect_threshold: 0.4  # 0.3 → 0.4 (덜 민감)
```

### Q6: RViz2가 안 뜸
**원인:** Display 설정 문제
```bash
# 호스트에서 (Docker 실행 전)
xhost +local:docker

# 또는 Docker run 시
docker run -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ...
```

---

## 🎯 성능 비교

| 모드 | FPS | 안정성 | 장애물 회피 | 추천도 |
|------|-----|--------|-----------|--------|
| **Hybrid (HSV+YOLO)** | 30Hz | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ✅✅✅ |
| YOLO만 | 10Hz | ⭐⭐⭐ | ⭐⭐⭐⭐ | ⚠️ |
| HSV만 | 30Hz | ⭐⭐⭐⭐⭐ | ❌ | ⚠️ |

---

## 📚 추가 자료

- **튜닝 가이드**: [TUNING_GUIDE.md](TUNING_GUIDE.md)
- **에러 로그**: [archive/docs_archive/error.md](archive/docs_archive/error.md)
- **변경 이력**: [archive/docs_archive/change_code.md](archive/docs_archive/change_code.md)

---

## 🏆 대회 전략

### 트랙 주행 (장애물 없음)
```bash
# HSV만으로도 충분하지만, hybrid 사용 추천
./start.sh hybrid-motor
```
- 30Hz 빠른 제어
- 안정적인 차선 추종

### 장애물 회피
```bash
# HSV + YOLO 결합 (필수!)
./start.sh hybrid-motor
```
- HSV로 빠른 차선 추종
- YOLO로 정확한 장애물 검출
- 자동 차선 변경 및 복귀

---

## ⚠️ 주의사항

1. **빌드 필수**: 새 노드를 추가했으므로 반드시 빌드 필요
   ```bash
   ./start.sh build
   source install/setup.bash
   ```

2. **파라미터 수정 후 재시작**: YAML 파일 수정 후 노드 재시작 필요

3. **모터 테스트 주의**: rosbag 테스트 시에도 모터가 실제로 움직임!

4. **RViz2 Display**: X11 forwarding 설정 확인

5. **GPU 필요**: YOLO 노드는 GPU 권장 (Jetson 등)

---

## 📞 문제 발생 시

1. 로그 확인: `ros2 topic echo /rosout`
2. 노드 상태: `ros2 node list`
3. 토픽 확인: `ros2 topic list`
4. 이슈 리포트: GitHub Issues

---

**Happy Driving! 🚗💨**
