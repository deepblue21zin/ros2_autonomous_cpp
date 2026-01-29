# Code Change Log

코드 변경 이력을 기록합니다.

---

## 2026-01-29

### 1. README.md 전체 재작성

**커밋:** `f210a94`

**변경 내용:**
- 프로젝트 구조 문서화 (8개 ROS2 패키지)
- 시스템 아키텍처 다이어그램 추가
- 하드웨어 연결 가이드 (Arduino, LiDAR, Camera)
- 설정 파일 예시 (yaml)
- 빌드 및 실행 방법
- 토픽 목록 (센서, 인식, 제어)
- Arduino 명령 프로토콜 문서화
- 실차 테스트 절차

---

### 2. requirements.txt 추가

**커밋:** `3ba7db7`

**파일:** `requirements.txt`

**내용:**
```
numpy>=1.21.0
opencv-python>=4.5.0
ultralytics>=8.0.0
pyserial>=3.5
```

---

### 3. README.md 의존성 설치 섹션 추가

**커밋:** `3ba7db7`

**추가된 내용:**
- ROS2 apt 패키지 설치 명령
- Python pip 패키지 설치 명령

---

### 4. .gitignore 업데이트

**커밋:** `f210a94`

**추가된 항목:**
```
.vscode/
*.db
*.ipch
build/
install/
log/
```

---

### 5. Docker 환경 설정

**파일:** `/home/deepblue/target_projects/adas_env/compose.yaml`

**변경 내용:**
- YAML 들여쓰기 수정
- 볼륨 마운트 경로 수정: `ros2_autonomous_cpp` → `/root/ros2_ws`
- 디바이스 추가: `/dev/ttyACM0`, `/dev/ttyUSB0`, `/dev/video0`
- `privileged: true` 추가

**최종 설정:**
```yaml
services:
  adas-dev:
    image: osrf/ros:humble-desktop
    container_name: adas_container
    network_mode: host
    ipc: host
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
      - /home/deepblue/target_projects/ros2_autonomous_cpp/ros2_autonomous_cpp:/root/ros2_ws
    environment:
      - DISPLAY=${DISPLAY}
    stdin_open: true
    tty: true
    privileged: true
    devices:
      - /dev/ttyACM0:/dev/ttyACM0
      - /dev/ttyUSB0:/dev/ttyUSB0
      - /dev/video0:/dev/video0
```

---

## 예정된 변경

- [ ] 차선 추적 YOLO 모델 적용 (GPU 환경 필요)
- [ ] Docker GPU 지원 추가
- [ ] 실차 테스트 후 파라미터 튜닝
