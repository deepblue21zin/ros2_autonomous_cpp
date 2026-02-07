# 후방 LiDAR 기반 수직 주차 시스템

2026 경기도 대학생 자율주행 경진대회 주차 미션 (별도 4분 경기, 200점).

## 대회 규정 요약

- 주차 공간: 950mm(W) x 1500mm(L), 수직 주차
- 2초 이상 정차 후 빠져나와 OUT 라인까지 주행
- 하드웨어: RPLiDAR를 차량 **후면**에 장착

## 파일 구조

| 파일 | 위치 | 설명 |
|------|------|------|
| `parking_node.hpp` | `decision/include/decision/` | 주차 노드 헤더 |
| `parking_node.cpp` | `decision/src/` | 주차 상태머신 + LiDAR 처리 |
| `parking_params.yaml` | `decision/config/` | 튜닝 파라미터 |
| `parking_launch.py` | `bringup/launch/` | 주차 전용 런치 파일 |

수정 파일: `decision/CMakeLists.txt` — `parking_node` 실행파일 + config install 추가

## 실행 방법

```bash
# 실차 실행
ros2 launch bringup parking_launch.py

# 단독 테스트 (auto_start 끔)
ros2 run decision parking_node --ros-args -p auto_start:=false

# 디버그: 현재 상태 모니터링
ros2 topic echo /parking/state
```

## 상태머신

```
IDLE → SEARCH → ALIGN → REVERSE_ENTER → STRAIGHTEN → HOLD → EXIT_FORWARD → SEEK_OUT → DONE
```

| 상태 | 동작 | 전이 조건 |
|------|------|-----------|
| **IDLE** | 정지, 시작 대기 | `auto_start=true` 또는 토픽 트리거 |
| **SEARCH** | 저속 전진, LiDAR 측면 스캔으로 주차 공간 탐색 | 빈 공간(gap) 감지 → ALIGN |
| **ALIGN** | 주차 공간을 지나친 후 정지 (overshoot) | 정지 완료 → REVERSE_ENTER |
| **REVERSE_ENTER** | 후진 + 조향 (공간 안으로 진입) | 후방 거리 < `back_wall_target_m` → STRAIGHTEN |
| **STRAIGHTEN** | 후진 + P제어 (좌우 벽 거리로 중앙 정렬) | 후방 거리 < `park_depth_m` → HOLD |
| **HOLD** | 완전 정지, 2초 이상 대기 | 타이머 만료 → EXIT_FORWARD |
| **EXIT_FORWARD** | 전진 + 조향 (공간 탈출) | 탈출 시간 경과 → SEEK_OUT |
| **SEEK_OUT** | 전진, `/parking/line_detected` 구독 | OUT 라인 감지 → DONE |
| **DONE** | 완전 정지 | 종료 |

## LiDAR 각도 규약 (후방 장착)

LiDAR의 0° (전방) = 차량의 **후방**:

```
         차량 전방 (LiDAR 180°)
              ↑
   좌 (270°) ← → 우 (90°)    ← LiDAR 기준
              ↓
         차량 후방 (LiDAR 0°)  ← LiDAR 전면
```

| 감지 방향 | LiDAR 각도 범위 | 용도 |
|-----------|-----------------|------|
| 차량 후방 | 350°~10° | 뒤 벽 거리 측정 |
| 차량 우측 | 80°~100° | 측면 공간 탐색 / 우측 벽 |
| 차량 좌측 | 260°~280° | 좌측 벽 |
| 차량 전방 | 170°~190° | (미사용) |

모든 각도는 파라미터로 조정 가능 (주차 공간이 좌/우 어느 쪽이든 대응).

## 주차 공간 감지 알고리즘 (SEARCH)

1. 측면 각도 범위 (예: 우측 80°~100°) 스캔
2. 가까운 장애물(벽) 있다가 → 갑자기 멀어지는 구간 = 공간 시작
3. 연속 원거리 포인트 수 >= `min_gap_points` → 유효한 주차 공간

## 주요 파라미터 (parking_params.yaml)

```yaml
/**:
  ros__parameters:
    auto_start: true              # 시작 즉시 SEARCH 진입

    # SEARCH
    search_speed_mps: 0.3         # 탐색 전진 속도
    side_scan_angle_min_deg: 80.0 # 측면 스캔 범위 (LiDAR 기준)
    side_scan_angle_max_deg: 100.0
    side_near_threshold_m: 0.6    # 벽/장애물 판정 거리
    side_far_threshold_m: 1.5     # 빈 공간 판정 거리
    min_gap_points: 5             # 공간 확정 최소 포인트 수

    # ALIGN
    align_duration_sec: 1.5       # 오버슈트 전진 시간

    # REVERSE_ENTER
    reverse_speed_mps: -0.25      # 후진 속도
    entry_steer_rad: 0.4          # 진입 조향각 (+좌/-우)
    back_wall_target_m: 0.5       # 뒤 벽 목표 거리 → STRAIGHTEN

    # STRAIGHTEN
    straighten_speed_mps: -0.15   # 정렬 후진 속도
    straighten_steer_kp: 2.0      # 좌우 벽 거리 차이 P 게인
    park_depth_m: 0.25            # 최종 뒤 벽 거리 → HOLD

    # HOLD
    hold_duration_sec: 3.0        # 정차 시간 (규정 2초 + 여유)

    # EXIT_FORWARD
    exit_speed_mps: 0.3           # 탈출 전진 속도
    exit_steer_rad: -0.4          # 탈출 조향 (진입 반대)
    exit_duration_sec: 2.5        # 탈출 기동 시간

    # SEEK_OUT
    seek_speed_mps: 0.3           # OUT 라인 탐색 속도

    # 안전
    emergency_stop_distance_m: 0.10
```

## 런치 파일 구성 (parking_launch.py)

포함:
1. RPLiDAR 드라이버 (후방 장착)
2. Arduino bridge (모터 제어)
3. 후방 카메라 (USB cam)
4. `parking_line_node` (OUT 라인 감지)
5. `parking_node` (주차 상태머신)
6. Static TF: `base_link → laser` (후방 위치), `base_link → camera_rear`

미포함 (불필요):
- 전방 카메라 / 차선 인식
- 초음파
- `decision_node` (parking_node가 직접 `/arduino/cmd` 제어)

## ROS 토픽

| 토픽 | 타입 | 방향 | 설명 |
|------|------|------|------|
| `/scan` | LaserScan | 구독 | RPLiDAR 스캔 데이터 |
| `/parking/line_detected` | Bool | 구독 | OUT 라인 감지 여부 |
| `/decision/cmd` → `/arduino/cmd` | AckermannDrive | 발행 | 모터 명령 |
| `/parking/state` | String | 발행 | 현재 상태 (디버그) |
| `/parking/overlay` | Image | 발행 | 후방 카메라 디버그 오버레이 |

## 주의사항

- `decision_node`와 동시 실행 금지 — 같은 `/arduino/cmd` 토픽에 publish 충돌
- 주차는 별도 경기이므로 `parking_launch.py`를 단독 실행
- LiDAR static TF: 후방 장착 반영 (`-0.15, 0, 0.05, 0, 0, 3.14159`)
- `entry_steer_rad` 부호로 주차 공간 방향 결정 (양수=좌측, 음수=우측)

## 튜닝 가이드

1. **SEARCH 감도**: `side_near_threshold_m`, `side_far_threshold_m`, `min_gap_points` 조정
2. **진입 각도**: `entry_steer_rad` (크면 급회전, 작으면 완만)
3. **정렬 정밀도**: `straighten_steer_kp` (크면 민감, 작으면 둔감)
4. **정차 위치**: `park_depth_m` (뒤 벽까지 최종 거리)
5. **탈출 기동**: `exit_steer_rad`, `exit_duration_sec`

실시간 파라미터 변경:
```bash
ros2 param set /parking_node entry_steer_rad 0.35
ros2 param set /parking_node hold_duration_sec 2.5
```
