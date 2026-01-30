# 센서 위치 캘리브레이션 가이드

## base_link 기준점 정의

**선택한 기준:** 뒷바퀴 축 중심, 지면 높이

```
좌표계:
- x축: 전방(+) / 후방(-)
- y축: 오른쪽(+) / 왼쪽(-)
- z축: 위(+) / 아래(-)
```

---

## 측정 방법

### 1. 준비물
- 줄자 또는 자
- 마스킹 테이프 (기준점 표시)
- 수평계 (선택)

### 2. base_link 표시
1. 차량을 평평한 곳에 놓기
2. 뒷바퀴 축의 정중앙에 테이프로 표시
3. 지면에서 수직으로 올라간 점이 base_link (x=0, y=0, z=0)

### 3. 센서별 측정

#### LiDAR (RPLiDAR A1)
```
측정 위치: LiDAR 회전축 중심

x: _____ cm (앞쪽이면 +, 뒤쪽이면 -)
y: _____ cm (오른쪽이면 +, 왼쪽이면 -)
z: _____ cm (지면에서 높이)

예상값:
- x: 0.0 ~ 0.1 (차량 중앙 또는 약간 앞)
- y: 0.0 (중앙)
- z: 0.08 ~ 0.15 (8~15cm 높이)
```

#### 전방 카메라 (Logitech C920)
```
측정 위치: 카메라 렌즈 중심

x: _____ cm (차량 앞쪽에 설치, 양수)
y: _____ cm (중앙이면 0)
z: _____ cm (차선이 잘 보이는 높이)

카메라 각도:
- pitch (위아래): _____ 도 (아래를 보면 -, 위를 보면 +)
  → 라디안: pitch_deg × 0.01745

예상값:
- x: 0.15 ~ 0.20 (15~20cm 앞)
- y: 0.0
- z: 0.12 ~ 0.18 (12~18cm 높이)
- pitch: -5 ~ -10도 (도로를 향해 약간 아래)
```

#### 후방 카메라 (Logitech C920)
```
측정 위치: 카메라 렌즈 중심

x: _____ cm (차량 뒤쪽, 음수)
y: _____ cm (중앙이면 0)
z: _____ cm (주차 라인이 잘 보이는 높이)

카메라 각도:
- yaw: 180도 (뒤를 봄) = 3.14159 라디안
- pitch: _____ 도 (바닥을 보면 -)

예상값:
- x: -0.12 ~ -0.18 (-12~-18cm 뒤)
- y: 0.0
- z: 0.08 ~ 0.12 (8~12cm 높이)
- pitch: -8 ~ -15도 (주차 라인 보기)
```

---

## 측정 후 launch 파일 업데이트

### track_launch.py

```python
# base_link -> laser
static_tf_laser = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='static_tf_laser',
    arguments=['x', 'y', 'z', 'yaw', 'pitch', 'roll', 'base_link', 'laser']
    # 예: ['0.05', '0', '0.12', '0', '0', '0', 'base_link', 'laser']
)

# base_link -> camera_front
static_tf_camera_front = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='static_tf_camera_front',
    arguments=['x', 'y', 'z', 'yaw', 'pitch', 'roll', 'base_link', 'camera_front']
    # 예: ['0.18', '0', '0.15', '0', '-0.1', '0', 'base_link', 'camera_front']
)
```

### mission_launch.py

```python
# base_link -> camera_rear 추가
static_tf_camera_rear = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='static_tf_camera_rear',
    arguments=['x', 'y', 'z', 'yaw', 'pitch', 'roll', 'base_link', 'camera_rear']
    # 예: ['-0.16', '0', '0.10', '3.14159', '-0.15', '0', 'base_link', 'camera_rear']
)
```

---

## 검증 방법

### 1. TF 확인
```bash
# TF 트리 확인
ros2 run tf2_tools view_frames

# PDF 파일 생성됨: frames.pdf
# base_link -> laser, camera_front, camera_rear 연결 확인
```

### 2. RViz2에서 시각화
```bash
rviz2

# 설정:
1. Fixed Frame: base_link
2. Add → LaserScan (/scan)
3. Add → Image (/camera/front/image)
4. Add → Image (/camera/rear/image)
5. Add → TF (모든 frame 표시)

# 확인 사항:
- LiDAR 데이터가 차량 기준으로 올바른 위치에 표시되는가?
- 카메라 frame이 차량의 앞/뒤에 올바르게 표시되는가?
```

### 3. 각도 검증 (카메라)
```bash
# 전방 카메라: 차선이 이미지 하단 1/3에 나타나야 함
ros2 run rqt_image_view rqt_image_view /camera/front/image

# 후방 카메라: 주차 라인이 이미지 하단 절반에 나타나야 함
ros2 run rqt_image_view rqt_image_view /camera/rear/image
```

---

## 측정 시트

| 센서 | x (cm) | y (cm) | z (cm) | yaw (도) | pitch (도) | roll (도) |
|------|--------|--------|--------|----------|-----------|-----------|
| LiDAR | | | | 0 | 0 | 0 |
| 전방 카메라 | | | | 0 | | 0 |
| 후방 카메라 | | | | 180 | | 0 |

**측정 날짜:** _______________

**측정자:** _______________

**비고:**
