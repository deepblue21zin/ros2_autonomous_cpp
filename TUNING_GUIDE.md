# YOLO Lane Drive 정밀 조정 가이드

## 🎯 조향각 디버깅 및 튜닝

### 1단계: 현재 상태 확인

```bash
# 디버그 스크립트 실행
chmod +x debug_steering.sh
./debug_steering.sh

# 또는 직접 모니터링
ros2 topic echo /decision/cmd
```

**확인할 로그:**
```
DRIVE: offset=+0.15, steer=+0.180rad, speed=0.30m/s
```

---

## 📊 값 분석 및 조정

### A. `offset` 값 분석

**offset**: 차선 중심이 이미지 중앙에서 얼마나 떨어져 있는지 (-1.0 ~ +1.0)

| offset 범위 | 의미 | 정상 여부 |
|------------|------|----------|
| -0.1 ~ +0.1 | 차선이 거의 중앙 | ✅ 정상 |
| ±0.1 ~ ±0.3 | 약간 벗어남 | ✅ 정상 |
| ±0.3 ~ ±0.6 | 많이 벗어남 | ⚠️ 주의 |
| ±0.6 이상 | 차선 이탈 직전 | ❌ 위험 |

**offset이 항상 0에 가까우면**: 차선이 이미 중앙 → 조향 필요 없음 (정상)

---

### B. `steer` 값 분석

**steer**: 계산된 조향각 (라디안)

| steer 범위 | 각도 | 서보 값 | 조향 강도 |
|-----------|------|---------|----------|
| 0.0 ~ 0.1 | 0~6° | 90~96 | 매우 약함 ❌ |
| 0.1 ~ 0.2 | 6~11° | 96~101 | 약함 ⚠️ |
| 0.2 ~ 0.3 | 11~17° | 101~107 | 보통 ✅ |
| 0.3 ~ 0.45 | 17~26° | 107~116 | 강함 ✅ |

**문제 진단:**
- **offset은 크고 steer가 작음** → PID 게인 증가 필요
- **steer는 크고 서보 반응 없음** → 서보 각도 범위 문제

---

## 🔧 조정 방법

### Case 1: 조향각이 너무 약함 (steer < 0.2)

**원인**: PID 게인이 너무 작음

**해결**:
```python
# yolo_bag_test_launch.py (86-87줄)
'kp': 2.5,    # 1.2 → 2.5 (조향 강도 2배)
'kd': 0.6,    # 0.3 → 0.6 (안정성 2배)
```

**효과**:
```
기존: offset=0.3 → steer = 1.2*0.3 = 0.36 rad (21°)
변경: offset=0.3 → steer = 2.5*0.3 = 0.75 rad → 0.45로 제한 (26°)
```

---

### Case 2: 조향 반응이 불안정 (진동)

**원인**:
- Kp가 너무 큼 (과도한 반응)
- Kd가 너무 작음 (댐핑 부족)

**해결**:
```python
'kp': 1.5,    # Kp 줄이기
'kd': 0.8,    # Kd 높이기 (안정화)
```

---

### Case 3: ROI 영역 조정 (더 가까운 곳 보기)

**현재 설정**:
```python
'roi_top_ratio': 0.4,     # 상단 40% (먼 곳)
'roi_bottom_ratio': 0.9,  # 하단 90% (가까운 곳)
```

**가까운 곳 보기** (더 민감한 반응):
```python
'roi_top_ratio': 0.6,     # 0.4 → 0.6 (더 가까이)
'roi_bottom_ratio': 0.95, # 0.9 → 0.95
```

**먼 곳 보기** (더 부드러운 주행):
```python
'roi_top_ratio': 0.3,     # 0.4 → 0.3 (더 멀리)
'roi_bottom_ratio': 0.85, # 0.9 → 0.85
```

---

### Case 4: 최대 조향각 증가

**Launch 파일**:
```python
# yolo_bag_test_launch.py (51줄)
default_value='0.7',  # 0.45 → 0.7 (40도)
```

**arduino.yaml**:
```yaml
max_steer_deg: 50.0  # 45 → 50 (서보 범위: 40~140도)
```

---

## 🧪 튜닝 절차

### 1️⃣ 기본 확인
```bash
# 로그 모니터링
ros2 topic echo /decision/cmd

# 확인 사항:
# - offset 값이 나오는가? → 마스크 정상
# - steer 값이 나오는가? → 계산 정상
# - speed 값이 0이 아닌가? → 모터 명령 정상
```

### 2️⃣ PID 게인 증가 (추천 순서)
```python
# Step 1: Kp만 증가
'kp': 2.0,  # 1.2 → 2.0
'kd': 0.3,  # 유지

# 테스트 후 여전히 약하면
# Step 2: Kp 더 증가 + Kd 증가
'kp': 2.5,
'kd': 0.6,

# 테스트 후 진동이 생기면
# Step 3: Kp 줄이고 Kd 늘리기
'kp': 2.0,
'kd': 0.8,
```

### 3️⃣ ROI 조정 (필요 시)
```python
# 더 민감하게 (가까운 곳 보기)
'roi_top_ratio': 0.6,
'roi_bottom_ratio': 0.95,
```

### 4️⃣ 최대 조향각 증가 (마지막 수단)
```python
# Launch 파일
default_value='0.7',  # 40도

# arduino.yaml
max_steer_deg: 50.0
```

---

## 🎬 실시간 튜닝 팁

### 로그 읽는 법
```
[yolo_lane_drive] DRIVE: offset=+0.25, steer=+0.300rad, speed=0.30m/s
                          ↑           ↑              ↑
                       차선 위치   조향 명령      속도 명령
```

### 정상 동작 예시
```
offset=+0.15, steer=+0.180rad (10도), speed=0.30m/s
offset=+0.08, steer=+0.096rad (5도),  speed=0.30m/s
offset=-0.05, steer=-0.060rad (-3도), speed=0.30m/s
```

### 비정상 동작 예시
```
❌ STOP: no drivable mask
   → YOLO 마스크가 안 들어옴, /perception/drivable_mask 확인

❌ STOP: no drivable area in mask
   → 마스크가 비어있음, YOLO 모델 또는 이미지 확인

❌ offset=+0.40, steer=+0.080rad (5도), speed=0.30m/s
   → offset은 크지만 steer가 작음 → Kp 증가 필요!
```

---

## 📈 추천 설정값

### 일반 주행 (안정적)
```python
'cruise_speed_mps': 0.3,
'max_steer_rad': 0.45,
'kp': 2.0,
'kd': 0.5,
'roi_top_ratio': 0.4,
'roi_bottom_ratio': 0.9,
```

### 공격적 주행 (빠른 반응)
```python
'cruise_speed_mps': 0.35,
'max_steer_rad': 0.7,
'kp': 3.0,
'kd': 0.8,
'roi_top_ratio': 0.6,
'roi_bottom_ratio': 0.95,
```

### 느린 주행 (안전)
```python
'cruise_speed_mps': 0.2,
'max_steer_rad': 0.35,
'kp': 1.5,
'kd': 0.4,
'roi_top_ratio': 0.3,
'roi_bottom_ratio': 0.85,
```

---

## ⚠️ 주의사항

1. **한 번에 하나씩 조정**: 여러 값을 동시에 바꾸면 어떤 게 효과가 있는지 모름
2. **점진적으로 증가**: Kp를 1.2 → 1.5 → 2.0 → 2.5 순서로
3. **진동 발생 시**: Kd를 높이거나 Kp를 낮추기
4. **로그 확인 필수**: offset/steer 값을 보고 조정
5. **서보 물리적 한계**: 실제 서보가 몇 도까지 돌 수 있는지 확인

---

## 🔍 문제 해결

### Q1: offset은 크지만 steer가 작음
**A**: Kp를 2배 이상 증가 (1.2 → 2.5)

### Q2: 조향이 진동함
**A**: Kd를 증가하고 Kp를 약간 감소 (Kp: 2.0, Kd: 0.8)

### Q3: 서보가 전혀 안 움직임
**A**:
1. Arduino 연결 확인 (`ros2 topic echo /arduino/cmd`)
2. 서보 물리적 연결 확인
3. max_steer_deg를 50으로 증가

### Q4: 차선이 중앙인데도 계속 조향함
**A**: offset이 0에 가까운지 확인, Kp가 너무 크면 줄이기

---

## 📝 변경 후 테스트 명령

```bash
# 1. Launch 파일 수정 후
cd /root/ros2_ws
colcon build --packages-select bringup

# 2. 실행
ros2 launch bringup yolo_bag_test_launch.py

# 3. 모니터링 (새 터미널)
ros2 topic echo /decision/cmd
```
