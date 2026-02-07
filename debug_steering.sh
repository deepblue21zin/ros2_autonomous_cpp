#!/bin/bash
# 조향각 실시간 모니터링 스크립트

echo "============================================"
echo "  YOLO Lane Drive 조향각 디버깅"
echo "============================================"
echo ""

echo "📊 1. yolo_lane_drive 로그 확인 (실시간)"
echo "   로그에서 확인할 것:"
echo "   - DRIVE: offset=XXX, steer=XXX, speed=XXX"
echo "   - STOP: 메시지 (마스크, 초음파, 신호등 등)"
echo ""
echo "   실행: ros2 node info /yolo_lane_drive"
echo "   또는: ros2 topic echo /decision/cmd"
echo ""

echo "📊 2. 조향 명령 실시간 모니터링"
echo "   steering_angle 값이 얼마나 큰지 확인"
echo ""
ros2 topic echo /decision/cmd --field steering_angle &
PID1=$!

echo ""
echo "📊 3. Arduino 명령 실시간 모니터링"
echo "   V:XXX (속도), S:XXX (서보 각도) 확인"
echo "   서보 범위: 60~120 (중앙 90)"
echo ""

# 2초 대기 후 종료
sleep 2
kill $PID1 2>/dev/null

echo ""
echo "============================================"
echo "  완전 모니터링 (Ctrl+C로 종료)"
echo "============================================"
echo ""
echo "다음 명령으로 실시간 모니터링:"
echo ""
echo "# 터미널 1: 조향각 모니터링"
echo "ros2 topic echo /decision/cmd"
echo ""
echo "# 터미널 2: yolo_lane_drive 로그"
echo "ros2 node list | grep yolo_lane_drive"
echo ""
echo "# 터미널 3: YOLO 마스크 확인"
echo "ros2 topic hz /perception/drivable_mask"
