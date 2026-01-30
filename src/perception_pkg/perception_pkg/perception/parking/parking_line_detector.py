"""
후방 카메라 주차 라인 감지 모듈.
대회 규정: 수직 주차 후 2초 이상 정차 후 빠져나오기
"""
from typing import Tuple
import cv2
import numpy as np


def detect_parking_end_line(
    image: np.ndarray,
    roi_y_start_ratio: float = 0.6,
    canny_low: int = 50,
    canny_high: int = 150,
    hough_threshold: int = 50,
    min_line_length: int = 100,
    debug: bool = False
) -> Tuple[bool, np.ndarray]:
    """
    후방 카메라에서 주차 종료 라인(흰색 수평선) 감지.

    Args:
        image: BGR 이미지
        roi_y_start_ratio: ROI 시작 y 비율 (0.6 = 하단 40%만 사용)
        canny_low: Canny 엣지 검출 low threshold
        canny_high: Canny 엣지 검출 high threshold
        hough_threshold: HoughLinesP threshold
        min_line_length: 최소 직선 길이
        debug: 디버그 오버레이 생성 여부

    Returns:
        (line_detected, overlay): 라인 감지 여부, 디버그 오버레이
    """
    h, w = image.shape[:2]

    # ROI 추출 (하단 40%)
    roi_y_start = int(h * roi_y_start_ratio)
    roi = image[roi_y_start:h, 0:w]

    # 그레이스케일 변환
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    # 가우시안 블러
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # Canny 엣지 검출
    edges = cv2.Canny(blur, canny_low, canny_high)

    # HoughLinesP로 직선 검출
    lines = cv2.HoughLinesP(
        edges, 1, np.pi / 180,
        threshold=hough_threshold,
        minLineLength=min_line_length,
        maxLineGap=50
    )

    # 오버레이 생성
    overlay = image.copy() if debug else image

    line_detected = False
    horizontal_lines = []

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]

            # 기울기 계산
            if x2 == x1:
                continue
            slope = abs((y2 - y1) / (x2 - x1))

            # 거의 수평인 선만 선택 (기울기 < 0.3)
            if slope < 0.3:
                # 전체 이미지 좌표로 변환
                y1_global = y1 + roi_y_start
                y2_global = y2 + roi_y_start

                horizontal_lines.append((x1, y1_global, x2, y2_global))

                if debug:
                    cv2.line(overlay, (x1, y1_global), (x2, y2_global),
                             (0, 255, 0), 3)

    # 수평선이 충분히 길고 여러 개 감지되면 주차 라인으로 판정
    if len(horizontal_lines) >= 2:
        line_detected = True

        if debug:
            # 감지 메시지 표시
            cv2.putText(overlay, "PARKING LINE DETECTED", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    if debug:
        # ROI 영역 표시
        cv2.rectangle(overlay, (0, roi_y_start), (w, h), (255, 0, 0), 2)

    return line_detected, overlay
