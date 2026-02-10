"""차선 중심 탐지 모듈 (RANSAC + 2차 다항식 피팅)."""
import warnings
from typing import List, Optional, Tuple
import cv2
import numpy as np


def _sample_line_points(x1: int, y1: int, x2: int, y2: int,
                        step: float = 5.0) -> np.ndarray:
    """Hough 세그먼트를 따라 일정 간격으로 포인트 샘플링.

    긴 세그먼트에서 더 많은 포인트가 생성되어 자연스러운 가중치 효과.
    """
    length = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    n_points = max(int(length / step), 2)
    xs = np.linspace(x1, x2, n_points)
    ys = np.linspace(y1, y2, n_points)
    return np.column_stack([xs, ys]).astype(np.float32)


def _fit_lane_ransac(points: np.ndarray, y_bottom: int, y_top: int,
                     degree: int = 2, ransac_iters: int = 60,
                     inlier_thresh: float = 12.0
                     ) -> Optional[Tuple[np.ndarray, np.ndarray, int]]:
    """RANSAC + 다항식 피팅으로 차선 근사.

    Args:
        points: (N, 2) 배열. 각 행 = [x, y].
        y_bottom: ROI 하단 y 좌표.
        y_top: 피팅 상단 y 좌표.
        degree: 다항식 차수 (2 = 곡선).
        ransac_iters: RANSAC 반복 횟수.
        inlier_thresh: 인라이어 판정 거리 (px).
    Returns:
        (y_points, x_points, x_bottom) 또는 None.
    """
    min_samples = degree + 1
    if points.shape[0] < min_samples * 2:
        return None

    x = points[:, 0]
    y = points[:, 1]

    best_coef = None
    best_inlier_count = 0

    for _ in range(ransac_iters):
        idx = np.random.choice(len(x), min_samples, replace=False)
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', np.RankWarning)
            try:
                coef = np.polyfit(y[idx], x[idx], degree)
            except np.linalg.LinAlgError:
                continue

        x_pred = np.polyval(coef, y)
        errors = np.abs(x - x_pred)
        inlier_count = int(np.sum(errors < inlier_thresh))

        if inlier_count > best_inlier_count:
            best_inlier_count = inlier_count
            best_coef = coef

    if best_coef is None or best_inlier_count < min_samples:
        return None

    # 인라이어만으로 다시 피팅 (정밀도 향상)
    x_pred = np.polyval(best_coef, y)
    inlier_mask = np.abs(x - x_pred) < inlier_thresh

    if np.sum(inlier_mask) < min_samples:
        return None

    try:
        coef = np.polyfit(y[inlier_mask], x[inlier_mask], degree)
    except (np.linalg.LinAlgError, np.RankWarning):
        return None

    # 곡선 포인트 생성
    y_pts = np.linspace(y_top, y_bottom, 30).astype(int)
    x_pts = np.polyval(coef, y_pts).astype(int)
    x_bottom = int(np.polyval(coef, y_bottom))

    return y_pts, x_pts, x_bottom


def detect_lane_center(edges: np.ndarray, roi_color: np.ndarray,
                        debug: bool = False,
                        use_bev: bool = False) -> Tuple[float, np.ndarray]:
    """허프 변환 + RANSAC 피팅으로 차선 중심을 추정.

    Args:
        edges: Canny 엣지 이미지.
        roi_color: ROI BGR 이미지.
        debug: True일 때 검출 선 오버레이.
        use_bev: BEV 모드 (위치 기반 좌/우 분류).
    Returns:
        center_x: 추정된 차선 중심 x좌표.
        overlay: 디버그용 BGR 이미지.
    """
    h, w = edges.shape[:2]
    mid_x = w // 2

    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=25,
                            minLineLength=10, maxLineGap=150)
    overlay = roi_color.copy()

    left_points: List[np.ndarray] = []
    right_points: List[np.ndarray] = []

    if lines is not None:
        for x1, y1, x2, y2 in lines[:, 0]:
            avg_x = (x1 + x2) / 2.0

            if use_bev:
                # BEV: 차선이 거의 수직 → 위치(x좌표)만으로 좌/우 분류
                # 수평선(abs(slope) < 0.3) 필터링, 수직선 허용
                if x2 != x1:
                    slope = float(y2 - y1) / float(x2 - x1)
                    if abs(slope) < 0.3:
                        continue

                sampled = _sample_line_points(x1, y1, x2, y2)

                if avg_x < mid_x:
                    left_points.append(sampled)
                else:
                    right_points.append(sampled)
            else:
                # 원근 시점: 기울기 + 위치 기반 좌/우 분류
                if x2 == x1:
                    continue
                slope = float(y2 - y1) / float(x2 - x1)
                if abs(slope) < 0.12:
                    continue

                sampled = _sample_line_points(x1, y1, x2, y2)

                if slope < 0 and avg_x < mid_x + w * 0.1:
                    left_points.append(sampled)
                elif slope > 0 and avg_x > mid_x - w * 0.1:
                    right_points.append(sampled)

            if debug:
                cv2.line(overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)

    y_bottom = h - 1
    y_top = int(h * 0.6)

    left_result = _fit_lane_ransac(
        np.vstack(left_points), y_bottom, y_top) if left_points else None
    right_result = _fit_lane_ransac(
        np.vstack(right_points), y_bottom, y_top) if right_points else None

    lane_positions = []

    if left_result is not None:
        ly, lx, lx_bottom = left_result
        lane_positions.append(lx_bottom)
        # 곡선으로 차선 표시 (직선 → polylines)
        pts = np.column_stack([lx, ly]).reshape(-1, 1, 2)
        cv2.polylines(overlay, [pts], False, (255, 0, 0), 3)

    if right_result is not None:
        ry, rx, rx_bottom = right_result
        lane_positions.append(rx_bottom)
        pts = np.column_stack([rx, ry]).reshape(-1, 1, 2)
        cv2.polylines(overlay, [pts], False, (0, 0, 255), 3)

    # Calculate center with single-lane fallback
    # 대회 트랙: 우측 차선 반시계 주행 → 오른쪽=외측 실선, 왼쪽=중앙 점선
    left_detected = left_result is not None
    right_detected = right_result is not None

    if left_detected and right_detected:
        center_x = float(np.mean(lane_positions))
    elif right_detected and not left_detected:
        estimated_lane_width = int(w * 0.35)
        center_x = float(lane_positions[0] - estimated_lane_width // 2)
        center_x = max(0.0, min(center_x, float(w - 1)))
    elif left_detected and not right_detected:
        estimated_lane_width = int(w * 0.35)
        center_x = float(lane_positions[0] + estimated_lane_width // 2)
        center_x = max(0.0, min(center_x, float(w - 1)))
    else:
        center_x = float(w) / 2.0

    cv2.circle(overlay, (int(center_x), y_bottom), 6, (0, 255, 255), -1)
    return center_x, overlay
