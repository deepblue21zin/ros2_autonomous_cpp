"""전처리 모듈: ROI 추출, 색상/밝기 보정, 엣지 생성, BEV 변환."""
from typing import Tuple
import cv2
import numpy as np


def warp_bev(roi_color: np.ndarray,
             src_tl_x: float = 0.30,
             src_tr_x: float = 0.70,
             src_bl_x: float = 0.0,
             src_br_x: float = 1.0,
             draw_trapezoid: bool = False) -> np.ndarray:
    """ROI를 Bird's Eye View(탑뷰)로 변환.

    명시적 4포인트 지정으로 카메라 기울기에 맞춰 캘리브레이션 가능.
    소스 사다리꼴의 x좌표를 ROI 폭 비율(0.0~1.0)로 지정.
    y좌표는 자동: 상단 2점 = ROI 꼭대기, 하단 2점 = ROI 바닥.

    Args:
        roi_color: ROI BGR 이미지.
        src_tl_x: 소스 사다리꼴 상단-좌 x비율 (0.0~1.0).
        src_tr_x: 소스 사다리꼴 상단-우 x비율 (0.0~1.0).
        src_bl_x: 소스 사다리꼴 하단-좌 x비율 (0.0~1.0).
        src_br_x: 소스 사다리꼴 하단-우 x비율 (0.0~1.0).
        draw_trapezoid: True시 변환 전 원본에 사다리꼴 그리기 (캘리브레이션용).
    Returns:
        warped: BEV 변환된 이미지.
    """
    h, w = roi_color.shape[:2]

    src = np.float32([
        [w * src_tl_x, 0],          # top-left
        [w * src_tr_x, 0],          # top-right
        [w * src_br_x, h - 1],      # bottom-right
        [w * src_bl_x, h - 1],      # bottom-left
    ])
    dst = np.float32([
        [0, 0],
        [w - 1, 0],
        [w - 1, h - 1],
        [0, h - 1],
    ])

    if draw_trapezoid:
        pts = src.astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(roi_color, [pts], True, (0, 255, 255), 2)

    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(roi_color, M, (w, h))
    return warped


def preprocess_image(frame: np.ndarray, roi_y_ratio: float,
                     canny_low: int, canny_high: int,
                     gaussian_kernel: int,
                     use_bev: bool = False,
                     bev_src_tl_x: float = 0.30,
                     bev_src_tr_x: float = 0.70,
                     bev_src_bl_x: float = 0.0,
                     bev_src_br_x: float = 1.0,
                     bev_draw_trapezoid: bool = False,
                     ) -> Tuple[np.ndarray, np.ndarray, int]:
    """이미지를 전처리하여 ROI와 엣지 맵을 반환.

    Args:
        frame: BGR 원본 이미지.
        roi_y_ratio: ROI 시작 비율 (예: 0.55).
        canny_low: Canny 하한값.
        canny_high: Canny 상한값.
        gaussian_kernel: 가우시안 블러 커널 크기(홀수).
        use_bev: BEV 변환 사용 여부.
        bev_src_tl_x: BEV 소스 상단-좌 x비율.
        bev_src_tr_x: BEV 소스 상단-우 x비율.
        bev_src_bl_x: BEV 소스 하단-좌 x비율.
        bev_src_br_x: BEV 소스 하단-우 x비율.
        bev_draw_trapezoid: BEV 캘리브레이션 사다리꼴 표시.
    Returns:
        edges: 엣지 바이너리 이미지.
        roi_color: ROI 색상 이미지(BGR). BEV 적용 시 변환된 이미지.
        y_start: ROI가 시작되는 세로 픽셀 위치.
    """
    h, _ = frame.shape[:2]
    y_start = int(h * roi_y_ratio)
    roi_color = frame[y_start:, :]

    # BEV 변환 (활성화 시)
    if use_bev:
        roi_color = warp_bev(roi_color,
                             src_tl_x=bev_src_tl_x,
                             src_tr_x=bev_src_tr_x,
                             src_bl_x=bev_src_bl_x,
                             src_br_x=bev_src_br_x,
                             draw_trapezoid=bev_draw_trapezoid)

    hsv = cv2.cvtColor(roi_color, cv2.COLOR_BGR2HSV)
    # 흰색과 노란색 마스크
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 40, 255])
    lower_yellow = np.array([15, 80, 80])
    upper_yellow = np.array([35, 255, 255])
    mask_white = cv2.inRange(hsv, lower_white, upper_white)
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask = cv2.bitwise_or(mask_white, mask_yellow)

    masked = cv2.bitwise_and(roi_color, roi_color, mask=mask)
    gray = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    gray = clahe.apply(gray)

    blur = cv2.GaussianBlur(gray, (gaussian_kernel, gaussian_kernel), 0)
    edges = cv2.Canny(blur, canny_low, canny_high)
    return edges, roi_color, y_start
