"""YOLOv8n-seg 기반 통합 감지기.

목적:
  1. 신호등 감지: 빨간불 -> 정지, 초록불 -> 주행
  2. 장애물 감지: obstacle 클래스
  3. 차선 인식 보조: 세그멘테이션 마스크로 주행 가능 영역 추출

모델: best.pt (커스텀 4 classes, segmentation)
클래스:
  - green_light (0): 초록 신호등
  - obstacle (1): 장애물
  - red_light (2): 빨간 신호등
  - road_objects2 (3): 도로 객체
"""

from __future__ import annotations

import os
import threading
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from .detector import Detection

try:
    from ultralytics import YOLO
except ImportError as exc:
    raise RuntimeError(
        "ultralytics 패키지가 필요합니다: pip install ultralytics"
    ) from exc


# -- 커스텀 클래스 ID (best.pt) --
GREEN_LIGHT_ID = 0
OBSTACLE_ID = 1
RED_LIGHT_ID = 2
ROAD_OBJECTS_ID = 3

# 신호등 클래스
TRAFFIC_LIGHT_IDS = {GREEN_LIGHT_ID, RED_LIGHT_ID}

# 장애물로 취급할 클래스
OBSTACLE_IDS = {OBSTACLE_ID}

# 클래스 ID -> 신호등 상태 매핑
LIGHT_STATE_MAP = {
    GREEN_LIGHT_ID: "green",
    RED_LIGHT_ID: "red",
}


@dataclass(frozen=True)
class TrafficLightState:
    """신호등 판별 결과."""
    state: str          # "red", "green", "unknown"
    confidence: float   # YOLO 감지 신뢰도
    bbox: Tuple[int, int, int, int]  # x1, y1, x2, y2


@dataclass(frozen=True)
class ObstacleInfo:
    """장애물 정보."""
    label: str
    confidence: float
    bbox: Tuple[int, int, int, int]
    mask: Optional[np.ndarray] = field(default=None, repr=False)


@dataclass
class Yolov8nSegConfig:
    """YOLOv8n-seg 설정."""
    model_path: str = ""
    conf_threshold: float = 0.4
    iou_threshold: float = 0.45
    imgsz: int = 640
    device: Optional[str] = None
    # 신호등 최소 면적 비율 (현재 미사용: API 호환용)
    traffic_light_min_ratio: float = 0.0
    # 세그멘테이션 마스크 사용 여부
    use_seg_mask: bool = True
    # 장애물 감지 사용 여부
    detect_obstacles: bool = True


class Yolov8nSegDetector:
    """YOLOv8n-seg 통합 감지기 (best.pt 커스텀 모델).

    기능:
      1. 신호등 감지: green_light(0), red_light(2) 직접 감지
      2. 장애물 감지: obstacle(1) 클래스
      3. 세그멘테이션 마스크 -> 주행 가능 영역 추출 (차선 보조)
    """

    _model_cache: Dict[str, YOLO] = {}
    _model_lock = threading.Lock()

    def __init__(self, config: Yolov8nSegConfig) -> None:
        self.config = config

        if not config.model_path or not os.path.exists(config.model_path):
            raise RuntimeError(
                f"모델 파일을 찾을 수 없습니다: {config.model_path}"
            )

        with Yolov8nSegDetector._model_lock:
            if config.model_path not in Yolov8nSegDetector._model_cache:
                model = YOLO(config.model_path)
                if config.device:
                    model.to(config.device)
                Yolov8nSegDetector._model_cache[config.model_path] = model
            self.model = Yolov8nSegDetector._model_cache[config.model_path]

        # 모델 클래스 이름
        names = getattr(self.model, "names", {})
        if isinstance(names, dict):
            self.names = {int(k): str(v) for k, v in names.items()}
        elif isinstance(names, (list, tuple)):
            self.names = {i: str(n) for i, n in enumerate(names)}
        else:
            self.names = {}

    # -- 메인 추론 --

    def predict(self, frame: np.ndarray):
        """YOLO 추론 실행. 결과 객체 반환."""
        return self.model.predict(
            source=frame,
            imgsz=self.config.imgsz,
            conf=self.config.conf_threshold,
            iou=self.config.iou_threshold,
            device=self.config.device,
            verbose=False,
        )

    # -- 신호등 감지 --

    def detect_traffic_light(self, frame: np.ndarray,
                              results=None) -> Optional[TrafficLightState]:
        """프레임에서 신호등을 감지.

        best.pt 모델이 직접 green_light/red_light를 분류하므로
        HSV 색상 분석이 필요 없음.

        Returns:
            TrafficLightState 또는 None (미감지)
        """
        if results is None:
            results = self.predict(frame)

        best: Optional[TrafficLightState] = None

        for result in results:
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue

            for i, box in enumerate(boxes):
                cls_id = int(box.cls.item())
                if cls_id not in TRAFFIC_LIGHT_IDS:
                    continue

                score = float(box.conf.item())
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]

                state = LIGHT_STATE_MAP.get(cls_id, "unknown")

                tl = TrafficLightState(
                    state=state,
                    confidence=score,
                    bbox=(x1, y1, x2, y2),
                )

                # 가장 큰 (가까운) 신호등 선택
                if best is None or (x2 - x1) * (y2 - y1) > \
                        (best.bbox[2] - best.bbox[0]) * (best.bbox[3] - best.bbox[1]):
                    best = tl

        return best

    # -- 장애물 감지 --

    def detect_obstacles(self, frame: np.ndarray,
                          results=None) -> List[ObstacleInfo]:
        """obstacle 클래스 감지.

        Returns:
            ObstacleInfo 리스트
        """
        if not self.config.detect_obstacles:
            return []

        if results is None:
            results = self.predict(frame)

        obstacles: List[ObstacleInfo] = []

        for result in results:
            boxes = getattr(result, "boxes", None)
            masks = getattr(result, "masks", None)
            if boxes is None:
                continue

            for i, box in enumerate(boxes):
                cls_id = int(box.cls.item())
                if cls_id not in OBSTACLE_IDS:
                    continue

                score = float(box.conf.item())
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                label = self.names.get(cls_id, str(cls_id))

                mask = None
                if self.config.use_seg_mask and masks is not None:
                    mask_data = masks.data
                    if mask_data is not None and i < len(mask_data):
                        m = mask_data[i].cpu().numpy()
                        mask = cv2.resize(
                            m, (frame.shape[1], frame.shape[0]),
                            interpolation=cv2.INTER_NEAREST
                        ).astype(np.uint8)

                obstacles.append(ObstacleInfo(
                    label=label,
                    confidence=score,
                    bbox=(x1, y1, x2, y2),
                    mask=mask,
                ))

        return obstacles

    # -- 세그멘테이션 마스크 기반 주행 영역 추출 (차선 보조) --

    def get_drivable_mask(self, frame: np.ndarray,
                           results=None) -> np.ndarray:
        """세그멘테이션 마스크를 이용해 주행 불가 영역을 표시.

        장애물 영역을 마스킹하여, OpenCV 차선 인식에서
        해당 영역을 제외할 수 있도록 한다.

        Returns:
            주행 가능 영역 마스크 (255=주행 가능, 0=장애물)
        """
        h, w = frame.shape[:2]
        drivable = np.full((h, w), 255, dtype=np.uint8)

        if not self.config.use_seg_mask:
            return drivable

        if results is None:
            results = self.predict(frame)

        for result in results:
            boxes = getattr(result, "boxes", None)
            masks = getattr(result, "masks", None)
            if boxes is None or masks is None:
                continue

            mask_data = masks.data
            if mask_data is None:
                continue

            for i, box in enumerate(boxes):
                cls_id = int(box.cls.item())
                # 장애물 클래스만 마스킹
                if cls_id not in OBSTACLE_IDS:
                    continue
                if i >= len(mask_data):
                    continue

                m = mask_data[i].cpu().numpy()
                m_resized = cv2.resize(
                    m, (w, h), interpolation=cv2.INTER_NEAREST
                )
                # 장애물 영역을 0으로
                drivable[m_resized > 0.5] = 0

        return drivable

    # -- 통합 감지 (Detection 호환) --

    def detect(self, frame: np.ndarray) -> List[Detection]:
        """기존 Detection 형식 호환 출력.

        신호등 + 장애물을 모두 Detection 리스트로 반환.
        """
        results = self.predict(frame)
        detections: List[Detection] = []

        for result in results:
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue

            for box in boxes:
                cls_id = int(box.cls.item())
                score = float(box.conf.item())
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]

                label = self.names.get(cls_id, str(cls_id))

                detections.append(Detection(
                    label=label,
                    score=score,
                    bbox=(x1, y1, x2, y2),
                ))

        return detections

    # -- 디버그 오버레이 --

    def draw_overlay(self, frame: np.ndarray, results=None) -> np.ndarray:
        """감지 결과를 프레임에 오버레이 (디버그용).

        Returns:
            오버레이된 프레임 복사본
        """
        overlay = frame.copy()

        if results is None:
            results = self.predict(frame)

        for result in results:
            boxes = getattr(result, "boxes", None)
            masks = getattr(result, "masks", None)
            if boxes is None:
                continue

            for i, box in enumerate(boxes):
                cls_id = int(box.cls.item())
                score = float(box.conf.item())
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                label = self.names.get(cls_id, str(cls_id))

                # 색상 결정
                if cls_id == GREEN_LIGHT_ID:
                    box_color = (0, 255, 0)  # 초록
                    label = f"GREEN ({score:.0%})"
                elif cls_id == RED_LIGHT_ID:
                    box_color = (0, 0, 255)  # 빨강
                    label = f"RED ({score:.0%})"
                elif cls_id == OBSTACLE_ID:
                    box_color = (0, 165, 255)  # 주황
                    label = f"obstacle ({score:.0%})"
                elif cls_id == ROAD_OBJECTS_ID:
                    box_color = (255, 200, 0)  # 파랑-노랑
                    label = f"road_obj ({score:.0%})"
                else:
                    box_color = (200, 200, 200)

                # 바운딩 박스
                cv2.rectangle(overlay, (x1, y1), (x2, y2), box_color, 2)
                cv2.putText(overlay, label,
                            (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, box_color, 1, cv2.LINE_AA)

                # 세그멘테이션 마스크 오버레이
                if masks is not None and i < len(masks.data):
                    m = masks.data[i].cpu().numpy()
                    m_resized = cv2.resize(
                        m, (overlay.shape[1], overlay.shape[0]),
                        interpolation=cv2.INTER_NEAREST
                    )
                    mask_bool = m_resized > 0.5
                    overlay[mask_bool] = (
                        overlay[mask_bool] * 0.6 +
                        np.array(box_color) * 0.4
                    ).astype(np.uint8)

        return overlay
