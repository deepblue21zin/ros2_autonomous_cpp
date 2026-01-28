"""객체 검출 공용 유틸리티."""

from __future__ import annotations

import os
import threading
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

try:  # ROS 로그 사용 여부
    import rospy
except ImportError:  # pragma: no cover - 테스트 환경 대비
    rospy = None

try:
    from ultralytics import YOLO
except Exception:  # pragma: no cover - 선택적 의존성
    YOLO = None

# 라벨 네이밍 컨벤션: speed_sign_30, traffic_light_red 등


@dataclass(frozen=True)
class Detection:
    """객체 검출 결과."""

    label: str
    score: float
    bbox: Tuple[int, int, int, int]  # x_min, y_min, x_max, y_max


def _logwarn(message: str) -> None:
    if rospy is not None:
        rospy.logwarn(message)
    else:  # pragma: no cover
        print(f"[ObjectDetector] {message}")


class ObjectDetector:
    """YOLO 기반 객체 검출 추상화.

    model_path 인자가 주어지면 ultralytics YOLO로 추론하며, 미제공 시
    detect_objects 스텁을 호출한다. 하나의 모델을 여러 노드에서 공유하기 위해
    클래스 단위 캐시를 사용한다.
    """

    _model_cache: Dict[str, YOLO] = {}
    _model_lock = threading.Lock()

    def __init__(
        self,
        score_threshold: float = 0.3,
        model_path: Optional[str] = None,
        class_map: Optional[Dict[str, str]] = None,
        imgsz: int = 640,
        device: Optional[str] = None,
    ) -> None:
        self.score_threshold = float(score_threshold)
        self.model_path = model_path
        self.class_map = {str(k): str(v) for k, v in (class_map or {}).items()}
        self.imgsz = int(imgsz)
        self.device = device

        self.model: Optional[YOLO] = None
        self.names: Dict[int, str] = {}

        if self.model_path:
            if YOLO is None:
                raise RuntimeError(
                    "ultralytics 패키지가 설치되어 있지 않아 YOLO 모델을 로드할 수 없습니다."
                )
            if not os.path.exists(self.model_path):
                raise RuntimeError(f"YOLO 모델 파일을 찾을 수 없습니다: {self.model_path}")

            with ObjectDetector._model_lock:
                if self.model_path not in ObjectDetector._model_cache:
                    model = YOLO(self.model_path)
                    ObjectDetector._model_cache[self.model_path] = model
                self.model = ObjectDetector._model_cache[self.model_path]
                # getattr 보호: 구버전 호환
                model_names = getattr(self.model, "names", None)
                if isinstance(model_names, dict):
                    self.names = {int(k): str(v) for k, v in model_names.items()}
                elif isinstance(model_names, (list, tuple)):
                    self.names = {idx: str(name) for idx, name in enumerate(model_names)}
                else:
                    self.names = {}

    def detect(self, frame: np.ndarray) -> List[Detection]:
        raw: List[Tuple[str, float, Tuple[int, int, int, int]]] = []

        if self.model is None:
            raw = detect_objects(frame)
        else:
            try:
                results = self.model.predict(
                    source=frame,
                    imgsz=self.imgsz,
                    conf=self.score_threshold,
                    device=self.device,
                    verbose=False,
                )
            except Exception as exc:  # pragma: no cover - 예외 메시지 전달
                _logwarn(f"YOLO 추론 실패: {exc}")
                results = []

            for result in results:
                boxes = getattr(result, "boxes", None)
                if boxes is None:
                    continue
                for box in boxes:
                    conf_tensor = getattr(box, "conf", None)
                    cls_tensor = getattr(box, "cls", None)
                    xyxy_tensor = getattr(box, "xyxy", None)
                    if conf_tensor is None or cls_tensor is None or xyxy_tensor is None:
                        continue

                    score = float(conf_tensor.item())
                    if score < self.score_threshold:
                        continue

                    cls_idx = int(cls_tensor.item())
                    label_raw = self.names.get(cls_idx, str(cls_idx))
                    label = self.class_map.get(label_raw, label_raw)

                    x1, y1, x2, y2 = [int(round(v)) for v in xyxy_tensor[0].tolist()]
                    raw.append((label, score, (x1, y1, x2, y2)))

        detections = [
            Detection(label=label, score=score, bbox=bbox)
            for (label, score, bbox) in raw
            if score >= self.score_threshold
        ]
        return detections


def detect_objects(frame: np.ndarray) -> List[Tuple[str, float, Tuple[int, int, int, int]]]:
    """객체 검출 결과 반환 (라벨, 확률, 바운딩박스).

    기본 구현은 빈 리스트를 반환하며, 사용자 커스텀 모델과의 호환을 위해 남겨둔다.
    """
    return []


def filter_by_labels(
    detections: Sequence[Detection], candidates: Sequence[str]
) -> List[Detection]:
    """라벨 집합에 해당하는 검출만 반환."""
    candidate_set = set(candidates)
    return [det for det in detections if det.label in candidate_set]
