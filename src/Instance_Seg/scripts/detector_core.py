"""Pure YOLO result decoding; no ROS or model dependencies.

The ROS node uses these helpers so detection filtering can be tested without a
GPU, model file or ROS graph.  Both oriented (OBB) and axis-aligned detectors
are normalized to four image-plane pixel corners.
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class DetectedBox:
    class_id: int
    class_name: str
    confidence: float
    polygon: np.ndarray  # (4, 2) original-image pixels
    oriented: bool


def _array(value):
    """Convert a tensor or array-like value to float without keeping autograd."""
    if hasattr(value, 'detach'):
        value = value.detach()
    if hasattr(value, 'cpu'):
        value = value.cpu()
    return np.asarray(value, dtype=float)


def normalize_names(names):
    """Return an int -> class-name mapping for list or dict model metadata."""
    if isinstance(names, dict):
        items = names.items()
    elif isinstance(names, (list, tuple)):
        items = enumerate(names)
    else:
        raise ValueError('model class names must be a list or mapping')
    normalized = {}
    for key, name in items:
        try:
            index = int(key)
        except (TypeError, ValueError) as exc:
            raise ValueError('model class indexes must be integers') from exc
        if index < 0:
            raise ValueError('model class indexes must be non-negative')
        name = str(name).strip()
        if not name:
            raise ValueError('model class names must not be empty')
        normalized[index] = name
    if not normalized:
        raise ValueError('model class names must not be empty')
    return normalized


def target_class_ids(names, wanted):
    """Resolve configured class names to model indexes.

    An empty wanted list means every class produced by the model.
    """
    normalized = normalize_names(names)
    if not wanted:
        return set(normalized)
    if isinstance(wanted, str):
        wanted = [wanted]
    wanted = [str(name).strip().lower() for name in wanted if str(name).strip()]
    available = {name.lower(): index for index, name in normalized.items()}
    missing = sorted(set(wanted) - set(available))
    if missing:
        raise ValueError(
            f'missing target classes {missing}; actual names: {normalized}'
        )
    return {available[name] for name in wanted}


def polygon_area(polygon):
    polygon = np.asarray(polygon, dtype=float)
    if polygon.ndim != 2 or polygon.shape[1] != 2 or len(polygon) < 3:
        raise ValueError('polygon must have at least three (x, y) points')
    if not np.isfinite(polygon).all():
        raise ValueError('polygon must contain finite pixel coordinates')
    x, y = polygon[:, 0], polygon[:, 1]
    return float(0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


def _decode_polygon(box, oriented):
    if oriented:
        polygon = _array(box.xyxyxyxy).reshape(4, 2)
        if not np.isfinite(polygon).all():
            raise ValueError('model returned a non-finite oriented box')
        geometry = 'oriented'
    else:
        x1, y1, x2, y2 = _array(box.xyxy).reshape(4)
        if not np.isfinite([x1, y1, x2, y2]).all():
            raise ValueError('model returned a non-finite axis-aligned box')
        if x2 < x1 or y2 < y1:
            raise ValueError('model returned an inverted axis-aligned box')
        polygon = np.array(
            [[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=float
        )
        geometry = 'axis-aligned'
    if polygon_area(polygon) <= 0.0:
        raise ValueError(f'model returned a degenerate {geometry} box')
    return polygon


def decode_result(result, task, target_classes, threshold, max_detections=300):
    """Decode one Ultralytics result into filtered DetectedBox items.

    result may be a real Ultralytics result or a test double exposing names
    and either obb or boxes.  Non-target classes and scores below threshold are
    excluded; other detections are not silently padded or fabricated.
    """
    if task not in ('obb', 'detect'):
        raise ValueError("task must be 'obb' or 'detect'")
    threshold = float(threshold)
    if not math.isfinite(threshold) or not 0.0 <= threshold <= 1.0:
        raise ValueError('threshold must be finite and in [0, 1]')
    max_detections = int(max_detections)
    if max_detections <= 0:
        raise ValueError('max_detections must be positive')
    names = normalize_names(result.names)
    target_ids = target_class_ids(names, target_classes)
    boxes = getattr(result, 'obb' if task == 'obb' else 'boxes', None)
    if boxes is None:
        raise ValueError(f'model result has no {task} output')

    detections = []
    for box in boxes:
        class_id = int(_array(box.cls).reshape(-1)[0])
        confidence = float(_array(box.conf).reshape(-1)[0])
        if class_id not in names:
            raise ValueError(f'model returned unknown class index {class_id}')
        if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            raise ValueError('model returned an invalid confidence')
        if class_id not in target_ids or confidence < threshold:
            continue
        detections.append(DetectedBox(
            class_id=class_id,
            class_name=names[class_id],
            confidence=confidence,
            polygon=_decode_polygon(box, task == 'obb'),
            oriented=task == 'obb',
        ))
        if len(detections) > max_detections:
            raise ValueError(
                'model returned more detections than max_detections; '
                'the configured max_det should have prevented this'
            )
    return detections
