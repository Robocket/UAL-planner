"""Select 3D points inside camera-detector image polygons.

The detector reports boxes in original-image pixels.  LiDAR points keep their
source coordinates; only their projected pixel positions are tested here.
"""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class DetectionBox:
    track_id: int
    class_name: str
    confidence: float
    polygon: np.ndarray  # (N, 2) original-image pixels
    oriented: bool = False
    class_id: int = 0


def _points(points):
    points = np.asarray(points, dtype=np.float64)
    if points.shape == (0,):
        return points.reshape((0, 3))
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('points must have shape (N, 3)')
    return points


def _polygon(polygon):
    polygon = np.asarray(polygon, dtype=np.float64)
    if polygon.ndim != 2 or polygon.shape[1] != 2 or len(polygon) < 3:
        raise ValueError('detection polygon must have at least three (x, y) points')
    if not np.isfinite(polygon).all():
        raise ValueError('detection polygon must be finite')
    # A repeated closing vertex is common in drawing code and harmless.
    if len(polygon) > 3 and np.allclose(polygon[0], polygon[-1]):
        polygon = polygon[:-1]
    x, y = polygon[:, 0], polygon[:, 1]
    area = 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))
    if not np.isfinite(area) or area <= 0.0:
        raise ValueError('detection polygon must have positive area')
    return polygon, float(area)


def _segment_distance2(x, y, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    denominator = dx * dx + dy * dy
    if denominator <= 0.0:
        return (x - x1) ** 2 + (y - y1) ** 2
    t = np.clip(((x - x1) * dx + (y - y1) * dy) / denominator, 0.0, 1.0)
    return (x - (x1 + t * dx)) ** 2 + (y - (y1 + t * dy)) ** 2


def points_in_polygon(pixels, polygon, boundary_epsilon=1e-6):
    """Vectorized even-odd point-in-polygon test with inclusive edges.

    pixels is (N, 2) in continuous image coordinates.  NaN/inf points are never
    selected.  A point whose distance to an edge is within boundary_epsilon
    pixels is treated as inside, so a projected point on the reported boundary
    is not lost to floating-point roundoff.
    """
    pixels = np.asarray(pixels, dtype=np.float64)
    if pixels.ndim != 2 or pixels.shape[1] != 2:
        raise ValueError('pixels must have shape (N, 2)')
    polygon, _ = _polygon(polygon)
    epsilon = float(boundary_epsilon)
    if not np.isfinite(epsilon) or epsilon < 0.0:
        raise ValueError('boundary_epsilon must be finite and non-negative')
    x, y = pixels[:, 0], pixels[:, 1]
    finite = np.isfinite(x) & np.isfinite(y)
    inside = np.zeros(len(pixels), dtype=bool)
    boundary = np.zeros(len(pixels), dtype=bool)
    for index in range(len(polygon)):
        x1, y1 = polygon[index]
        x2, y2 = polygon[(index + 1) % len(polygon)]
        boundary |= _segment_distance2(x, y, x1, y1, x2, y2) <= epsilon * epsilon
        crosses = (y1 > y) != (y2 > y)
        if y1 == y2:
            continue
        crossing_x = x1 + (x2 - x1) * (y - y1) / (y2 - y1)
        inside ^= crosses & (x < crossing_x)
    return finite & (inside | boundary)


def select_detection_groups(
    points, pixels, detections, selection='largest', target_id=0,
    target_class='', boundary_epsilon=1e-6,
):
    """Return {track_id: original 3D points} for eligible detections.

    Largest mirrors mask selection: it chooses the largest image polygon that
    still contains at least one valid projected point.  All keeps every eligible
    non-empty polygon.  Boxes are independent; overlapping image areas may
    therefore carry the same 3D point in more than one group.
    """
    points = _points(points)
    pixels = np.asarray(pixels, dtype=np.float64)
    if pixels.shape != (len(points), 2):
        raise ValueError('pixel coordinates must correspond one-to-one with points')
    if selection not in ('largest', 'all'):
        raise ValueError('instance_selection must be largest or all')
    if int(target_id) < 0:
        raise ValueError('target_instance_id must be >= 0')
    target_id = int(target_id)
    target_class = str(target_class or '')

    eligible = []
    seen = set()
    for detection in detections:
        track_id = int(detection.track_id)
        if track_id <= 0:
            raise ValueError('detection track IDs must be positive')
        if track_id in seen:
            raise ValueError('detection metadata contains duplicate IDs')
        seen.add(track_id)
        if target_id and track_id != target_id:
            continue
        if target_class and str(detection.class_name) != target_class:
            continue
        polygon, area = _polygon(detection.polygon)
        mask = points_in_polygon(pixels, polygon, boundary_epsilon)
        if np.any(mask):
            eligible.append((track_id, area, points[mask]))

    if selection == 'largest' and eligible:
        eligible = [max(eligible, key=lambda item: (item[1], -item[0]))]
    return {track_id: group for track_id, _, group in eligible}
