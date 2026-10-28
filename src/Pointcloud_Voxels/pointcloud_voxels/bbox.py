"""Point-cloud oriented bbox generation and persistence.

The bbox construction follows the available ws_ual cluster-bbox logic:
project xyz onto xy/xz/yz, voxelize the projection, connect voxel centroids,
fit a PCA oriented rectangle, then add the extent along the projection normal.
The tracker keeps a 3D bbox across frames; a nearby lower-confidence bbox does
not replace it.
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class PointCloudBBox:
    center: np.ndarray
    dimensions: np.ndarray  # length, width, height
    corners: np.ndarray     # (8, 3), low normal first
    point_count: int
    projection_plane: str


@dataclass(frozen=True)
class BBoxObservation:
    class_id: int
    class_name: str
    confidence: float
    bbox: PointCloudBBox


@dataclass(frozen=True)
class TrackedPointCloudBBox:
    track_id: int
    class_id: int
    class_name: str
    confidence: float
    center: np.ndarray
    dimensions: np.ndarray
    corners: np.ndarray
    point_count: int


def _points(points):
    points = np.asarray(points, dtype=np.float64)
    if points.shape == (0,):
        return points.reshape((0, 3))
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('points must have shape (N, 3)')
    if not np.isfinite(points).all():
        raise ValueError('points must be finite')
    return points


def _project(points, plane):
    if plane == 'xy':
        return points[:, (0, 1)], points[:, 2]
    if plane == 'xz':
        return points[:, (0, 2)], points[:, 1]
    if plane == 'yz':
        return points[:, (1, 2)], points[:, 0]
    raise ValueError('projection_plane must be xy, xz or yz')


def _unproject(uv, normal, plane):
    uv = np.asarray(uv, dtype=float)
    normal = np.asarray(normal, dtype=float)
    if plane == 'xy':
        return np.column_stack((uv[:, 0], uv[:, 1], normal))
    if plane == 'xz':
        return np.column_stack((uv[:, 0], normal, uv[:, 1]))
    if plane == 'yz':
        return np.column_stack((normal, uv[:, 0], uv[:, 1]))
    raise ValueError('projection_plane must be xy, xz or yz')


def _cluster_projected(uv, tolerance, voxel_size, min_points, max_points, max_clusters):
    cells = np.floor(uv / voxel_size).astype(np.int64)
    unique_cells, inverse = np.unique(cells, axis=0, return_inverse=True)
    count = len(unique_cells)
    point_counts = np.bincount(inverse, minlength=count).astype(float)
    centers = np.empty((count, 2), dtype=float)
    centers[:, 0] = np.bincount(inverse, weights=uv[:, 0], minlength=count) / point_counts
    centers[:, 1] = np.bincount(inverse, weights=uv[:, 1], minlength=count) / point_counts
    lookup = {tuple(cell): index for index, cell in enumerate(unique_cells)}
    labels = np.full(count, -1, dtype=np.int32)
    radius = int(math.ceil(tolerance / voxel_size))
    tolerance_sq = tolerance * tolerance
    cluster_id = 0
    for seed in range(count):
        if labels[seed] >= 0:
            continue
        labels[seed] = cluster_id
        stack = [seed]
        while stack:
            current = stack.pop()
            cx, cy = unique_cells[current]
            center = centers[current]
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    neighbor = lookup.get((cx + dx, cy + dy))
                    if neighbor is None or labels[neighbor] >= 0:
                        continue
                    delta = centers[neighbor] - center
                    if float(delta @ delta) <= tolerance_sq:
                        labels[neighbor] = cluster_id
                        stack.append(neighbor)
        cluster_id += 1
    point_labels = labels[inverse]
    counts = np.bincount(point_labels, minlength=cluster_id)
    accepted = [
        index for index, value in enumerate(counts)
        if min_points <= value <= max_points
    ]
    accepted.sort(key=lambda index: int(counts[index]), reverse=True)
    accepted = accepted[:max_clusters]
    return [(point_labels == index, int(counts[index])) for index in accepted]


def _oriented_rectangle(uv, box_type, min_box_size):
    center = np.mean(uv, axis=0, dtype=np.float64)
    if box_type == 'axis_aligned' or len(uv) < 3:
        basis = np.eye(2, dtype=np.float64)
    else:
        centered = uv.astype(np.float64) - center
        covariance = centered.T @ centered / float(len(centered))
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        basis = eigenvectors[:, np.argsort(eigenvalues)[::-1]]
    local = (uv.astype(np.float64) - center) @ basis
    lower = np.min(local, axis=0)
    upper = np.max(local, axis=0)
    dimensions = upper - lower
    if np.min(dimensions) < min_box_size:
        return None
    local_corners = np.array([
        [lower[0], lower[1]], [upper[0], lower[1]],
        [upper[0], upper[1]], [lower[0], upper[1]],
    ])
    corners = center + local_corners @ basis.T
    return corners, center, dimensions


def _aabb_bbox(points, plane, min_height):
    lower = points.min(axis=0)
    upper = points.max(axis=0)
    center = (lower + upper) / 2.0
    dimensions = upper - lower
    if dimensions[0] <= 0.0 or dimensions[1] <= 0.0:
        dimensions = np.maximum(dimensions, min_height)
    if dimensions[2] <= 0.0:
        dimensions[2] = min_height
    offset = dimensions / 2.0
    signs = np.array([
        [0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
        [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1],
    ], dtype=float)
    corners = center - offset + signs * dimensions
    return PointCloudBBox(center, dimensions, corners, len(points), plane)


def pointcloud_bbox(
    points, projection_plane='yz', voxel_size=0.25, cluster_tolerance=0.8,
    min_cluster_points=20, max_cluster_points=200000, box_type='oriented',
    min_box_size=0.1, min_box_height=0.1, cluster_selection='largest_area',
    max_clusters=100, fallback_to_aabb=True,
):
    """Fit one 3D bbox to the dominant projected point cluster."""
    points = _points(points)
    if projection_plane not in ('xy', 'xz', 'yz'):
        raise ValueError('projection_plane must be xy, xz or yz')
    if box_type not in ('oriented', 'axis_aligned'):
        raise ValueError('box_type must be oriented or axis_aligned')
    if cluster_selection not in ('largest_area', 'largest_count'):
        raise ValueError('cluster_selection must be largest_area or largest_count')
    voxel_size = float(voxel_size)
    cluster_tolerance = float(cluster_tolerance)
    min_box_size = float(min_box_size)
    min_box_height = float(min_box_height)
    if not np.isfinite(voxel_size) or voxel_size <= 0:
        raise ValueError('voxel_size must be finite and positive')
    if not np.isfinite(cluster_tolerance) or cluster_tolerance < 0:
        raise ValueError('cluster_tolerance must be finite and non-negative')
    if not np.isfinite(min_box_size) or min_box_size <= 0:
        raise ValueError('min_box_size must be finite and positive')
    if not np.isfinite(min_box_height) or min_box_height <= 0:
        raise ValueError('min_box_height must be finite and positive')
    if int(min_cluster_points) <= 0 or int(max_cluster_points) < int(min_cluster_points):
        raise ValueError('invalid cluster point limits')
    if int(max_clusters) <= 0:
        raise ValueError('max_clusters must be positive')
    if len(points) == 0:
        return None
    uv, normal = _project(points, projection_plane)
    clusters = _cluster_projected(
        uv, cluster_tolerance, voxel_size, int(min_cluster_points),
        int(max_cluster_points), int(max_clusters),
    )
    candidates = []
    for mask, count in clusters:
        rectangle = _oriented_rectangle(uv[mask], box_type, min_box_size)
        if rectangle is None:
            continue
        corners2d, center2d, dimensions2d = rectangle
        area = float(dimensions2d[0] * dimensions2d[1])
        candidates.append((area, count, mask, corners2d, center2d, dimensions2d))
    if not candidates:
        return _aabb_bbox(points, projection_plane, min_box_height) if fallback_to_aabb else None
    if cluster_selection == 'largest_area':
        chosen = max(candidates, key=lambda item: (item[0], item[1]))
    else:
        chosen = max(candidates, key=lambda item: (item[1], item[0]))
    _, count, mask, corners2d, center2d, dimensions2d = chosen
    normal_values = normal[mask]
    normal_min = float(np.min(normal_values))
    normal_max = float(np.max(normal_values))
    if normal_max - normal_min < min_box_height:
        midpoint = (normal_min + normal_max) / 2.0
        normal_min = midpoint - min_box_height / 2.0
        normal_max = midpoint + min_box_height / 2.0
    low = _unproject(corners2d, np.full(4, normal_min), projection_plane)
    high = _unproject(corners2d, np.full(4, normal_max), projection_plane)
    corners = np.vstack((low, high))
    center = _unproject(center2d.reshape(1, 2),
                        np.array([(normal_min + normal_max) / 2.0]),
                        projection_plane)[0]
    dimensions = np.array([dimensions2d[0], dimensions2d[1], normal_max - normal_min])
    return PointCloudBBox(center, dimensions, corners, count, projection_plane)


def bbox_aabb(corners):
    corners = np.asarray(corners, dtype=float)
    if corners.shape != (8, 3) or not np.isfinite(corners).all():
        raise ValueError('bbox corners must be finite with shape (8, 3)')
    return corners.min(axis=0), corners.max(axis=0)


def bbox_volume(bbox):
    return float(np.prod(bbox.dimensions))


def _aabb_iou(first, second):
    first_lower, first_upper = bbox_aabb(first)
    second_lower, second_upper = bbox_aabb(second)
    inter = np.maximum(np.minimum(first_upper, second_upper)
                       - np.maximum(first_lower, second_lower), 0.0)
    inter_volume = float(np.prod(inter))
    first_volume = float(np.prod(first_upper - first_lower))
    second_volume = float(np.prod(second_upper - second_lower))
    union = first_volume + second_volume - inter_volume
    return inter_volume / union if union > 0.0 else 0.0


class PointCloudBBoxTracker:
    """Persist 3D bboxes; nearby lower-confidence observations do not update."""

    def __init__(self, match_iou=0.2, center_ratio=0.5,
                 confidence_margin=0.0, timeout_sec=2.0, max_tracks=100):
        self.match_iou = float(match_iou)
        self.center_ratio = float(center_ratio)
        self.confidence_margin = float(confidence_margin)
        self.timeout_sec = float(timeout_sec)
        self.max_tracks = int(max_tracks)
        if not 0.0 <= self.match_iou <= 1.0:
            raise ValueError('match_iou must be in [0, 1]')
        if not np.isfinite(self.center_ratio) or self.center_ratio < 0.0:
            raise ValueError('center_ratio must be finite and non-negative')
        if not np.isfinite(self.confidence_margin) or self.confidence_margin < 0.0:
            raise ValueError('confidence_margin must be finite and non-negative')
        if not np.isfinite(self.timeout_sec) or self.timeout_sec < 0.0:
            raise ValueError('timeout_sec must be finite and non-negative')
        if self.max_tracks <= 0:
            raise ValueError('max_tracks must be positive')
        self._tracks = []
        self._next_id = 1

    def _geometry(self, observation, track):
        iou = _aabb_iou(observation.bbox.corners, track['corners'])
        distance = float(np.linalg.norm(observation.bbox.center - track['center']))
        diagonals = [
            float(np.linalg.norm(observation.bbox.dimensions)),
            float(np.linalg.norm(track['dimensions'])),
        ]
        return iou, distance, max(diagonals)

    def _matches(self, observation, track):
        if int(observation.class_id) != int(track['class_id']):
            return False
        iou, distance, diagonal = self._geometry(observation, track)
        return iou >= self.match_iou or distance <= self.center_ratio * diagonal

    def _best(self, observation, tracks):
        best, best_score = None, None
        for track in tracks:
            if not self._matches(observation, track):
                continue
            iou, distance, diagonal = self._geometry(observation, track)
            score = (iou, -distance, -int(track['track_id']))
            if best_score is None or score > best_score:
                best, best_score = track, score
        return best

    def update(self, observations, now_sec):
        now = float(now_sec)
        if not np.isfinite(now):
            raise ValueError('now_sec must be finite')
        observations = list(observations)
        used = set()
        for observation in sorted(observations, key=lambda item: -float(item.confidence)):
            unused = [track for track in self._tracks
                      if int(track['track_id']) not in used]
            track = self._best(observation, unused)
            if track is None:
                if self._best(observation, [item for item in self._tracks
                                            if int(item['track_id']) in used]) is not None:
                    continue
                track = {
                    'track_id': self._next_id,
                    'class_id': int(observation.class_id),
                    'class_name': str(observation.class_name),
                    'confidence': float(observation.confidence),
                    'center': np.asarray(observation.bbox.center, dtype=float).copy(),
                    'dimensions': np.asarray(observation.bbox.dimensions, dtype=float).copy(),
                    'corners': np.asarray(observation.bbox.corners, dtype=float).copy(),
                    'point_count': int(observation.bbox.point_count),
                    'last_seen': now,
                }
                self._next_id += 1
                self._tracks.append(track)
                used.add(track['track_id'])
                continue
            used.add(track['track_id'])
            track['last_seen'] = now
            if float(observation.confidence) > float(track['confidence']) + self.confidence_margin:
                track['class_name'] = str(observation.class_name)
                track['confidence'] = float(observation.confidence)
                track['center'] = np.asarray(observation.bbox.center, dtype=float).copy()
                track['dimensions'] = np.asarray(observation.bbox.dimensions, dtype=float).copy()
                track['corners'] = np.asarray(observation.bbox.corners, dtype=float).copy()
                track['point_count'] = int(observation.bbox.point_count)
        self._tracks = [
            track for track in self._tracks
            if now - float(track['last_seen']) <= self.timeout_sec
        ]
        if len(self._tracks) > self.max_tracks:
            self._tracks.sort(
                key=lambda track: (float(track['last_seen']),
                                   float(track['confidence'])),
                reverse=True,
            )
            self._tracks = self._tracks[:self.max_tracks]
        return [
            TrackedPointCloudBBox(
                track_id=int(track['track_id']),
                class_id=int(track['class_id']),
                class_name=str(track['class_name']),
                confidence=float(track['confidence']),
                center=np.asarray(track['center'], dtype=float).copy(),
                dimensions=np.asarray(track['dimensions'], dtype=float).copy(),
                corners=np.asarray(track['corners'], dtype=float).copy(),
                point_count=int(track['point_count']),
            )
            for track in sorted(self._tracks, key=lambda item: int(item['track_id']))
        ]
