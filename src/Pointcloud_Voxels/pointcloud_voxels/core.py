"""Project LiDAR points into image pixels and classify occupied 3D cells.

The input and selected points remain in the LiDAR frame.  Camera transforms are
used only to look up original-image pixels.  Instance masks use exact pixel
labels; camera detections use the reported image polygon/box.
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class VoxelGrid:
    indices: np.ndarray
    centers: np.ndarray
    point_counts: np.ndarray
    empty_faces: np.ndarray
    classes: np.ndarray  # 0: blue plane, 1: green edge, 2: red corner


def _points(points):
    points = np.asarray(points, dtype=np.float64)
    if points.shape == (0,):
        return points.reshape((0, 3))
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('points must have shape (N, 3)')
    return points


def _positive(value, name):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be finite and positive')
    return value


def _integer(value, name, minimum=0):
    try:
        integer = int(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f'{name} must be an integer >= {minimum}') from exc
    if isinstance(value, (bool, np.bool_)) or integer != value or value < minimum:
        raise ValueError(f'{name} must be an integer >= {minimum}')
    return integer


def _cell_indices(points, size):
    indices = np.floor(points / size)
    # Reserve one lattice step at either end for neighbour lookups.
    limit = np.iinfo(np.int64)
    if not np.isfinite(indices).all() or np.any(indices <= limit.min) or np.any(indices >= limit.max):
        raise ValueError('point coordinates exceed the integer voxel lattice')
    return indices.astype(np.int64)


def filter_points(points, roi_min, roi_max, zero_epsilon=1e-6):
    """Remove non-finite/zero samples and apply inclusive LiDAR-frame ROI."""
    points = _points(points)
    lower, upper = np.asarray(roi_min, dtype=float), np.asarray(roi_max, dtype=float)
    if lower.shape != (3,) or upper.shape != (3,):
        raise ValueError('ROI bounds must contain x, y, z')
    if np.isnan(lower).any() or np.isnan(upper).any() or np.any(lower > upper):
        raise ValueError('ROI bounds must be ordered and must not contain NaN')
    zero_epsilon = float(zero_epsilon)
    if not math.isfinite(zero_epsilon) or zero_epsilon < 0:
        raise ValueError('zero_epsilon must be finite and nonnegative')
    keep = (
        np.isfinite(points).all(axis=1)
        & (np.linalg.norm(points, axis=1) > zero_epsilon)
        & ((points >= lower) & (points <= upper)).all(axis=1)
    )
    return points[keep]


def validate_transform(transform):
    """Validate a rigid LiDAR-to-camera-optical homogeneous transform."""
    transform = np.asarray(transform, dtype=float)
    if transform.shape == (16,):
        transform = transform.reshape((4, 4))
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError('lidar_to_camera must be a finite 4x4 transform')
    rotation = transform[:3, :3]
    if not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-7, rtol=0):
        raise ValueError('transform last row must be [0, 0, 0, 1]')
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0):
        raise ValueError('transform rotation must be orthonormal')
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6, rtol=0):
        raise ValueError('transform rotation must have determinant +1')
    return transform


def project_pixels(
    points, camera_matrix, lidar_to_camera, distortion=(),
    distortion_model='plumb_bob', near=0.1, far=100.0,
):
    """Return continuous camera pixel coordinates for each source point.

    Invalid, non-finite, behind-camera and outside-range points receive NaN.
    Intrinsics must describe the image whose detections are being consumed.
    No image resize, box fill, timestamp change or world-frame conversion is
    performed here.
    """
    points = _points(points)
    matrix = np.asarray(camera_matrix, dtype=float)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError('camera_matrix must be finite with shape (3, 3)')
    if matrix[0, 0] <= 0 or matrix[1, 1] <= 0 or not np.allclose(matrix[2], [0, 0, 1]):
        raise ValueError('camera_matrix must have positive fx/fy and last row [0, 0, 1]')
    transform = validate_transform(lidar_to_camera)
    coefficients = np.asarray(distortion, dtype=float)
    expected = {'plumb_bob': (0, 4, 5), 'rational_polynomial': (8,)}
    if distortion_model not in expected or coefficients.ndim != 1:
        raise ValueError('unsupported camera distortion model')
    if len(coefficients) not in expected[distortion_model] or not np.isfinite(coefficients).all():
        raise ValueError('invalid distortion coefficients for camera model')
    near, far = float(near), float(far)
    if not math.isfinite(near) or not math.isfinite(far) or not 0 <= near < far:
        raise ValueError('projection range must satisfy finite 0 <= near < far')

    u = np.full(len(points), np.nan, dtype=float)
    v = np.full(len(points), np.nan, dtype=float)
    finite = np.isfinite(points).all(axis=1)
    source_indices = np.flatnonzero(finite)
    if len(source_indices) == 0:
        return u, v
    camera = points[finite] @ transform[:3, :3].T + transform[:3, 3]
    visible = (camera[:, 2] > near) & (camera[:, 2] < far)
    camera, source_indices = camera[visible], source_indices[visible]
    if len(camera) == 0:
        return u, v
    x, y = camera[:, 0] / camera[:, 2], camera[:, 1] / camera[:, 2]
    if len(coefficients):
        k1, k2, p1, p2 = coefficients[:4]
        k3 = coefficients[4] if len(coefficients) >= 5 else 0.0
        r2 = x*x + y*y
        with np.errstate(over='ignore', invalid='ignore', divide='ignore'):
            radial = 1 + r2 * (k1 + r2 * (k2 + r2 * k3))
            if distortion_model == 'rational_polynomial':
                k4, k5, k6 = coefficients[5:8]
                radial /= 1 + r2 * (k4 + r2 * (k5 + r2 * k6))
            distorted_x = x * radial + 2*p1*x*y + p2*(r2 + 2*x*x)
            distorted_y = y * radial + p1*(r2 + 2*y*y) + 2*p2*x*y
        x, y = distorted_x, distorted_y
    projected_u = matrix[0, 0]*x + matrix[0, 1]*y + matrix[0, 2]
    projected_v = matrix[1, 0]*x + matrix[1, 1]*y + matrix[1, 2]
    finite_pixels = np.isfinite(projected_u) & np.isfinite(projected_v)
    u[source_indices[finite_pixels]] = projected_u[finite_pixels]
    v[source_indices[finite_pixels]] = projected_v[finite_pixels]
    return u, v


def project_instance_ids(
    points, id_map, camera_matrix, lidar_to_camera, distortion=(),
    distortion_model='plumb_bob', near=0.1, far=100.0,
):
    """Return the exact instance-mask ID at each point's camera pixel.

    Background, invalid, behind-camera and out-of-image points receive ID 0.
    Intrinsics must describe this mask's original-image resolution.  No mask
    resize, bbox fill, timestamp change, or world-frame conversion is done.
    """
    points = _points(points)
    id_map = np.asarray(id_map)
    if id_map.ndim != 2 or not all(id_map.shape) or id_map.dtype.kind not in 'iu':
        raise ValueError('id_map must be a nonempty 2D integer image')
    if np.any(id_map < 0) or np.any(id_map > np.iinfo(np.int32).max):
        raise ValueError('instance IDs must be nonnegative int32 values')
    u, v = project_pixels(
        points, camera_matrix, lidar_to_camera, distortion,
        distortion_model, near, far,
    )
    ids = np.zeros(len(points), dtype=np.int32)
    height, width = id_map.shape
    finite_pixels = (
        np.isfinite(u) & np.isfinite(v)
        & (u >= -1) & (u <= width) & (v >= -1) & (v <= height)
    )
    source_indices = np.flatnonzero(finite_pixels)
    if len(source_indices) == 0:
        return ids
    rounded_u = np.rint(u[source_indices]).astype(np.int64)
    rounded_v = np.rint(v[source_indices]).astype(np.int64)
    inside = (
        (rounded_u >= 0) & (rounded_u < width)
        & (rounded_v >= 0) & (rounded_v < height)
    )
    ids[source_indices[inside]] = id_map[
        rounded_v[inside], rounded_u[inside]
    ]
    return ids


def voxelize(points, voxel_size=0.5, min_points=1, edge_empty_faces=4, corner_min_empty_faces=5):
    """Classify cells before any visualization cap is applied.

    Exactly ``edge_empty_faces`` empty faces is green; at least
    ``corner_min_empty_faces`` is red; the remaining cells are blue.
    """
    points = _points(points)
    size = _positive(voxel_size, 'voxel_size')
    min_points = _integer(min_points, 'min_points', minimum=1)
    edge = _integer(edge_empty_faces, 'edge_empty_faces')
    corner = _integer(corner_min_empty_faces, 'corner_min_empty_faces')
    if not edge < corner <= 6:
        raise ValueError('thresholds must satisfy 0 <= edge < corner <= 6')
    if not np.isfinite(points).all():
        raise ValueError('voxel points must be finite; filter them before voxelizing')
    indices, counts = np.unique(_cell_indices(points, size), axis=0, return_counts=True)
    keep = counts >= min_points
    indices, counts = indices[keep], counts[keep]
    occupied = {tuple(index) for index in indices.tolist()}
    offsets = ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))
    empty = np.fromiter(
        (sum((x+dx, y+dy, z+dz) not in occupied for dx, dy, dz in offsets)
         for x, y, z in indices.tolist()),
        dtype=np.uint8, count=len(indices),
    )
    classes = np.zeros(len(indices), dtype=np.uint8)
    classes[empty == edge] = 1
    classes[empty >= corner] = 2
    return VoxelGrid(indices, (indices.astype(float) + 0.5)*size, counts, empty, classes)


def select_bbox_points(
    points, projection_plane='yz', voxel_size=0.25, cluster_tolerance=0.8,
    min_cluster_points=20, max_cluster_points=200000, min_box_size=0.1,
):
    """Legacy ws_ual diagnostic: select the cluster with largest PCA box.

    This path is independent of instance masks and must be explicitly enabled
    by the caller.  It returns original 3D cluster points, not all box contents.
    """
    points = _points(points)
    axes = {'xy': (0, 1), 'xz': (0, 2), 'yz': (1, 2)}
    if projection_plane not in axes:
        raise ValueError('projection_plane must be xy, xz or yz')
    size = _positive(voxel_size, 'voxel_size')
    tolerance = _positive(cluster_tolerance, 'cluster_tolerance')
    minimum = _integer(min_cluster_points, 'min_cluster_points', 1)
    maximum = _integer(max_cluster_points, 'max_cluster_points', minimum)
    min_box_size = _positive(min_box_size, 'min_box_size')
    if not np.isfinite(points).all():
        raise ValueError('bbox points must be finite')
    if len(points) == 0:
        return points.copy()
    uv = points[:, axes[projection_plane]]
    cells, inverse = np.unique(_cell_indices(uv, size), axis=0, return_inverse=True)
    counts = np.bincount(inverse)
    centers = np.column_stack([
        np.bincount(inverse, weights=uv[:, axis])/counts for axis in (0, 1)
    ])
    lookup = {tuple(cell): index for index, cell in enumerate(cells.tolist())}
    labels = np.full(len(cells), -1, dtype=np.int32)
    radius = int(math.ceil(tolerance / size))
    offsets = [(dx, dy) for dx in range(-radius, radius+1) for dy in range(-radius, radius+1)]
    cluster_id = 0
    for seed in range(len(cells)):
        if labels[seed] >= 0:
            continue
        labels[seed] = cluster_id
        stack = [seed]
        while stack:
            current = stack.pop()
            cx, cy = cells[current]
            for dx, dy in offsets:
                neighbor = lookup.get((cx+dx, cy+dy))
                if neighbor is None or labels[neighbor] >= 0:
                    continue
                delta = centers[neighbor] - centers[current]
                if float(delta @ delta) <= tolerance*tolerance:
                    labels[neighbor] = cluster_id
                    stack.append(neighbor)
        cluster_id += 1
    point_labels = labels[inverse]
    cluster_counts = np.bincount(point_labels)
    best_area, best = -1.0, points[:0]
    for cluster_id, count in enumerate(cluster_counts):
        if not minimum <= count <= maximum:
            continue
        selected = point_labels == cluster_id
        projected = uv[selected]
        centered = projected - projected.mean(axis=0)
        if len(projected) < 3:
            basis = np.eye(2)
        else:
            _, vectors = np.linalg.eigh(centered.T @ centered / len(projected))
            basis = vectors[:, ::-1]
        dimensions = np.ptp(centered @ basis, axis=0)
        area = float(np.prod(dimensions))
        if np.min(dimensions) >= min_box_size and area > best_area:
            best_area, best = area, points[selected]
    return best
