"""Simple 3D A* path planning for a bbox-selected target point.

The planner works in the same LiDAR frame as the selected point clouds.  It is
deliberately independent of ROS so the collision grid and path can be tested
offline.  It is a local demonstration, not a replacement for EGO-Planner's
dynamic trajectory optimization or a closed-loop flight controller.
"""

from dataclasses import dataclass
import heapq
import math

import numpy as np


@dataclass(frozen=True)
class PlanResult:
    waypoints: np.ndarray
    length: float
    visited: int


def _points(points):
    points = np.asarray(points, dtype=np.float64)
    if points.shape == (0,):
        return points.reshape((0, 3))
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('points must have shape (N, 3)')
    if not np.isfinite(points).all():
        raise ValueError('points must be finite')
    return points


def point_bounds(points):
    """Return (minimum, maximum, center) of a non-empty 3D point set."""
    points = _points(points)
    if len(points) == 0:
        raise ValueError('at least one target point is required')
    lower = points.min(axis=0)
    upper = points.max(axis=0)
    return lower, upper, (lower + upper) / 2.0


def _positive(value, name):
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f'{name} must be finite and positive')
    return value


def _nonnegative(value, name):
    value = float(value)
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(f'{name} must be finite and non-negative')
    return value


def _cell(point, lower, resolution):
    return np.floor((np.asarray(point, dtype=float) - lower) / resolution).astype(np.int64)


def _clear_sphere(occupied, center, radius_cells):
    if radius_cells < 0:
        return
    for dx in range(-radius_cells, radius_cells + 1):
        for dy in range(-radius_cells, radius_cells + 1):
            for dz in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy + dz * dz > radius_cells * radius_cells:
                    continue
                index = (int(center[0]) + dx, int(center[1]) + dy, int(center[2]) + dz)
                if all(0 <= index[axis] < occupied.shape[axis] for axis in range(3)):
                    occupied[index] = False


def _segment_clear(start, end, occupied, lower, resolution):
    length = float(np.linalg.norm(end - start))
    steps = max(2, int(math.ceil(length / (resolution * 0.35))) + 1)
    samples = np.linspace(start, end, steps)
    cells = np.floor((samples - lower) / resolution).astype(np.int64)
    if not np.all((cells >= 0) & (cells < np.asarray(occupied.shape))):
        return False
    return not np.any(occupied[cells[:, 0], cells[:, 1], cells[:, 2]])


def _smooth(waypoints, occupied, lower, resolution):
    if len(waypoints) <= 2:
        return waypoints
    result = [waypoints[0]]
    index = 0
    while index < len(waypoints) - 1:
        candidate = len(waypoints) - 1
        while candidate > index + 1:
            if _segment_clear(waypoints[index], waypoints[candidate], occupied, lower, resolution):
                break
            candidate -= 1
        result.append(waypoints[candidate])
        index = candidate
    return np.asarray(result, dtype=float)


def plan_path(
    start, target, obstacles, resolution=1.0, inflation=0.5,
    margin_xy=5.0, margin_z=3.0, start_clearance=0.5,
    goal_clearance=0.5, max_grid_cells=200000, smooth=True,
):
    """Plan a 3D grid path from start to target around obstacle points.

    The search bounds are the start/target box expanded by the requested
    margins.  Obstacle points are inflated by the requested radius, and small
    spheres around start and target are cleared so those nodes are reachable.
    Returns None when the goal is outside the bounded grid or no path exists.
    """
    start = np.asarray(start, dtype=float).reshape(-1)
    target = np.asarray(target, dtype=float).reshape(-1)
    if start.shape != (3,) or target.shape != (3,) or not np.isfinite(start).all() or not np.isfinite(target).all():
        raise ValueError('start and target must be finite 3D points')
    obstacles = _points(obstacles)
    resolution = _positive(resolution, 'resolution')
    inflation = _nonnegative(inflation, 'inflation')
    margin_xy = _nonnegative(margin_xy, 'margin_xy')
    margin_z = _nonnegative(margin_z, 'margin_z')
    start_clearance = _nonnegative(start_clearance, 'start_clearance')
    goal_clearance = _nonnegative(goal_clearance, 'goal_clearance')
    max_grid_cells = int(max_grid_cells)
    if max_grid_cells <= 0:
        raise ValueError('max_grid_cells must be positive')

    margin = np.array([margin_xy, margin_xy, margin_z], dtype=float)
    lower = np.minimum(start, target) - margin
    upper = np.maximum(start, target) + margin
    dims = np.floor((upper - lower) / resolution).astype(np.int64) + 1
    if np.any(dims < 1):
        raise ValueError('invalid planning bounds')
    if int(np.prod(dims, dtype=np.int64)) > max_grid_cells:
        raise ValueError(
            f'planning grid {tuple(int(v) for v in dims)} exceeds max_grid_cells={max_grid_cells}'
        )

    occupied = np.zeros(tuple(int(v) for v in dims), dtype=bool)
    if len(obstacles):
        cells = np.floor((obstacles - lower) / resolution).astype(np.int64)
        inside = np.all((cells >= 0) & (cells < dims), axis=1)
        cells = np.unique(cells[inside], axis=0)
        radius_cells = int(math.ceil(inflation / resolution))
        offsets = [
            (dx, dy, dz)
            for dx in range(-radius_cells, radius_cells + 1)
            for dy in range(-radius_cells, radius_cells + 1)
            for dz in range(-radius_cells, radius_cells + 1)
            if dx * dx + dy * dy + dz * dz <= radius_cells * radius_cells
        ]
        for offset in offsets:
            shifted = cells + np.asarray(offset)
            valid = np.all((shifted >= 0) & (shifted < dims), axis=1)
            shifted = shifted[valid]
            occupied[shifted[:, 0], shifted[:, 1], shifted[:, 2]] = True

    start_cell = tuple(int(v) for v in _cell(start, lower, resolution))
    goal_cell = tuple(int(v) for v in _cell(target, lower, resolution))
    if not all(0 <= start_cell[i] < dims[i] for i in range(3)):
        raise ValueError('start point is outside the bounded planning grid')
    if not all(0 <= goal_cell[i] < dims[i] for i in range(3)):
        raise ValueError('target point is outside the bounded planning grid')
    _clear_sphere(occupied, start_cell, int(math.ceil(start_clearance / resolution)))
    _clear_sphere(occupied, goal_cell, int(math.ceil(goal_clearance / resolution)))

    if start_cell == goal_cell:
        points = np.vstack((start, target))
        return PlanResult(points, float(np.linalg.norm(target - start)), 1)

    neighbors = [
        (dx, dy, dz)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        for dz in (-1, 0, 1)
        if not (dx == 0 and dy == 0 and dz == 0)
    ]
    goal_array = np.asarray(goal_cell, dtype=float)

    def heuristic(cell):
        return float(np.linalg.norm(np.asarray(cell, dtype=float) - goal_array) * resolution)

    frontier = [(heuristic(start_cell), 0.0, start_cell)]
    cost = {start_cell: 0.0}
    came_from = {}
    closed = set()
    while frontier:
        _, current_cost, current = heapq.heappop(frontier)
        if current in closed:
            continue
        closed.add(current)
        if current == goal_cell:
            break
        if len(closed) > max_grid_cells:
            break
        for dx, dy, dz in neighbors:
            neighbor = (current[0] + dx, current[1] + dy, current[2] + dz)
            if not all(0 <= neighbor[i] < dims[i] for i in range(3)):
                continue
            if occupied[neighbor]:
                continue
            step = math.sqrt(dx * dx + dy * dy + dz * dz) * resolution
            new_cost = current_cost + step
            if new_cost >= cost.get(neighbor, math.inf):
                continue
            cost[neighbor] = new_cost
            came_from[neighbor] = current
            heapq.heappush(frontier, (new_cost + heuristic(neighbor), new_cost, neighbor))

    if goal_cell not in came_from and goal_cell != start_cell:
        return None
    cells = [goal_cell]
    while cells[-1] != start_cell:
        cells.append(came_from[cells[-1]])
    cells.reverse()
    waypoints = lower + (np.asarray(cells, dtype=float) + 0.5) * resolution
    waypoints[0] = start
    waypoints[-1] = target
    if smooth:
        waypoints = _smooth(waypoints, occupied, lower, resolution)
    length = float(np.sum(np.linalg.norm(np.diff(waypoints, axis=0), axis=1)))
    return PlanResult(waypoints, length, len(closed))
