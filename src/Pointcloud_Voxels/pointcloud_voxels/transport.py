"""ROS-message-shaped binary decoders and time matching; no ROS imports."""

from collections import deque

import numpy as np


def stamp_ns(message):
    stamp = message.header.stamp
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    if value <= 0 or not 0 <= int(stamp.nanosec) < 1_000_000_000:
        raise ValueError('A non-zero, valid sensor header timestamp is required')
    return value


def read_xyz(message):
    """Decode FLOAT32/FLOAT64 XYZ including row padding and either byte order."""
    fields = {field.name: field for field in message.fields}
    endian = '>' if message.is_bigendian else '<'
    formats, offsets = [], []
    for name in ('x', 'y', 'z'):
        field = fields.get(name)
        if field is None or field.count != 1 or field.datatype not in (7, 8):
            raise ValueError(f'PointCloud2 field {name} must be one FLOAT32/FLOAT64')
        width = 4 if field.datatype == 7 else 8
        if field.offset < 0 or field.offset + width > message.point_step:
            raise ValueError(f'Invalid {name} offset/point_step')
        formats.append(endian + ('f4' if width == 4 else 'f8'))
        offsets.append(field.offset)
    if message.height == 0 or message.width == 0:
        return np.empty((0, 3), dtype=np.float64)
    if message.row_step < message.width * message.point_step:
        raise ValueError('PointCloud2 row_step is smaller than its row width')
    if len(message.data) < message.height * message.row_step:
        raise ValueError('PointCloud2 payload is truncated')
    dtype = np.dtype({
        'names': ['x', 'y', 'z'], 'formats': formats,
        'offsets': offsets, 'itemsize': message.point_step,
    })
    rows = np.ndarray(
        shape=(message.height, message.width), dtype=dtype,
        buffer=message.data, strides=(message.row_step, message.point_step),
    )
    return np.column_stack([rows[name].reshape(-1) for name in ('x', 'y', 'z')])


def read_id_map(message):
    """Decode the original-resolution signed instance IDs, never a bbox image."""
    if message.encoding.lower() != '32sc1':
        raise ValueError('instance_id_map must use 32SC1 encoding')
    if message.height <= 0 or message.width <= 0:
        raise ValueError('instance_id_map must have positive dimensions')
    if message.step < message.width * 4:
        raise ValueError('instance_id_map step is smaller than its row width')
    if len(message.data) < message.height * message.step:
        raise ValueError('instance_id_map payload is truncated')
    dtype = np.dtype('>i4' if message.is_bigendian else '<i4')
    result = np.ndarray(
        shape=(message.height, message.width), dtype=dtype,
        buffer=message.data, strides=(message.step, 4),
    ).astype(np.int32, copy=False)
    if np.any(result < 0):
        raise ValueError('Instance IDs must be non-negative; zero is background')
    return result


class CloudCache:
    """Bounded history for delayed masks, matched by sensor time, not arrival."""

    def __init__(self, capacity=800, reset_threshold_sec=30.0):
        if capacity <= 0 or not np.isfinite(reset_threshold_sec) or reset_threshold_sec <= 0:
            raise ValueError('Cloud cache capacity/reset threshold must be positive')
        self.messages = deque(maxlen=capacity)
        self.reset_ns = int(reset_threshold_sec * 1e9)

    def append(self, message):
        current = stamp_ns(message)
        reset = bool(self.messages and current < stamp_ns(self.messages[-1]) - self.reset_ns)
        if reset:
            self.messages.clear()
        self.messages.append(message)
        return reset

    def nearest(self, target_ns, slop_sec):
        if not np.isfinite(slop_sec) or slop_sec < 0:
            raise ValueError('sync_slop must be finite and non-negative')
        if not self.messages:
            return None
        candidate = min(self.messages, key=lambda msg: abs(stamp_ns(msg) - target_ns))
        if abs(stamp_ns(candidate) - target_ns) > slop_sec * 1e9:
            return None
        return candidate
