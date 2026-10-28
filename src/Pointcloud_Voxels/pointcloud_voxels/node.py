"""ROS 2 adapter: camera masks/detections -> original-frame LiDAR voxels."""

from collections import deque
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PointStamped
from ins_seg.msg import DetectionResult, SegmentationResult
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from .bbox import (
    BBoxObservation, PointCloudBBoxTracker, bbox_volume, pointcloud_bbox,
)
from .core import (
    filter_points, project_instance_ids, project_pixels, select_bbox_points,
    validate_transform, voxelize,
)
from .detection import DetectionBox, select_detection_groups
from .selection import select_mask_groups
from .transport import CloudCache, read_id_map, read_xyz, stamp_ns


class MaskVoxelsNode(Node):
    def __init__(self):
        super().__init__('mask_voxels')
        defaults = {
            'selection_mode': 'detection',
            'pointcloud_topic': '/livox/lidar/points',
            'pointcloud_reliability': 'best_effort',
            'segmentation_topic': '/sam3/segmentation',
            'detection_topic': '/ship/detections',
            'camera_info_topic': '/left_camera/camera_info',
            'marker_topic': '/ship/voxels',
            'selected_cloud_topic': '/ship/points',
            'calibration_confirmed': False,
            'allow_approximate_calibration': False,
            'lidar_frame': 'avia_frame',
            'camera_optical_frame': '',
            'image_width': 1280,
            'image_height': 1024,
            'camera_matrix': [385.45, 0.0, 640.0, 0.0, 385.45, 512.0, 0.0, 0.0, 1.0],
            'distortion': [0.0, 0.0, 0.0, 0.0, 0.0],
            'distortion_model': 'plumb_bob',
            'lidar_to_camera': np.eye(4).flatten().tolist(),
            'near_plane': 0.1,
            'far_plane': 100.0,
            'sync_slop': 0.2,
            'pointcloud_cache_size': 800,
            'pending_mask_limit': 4,
            'timestamp_reset_threshold_sec': 30.0,
            'instance_selection': 'largest',
            'target_instance_id': 0,
            'target_class_name': '',
            'detection_boundary_epsilon': 1e-6,
            'point_bbox_enabled': True,
            'bbox_topic': '/ship/bboxes',
            'bbox_target_topic': '/ship/bbox_target',
            'bbox_projection_plane': 'yz',
            'bbox_voxel_size': 0.25,
            'bbox_cluster_tolerance': 0.8,
            'bbox_min_cluster_points': 20,
            'bbox_max_cluster_points': 200000,
            'bbox_box_type': 'oriented',
            'bbox_min_box_size': 0.1,
            'bbox_min_height': 0.1,
            'bbox_cluster_selection': 'largest_area',
            'bbox_fallback_to_aabb': True,
            'bbox_confidence_margin': 0.0,
            'bbox_match_iou': 0.2,
            'bbox_match_center_ratio': 0.5,
            'bbox_timeout_sec': 2.0,
            'max_bboxes': 100,
            'bbox_target_selection': 'largest_volume',
            'bbox_line_width': 0.08,
            'bbox_label_height': 0.8,
            'roi_min': [0.5, -10.0, -25.0],
            'roi_max': [80.0, 10.0, 25.0],
            'voxel_3d_size': 0.5,
            'voxel_3d_min_points': 1,
            'voxel_edge_empty_faces': 4,
            'voxel_corner_min_empty_faces': 5,
            'max_visualized_3d_voxels': 30000,
            'marker_lifetime': 0.0,
            'show_labels': True,
            'plane_rgba': [0.05, 0.35, 1.0, 0.22],
            'edge_rgba': [0.10, 0.80, 0.20, 1.0],
            'corner_rgba': [0.95, 0.0, 0.0, 1.0],
            'projection_plane': 'yz',
            'voxel_size': 0.25,
            'cluster_tolerance': 0.8,
            'min_cluster_points': 20,
            'max_cluster_points': 200000,
            'min_box_size': 0.1,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.cfg = {name: self.get_parameter(name).value for name in defaults}
        p = self.cfg
        if p['selection_mode'] not in ('mask', 'detection', 'cluster', 'bbox'):
            raise ValueError('selection_mode must be mask, detection, cluster or bbox')
        # Keep the historical name as an alias for the LiDAR-only diagnostic.
        if p['selection_mode'] == 'bbox':
            p['selection_mode'] = 'cluster'
        if p['pointcloud_reliability'] not in ('best_effort', 'reliable'):
            raise ValueError('pointcloud_reliability must be best_effort or reliable')
        if p['instance_selection'] not in ('largest', 'all'):
            raise ValueError('instance_selection must be largest or all')
        for name in ('image_width', 'image_height', 'pending_mask_limit', 'max_visualized_3d_voxels'):
            if p[name] <= 0:
                raise ValueError(f'{name} must be positive')
        if p['target_instance_id'] < 0:
            raise ValueError('target_instance_id must be >= 0')
        if not np.isfinite(p['sync_slop']) or p['sync_slop'] < 0:
            raise ValueError('sync_slop must be finite and >= 0')
        if not np.isfinite(p['marker_lifetime']) or p['marker_lifetime'] < 0:
            raise ValueError('marker_lifetime must be finite and >= 0')
        if not np.isfinite(p['detection_boundary_epsilon']) or p['detection_boundary_epsilon'] < 0:
            raise ValueError('detection_boundary_epsilon must be finite and >= 0')
        if p['bbox_target_selection'] not in ('largest_volume', 'highest_confidence'):
            raise ValueError('bbox_target_selection must be largest_volume or highest_confidence')
        for name in ('bbox_line_width', 'bbox_label_height'):
            if not np.isfinite(float(p[name])) or float(p[name]) <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        if p['point_bbox_enabled']:
            pointcloud_bbox(
                np.empty((0, 3)), p['bbox_projection_plane'],
                p['bbox_voxel_size'], p['bbox_cluster_tolerance'],
                p['bbox_min_cluster_points'], p['bbox_max_cluster_points'],
                p['bbox_box_type'], p['bbox_min_box_size'], p['bbox_min_height'],
                p['bbox_cluster_selection'], p['max_bboxes'],
                p['bbox_fallback_to_aabb'],
            )
            self.bbox_tracker = PointCloudBBoxTracker(
                match_iou=p['bbox_match_iou'],
                center_ratio=p['bbox_match_center_ratio'],
                confidence_margin=p['bbox_confidence_margin'],
                timeout_sec=p['bbox_timeout_sec'],
                max_tracks=p['max_bboxes'],
            )
        else:
            self.bbox_tracker = None
        for name in ('plane_rgba', 'edge_rgba', 'corner_rgba'):
            rgba = np.asarray(p[name])
            if rgba.shape != (4,) or not np.isfinite(rgba).all() or np.any((rgba < 0) | (rgba > 1)):
                raise ValueError(f'{name} must be four finite values in [0, 1]')
        # Validate numerical configuration even before input messages arrive.
        filter_points(np.empty((0, 3)), p['roi_min'], p['roi_max'])
        voxelize(np.empty((0, 3)), p['voxel_3d_size'], p['voxel_3d_min_points'],
                 p['voxel_edge_empty_faces'], p['voxel_corner_min_empty_faces'])
        if p['selection_mode'] in ('mask', 'detection'):
            self.transform = validate_transform(p['lidar_to_camera'])
            project_pixels(
                np.empty((0, 3)),
                np.asarray(p['camera_matrix']).reshape(3, 3), self.transform,
                p['distortion'], p['distortion_model'], p['near_plane'], p['far_plane'],
            )
        else:
            select_bbox_points(
                np.empty((0, 3)), **{name: p[name] for name in (
                    'projection_plane', 'voxel_size', 'cluster_tolerance',
                    'min_cluster_points', 'max_cluster_points', 'min_box_size',
                )},
            )
        self.cache = CloudCache(p['pointcloud_cache_size'], p['timestamp_reset_threshold_sec'])
        self.pending = deque(maxlen=p['pending_mask_limit'])
        self.camera_info = None
        self.last_processed_ns = None
        self.marker_pub = self.create_publisher(
            MarkerArray, p['marker_topic'],
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE),
        )
        self.cloud_pub = self.create_publisher(PointCloud2, p['selected_cloud_topic'], 1)
        if p['point_bbox_enabled']:
            self.bbox_pub = self.create_publisher(MarkerArray, p['bbox_topic'], 1)
            self.bbox_target_pub = self.create_publisher(
                PointStamped, p['bbox_target_topic'], 1)
        self.cloud_sub = self.create_subscription(
            PointCloud2, p['pointcloud_topic'], self._cloud_callback,
            QoSProfile(depth=5, reliability=(
                ReliabilityPolicy.RELIABLE if p['pointcloud_reliability'] == 'reliable'
                else ReliabilityPolicy.BEST_EFFORT
            )),
        )
        if p['selection_mode'] == 'mask':
            # The image-sized id map needs reliable delivery, matching ins_seg.
            self.mask_sub = self.create_subscription(
                SegmentationResult, p['segmentation_topic'], self._mask_callback, 1,
            )
        elif p['selection_mode'] == 'detection':
            # Detection results are small, but reliable delivery keeps a
            # complete frame after image/DDS bursts.
            self.detection_sub = self.create_subscription(
                DetectionResult, p['detection_topic'], self._detection_callback,
                QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE),
            )
        if p['selection_mode'] in ('mask', 'detection') and p['camera_info_topic']:
            self.info_sub = self.create_subscription(
                CameraInfo, p['camera_info_topic'], self._info_callback, qos_profile_sensor_data,
            )
        if p['selection_mode'] in ('mask', 'detection'):
            if not p['calibration_confirmed'] and p['allow_approximate_calibration']:
                self._warn('APPROXIMATE calibration enabled: provisional intrinsics/extrinsics; '
                           'outputs are for pipeline checks, not calibrated geometry.')
            elif not p['calibration_confirmed']:
                self._warn('Camera selection awaits verified calibration: fill sensor config and set '
                           'calibration_confirmed=true; current matrices are placeholders.')
        self.get_logger().info(
            f"mode={p['selection_mode']} input={p['pointcloud_topic']} -> "
            f"{p['selected_cloud_topic']}, {p['marker_topic']}"
        )

    def _warn(self, message):
        self.get_logger().warning(message, throttle_duration_sec=5.0)

    def _info_callback(self, message):
        self.camera_info = message

    def _cloud_callback(self, message):
        try:
            if self.cfg['selection_mode'] == 'cluster':
                points = filter_points(read_xyz(message), self.cfg['roi_min'], self.cfg['roi_max'])
                selected = select_bbox_points(
                    points, **{name: self.cfg[name] for name in (
                        'projection_plane', 'voxel_size', 'cluster_tolerance',
                        'min_cluster_points', 'max_cluster_points', 'min_box_size',
                    )},
                )
                self._publish(message.header, {1: selected} if len(selected) else {})
                return
            if self.cache.append(message):
                self.pending.clear()
                self.last_processed_ns = None
                self._publish(message.header, {})
                self._warn('Cloud time jumped backwards; cleared cloud/perception caches (bag loop).')
            self._process_pending()
        except (ValueError, TypeError, OverflowError) as error:
            self._warn(f'Point cloud rejected: {error}')
            self._publish(message.header, {})

    def _queue_perception(self, kind, message):
        try:
            target = stamp_ns(message)
            if kind == 'mask':
                if stamp_ns(message.instance_id_map) != target:
                    raise ValueError('Segmentation and id-map header timestamps differ')
                map_frame = message.instance_id_map.header.frame_id
                if map_frame and message.header.frame_id and map_frame != message.header.frame_id:
                    raise ValueError('Segmentation and id-map camera frames differ')
            if self.last_processed_ns is not None and target <= self.last_processed_ns:
                self._warn(f'Ignoring duplicated/out-of-order {kind} timestamp.')
                return
            if len(self.pending) == self.pending.maxlen:
                self._warn(f'Pending {kind} queue full; oldest unmatched result is being dropped.')
            self.pending.append((kind, message))
            self._process_pending()
        except (ValueError, TypeError, OverflowError) as error:
            self._warn(f'{kind.capitalize()} rejected: {error}')

    def _mask_callback(self, message):
        self._queue_perception('mask', message)

    def _detection_callback(self, message):
        self._queue_perception('detection', message)

    def _process_pending(self):
        for _ in range(len(self.pending)):
            kind, perception = self.pending.popleft()
            target = stamp_ns(perception)
            if self.last_processed_ns is not None and target <= self.last_processed_ns:
                continue
            cloud = self.cache.nearest(target, self.cfg['sync_slop'])
            if cloud is None:
                if self.cache.messages and stamp_ns(self.cache.messages[-1]) > target + self.cfg['sync_slop'] * 1e9:
                    self._warn(f'No cloud within sync_slop of {kind} acquisition time; '
                               'check sensor header time bases/cache size. Latest cloud is not substituted.')
                else:
                    self.pending.append((kind, perception))
                continue
            try:
                if kind == 'mask':
                    groups = self._select_mask_points(cloud, perception)
                else:
                    groups, observations = self._select_detection_points(cloud, perception)
                    self._publish_bboxes(cloud.header, observations)
                self._publish(cloud.header, groups)
            except (ValueError, TypeError, OverflowError) as error:
                self._warn(f'{kind.capitalize()}/cloud pair rejected: {error}')
                if kind == 'detection' and self.bbox_tracker is not None:
                    self._publish_bboxes(cloud.header, [])
                self._publish(cloud.header, {})
            self.last_processed_ns = target

    def _camera_parameters(self, message, width, height, extra_frames=()):
        p = self.cfg
        if not p['calibration_confirmed'] and not p['allow_approximate_calibration']:
            raise ValueError('calibration_confirmed=false: supply verified intrinsics/extrinsics first')
        camera_frame = p['camera_optical_frame']
        if not camera_frame:
            raise ValueError('Set camera_optical_frame to the calibrated lidar_to_camera target frame')
        info = self.camera_info
        if info is not None:
            if info.binning_x not in (0, 1) or info.binning_y not in (0, 1) or info.roi.x_offset or info.roi.y_offset:
                raise ValueError('Binned/cropped CameraInfo requires matching full-image calibration')
            if info.roi.width not in (0, info.width) or info.roi.height not in (0, info.height):
                raise ValueError('Cropped CameraInfo ROI does not describe the complete image')
            if info.header.frame_id and info.header.frame_id != camera_frame:
                raise ValueError('CameraInfo frame differs from the calibrated lidar_to_camera target frame')
            if (width, height) != (info.width, info.height):
                raise ValueError('Input image and camera calibration sizes differ; images will not be resized')
            matrix = np.asarray(info.k).reshape(3, 3)
            distortion, model = info.d, info.distortion_model
        else:
            if (width, height) != (p['image_width'], p['image_height']):
                raise ValueError('Input image differs from configured calibration image size; '
                                 'images will not be resized')
            matrix = np.asarray(p['camera_matrix']).reshape(3, 3)
            distortion, model = p['distortion'], p['distortion_model']
        for frame in (message.header.frame_id, *extra_frames):
            if frame and frame != camera_frame:
                raise ValueError(f'Camera input frame {frame} differs from calibrated frame {camera_frame}')
        return matrix, distortion, model

    def _select_mask_points(self, cloud, mask):
        p = self.cfg
        if not p['lidar_frame'] or cloud.header.frame_id != p['lidar_frame']:
            raise ValueError('PointCloud2 frame must match the source frame of lidar_to_camera')
        id_map = read_id_map(mask.instance_id_map)
        matrix, distortion, model = self._camera_parameters(
            mask, id_map.shape[1], id_map.shape[0],
            (mask.instance_id_map.header.frame_id,),
        )
        points = filter_points(read_xyz(cloud), p['roi_min'], p['roi_max'])
        ids = project_instance_ids(
            points, id_map, matrix, self.transform, distortion, model,
            p['near_plane'], p['far_plane'],
        )
        metadata = {int(item.track_id): item.class_name for item in mask.instances}
        if len(metadata) != len(mask.instances):
            raise ValueError('Instance metadata contains duplicate IDs')
        return select_mask_groups(
            points, ids, id_map, metadata, p['instance_selection'],
            p['target_instance_id'], p['target_class_name'],
        )

    def _detection_image_size(self, message):
        width, height = int(message.image_width), int(message.image_height)
        if width <= 0 or height <= 0:
            width, height = int(self.cfg['image_width']), int(self.cfg['image_height'])
        return width, height

    @staticmethod
    def _decode_detections(message):
        detections = []
        for item in message.detections:
            class_name = str(item.class_name).strip()
            if not class_name:
                raise ValueError('Detection class name must not be empty')
            confidence = float(item.confidence)
            if not np.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
                raise ValueError('Detection confidence must be finite and in [0, 1]')
            polygon = np.asarray(
                [[float(point.x), float(point.y)] for point in item.polygon.points],
                dtype=float,
            )
            if polygon.shape != (4, 2):
                raise ValueError('Detection polygon must contain exactly four corners')
            detections.append(DetectionBox(
                track_id=int(item.track_id),
                class_name=class_name,
                confidence=confidence,
                polygon=polygon,
                oriented=bool(item.oriented),
                class_id=int(item.class_id),
            ))
        return detections

    def _select_detection_points(self, cloud, message):
        p = self.cfg
        if not p['lidar_frame'] or cloud.header.frame_id != p['lidar_frame']:
            raise ValueError('PointCloud2 frame must match the source frame of lidar_to_camera')
        width, height = self._detection_image_size(message)
        matrix, distortion, model = self._camera_parameters(message, width, height)
        detections = self._decode_detections(message)
        points = filter_points(read_xyz(cloud), p['roi_min'], p['roi_max'])
        u, v = project_pixels(
            points, matrix, self.transform, distortion, model,
            p['near_plane'], p['far_plane'],
        )
        # A detector polygon may extend beyond the image.  Never select a
        # point that has no pixel in this image, even if that polygon covers it.
        inside = (
            np.isfinite(u) & np.isfinite(v)
            & (u >= 0) & (u < width) & (v >= 0) & (v < height)
        )
        pixels = np.column_stack((
            np.where(inside, u, np.nan),
            np.where(inside, v, np.nan),
        ))
        groups = select_detection_groups(
            points, pixels, detections,
            p['instance_selection'], p['target_instance_id'],
            p['target_class_name'], p['detection_boundary_epsilon'],
        )
        observations = []
        if self.bbox_tracker is not None:
            by_id = {int(item.track_id): item for item in detections}
            for track_id, group_points in groups.items():
                detection = by_id.get(int(track_id))
                if detection is None or len(group_points) == 0:
                    continue
                bbox = pointcloud_bbox(
                    group_points, p['bbox_projection_plane'],
                    p['bbox_voxel_size'], p['bbox_cluster_tolerance'],
                    p['bbox_min_cluster_points'], p['bbox_max_cluster_points'],
                    p['bbox_box_type'], p['bbox_min_box_size'], p['bbox_min_height'],
                    p['bbox_cluster_selection'], p['max_bboxes'],
                    p['bbox_fallback_to_aabb'],
                )
                if bbox is not None:
                    observations.append(BBoxObservation(
                        class_id=int(detection.class_id),
                        class_name=detection.class_name,
                        confidence=float(detection.confidence),
                        bbox=bbox,
                    ))
        return groups, observations

    def _publish_bboxes(self, header, observations):
        if self.bbox_tracker is None:
            return
        tracked = self.bbox_tracker.update(observations, time.monotonic())
        markers = [Marker(header=header, action=Marker.DELETEALL)]
        p = self.cfg
        lifetime_ns = int(p['marker_lifetime'] * 1e9)
        lifetime = Duration(sec=lifetime_ns // 10**9, nanosec=lifetime_ns % 10**9)
        edges = ((0, 1), (1, 2), (2, 3), (3, 0),
                 (4, 5), (5, 6), (6, 7), (7, 4),
                 (0, 4), (1, 5), (2, 6), (3, 7))
        for bbox in tracked:
            box = Marker(
                header=header, ns='ship_point_bbox', id=bbox.track_id,
                type=Marker.LINE_LIST, action=Marker.ADD)
            box.pose.orientation.w = 1.0
            box.scale.x = float(p['bbox_line_width'])
            box.color = ColorRGBA(r=1.0, g=0.55, b=0.1, a=1.0)
            box.lifetime = lifetime
            box.points = [
                Point(x=float(bbox.corners[index][0]),
                      y=float(bbox.corners[index][1]),
                      z=float(bbox.corners[index][2]))
                for edge in edges for index in edge
            ]
            markers.append(box)
            label = Marker(
                header=header, ns='ship_point_bbox_labels', id=bbox.track_id,
                type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
            label.pose.orientation.w = 1.0
            label.pose.position = Point(
                x=float(bbox.center[0]), y=float(bbox.center[1]),
                z=float(bbox.center[2] + p['bbox_label_height']))
            label.scale.z = 0.5
            label.color = ColorRGBA(r=1.0, g=0.85, b=0.2, a=1.0)
            label.lifetime = lifetime
            label.text = (
                f'BBox {bbox.track_id} {bbox.class_name} conf={bbox.confidence:.2f} '
                f'L/W/H={bbox.dimensions[0]:.1f}/{bbox.dimensions[1]:.1f}/'
                f'{bbox.dimensions[2]:.1f} n={bbox.point_count}'
            )
            markers.append(label)
        self.bbox_pub.publish(MarkerArray(markers=markers))
        if not tracked:
            return
        if observations:
            if p['bbox_target_selection'] == 'highest_confidence':
                selected = max(observations, key=lambda item: (
                    item.confidence, bbox_volume(item.bbox)))
            else:
                selected = max(observations, key=lambda item: (
                    bbox_volume(item.bbox), item.confidence))
            candidates = [item for item in tracked
                          if item.class_id == selected.class_id]
            target = (min(candidates,
                          key=lambda item: float(np.linalg.norm(
                              item.center - selected.bbox.center)))
                      if candidates else None)
        elif p['bbox_target_selection'] == 'highest_confidence':
            target = max(tracked, key=lambda item: (item.confidence,
                                                    bbox_volume(item)))
        else:
            target = max(tracked, key=lambda item: (bbox_volume(item),
                                                    item.confidence))
        if target is None:
            return
        message = PointStamped()
        message.header = header
        message.point = Point(
            x=float(target.center[0]), y=float(target.center[1]),
            z=float(target.center[2]))
        self.bbox_target_pub.publish(message)

    def _publish(self, header, groups):
        clear = Marker(header=header, action=Marker.DELETEALL)
        markers = [clear]
        combined = []
        p = self.cfg
        palette = (('plane', p['plane_rgba']), ('edge', p['edge_rgba']), ('corner', p['corner_rgba']))
        for instance_id, points in groups.items():
            combined.append(points)
            grid = voxelize(points, p['voxel_3d_size'], p['voxel_3d_min_points'],
                            p['voxel_edge_empty_faces'], p['voxel_corner_min_empty_faces'])
            # Count empty faces on the complete grid first; a render limit must
            # not turn neighbours omitted from RViz into false empty faces.
            visible = np.arange(len(grid.indices))
            if len(visible) > p['max_visualized_3d_voxels']:
                visible = np.argsort(-grid.point_counts, kind='stable')[:p['max_visualized_3d_voxels']]
            for class_id, (name, rgba) in enumerate(palette):
                marker = Marker(header=header, ns=f'ship_voxel_{name}', id=instance_id,
                                type=Marker.CUBE_LIST, action=Marker.ADD)
                marker.pose.orientation.w = 1.0
                marker.scale.x = marker.scale.y = marker.scale.z = p['voxel_3d_size'] * 0.96
                marker.color = ColorRGBA(**dict(zip(('r', 'g', 'b', 'a'), rgba)))
                lifetime_ns = int(p['marker_lifetime'] * 1e9)
                marker.lifetime = Duration(sec=lifetime_ns // 10**9, nanosec=lifetime_ns % 10**9)
                subset = visible[grid.classes[visible] == class_id]
                marker.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in grid.centers[subset]]
                markers.append(marker)
            if p['show_labels'] and len(grid.centers):
                label = Marker(header=header, ns='ship_voxel_labels', id=instance_id,
                               type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
                label.pose.orientation.w = 1.0
                center = grid.centers.mean(axis=0)
                label.pose.position = Point(x=float(center[0]), y=float(center[1]), z=float(center[2] + 0.8))
                label.scale.z = 0.5
                label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                label.lifetime = markers[-1].lifetime
                counts = np.bincount(grid.classes, minlength=3)
                label.text = f'ID={instance_id} vox={len(grid.indices)} C/E/F={counts[2]}/{counts[1]}/{counts[0]}'
                markers.append(label)
        selected = np.concatenate(combined) if combined else np.empty((0, 3), dtype=np.float32)
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, selected.astype(np.float32)))
        self.marker_pub.publish(MarkerArray(markers=markers))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MaskVoxelsNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            # ros2 launch can forward a second SIGINT during cleanup.
            pass
