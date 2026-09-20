#!/usr/bin/env python3
"""Evaluate z-up point clouds for circular UAV landing footprints."""

import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray

from auto_landing.msg import LandingRegion, LandingRegionArray


class LandingEvaluator(Node):
    """Turn a point cloud into safe connected landing regions and confidence."""

    def __init__(self):
        super().__init__('landing_evaluator')
        defaults = {
            'input_topic': '/projected_cloud',
            'regions_topic': '/landing/regions',
            'confidence_cloud_topic': '/landing/confidence_cloud',
            'markers_topic': '/landing/markers',
            'drone_radius': 0.5,
            'safety_margin': 0.15,
            'grid_resolution': 0.10,
            'min_points_per_cell': 3,
            'max_slope_deg': 10.0,
            'max_roughness': 0.05,
            'max_height_range': 0.15,
            'target_points_per_cell': 8,
            'min_region_area': 0.25,
            'evaluation_radius': 20.0,
            'max_grid_cells': 1000000,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.drone_radius = float(self.get_parameter('drone_radius').value)
        self.safety_margin = float(self.get_parameter('safety_margin').value)
        self.resolution = float(self.get_parameter('grid_resolution').value)
        self.min_points = int(self.get_parameter('min_points_per_cell').value)
        self.max_slope = float(self.get_parameter('max_slope_deg').value)
        self.max_roughness = float(self.get_parameter('max_roughness').value)
        self.max_height_range = float(
            self.get_parameter('max_height_range').value
        )
        self.target_points = int(
            self.get_parameter('target_points_per_cell').value
        )
        self.min_region_area = float(self.get_parameter('min_region_area').value)
        self.evaluation_radius = float(
            self.get_parameter('evaluation_radius').value
        )
        self.max_grid_cells = int(self.get_parameter('max_grid_cells').value)
        self._validate_parameters()

        self.regions_pub = self.create_publisher(
            LandingRegionArray, self.get_parameter('regions_topic').value, 10
        )
        self.confidence_pub = self.create_publisher(
            PointCloud2, self.get_parameter('confidence_cloud_topic').value, 1
        )
        self.markers_pub = self.create_publisher(
            MarkerArray, self.get_parameter('markers_topic').value, 10
        )
        self.subscription = self.create_subscription(
            PointCloud2,
            self.get_parameter('input_topic').value,
            self.pointcloud_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            '降落评估节点已启动: '
            f'R={self.drone_radius:.2f} m, margin={self.safety_margin:.2f} m, '
            f'resolution={self.resolution:.2f} m'
        )

    def _validate_parameters(self):
        if self.drone_radius <= 0.0 or self.resolution <= 0.0:
            raise ValueError('drone_radius 和 grid_resolution 必须大于 0')
        if self.safety_margin < 0.0:
            raise ValueError('safety_margin 不能小于 0')
        if (
            self.max_slope <= 0.0
            or self.max_roughness <= 0.0
            or self.max_height_range <= 0.0
        ):
            raise ValueError('坡度和粗糙度阈值必须大于 0')

    @staticmethod
    def _read_xyz(message):
        cloud = point_cloud2.read_points(
            message, field_names=('x', 'y', 'z'), skip_nans=True
        )
        array = np.asarray(cloud)
        if array.dtype.names:
            return np.column_stack(
                [array[field] for field in ('x', 'y', 'z')]
            ).astype(np.float64, copy=False)
        return np.asarray(list(cloud), dtype=np.float64).reshape((-1, 3))

    def pointcloud_callback(self, message):
        try:
            points = self._read_xyz(message)
            result = self.evaluate_points(points)
            self._publish_result(message.header, result)
        except Exception as exc:
            self.get_logger().error(f'降落区域评估失败: {exc}')

    def evaluate_points(self, points):
        """Evaluate an Nx3 numpy array; separated for deterministic testing."""
        if len(points) == 0:
            return self._empty_result()

        finite = np.all(np.isfinite(points), axis=1)
        points = points[finite]
        if len(points) == 0:
            return self._empty_result()

        # Bound both runtime and the influence of distant outliers.
        center_xy = np.median(points[:, :2], axis=0)
        if self.evaluation_radius > 0.0:
            relative = points[:, :2] - center_xy
            points = points[
                np.einsum('ij,ij->i', relative, relative)
                <= self.evaluation_radius ** 2
            ]
        if len(points) == 0:
            return self._empty_result()

        xy_min = np.floor(points[:, :2].min(axis=0) / self.resolution)
        xy_max = np.floor(points[:, :2].max(axis=0) / self.resolution)
        width, height = (xy_max - xy_min + 1).astype(int)
        if width * height > self.max_grid_cells:
            raise ValueError(
                f'点云范围生成 {width * height} 个栅格，超过 max_grid_cells='
                f'{self.max_grid_cells}；请减小 evaluation_radius 或增大分辨率'
            )

        indices = np.floor(points[:, :2] / self.resolution) - xy_min
        indices = indices.astype(np.int32)
        flat = indices[:, 1] * width + indices[:, 0]
        order = np.argsort(flat)
        flat_sorted = flat[order]
        unique, starts, counts = np.unique(
            flat_sorted, return_index=True, return_counts=True
        )

        heights = np.full((height, width), np.nan, dtype=np.float32)
        roughness = np.full((height, width), np.inf, dtype=np.float32)
        height_range = np.full((height, width), np.inf, dtype=np.float32)
        density = np.zeros((height, width), dtype=np.int32)
        for cell, start, count in zip(unique, starts, counts):
            z_values = points[order[start:start + count], 2]
            median = float(np.median(z_values))
            # Robust sigma estimate; isolated vertical outliers have low influence.
            mad = float(np.median(np.abs(z_values - median)))
            row, column = divmod(int(cell), width)
            heights[row, column] = median
            roughness[row, column] = 1.4826 * mad
            height_range[row, column] = float(np.ptp(z_values))
            density[row, column] = int(count)

        observed = density >= self.min_points
        neighbor_valid = (
            observed
            & np.roll(observed, 1, axis=0)
            & np.roll(observed, -1, axis=0)
            & np.roll(observed, 1, axis=1)
            & np.roll(observed, -1, axis=1)
        )
        neighbor_valid[[0, -1], :] = False
        neighbor_valid[:, [0, -1]] = False

        slope = np.full_like(heights, np.inf)
        dz_dx = (
            np.roll(heights, -1, axis=1) - np.roll(heights, 1, axis=1)
        ) / (2.0 * self.resolution)
        dz_dy = (
            np.roll(heights, -1, axis=0) - np.roll(heights, 1, axis=0)
        ) / (2.0 * self.resolution)
        slope[neighbor_valid] = np.degrees(np.arctan(np.hypot(
            dz_dx[neighbor_valid], dz_dy[neighbor_valid]
        )))

        cell_safe = (
            neighbor_valid
            & (slope <= self.max_slope)
            & (roughness <= self.max_roughness)
            & (height_range <= self.max_height_range)
        )
        footprint_radius = self.drone_radius + self.safety_margin
        radius_cells = max(1, int(math.ceil(footprint_radius / self.resolution)))
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (2 * radius_cells + 1, 2 * radius_cells + 1)
        )
        footprint_safe = cv2.erode(
            cell_safe.astype(np.uint8), kernel, iterations=1
        ).astype(bool)

        slope_score = np.clip(1.0 - slope / self.max_slope, 0.0, 1.0)
        roughness_score = np.clip(
            1.0 - roughness / self.max_roughness, 0.0, 1.0
        )
        density_score = np.clip(
            density.astype(np.float32) / max(self.target_points, 1), 0.0, 1.0
        )
        height_range_score = np.clip(
            1.0 - height_range / self.max_height_range, 0.0, 1.0
        )
        clearance = cv2.distanceTransform(
            cell_safe.astype(np.uint8), cv2.DIST_L2, 5
        ) * self.resolution
        clearance_score = np.clip(
            clearance / max(2.0 * footprint_radius, self.resolution), 0.0, 1.0
        )

        # Geometric mean: one weak factor strongly reduces overall confidence.
        cell_confidence = np.power(
            np.maximum(slope_score, 0.0)
            * np.maximum(roughness_score, 0.0)
            * np.maximum(density_score, 0.0)
            * np.maximum(height_range_score, 0.0)
            * np.maximum(clearance_score, 0.0),
            0.2,
        )
        # Minimum confidence across the whole circular UAV footprint.
        footprint_confidence = cv2.erode(
            cell_confidence.astype(np.float32), kernel, iterations=1
        )
        footprint_confidence[~footprint_safe] = 0.0

        component_count, labels, stats, _ = cv2.connectedComponentsWithStats(
            footprint_safe.astype(np.uint8), connectivity=8
        )
        regions = []
        for component in range(1, component_count):
            mask = labels == component
            area = int(stats[component, cv2.CC_STAT_AREA]) * self.resolution ** 2
            if area < self.min_region_area:
                continue

            rows, columns = np.nonzero(mask)
            scores = footprint_confidence[mask]
            best_index = int(np.argmax(scores))
            best_row, best_column = rows[best_index], columns[best_index]
            regions.append({
                'id': len(regions),
                'center': (
                    (best_column + xy_min[0] + 0.5) * self.resolution,
                    (best_row + xy_min[1] + 0.5) * self.resolution,
                    float(heights[best_row, best_column]),
                ),
                # Blend the safest point with the region-average repeatability.
                'confidence': float(
                    0.7 * scores[best_index] + 0.3 * np.mean(scores)
                ),
                'area': float(area),
                'equivalent_radius': float(math.sqrt(area / math.pi)),
                'mean_slope_deg': float(np.mean(slope[mask])),
                'mean_roughness': float(np.mean(roughness[mask])),
                'supporting_points': int(np.sum(density[mask])),
            })

        candidate_rows, candidate_columns = np.nonzero(footprint_safe)
        confidence_points = np.column_stack((
            (candidate_columns + xy_min[0] + 0.5) * self.resolution,
            (candidate_rows + xy_min[1] + 0.5) * self.resolution,
            heights[candidate_rows, candidate_columns],
            footprint_confidence[candidate_rows, candidate_columns],
        )).astype(np.float32)
        return {'regions': regions, 'confidence_points': confidence_points}

    @staticmethod
    def _empty_result():
        return {
            'regions': [],
            'confidence_points': np.empty((0, 4), dtype=np.float32),
        }

    def _publish_result(self, header, result):
        region_array = LandingRegionArray()
        region_array.header = header
        for data in result['regions']:
            region = LandingRegion()
            region.id = data['id']
            region.center.x, region.center.y, region.center.z = data['center']
            region.confidence = data['confidence']
            region.area = data['area']
            region.equivalent_radius = data['equivalent_radius']
            region.mean_slope_deg = data['mean_slope_deg']
            region.mean_roughness = data['mean_roughness']
            region.supporting_points = data['supporting_points']
            region_array.regions.append(region)
        self.regions_pub.publish(region_array)

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name='confidence', offset=12,
                datatype=PointField.FLOAT32, count=1
            ),
        ]
        cloud = point_cloud2.create_cloud(
            header, fields, result['confidence_points']
        )
        self.confidence_pub.publish(cloud)
        self._publish_markers(header, result['regions'])

    def _publish_markers(self, header, regions):
        markers = MarkerArray()
        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        diameter = 2.0 * (self.drone_radius + self.safety_margin)
        for data in regions:
            marker = Marker()
            marker.header = header
            marker.ns = 'landing_regions'
            marker.id = data['id']
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = (
                data['center']
            )
            marker.pose.position.z += 0.015
            marker.pose.orientation.w = 1.0
            marker.scale.x = diameter
            marker.scale.y = diameter
            marker.scale.z = 0.03
            marker.color.r = 1.0 - data['confidence']
            marker.color.g = data['confidence']
            marker.color.b = 0.1
            marker.color.a = 0.75
            marker.lifetime.sec = 1
            markers.markers.append(marker)

            text = Marker()
            text.header = header
            text.ns = 'landing_scores'
            text.id = data['id']
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x, text.pose.position.y, text.pose.position.z = (
                data['center']
            )
            text.pose.position.z += 0.3
            text.pose.orientation.w = 1.0
            text.scale.z = 0.22
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = f"ID {data['id']}: {data['confidence']:.2f}"
            text.lifetime.sec = 1
            markers.markers.append(text)
        self.markers_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = LandingEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
