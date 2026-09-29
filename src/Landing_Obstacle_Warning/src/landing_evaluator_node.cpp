#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

#include <landing_evaluator/HeightMap.h>
#include <landing_evaluator/LandingStatus.h>
#include <landing_evaluator/evaluator.hpp>

namespace landing_evaluator
{
class LandingEvaluatorNode
{
public:
  LandingEvaluatorNode() : nh_(), pnh_("~")
  {
    param("input_topic", input_topic_, std::string("/livox/lidar"));
    param("output_topic", output_topic_, std::string("/landing/status"));
    param("height_map_topic", height_map_topic_, std::string("/landing/height_map"));
    param("enable_topic", enable_topic_, std::string("/landing_evaluator/enable"));
    param("output_frame", output_frame_, std::string("base_link"));
    param("imu/enabled", use_imu_, false);
    param("imu/required", imu_required_, true);
    param("imu/topic", imu_topic_, std::string("/livox/imu"));
    param("imu/timeout_sec", imu_timeout_sec_, 0.15);
    param("imu/filter_alpha", imu_filter_alpha_, 0.12);
    param("imu/gravity_norm_filter_alpha", imu_gravity_norm_filter_alpha_, 0.01);
    param("imu/max_angular_velocity", imu_max_gyro_, 0.8);
    param("imu/max_accel_deviation_ratio", imu_max_accel_deviation_, 0.30);

    param("landing_radius", params_.landing_radius, params_.landing_radius);
    param("min_range", params_.min_range, params_.min_range);
    param("max_range", params_.max_range, params_.max_range);
    param("min_roi_points", params_.min_roi_points, params_.min_roi_points);
    param("ransac_iterations", params_.ransac_iterations, params_.ransac_iterations);
    int ransac_seed = static_cast<int>(params_.ransac_seed);
    param("ransac_seed", ransac_seed, ransac_seed);
    param("ransac_distance", params_.ransac_distance, params_.ransac_distance);
    param("ransac_early_exit_ratio", params_.ransac_early_exit_ratio,
      params_.ransac_early_exit_ratio);
    param("max_slope_deg", params_.max_slope_deg, params_.max_slope_deg);
    param("max_roughness_rms", params_.max_roughness_rms, params_.max_roughness_rms);
    param("max_roughness_p95", params_.max_roughness_p95, params_.max_roughness_p95);
    param("airspace_clearance_height", params_.airspace_clearance_height,
      params_.airspace_clearance_height);
    param("airspace_ceiling_z", params_.airspace_ceiling_z, params_.airspace_ceiling_z);
    param("airspace_min_points", params_.airspace_min_points, params_.airspace_min_points);
    param("max_occupied_cell_ratio", params_.max_occupied_cell_ratio,
      params_.max_occupied_cell_ratio);
    param("max_step_height", params_.max_step_height, params_.max_step_height);
    param("grid_resolution", params_.grid_resolution, params_.grid_resolution);
    int max_grid_cells = static_cast<int>(params_.max_grid_cells);
    param("max_grid_cells", max_grid_cells, max_grid_cells);
    param("grid_min_points", params_.grid_min_points, params_.grid_min_points);
    param("min_coverage_ratio", params_.min_coverage_ratio, params_.min_coverage_ratio);
    param("min_inlier_ratio", params_.min_inlier_ratio, params_.min_inlier_ratio);
    param("confirmation_frames", confirm_frames_, 3);
    param("landable_log_delay_sec", landable_log_delay_sec_, 5.0);
    param("watchdog/cloud_timeout_sec", cloud_timeout_sec_, 0.30);
    param("watchdog/publish_period_sec", watchdog_publish_period_sec_, 1.0);
    param("startup_enabled", startup_enabled_, true);

    std::vector<double> xyz{0.0, 0.0, 0.0};
    std::vector<double> rpy{0.0, 0.0, 0.0};
    param("extrinsics/translation", xyz, xyz);
    param("extrinsics/rotation_rpy", rpy, rpy);
    std::vector<double> imu_rpy = rpy;
    param("imu/extrinsics/rotation_rpy", imu_rpy, imu_rpy);
    if (xyz.size() != 3 || rpy.size() != 3 || imu_rpy.size() != 3) {
      throw std::invalid_argument("extrinsics vectors must contain exactly 3 values");
    }

    params_.ransac_seed = static_cast<uint32_t>(ransac_seed);
    params_.max_grid_cells = static_cast<size_t>(max_grid_cells);
    confirm_frames_ = std::max(1, confirm_frames_);
    validateParameters(ransac_seed, max_grid_cells);
    tx_ = xyz[0];
    ty_ = xyz[1];
    tz_ = xyz[2];
    makeRotation(rpy[0], rpy[1], rpy[2], rot_);
    makeRotation(imu_rpy[0], imu_rpy[1], imu_rpy[2], imu_rot_);

    status_pub_ = nh_.advertise<LandingStatus>(output_topic_, 10);
    height_map_pub_ = nh_.advertise<HeightMap>(height_map_topic_, 2);
    enable_service_ = pnh_.advertiseService("set_enabled", &LandingEvaluatorNode::setEnabled, this);
    enable_sub_ = nh_.subscribe(enable_topic_, 1, &LandingEvaluatorNode::onEnable, this);
    if (use_imu_) {
      imu_sub_ = nh_.subscribe(imu_topic_, 1, &LandingEvaluatorNode::onImu, this,
        ros::TransportHints().tcpNoDelay());
    }
    if (startup_enabled_) {
      createCloudSubscription();
    }
    const double watchdog_period = std::max(0.01, std::min(0.1, cloud_timeout_sec_ / 2.0));
    watchdog_timer_ = nh_.createTimer(
      ros::Duration(watchdog_period), &LandingEvaluatorNode::onWatchdog, this);
    ROS_INFO(
      "Landing evaluator %s; input %s, frame %s, radius %.2f m, radar IMU %s (%s)",
      startup_enabled_ ? "enabled" : "disabled", input_topic_.c_str(),
      output_frame_.c_str(), params_.landing_radius, use_imu_ ? "enabled" : "disabled",
      imu_topic_.c_str());
  }

private:
  template<typename T>
  void param(const std::string & name, T & value, const T & fallback)
  {
    pnh_.param<T>(name, value, fallback);
  }

  void validateParameters(int ransac_seed, int max_grid_cells) const
  {
    if (ransac_seed < 0 || max_grid_cells <= 0 || params_.landing_radius <= 0 ||
      params_.min_range < 0 || params_.max_range <= params_.min_range ||
      params_.min_roi_points < 3 || params_.grid_resolution <= 0 ||
      params_.ransac_distance <= 0 || params_.ransac_iterations <= 0 ||
      params_.ransac_early_exit_ratio <= 0 || params_.ransac_early_exit_ratio > 1 ||
      params_.grid_min_points <= 0 || params_.airspace_min_points <= 0 ||
      params_.airspace_clearance_height <= 0 || params_.max_occupied_cell_ratio < 0 ||
      params_.max_occupied_cell_ratio > 1 || params_.max_slope_deg < 0 ||
      params_.max_slope_deg > 90 || params_.max_roughness_rms < 0 ||
      params_.max_roughness_p95 < 0 || params_.max_step_height < 0 ||
      params_.min_coverage_ratio < 0 || params_.min_coverage_ratio > 1 ||
      params_.min_inlier_ratio < 0 || params_.min_inlier_ratio > 1 ||
      landable_log_delay_sec_ < 0 || imu_timeout_sec_ <= 0 || imu_filter_alpha_ <= 0 ||
      imu_filter_alpha_ > 1 || imu_gravity_norm_filter_alpha_ <= 0 ||
      imu_gravity_norm_filter_alpha_ > 1 || imu_max_gyro_ <= 0 ||
      imu_max_accel_deviation_ <= 0 || cloud_timeout_sec_ <= 0 ||
      watchdog_publish_period_sec_ <= 0)
    {
      throw std::invalid_argument("invalid landing evaluator parameter value");
    }
  }

  void createCloudSubscription()
  {
    if (cloud_sub_) {
      return;
    }
    last_cloud_receive_ = std::chrono::steady_clock::now();
    timeout_active_ = false;
    cloud_sub_ = nh_.subscribe(input_topic_, 1, &LandingEvaluatorNode::onCloud, this,
      ros::TransportHints().tcpNoDelay());
  }

  bool setEnabled(std_srvs::SetBool::Request & request, std_srvs::SetBool::Response & response)
  {
    response.success = true;
    response.message = applyEnabled(request.data, "service");
    return true;
  }

  void onEnable(const std_msgs::Bool::ConstPtr & command)
  {
    applyEnabled(command->data, "topic");
  }

  std::string applyEnabled(bool enable, const char * source)
  {
    if (enable == static_cast<bool>(cloud_sub_)) {
      return enable ? "already enabled" : "already disabled";
    }
    resetState();
    if (enable) {
      createCloudSubscription();
      ROS_INFO("Landing evaluation enabled by %s", source);
    } else {
      cloud_sub_.shutdown();
      ROS_INFO("Landing evaluation disabled by %s", source);
    }
    return enable ? "enabled" : "disabled";
  }

  void resetState()
  {
    candidate_ = Status::kUnknown;
    stable_ = Status::kUnknown;
    consecutive_ = 0;
    landable_since_valid_ = false;
    landable_logged_ = false;
    denial_logged_ = false;
  }

  static void makeRotation(double roll, double pitch, double yaw, std::array<double, 9> & out)
  {
    const double cr = std::cos(roll), sr = std::sin(roll);
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    out = {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
      sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
      -sp, cp * sr, cp * cr};
  }

  static Point3 rotate(const std::array<double, 9> & rotation, const Point3 & point)
  {
    return {
      rotation[0] * point.x + rotation[1] * point.y + rotation[2] * point.z,
      rotation[3] * point.x + rotation[4] * point.y + rotation[5] * point.z,
      rotation[6] * point.x + rotation[7] * point.y + rotation[8] * point.z};
  }

  void onImu(const sensor_msgs::Imu::ConstPtr & imu)
  {
    const Point3 sensor_accel{
      imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z};
    const Point3 sensor_gyro{
      imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z};
    Point3 up = rotate(imu_rot_, sensor_accel);
    const double norm = std::sqrt(up.x * up.x + up.y * up.y + up.z * up.z);
    const Point3 gyro_body = rotate(imu_rot_, sensor_gyro);
    const double gyro = std::sqrt(
      gyro_body.x * gyro_body.x + gyro_body.y * gyro_body.y + gyro_body.z * gyro_body.z);
    if (!std::isfinite(norm) || norm < 1e-3 || !std::isfinite(gyro)) {
      return;
    }
    if (gravity_norm_ <= 0) {
      gravity_norm_ = norm;
    }
    const double deviation = std::abs(norm - gravity_norm_) / gravity_norm_;
    if (gyro > imu_max_gyro_ || deviation > imu_max_accel_deviation_) {
      ROS_WARN_THROTTLE(2.0,
        "Rejecting radar IMU sample during motion (gyro=%.2f, accel deviation=%.2f)",
        gyro, deviation);
      return;
    }
    gravity_norm_ = (1.0 - imu_gravity_norm_filter_alpha_) * gravity_norm_ +
      imu_gravity_norm_filter_alpha_ * norm;
    up.x /= norm;
    up.y /= norm;
    up.z /= norm;
    if (!imu_valid_) {
      filtered_up_ = up;
      imu_valid_ = true;
    } else {
      filtered_up_.x = (1 - imu_filter_alpha_) * filtered_up_.x + imu_filter_alpha_ * up.x;
      filtered_up_.y = (1 - imu_filter_alpha_) * filtered_up_.y + imu_filter_alpha_ * up.y;
      filtered_up_.z = (1 - imu_filter_alpha_) * filtered_up_.z + imu_filter_alpha_ * up.z;
      const double filtered_norm = std::sqrt(
        filtered_up_.x * filtered_up_.x + filtered_up_.y * filtered_up_.y +
        filtered_up_.z * filtered_up_.z);
      filtered_up_.x /= filtered_norm;
      filtered_up_.y /= filtered_norm;
      filtered_up_.z /= filtered_norm;
    }
    imu_receive_time_ = std::chrono::steady_clock::now();
  }

  std::array<double, 9> levelingRotation(const Point3 & up) const
  {
    if (up.z < -0.999999) {
      return {-1, 0, 0, 0, 1, 0, 0, 0, -1};
    }
    const double k = 1.0 / (1.0 + up.z);
    return {1 - up.x * up.x * k, -up.x * up.y * k, -up.x,
      -up.x * up.y * k, 1 - up.y * up.y * k, -up.y, up.x, up.y, up.z};
  }

  static int fieldOffset(const sensor_msgs::PointCloud2 & cloud, const std::string & name)
  {
    for (const auto & field : cloud.fields) {
      if (field.name == name && field.datatype == sensor_msgs::PointField::FLOAT32) {
        return static_cast<int>(field.offset);
      }
    }
    return -1;
  }

  void onWatchdog(const ros::TimerEvent &)
  {
    if (!cloud_sub_) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - last_cloud_receive_).count() < cloud_timeout_sec_) {
      return;
    }
    const bool first = !timeout_active_;
    if (!first &&
      std::chrono::duration<double>(now - last_watchdog_publish_).count() <
      watchdog_publish_period_sec_)
    {
      return;
    }
    const auto previous = stable_;
    candidate_ = Status::kUnknown;
    stable_ = Status::kUnknown;
    consecutive_ = 0;
    landable_since_valid_ = false;
    landable_logged_ = false;
    denial_logged_ = false;
    timeout_active_ = true;
    last_watchdog_publish_ = now;

    LandingStatus output;
    output.header.stamp = ros::Time::now();
    output.header.frame_id = output_frame_;
    output.status = LandingStatus::UNKNOWN;
    output.raw_status = LandingStatus::UNKNOWN;
    output.status_changed = first && previous != Status::kUnknown;
    output.reason_mask = kCloudTimeout;
    output.reason = ReasonsToString(kCloudTimeout);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    output.slope_deg = nan;
    output.roughness_rms = nan;
    output.roughness_p95 = nan;
    output.max_obstacle_height = nan;
    output.occupied_cell_ratio = nan;
    output.max_height_step = nan;
    output.coverage_ratio = 0.0F;
    output.inlier_ratio = 0.0F;
    output.plane_height = nan;
    output.processing_time_ms = 0.0F;
    status_pub_.publish(output);
    if (first) {
      ROS_WARN("Point cloud timeout after %.2f s; publishing UNKNOWN", cloud_timeout_sec_);
    }
  }

  void onCloud(const sensor_msgs::PointCloud2::ConstPtr & cloud)
  {
    last_cloud_receive_ = std::chrono::steady_clock::now();
    if (timeout_active_) {
      timeout_active_ = false;
      ROS_INFO("Point cloud stream recovered");
    }
    processCloud(*cloud, !use_imu_ || imu_valid_, use_imu_ ? filtered_up_ : Point3{0, 0, 1});
  }

  void processCloud(
    const sensor_msgs::PointCloud2 & cloud, bool attitude_available, const Point3 & up)
  {
    const auto started = std::chrono::steady_clock::now();
    const int ox = fieldOffset(cloud, "x");
    const int oy = fieldOffset(cloud, "y");
    const int oz = fieldOffset(cloud, "z");
    if (ox < 0 || oy < 0 || oz < 0 || cloud.point_step == 0 ||
      std::max({ox, oy, oz}) + 4 > static_cast<int>(cloud.point_step))
    {
      ROS_WARN_THROTTLE(5.0, "PointCloud2 requires valid FLOAT32 x/y/z fields");
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    const bool imu_fresh = use_imu_ && attitude_available &&
      std::chrono::duration<double>(now - imu_receive_time_).count() <= imu_timeout_sec_;
    const bool attitude_ok = !use_imu_ || imu_fresh;
    std::array<double, 9> level{1, 0, 0, 0, 1, 0, 0, 0, 1};
    if (use_imu_ && attitude_ok) {
      level = levelingRotation(up);
    }

    std::vector<Point3> points;
    const double radius2 = params_.landing_radius * params_.landing_radius;
    const double min_range2 = params_.min_range * params_.min_range;
    const double max_range2 = params_.max_range * params_.max_range;
    for (uint32_t row = 0; row < cloud.height; ++row) {
      for (uint32_t column = 0; column < cloud.width; ++column) {
        const size_t offset = static_cast<size_t>(row) * cloud.row_step +
          static_cast<size_t>(column) * cloud.point_step;
        if (offset + cloud.point_step > cloud.data.size()) {
          continue;
        }
        float x, y, z;
        std::memcpy(&x, &cloud.data[offset + ox], sizeof(x));
        std::memcpy(&y, &cloud.data[offset + oy], sizeof(y));
        std::memcpy(&z, &cloud.data[offset + oz], sizeof(z));
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }
        const Point3 body{
          rot_[0] * x + rot_[1] * y + rot_[2] * z + tx_,
          rot_[3] * x + rot_[4] * y + rot_[5] * z + ty_,
          rot_[6] * x + rot_[7] * y + rot_[8] * z + tz_};
        const Point3 point = attitude_ok ? rotate(level, body) : body;
        const double range2 = point.x * point.x + point.y * point.y + point.z * point.z;
        if (point.x * point.x + point.y * point.y <= radius2 &&
          range2 >= min_range2 && range2 <= max_range2)
        {
          points.push_back(point);
        }
      }
    }

    Result result = Evaluate(points, params_);
    if (!attitude_ok && use_imu_ && imu_required_) {
      result.status = Status::kUnknown;
      result.reasons |= kImuUnavailable;
      result.slope_deg = std::numeric_limits<double>::quiet_NaN();
    }
    publishHeightMap(cloud.header, result);
    publishStatus(cloud.header, result, started);
  }

  void publishStatus(
    const std_msgs::Header & header, const Result & result,
    const std::chrono::steady_clock::time_point & started)
  {
    const auto raw = result.status;
    const auto previous = stable_;
    if (raw == Status::kNotLandable || raw == Status::kUnknown) {
      candidate_ = raw;
      consecutive_ = 0;
      stable_ = raw;
    } else {
      if (raw == candidate_) {
        ++consecutive_;
      } else {
        candidate_ = raw;
        consecutive_ = 1;
      }
      if (consecutive_ >= confirm_frames_) {
        stable_ = raw;
      }
    }
    uint32_t reasons = result.reasons;
    if (stable_ != raw) {
      reasons |= kStabilizing;
    }
    LandingStatus output;
    output.header = header;
    output.header.frame_id = output_frame_;
    output.status = static_cast<uint8_t>(stable_);
    output.raw_status = static_cast<uint8_t>(raw);
    output.status_changed = stable_ != previous;
    output.reason_mask = reasons;
    output.reason = ReasonsToString(reasons);
    output.roi_point_count = static_cast<uint32_t>(result.roi_point_count);
    output.slope_deg = result.slope_deg;
    output.roughness_rms = result.roughness_rms;
    output.roughness_p95 = result.roughness_p95;
    output.max_obstacle_height = result.max_obstacle_height;
    output.occupied_cell_ratio = result.occupied_cell_ratio;
    output.max_height_step = result.max_height_step;
    output.coverage_ratio = result.coverage_ratio;
    output.inlier_ratio = result.inlier_ratio;
    output.plane_height = result.plane_height;
    output.processing_time_ms = std::chrono::duration<float, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    status_pub_.publish(output);
    if (stable_ == Status::kNotLandable && !denial_logged_) {
      ROS_WARN("Landing denied: %s (slope=%.2f deg, coverage=%.2f, inliers=%.2f)",
        output.reason.c_str(), output.slope_deg, output.coverage_ratio, output.inlier_ratio);
      denial_logged_ = true;
    }
    updateLandableLog(stable_, output);
  }

  void publishHeightMap(const std_msgs::Header & header, const Result & result)
  {
    HeightMap map;
    map.header = header;
    map.header.frame_id = output_frame_;
    map.resolution = params_.grid_resolution;
    map.width = static_cast<uint32_t>(result.map_width);
    map.height = static_cast<uint32_t>(result.map_height);
    map.origin_x = result.map_origin_x;
    map.origin_y = result.map_origin_y;
    map.no_data_value = std::numeric_limits<float>::quiet_NaN();
    map.plane_a = result.plane_a;
    map.plane_b = result.plane_b;
    map.plane_c = result.plane_c;
    map.plane_d = result.plane_d;
    map.ground_height = result.ground_height;
    map.highest_point = result.highest_point;
    map.airspace_intrusion = result.airspace_intrusion;
    map.point_count = result.cell_point_count;
    map.ground_point_count = result.ground_point_count;
    map.airspace_occupied = result.airspace_occupied;
    height_map_pub_.publish(map);
  }

  void updateLandableLog(Status status, const LandingStatus & result)
  {
    const auto now = std::chrono::steady_clock::now();
    if (status != Status::kLandable) {
      landable_since_valid_ = false;
      landable_logged_ = false;
      return;
    }
    if (!landable_since_valid_) {
      landable_since_ = now;
      landable_since_valid_ = true;
      landable_logged_ = false;
    }
    const double elapsed = std::chrono::duration<double>(now - landable_since_).count();
    if (!landable_logged_ && elapsed >= landable_log_delay_sec_) {
      ROS_INFO("LANDABLE continuously for %.1f s (slope=%.2f, coverage=%.2f, inliers=%.2f)",
        elapsed, result.slope_deg, result.coverage_ratio, result.inlier_ratio);
      landable_logged_ = true;
      denial_logged_ = false;
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber cloud_sub_, enable_sub_, imu_sub_;
  ros::Publisher status_pub_, height_map_pub_;
  ros::ServiceServer enable_service_;
  ros::Timer watchdog_timer_;
  Parameters params_;
  std::string input_topic_, output_topic_, height_map_topic_, enable_topic_, output_frame_, imu_topic_;
  int confirm_frames_{3}, consecutive_{0};
  double landable_log_delay_sec_{5.0}, cloud_timeout_sec_{0.30};
  double watchdog_publish_period_sec_{1.0};
  bool startup_enabled_{true}, use_imu_{false}, imu_required_{true};
  Status candidate_{Status::kUnknown}, stable_{Status::kUnknown};
  std::chrono::steady_clock::time_point landable_since_{};
  bool landable_since_valid_{false}, landable_logged_{false}, denial_logged_{false};
  double tx_{0}, ty_{0}, tz_{0};
  std::array<double, 9> rot_{}, imu_rot_{};
  double imu_timeout_sec_{0.15}, imu_filter_alpha_{0.12};
  double imu_gravity_norm_filter_alpha_{0.01}, imu_max_gyro_{0.8};
  double imu_max_accel_deviation_{0.30}, gravity_norm_{0};
  bool imu_valid_{false};
  Point3 filtered_up_{0, 0, 1};
  std::chrono::steady_clock::time_point imu_receive_time_{};
  std::chrono::steady_clock::time_point last_cloud_receive_{}, last_watchdog_publish_{};
  bool timeout_active_{false};
};
}  // namespace landing_evaluator

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "landing_evaluator");
  try {
    landing_evaluator::LandingEvaluatorNode node;
    ros::spin();
  } catch (const std::exception & exception) {
    ROS_FATAL("Landing evaluator startup failed: %s", exception.what());
    std::cerr << "Landing evaluator startup failed: " << exception.what() << std::endl;
    return 1;
  }
  return 0;
}
