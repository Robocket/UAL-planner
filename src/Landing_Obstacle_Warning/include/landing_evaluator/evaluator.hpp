#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace landing_evaluator
{

struct Point3 {double x; double y; double z;};

struct Parameters
{
  double landing_radius{0.7};
  double min_range{0.3};
  double max_range{30.0};
  int min_roi_points{100};
  int ransac_iterations{60};
  uint32_t ransac_seed{0x4c414e44U};
  double ransac_distance{0.06};
  double ransac_early_exit_ratio{0.90};
  double max_slope_deg{8.0};
  double max_roughness_rms{0.05};
  double max_roughness_p95{0.08};
  double airspace_clearance_height{0.12};
  double airspace_ceiling_z{-0.10};
  int airspace_min_points{3};
  double max_occupied_cell_ratio{0.0};
  double max_step_height{0.10};
  double grid_resolution{0.15};
  int max_grid_cells{1000000};
  int grid_min_points{2};
  double min_coverage_ratio{0.80};
  double min_inlier_ratio{0.70};
};

enum class Status : uint8_t {kUnknown = 0, kLandable = 1, kNotLandable = 2};

enum Reason : uint32_t
{
  kNone = 0, kTooFewPoints = 1U, kNoPlane = 2U, kLowCoverage = 4U,
  kStabilizing = 8U, kSlope = 16U, kRoughness = 32U, kObstacle = 64U,
  kStep = 128U, kLowInlierRatio = 256U, kImuUnavailable = 512U,
  kCloudTimeout = 1024U, kGridTooLarge = 2048U
};

struct Result
{
  Status status{Status::kUnknown};
  uint32_t reasons{kNone};
  size_t roi_point_count{0};
  double slope_deg{0.0};
  double roughness_rms{0.0};
  double roughness_p95{0.0};
  double max_obstacle_height{0.0};
  double occupied_cell_ratio{0.0};
  double max_height_step{0.0};
  double coverage_ratio{0.0};
  double inlier_ratio{0.0};
  double plane_height{0.0};
  double plane_a{0.0};
  double plane_b{0.0};
  double plane_c{0.0};
  double plane_d{0.0};
  int map_width{0};
  int map_height{0};
  double map_origin_x{0.0};
  double map_origin_y{0.0};
  std::vector<float> ground_height;
  std::vector<float> highest_point;
  std::vector<float> airspace_intrusion;
  std::vector<uint32_t> cell_point_count;
  std::vector<uint32_t> ground_point_count;
  std::vector<uint8_t> airspace_occupied;
};

Result Evaluate(const std::vector<Point3> & points, const Parameters & params);
std::string ReasonsToString(uint32_t reasons);

}  // namespace landing_evaluator
