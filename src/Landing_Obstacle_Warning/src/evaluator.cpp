#include "landing_evaluator/evaluator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <sstream>

namespace landing_evaluator
{
namespace
{
struct Plane {double a{0}, b{0}, c{1}, d{0};};
double Dot(const Plane & p, const Point3 & q) {return p.a*q.x+p.b*q.y+p.c*q.z+p.d;}

bool FromThree(const Point3 & p, const Point3 & q, const Point3 & r, Plane & out)
{
  const double ux=q.x-p.x, uy=q.y-p.y, uz=q.z-p.z;
  const double vx=r.x-p.x, vy=r.y-p.y, vz=r.z-p.z;
  double a=uy*vz-uz*vy, b=uz*vx-ux*vz, c=ux*vy-uy*vx;
  const double n=std::sqrt(a*a+b*b+c*c);
  if (n < 1e-9) return false;
  a/=n; b/=n; c/=n;
  if (c < 0) {a=-a; b=-b; c=-c;}
  out={a,b,c,-(a*p.x+b*p.y+c*p.z)};
  return true;
}

// Jacobi rotations for the smallest eigenvector of a symmetric 3x3 covariance.
std::array<double, 3> SmallestEigenvector(std::array<std::array<double,3>,3> a)
{
  std::array<std::array<double,3>,3> v{{{{1,0,0}},{{0,1,0}},{{0,0,1}}}};
  for (int it=0; it<24; ++it) {
    int p=0,q=1;
    if (std::abs(a[0][2])>std::abs(a[p][q])) {p=0;q=2;}
    if (std::abs(a[1][2])>std::abs(a[p][q])) {p=1;q=2;}
    if (std::abs(a[p][q])<1e-12) break;
    const double phi=0.5*std::atan2(2*a[p][q],a[q][q]-a[p][p]);
    const double c=std::cos(phi), s=std::sin(phi);
    for (int k=0;k<3;++k) {const double ap=a[k][p], aq=a[k][q];a[k][p]=c*ap-s*aq;a[k][q]=s*ap+c*aq;}
    for (int k=0;k<3;++k) {const double ap=a[p][k], aq=a[q][k];a[p][k]=c*ap-s*aq;a[q][k]=s*ap+c*aq;}
    for (int k=0;k<3;++k) {const double vp=v[k][p], vq=v[k][q];v[k][p]=c*vp-s*vq;v[k][q]=s*vp+c*vq;}
  }
  int e=0; if(a[1][1]<a[e][e])e=1; if(a[2][2]<a[e][e])e=2;
  return {v[0][e],v[1][e],v[2][e]};
}

Plane Refine(const std::vector<Point3> & pts, const std::vector<size_t> & ids)
{
  Point3 c{0,0,0};
  for(auto i:ids){c.x+=pts[i].x;c.y+=pts[i].y;c.z+=pts[i].z;}
  const double n=static_cast<double>(ids.size()); c.x/=n;c.y/=n;c.z/=n;
  std::array<std::array<double,3>,3> m{};
  for(auto i:ids){const double x=pts[i].x-c.x,y=pts[i].y-c.y,z=pts[i].z-c.z;
    m[0][0]+=x*x;m[0][1]+=x*y;m[0][2]+=x*z;m[1][1]+=y*y;m[1][2]+=y*z;m[2][2]+=z*z;}
  m[1][0]=m[0][1];m[2][0]=m[0][2];m[2][1]=m[1][2];
  auto e=SmallestEigenvector(m); if(e[2]<0){e[0]*=-1;e[1]*=-1;e[2]*=-1;}
  return {e[0],e[1],e[2],-(e[0]*c.x+e[1]*c.y+e[2]*c.z)};
}
}

Result Evaluate(const std::vector<Point3> & points, const Parameters & p)
{
  Result out;
  std::vector<Point3> roi; roi.reserve(points.size());
  const double radius2=p.landing_radius*p.landing_radius;
  const double min_range2=p.min_range*p.min_range,max_range2=p.max_range*p.max_range;
  for(const auto & q:points){const double range2=q.x*q.x+q.y*q.y+q.z*q.z;
    if(std::isfinite(range2)&&range2>=min_range2&&range2<=max_range2&&q.x*q.x+q.y*q.y<=radius2)roi.push_back(q);}
  out.roi_point_count=roi.size();
  const double requested_side=std::max(1.0,std::ceil(2*p.landing_radius/p.grid_resolution));
  if(requested_side*requested_side>p.max_grid_cells){out.reasons=kGridTooLarge;return out;}
  const int side=static_cast<int>(requested_side);
  struct Cell{int count{0},ground_count{0},airspace_points{0};double ground_z_sum{0},residual_sum{0},highest_z{-std::numeric_limits<double>::infinity()},max_intrusion{0};};
  std::vector<Cell> grid(side*side);
  out.map_width=side;out.map_height=side;out.map_origin_x=-p.landing_radius;out.map_origin_y=-p.landing_radius;
  const float nan=std::numeric_limits<float>::quiet_NaN();const size_t cell_total=static_cast<size_t>(side)*side;
  out.ground_height.assign(cell_total,nan);out.highest_point.assign(cell_total,nan);out.airspace_intrusion.assign(cell_total,0.0F);
  out.cell_point_count.assign(cell_total,0U);out.ground_point_count.assign(cell_total,0U);out.airspace_occupied.assign(cell_total,0U);
  for(const auto & q:roi){int x=static_cast<int>((q.x+p.landing_radius)/p.grid_resolution),y=static_cast<int>((q.y+p.landing_radius)/p.grid_resolution);
    if(x<0||y<0||x>=side||y>=side)continue;auto & c=grid[y*side+x];++c.count;c.highest_z=std::max(c.highest_z,q.z);}
  for(size_t i=0;i<grid.size();++i){out.cell_point_count[i]=grid[i].count;if(grid[i].count)out.highest_point[i]=grid[i].highest_z;}
  if(roi.size()<static_cast<size_t>(std::max(3,p.min_roi_points))){out.reasons=kTooFewPoints;return out;}

  std::mt19937 rng(p.ransac_seed); std::uniform_int_distribution<size_t> pick(0,roi.size()-1);
  Plane best; size_t best_count=0;
  for(int it=0;it<p.ransac_iterations;++it){size_t i=pick(rng),j=pick(rng),k=pick(rng);Plane candidate;
    if(i==j||j==k||i==k||!FromThree(roi[i],roi[j],roi[k],candidate))continue;
    size_t count=0;for(const auto & q:roi)if(std::abs(Dot(candidate,q))<=p.ransac_distance)++count;
    if(count>best_count){best=candidate;best_count=count;}
    if(best_count>=static_cast<size_t>(p.ransac_early_exit_ratio*roi.size()))break;}
  if(best_count<3){out.reasons=kNoPlane;return out;}
  std::vector<size_t> best_ids;best_ids.reserve(best_count);
  for(size_t n=0;n<roi.size();++n)if(std::abs(Dot(best,roi[n]))<=p.ransac_distance)best_ids.push_back(n);
  best=Refine(roi,best_ids);
  out.plane_a=best.a;out.plane_b=best.b;out.plane_c=best.c;out.plane_d=best.d;
  // Reclassify after refinement.
  best_ids.clear();
  for(size_t n=0;n<roi.size();++n)if(std::abs(Dot(best,roi[n]))<=p.ransac_distance)best_ids.push_back(n);
  if(best_ids.size()<3){out.reasons=kNoPlane;return out;}
  out.inlier_ratio=static_cast<double>(best_ids.size())/roi.size();
  out.slope_deg=std::acos(std::clamp(std::abs(best.c),0.0,1.0))*180.0/M_PI;
  out.plane_height=std::abs(best.c)>1e-6 ? -best.d/best.c : 0.0;
  std::vector<double> errors;errors.reserve(best_ids.size());double squares=0;
  for(auto i:best_ids){double e=std::abs(Dot(best,roi[i]));errors.push_back(e);squares+=e*e;}
  out.roughness_rms=std::sqrt(squares/std::max<size_t>(1,errors.size()));
  const size_t p95_index=std::min(errors.size()-1,static_cast<size_t>(0.95*errors.size()));
  std::nth_element(errors.begin(),errors.begin()+p95_index,errors.end());out.roughness_p95=errors[p95_index];

  for(const auto & q:roi){int x=static_cast<int>((q.x+p.landing_radius)/p.grid_resolution),y=static_cast<int>((q.y+p.landing_radius)/p.grid_resolution);
    if(x<0||y<0||x>=side||y>=side)continue;auto & c=grid[y*side+x];const double h=Dot(best,q);
    if(std::abs(h)<=p.ransac_distance){++c.ground_count;c.ground_z_sum+=q.z;c.residual_sum+=h;}
    if(h>p.airspace_clearance_height&&q.z<=p.airspace_ceiling_z){++c.airspace_points;c.max_intrusion=std::max(c.max_intrusion,h);out.max_obstacle_height=std::max(out.max_obstacle_height,h);}}
  int expected=0,valid=0,occupied_cells=0;bool obstacle=false;
  for(int y=0;y<side;++y)for(int x=0;x<side;++x){double cx=-p.landing_radius+(x+0.5)*p.grid_resolution,cy=-p.landing_radius+(y+0.5)*p.grid_resolution;
    const size_t i=static_cast<size_t>(y)*side+x;if(cx*cx+cy*cy>radius2)continue;++expected;const auto & c=grid[i];
    if(c.ground_count>=p.grid_min_points)++valid;if(c.airspace_points>=p.airspace_min_points){++occupied_cells;out.airspace_occupied[i]=1U;}
    if(c.ground_count){out.ground_height[i]=c.ground_z_sum/c.ground_count;out.ground_point_count[i]=c.ground_count;}
    out.airspace_intrusion[i]=c.max_intrusion;}
  out.coverage_ratio=expected?static_cast<double>(valid)/expected:0;
  out.occupied_cell_ratio=expected?static_cast<double>(occupied_cells)/expected:0;obstacle=out.occupied_cell_ratio>p.max_occupied_cell_ratio;
  // Compare discontinuities relative to the fitted support plane. Raw Z
  // differences also count the expected height trend of a safe slope.
  // Keep this metric on the same circular domain as coverage and occupancy.
  for(int y=0;y<side;++y)for(int x=0;x<side;++x){
    const double cx=-p.landing_radius+(x+0.5)*p.grid_resolution;
    const double cy=-p.landing_radius+(y+0.5)*p.grid_resolution;
    const auto & a=grid[y*side+x];
    if(cx*cx+cy*cy>radius2||a.ground_count<p.grid_min_points)continue;
    for(auto d:std::array<std::array<int,2>,2>{{{{1,0}},{{0,1}}}}){
      const int nx=x+d[0],ny=y+d[1];if(nx>=side||ny>=side)continue;
      const double ncx=-p.landing_radius+(nx+0.5)*p.grid_resolution;
      const double ncy=-p.landing_radius+(ny+0.5)*p.grid_resolution;
      const auto & b=grid[ny*side+nx];
      if(ncx*ncx+ncy*ncy>radius2||b.ground_count<p.grid_min_points)continue;
      out.max_height_step=std::max(out.max_height_step,std::abs(
        a.residual_sum/a.ground_count-b.residual_sum/b.ground_count));}}

  if(out.coverage_ratio<p.min_coverage_ratio)out.reasons|=kLowCoverage;
  if(out.slope_deg>p.max_slope_deg)out.reasons|=kSlope;
  if(out.roughness_rms>p.max_roughness_rms||out.roughness_p95>p.max_roughness_p95)out.reasons|=kRoughness;
  if(obstacle)out.reasons|=kObstacle;
  if(out.max_height_step>p.max_step_height)out.reasons|=kStep;
  if(out.inlier_ratio<p.min_inlier_ratio)out.reasons|=kLowInlierRatio;
  const uint32_t hazards=kSlope|kRoughness|kObstacle|kStep|kLowInlierRatio;
  out.status=(out.reasons&hazards)?Status::kNotLandable:((out.reasons&kLowCoverage)?Status::kUnknown:Status::kLandable);
  return out;
}

std::string ReasonsToString(uint32_t r)
{
  if(r==kNone)return "none";std::vector<std::string> s;
  if(r&kTooFewPoints)s.emplace_back("too_few_points");if(r&kNoPlane)s.emplace_back("no_plane");if(r&kLowCoverage)s.emplace_back("low_coverage");
  if(r&kStabilizing)s.emplace_back("stabilizing");if(r&kSlope)s.emplace_back("slope");if(r&kRoughness)s.emplace_back("roughness");
  if(r&kObstacle)s.emplace_back("airspace_occupied");if(r&kStep)s.emplace_back("step");if(r&kLowInlierRatio)s.emplace_back("low_inlier_ratio");
  if(r&kImuUnavailable)s.emplace_back("imu_unavailable");
  if(r&kCloudTimeout)s.emplace_back("cloud_timeout");
  if(r&kGridTooLarge)s.emplace_back("grid_too_large");
  std::ostringstream os;for(size_t i=0;i<s.size();++i){if(i)os<<',';os<<s[i];}return os.str();
}
}  // namespace landing_evaluator
