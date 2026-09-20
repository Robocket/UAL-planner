#include <gtest/gtest.h>

#include <cmath>

#include "landing_evaluator/evaluator.hpp"

using landing_evaluator::Evaluate;using landing_evaluator::Parameters;using landing_evaluator::Point3;using landing_evaluator::Status;

static std::vector<Point3> Plane(double slope=0.0)
{
  std::vector<Point3> p;for(double x=-0.65;x<=0.65;x+=0.05)for(double y=-0.65;y<=0.65;y+=0.05)if(x*x+y*y<0.65*0.65)p.push_back({x,y,-2.0+slope*x});return p;
}
TEST(Evaluator,FlatPlaneIsLandable){Parameters p;p.min_coverage_ratio=0.7;auto r=Evaluate(Plane(),p);EXPECT_EQ(r.status,Status::kLandable);EXPECT_LT(r.slope_deg,0.1);}
TEST(Evaluator,RejectsSteepPlane){Parameters p;p.min_coverage_ratio=0.7;auto r=Evaluate(Plane(0.25),p);EXPECT_EQ(r.status,Status::kNotLandable);EXPECT_TRUE(r.reasons&landing_evaluator::kSlope);}
TEST(Evaluator,TooFewPointsIsUnknown){Parameters p;auto r=Evaluate({{0,0,-2}},p);EXPECT_EQ(r.status,Status::kUnknown);EXPECT_TRUE(r.reasons&landing_evaluator::kTooFewPoints);}
TEST(Evaluator,AirspaceIntrusionDeniesLanding){Parameters p;p.min_coverage_ratio=0.7;auto cloud=Plane();
  cloud.push_back({0.01,0.01,-1.50});cloud.push_back({0.02,0.01,-1.49});cloud.push_back({0.01,0.02,-1.51});
  auto r=Evaluate(cloud,p);EXPECT_EQ(r.status,Status::kNotLandable);EXPECT_TRUE(r.reasons&landing_evaluator::kObstacle);EXPECT_GT(r.occupied_cell_ratio,0.0);}
TEST(Evaluator,ProducesHeightMap){Parameters p;p.min_coverage_ratio=0.7;auto r=Evaluate(Plane(),p);
  EXPECT_GT(r.map_width,0);EXPECT_EQ(r.ground_height.size(),static_cast<size_t>(r.map_width*r.map_height));EXPECT_EQ(r.airspace_occupied.size(),r.ground_height.size());}
TEST(Evaluator,StepMetricRemovesFittedSlopeTrend){Parameters p;p.min_coverage_ratio=0.7;p.max_slope_deg=10.0;p.max_step_height=0.002;
  const double slope=std::tan(5.0*M_PI/180.0);auto r=Evaluate(Plane(slope),p);
  EXPECT_EQ(r.status,Status::kLandable);EXPECT_NEAR(r.slope_deg,5.0,0.1);EXPECT_LT(r.max_height_step,p.max_step_height);}
TEST(Evaluator,RejectsOversizedGridWithoutAllocating){Parameters p;p.grid_resolution=0.001;p.max_grid_cells=100;
  auto r=Evaluate(Plane(),p);EXPECT_EQ(r.status,Status::kUnknown);EXPECT_TRUE(r.reasons&landing_evaluator::kGridTooLarge);}
