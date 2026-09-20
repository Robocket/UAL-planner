#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <limits>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "landing_evaluator/evaluator.hpp"
#include "landing_evaluator/msg/landing_status.hpp"
#include "landing_evaluator/msg/height_map.hpp"

namespace landing_evaluator
{
class LandingEvaluatorNode : public rclcpp::Node
{
public:
  LandingEvaluatorNode() : Node("landing_evaluator")
  {
    input_topic_=declare_parameter("input_topic",std::string("/livox/lidar"));
    output_topic_=declare_parameter("output_topic",std::string("/landing/status"));
    height_map_topic_=declare_parameter("height_map_topic",std::string("/landing/height_map"));
    enable_topic_=declare_parameter("enable_topic",std::string("/landing_evaluator/enable"));
    output_frame_=declare_parameter("output_frame",std::string("base_link"));
    use_imu_=declare_parameter("imu.enabled",false);
    imu_required_=declare_parameter("imu.required",true);
    imu_topic_=declare_parameter("imu.topic",std::string("/livox/imu"));
    imu_timeout_sec_=declare_parameter("imu.timeout_sec",0.15);
    imu_filter_alpha_=declare_parameter("imu.filter_alpha",0.12);
    imu_gravity_norm_filter_alpha_=declare_parameter("imu.gravity_norm_filter_alpha",0.01);
    imu_max_gyro_=declare_parameter("imu.max_angular_velocity",0.8);
    imu_max_accel_deviation_=declare_parameter("imu.max_accel_deviation_ratio",0.30);
    params_.landing_radius=declare_parameter("landing_radius",params_.landing_radius);
    params_.min_range=declare_parameter("min_range",params_.min_range);params_.max_range=declare_parameter("max_range",params_.max_range);
    params_.min_roi_points=declare_parameter("min_roi_points",params_.min_roi_points);params_.ransac_iterations=declare_parameter("ransac_iterations",params_.ransac_iterations);
    const auto ransac_seed=declare_parameter<int64_t>("ransac_seed",params_.ransac_seed);
    params_.ransac_distance=declare_parameter("ransac_distance",params_.ransac_distance);params_.ransac_early_exit_ratio=declare_parameter("ransac_early_exit_ratio",params_.ransac_early_exit_ratio);params_.max_slope_deg=declare_parameter("max_slope_deg",params_.max_slope_deg);
    params_.max_roughness_rms=declare_parameter("max_roughness_rms",params_.max_roughness_rms);params_.max_roughness_p95=declare_parameter("max_roughness_p95",params_.max_roughness_p95);
    params_.airspace_clearance_height=declare_parameter("airspace_clearance_height",params_.airspace_clearance_height);
    params_.airspace_ceiling_z=declare_parameter("airspace_ceiling_z",params_.airspace_ceiling_z);
    params_.airspace_min_points=declare_parameter("airspace_min_points",params_.airspace_min_points);
    params_.max_occupied_cell_ratio=declare_parameter("max_occupied_cell_ratio",params_.max_occupied_cell_ratio);
    params_.max_step_height=declare_parameter("max_step_height",params_.max_step_height);params_.grid_resolution=declare_parameter("grid_resolution",params_.grid_resolution);
    params_.max_grid_cells=declare_parameter("max_grid_cells",params_.max_grid_cells);
    params_.grid_min_points=declare_parameter("grid_min_points",params_.grid_min_points);params_.min_coverage_ratio=declare_parameter("min_coverage_ratio",params_.min_coverage_ratio);
    params_.min_inlier_ratio=declare_parameter("min_inlier_ratio",params_.min_inlier_ratio);
    confirm_frames_=static_cast<int>(std::max<int64_t>(1,declare_parameter("confirmation_frames",3)));
    landable_log_delay_sec_=declare_parameter("landable_log_delay_sec",5.0);
    cloud_timeout_sec_=declare_parameter("watchdog.cloud_timeout_sec",0.30);
    watchdog_publish_period_sec_=declare_parameter("watchdog.publish_period_sec",1.0);
    startup_enabled_=declare_parameter("startup_enabled",true);
    const auto xyz=declare_parameter("extrinsics.translation",std::vector<double>{0,0,0});
    const auto rpy=declare_parameter("extrinsics.rotation_rpy",std::vector<double>{0,0,0});
    const auto imu_rpy=declare_parameter("imu.extrinsics.rotation_rpy",rpy);
    if(xyz.size()!=3||rpy.size()!=3||imu_rpy.size()!=3)throw std::invalid_argument("extrinsics and imu.extrinsics rotation_rpy must each contain 3 values");
    if(ransac_seed<0||ransac_seed>std::numeric_limits<uint32_t>::max())
      throw std::invalid_argument("ransac_seed must be in the uint32 range");
    params_.ransac_seed=static_cast<uint32_t>(ransac_seed);
    if(params_.landing_radius<=0||params_.min_range<0||params_.max_range<=params_.min_range||
      params_.min_roi_points<3||params_.grid_resolution<=0||params_.max_grid_cells<=0||params_.ransac_distance<=0||
      params_.ransac_iterations<=0||params_.ransac_early_exit_ratio<=0||params_.ransac_early_exit_ratio>1||
      params_.grid_min_points<=0||params_.airspace_min_points<=0||
      params_.airspace_clearance_height<=0||params_.max_occupied_cell_ratio<0||params_.max_occupied_cell_ratio>1||
      params_.max_slope_deg<0||params_.max_slope_deg>90||params_.max_roughness_rms<0||params_.max_roughness_p95<0||
      params_.max_step_height<0||params_.min_coverage_ratio<0||params_.min_coverage_ratio>1||
      params_.min_inlier_ratio<0||params_.min_inlier_ratio>1||landable_log_delay_sec_<0||
      imu_timeout_sec_<=0||imu_filter_alpha_<=0||imu_filter_alpha_>1||
      imu_gravity_norm_filter_alpha_<=0||imu_gravity_norm_filter_alpha_>1||
      imu_max_gyro_<=0||imu_max_accel_deviation_<=0||
      cloud_timeout_sec_<=0||watchdog_publish_period_sec_<=0)
      throw std::invalid_argument("invalid landing evaluator parameter value");
    tx_=xyz[0];ty_=xyz[1];tz_=xyz[2];makeRotation(rpy[0],rpy[1],rpy[2],rot_);makeRotation(imu_rpy[0],imu_rpy[1],imu_rpy[2],imu_rot_);
    pub_=create_publisher<msg::LandingStatus>(output_topic_,rclcpp::QoS(10));
    height_map_pub_=create_publisher<msg::HeightMap>(height_map_topic_,rclcpp::QoS(2));
    enable_service_=create_service<std_srvs::srv::SetBool>("~/set_enabled",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr request,std_srvs::srv::SetBool::Response::SharedPtr response){setEnabled(request->data,*response);});
    enable_sub_=create_subscription<std_msgs::msg::Bool>(enable_topic_,rclcpp::QoS(1).reliable(),
      [this](std_msgs::msg::Bool::ConstSharedPtr command){applyEnabled(command->data,"topic");});
    if(use_imu_){
      auto imu_qos=rclcpp::QoS(rclcpp::KeepLast(1));
      imu_qos.best_effort().durability_volatile();
      imu_sub_=create_subscription<sensor_msgs::msg::Imu>(imu_topic_,imu_qos,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr m){onImu(*m);});
    }
    if(startup_enabled_)createCloudSubscription();
    const auto watchdog_period=std::chrono::milliseconds(std::max(10,static_cast<int>(1000.0*std::min(0.1,cloud_timeout_sec_/2.0))));
    watchdog_timer_=create_wall_timer(watchdog_period,[this](){onWatchdog();});
    RCLCPP_INFO(get_logger(),"Landing evaluator %s; input %s, frame %s, radius %.2f m, radar IMU %s (%s)",startup_enabled_?"enabled":"disabled",input_topic_.c_str(),output_frame_.c_str(),params_.landing_radius,use_imu_?"enabled":"disabled",imu_topic_.c_str());
  }
private:
  void createCloudSubscription()
  {
    if(sub_)return;
    last_cloud_receive_=std::chrono::steady_clock::now();timeout_active_=false;
    // Keep only one transport-level cloud while the current frame is being
    // evaluated.  A deeper queue makes a CPU-bound evaluator process stale
    // clouds and appear to lag behind the live sensor.
    auto cloud_qos=rclcpp::QoS(rclcpp::KeepLast(1));
    cloud_qos.best_effort().durability_volatile();
    sub_=create_subscription<sensor_msgs::msg::PointCloud2>(input_topic_,cloud_qos,[this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m){onCloudReceived(m);});
  }
  void setEnabled(bool enable,std_srvs::srv::SetBool::Response & response)
  {
    response.success=true;response.message=applyEnabled(enable,"service");
  }
  std::string applyEnabled(bool enable,const char * source)
  {
    if(enable==static_cast<bool>(sub_))return enable?"already enabled":"already disabled";
    if(enable){resetState();createCloudSubscription();RCLCPP_INFO(get_logger(),"Landing evaluation enabled by %s",source);}
    else{sub_.reset();resetState();RCLCPP_INFO(get_logger(),"Landing evaluation disabled by %s",source);}
    return enable?"enabled":"disabled";
  }
  void resetState()
  {
    candidate_=Status::kUnknown;stable_=Status::kUnknown;consecutive_=0;landable_since_valid_=false;landable_logged_=false;denial_logged_=false;
  }
  static void makeRotation(double roll,double pitch,double yaw,std::array<double,9> & out)
  {
    const double cr=cos(roll),sr=sin(roll),cp=cos(pitch),sp=sin(pitch),cy=cos(yaw),sy=sin(yaw);
    out={cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr,sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr,-sp,cp*sr,cp*cr};
  }
  static Point3 rotate(const std::array<double,9> & r,const Point3 & p)
  {return {r[0]*p.x+r[1]*p.y+r[2]*p.z,r[3]*p.x+r[4]*p.y+r[5]*p.z,r[6]*p.x+r[7]*p.y+r[8]*p.z};}
  static bool normalize(Point3 & p)
  {const double n=std::sqrt(p.x*p.x+p.y*p.y+p.z*p.z);if(!std::isfinite(n)||n<1e-6)return false;p.x/=n;p.y/=n;p.z/=n;return true;}
  void onImu(const sensor_msgs::msg::Imu & imu)
  {
    const Point3 sensor_accel{imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z};
    const Point3 sensor_gyro{imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z};
    Point3 up=rotate(imu_rot_,sensor_accel);const double norm=std::sqrt(up.x*up.x+up.y*up.y+up.z*up.z);
    const Point3 gyro_body=rotate(imu_rot_,sensor_gyro);
    const double gyro=std::sqrt(gyro_body.x*gyro_body.x+gyro_body.y*gyro_body.y+gyro_body.z*gyro_body.z);
    if(!std::isfinite(norm)||norm<1e-3||!std::isfinite(gyro))return;
    if(gravity_norm_<=0)gravity_norm_=norm;
    const double deviation=std::abs(norm-gravity_norm_)/gravity_norm_;
    if(gyro>imu_max_gyro_||deviation>imu_max_accel_deviation_){
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,
        "Rejecting radar IMU sample during motion (gyro=%.2f rad/s, accel deviation=%.2f); gravity direction held",gyro,deviation);
      return;
    }
    gravity_norm_=(1.0-imu_gravity_norm_filter_alpha_)*gravity_norm_+
      imu_gravity_norm_filter_alpha_*norm;up.x/=norm;up.y/=norm;up.z/=norm;
    if(!imu_valid_){filtered_up_=up;imu_valid_=true;}
    else {filtered_up_.x=(1-imu_filter_alpha_)*filtered_up_.x+imu_filter_alpha_*up.x;filtered_up_.y=(1-imu_filter_alpha_)*filtered_up_.y+imu_filter_alpha_*up.y;filtered_up_.z=(1-imu_filter_alpha_)*filtered_up_.z+imu_filter_alpha_*up.z;
      const double n=std::sqrt(filtered_up_.x*filtered_up_.x+filtered_up_.y*filtered_up_.y+filtered_up_.z*filtered_up_.z);filtered_up_.x/=n;filtered_up_.y/=n;filtered_up_.z/=n;}
    // Use local monotonic arrival time. Some recorded Livox bags contain a
    // different clock epoch in PointCloud2 and Imu headers.
    const auto now=std::chrono::steady_clock::now();
    latest_imu_sample_={filtered_up_,now,true};imu_receive_time_=now;
  }
  std::array<double,9> levelingRotation(const Point3 & up) const
  {
    // Rodrigues rotation that maps measured aircraft-up onto gravity-frame +Z.
    const double x=up.x,y=up.y,z=up.z;
    if(z<-0.999999)return {-1,0,0,0,1,0,0,0,-1};
    const double k=1.0/(1.0+z);
    return {1-x*x*k,-x*y*k,-x,y==0?0:-x*y*k,1-y*y*k,-y,x,y,z};
  }
  static int fieldOffset(const sensor_msgs::msg::PointCloud2 & m,const std::string & name)
  {for(const auto & f:m.fields)if(f.name==name&&f.datatype==sensor_msgs::msg::PointField::FLOAT32)return static_cast<int>(f.offset);return -1;}
  void onWatchdog()
  {
    if(!sub_)return;
    const auto now_steady=std::chrono::steady_clock::now();
    if(std::chrono::duration<double>(now_steady-last_cloud_receive_).count()<cloud_timeout_sec_)return;
    const bool first=!timeout_active_;
    if(!first&&std::chrono::duration<double>(now_steady-last_watchdog_publish_).count()<watchdog_publish_period_sec_)return;
    const auto previous=stable_;candidate_=Status::kUnknown;stable_=Status::kUnknown;consecutive_=0;
    landable_since_valid_=false;landable_logged_=false;denial_logged_=false;timeout_active_=true;last_watchdog_publish_=now_steady;
    msg::LandingStatus out;out.header.stamp=get_clock()->now();out.header.frame_id=output_frame_;
    out.status=msg::LandingStatus::UNKNOWN;out.raw_status=msg::LandingStatus::UNKNOWN;
    out.status_changed=first&&previous!=Status::kUnknown;out.reason_mask=kCloudTimeout;out.reason=ReasonsToString(kCloudTimeout);
    const float nan=std::numeric_limits<float>::quiet_NaN();out.slope_deg=nan;out.roughness_rms=nan;out.roughness_p95=nan;
    out.max_obstacle_height=nan;out.occupied_cell_ratio=nan;out.max_height_step=nan;out.coverage_ratio=0.0F;
    out.inlier_ratio=0.0F;out.plane_height=nan;out.processing_time_ms=0.0F;pub_->publish(out);
    if(first)RCLCPP_WARN(get_logger(),"Point cloud timeout after %.2f s; publishing UNKNOWN",cloud_timeout_sec_);
  }
  struct ImuSample{Point3 up{0,0,1};std::chrono::steady_clock::time_point time{};bool valid{false};};
  void onCloudReceived(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
  {
    const auto now=std::chrono::steady_clock::now();last_cloud_receive_=now;
    if(timeout_active_){timeout_active_=false;RCLCPP_INFO(get_logger(),"Point cloud stream recovered");}
    if(!use_imu_){processCloud(*cloud,false,Point3{0,0,1});return;}
    // Process every cloud immediately with the latest received IMU sample.
    // No cloud or IMU history is retained, so this path has bounded memory and
    // never waits for a future sensor message.
    if(!latest_imu_sample_.valid){processCloud(*cloud,false,Point3{0,0,1});return;}
    processCloud(*cloud,true,latest_imu_sample_.up);
  }
  void processCloud(const sensor_msgs::msg::PointCloud2 & cloud,bool imu_interpolated,const Point3 & interpolated_up)
  {
    const auto processing_start=std::chrono::steady_clock::now();
    const int ox=fieldOffset(cloud,"x"),oy=fieldOffset(cloud,"y"),oz=fieldOffset(cloud,"z");
    if(ox<0||oy<0||oz<0||cloud.point_step==0||std::max({ox,oy,oz})+4>static_cast<int>(cloud.point_step)){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"PointCloud2 requires valid FLOAT32 x/y/z fields");return;}
    std::vector<Point3> points;
    const auto now_steady=std::chrono::steady_clock::now();
    const bool imu_ok=use_imu_&&imu_interpolated&&imu_valid_&&std::chrono::duration<double>(now_steady-imu_receive_time_).count()<=imu_timeout_sec_;
    Point3 corrected_up{0,0,1};bool attitude_ok=false;
    if(imu_ok){corrected_up=interpolated_up;attitude_ok=true;}
    else if(!use_imu_){attitude_ok=true;}
    std::array<double,9> level{1,0,0,0,1,0,0,0,1};
    if(attitude_ok&&use_imu_)level=levelingRotation(corrected_up);
    const double radius2=params_.landing_radius*params_.landing_radius;
    const double min_range2=params_.min_range*params_.min_range,max_range2=params_.max_range*params_.max_range;
    for(uint32_t row=0;row<cloud.height;++row)for(uint32_t col=0;col<cloud.width;++col){const size_t pos=static_cast<size_t>(row)*cloud.row_step+static_cast<size_t>(col)*cloud.point_step;
      if(pos+cloud.point_step>cloud.data.size())continue;float x,y,z;std::memcpy(&x,&cloud.data[pos+ox],4);std::memcpy(&y,&cloud.data[pos+oy],4);std::memcpy(&z,&cloud.data[pos+oz],4);
      if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z))continue;
      const Point3 body{rot_[0]*x+rot_[1]*y+rot_[2]*z+tx_,rot_[3]*x+rot_[4]*y+rot_[5]*z+ty_,rot_[6]*x+rot_[7]*y+rot_[8]*z+tz_};
      const Point3 q=attitude_ok?rotate(level,body):body;
      const double range2=q.x*q.x+q.y*q.y+q.z*q.z;
      if(q.x*q.x+q.y*q.y<=radius2&&range2>=min_range2&&range2<=max_range2)points.push_back(q);}
    Result result=Evaluate(points,params_);
    if(!attitude_ok&&(use_imu_&&imu_required_)){
      result.status=Status::kUnknown;result.reasons|=kImuUnavailable;
      // The fitted plane is expressed in the tilted body frame in this case;
      // publishing that angle as a valid ground slope is misleading.
      result.slope_deg=std::numeric_limits<double>::quiet_NaN();
    }
    const auto raw=result.status;const auto previous=stable_;
    publishHeightMap(cloud.header,result);
    // Safety state machine: hazards and missing evidence take effect immediately.
    // Only entry into LANDABLE is debounced over multiple consecutive frames.
    if(raw==Status::kNotLandable){candidate_=raw;consecutive_=0;stable_=raw;}
    else if(raw==Status::kUnknown){candidate_=raw;consecutive_=0;stable_=raw;}
    else {if(raw==candidate_)++consecutive_;else{candidate_=raw;consecutive_=1;}if(consecutive_>=confirm_frames_)stable_=raw;}
    uint32_t reasons=result.reasons;if(stable_!=raw)reasons|=kStabilizing;
    msg::LandingStatus out;out.header=cloud.header;out.header.frame_id=output_frame_;out.status=static_cast<uint8_t>(stable_);out.raw_status=static_cast<uint8_t>(raw);
    out.status_changed=stable_!=previous;out.reason_mask=reasons;out.reason=ReasonsToString(reasons);out.roi_point_count=static_cast<uint32_t>(result.roi_point_count);
    out.slope_deg=result.slope_deg;out.roughness_rms=result.roughness_rms;out.roughness_p95=result.roughness_p95;out.max_obstacle_height=result.max_obstacle_height;out.occupied_cell_ratio=result.occupied_cell_ratio;
    out.max_height_step=result.max_height_step;out.coverage_ratio=result.coverage_ratio;out.inlier_ratio=result.inlier_ratio;out.plane_height=result.plane_height;
    out.processing_time_ms=std::chrono::duration<float,std::milli>(std::chrono::steady_clock::now()-processing_start).count();pub_->publish(out);
    // Log one warning per hazardous episode. Brief internal recoveries do not
    // re-arm it; only a continuously confirmed LANDABLE interval does.
    if(stable_==Status::kNotLandable&&!denial_logged_){
      RCLCPP_WARN(get_logger(),"Landing denied: %s (slope=%.2f deg, coverage=%.2f, inliers=%.2f)",out.reason.c_str(),out.slope_deg,out.coverage_ratio,out.inlier_ratio);
      denial_logged_=true;
    }
    updateLandableLog(stable_,out);
  }
  void publishHeightMap(const std_msgs::msg::Header & input_header,const Result & result)
  {
    msg::HeightMap map;map.header=input_header;map.header.frame_id=output_frame_;map.resolution=params_.grid_resolution;
    map.width=static_cast<uint32_t>(result.map_width);map.height=static_cast<uint32_t>(result.map_height);
    map.origin_x=result.map_origin_x;map.origin_y=result.map_origin_y;map.no_data_value=std::numeric_limits<float>::quiet_NaN();
    map.plane_a=result.plane_a;map.plane_b=result.plane_b;map.plane_c=result.plane_c;map.plane_d=result.plane_d;
    map.ground_height=result.ground_height;map.highest_point=result.highest_point;map.airspace_intrusion=result.airspace_intrusion;
    map.point_count=result.cell_point_count;map.ground_point_count=result.ground_point_count;map.airspace_occupied=result.airspace_occupied;height_map_pub_->publish(map);
  }
  void updateLandableLog(Status status,const msg::LandingStatus & result)
  {
    const auto now=std::chrono::steady_clock::now();
    if(status!=Status::kLandable){landable_since_valid_=false;landable_logged_=false;return;}
    if(!landable_since_valid_){landable_since_=now;landable_since_valid_=true;landable_logged_=false;}
    const double elapsed=std::chrono::duration<double>(now-landable_since_).count();
    if(!landable_logged_&&elapsed>=landable_log_delay_sec_){
      RCLCPP_INFO(get_logger(),"LANDABLE continuously for %.1f s (slope=%.2f deg, coverage=%.2f, inliers=%.2f)",elapsed,result.slope_deg,result.coverage_ratio,result.inlier_ratio);
      landable_logged_=true;
      denial_logged_=false;
    }
  }
  Parameters params_;std::string input_topic_,output_topic_,height_map_topic_,enable_topic_,output_frame_,imu_topic_;int confirm_frames_{3},consecutive_{0};double landable_log_delay_sec_{5.0},cloud_timeout_sec_{0.30},watchdog_publish_period_sec_{1.0};bool startup_enabled_{true},use_imu_{false},imu_required_{true};Status candidate_{Status::kUnknown},stable_{Status::kUnknown};
  std::chrono::steady_clock::time_point landable_since_{};bool landable_since_valid_{false},landable_logged_{false},denial_logged_{false};
  double tx_{0},ty_{0},tz_{0};std::array<double,9> rot_{},imu_rot_{};
  double imu_timeout_sec_{0.15},imu_filter_alpha_{0.12},imu_gravity_norm_filter_alpha_{0.01},imu_max_gyro_{0.8},imu_max_accel_deviation_{0.30},gravity_norm_{0};bool imu_valid_{false};Point3 filtered_up_{0,0,1};std::chrono::steady_clock::time_point imu_receive_time_{};ImuSample latest_imu_sample_{};
  std::chrono::steady_clock::time_point last_cloud_receive_{},last_watchdog_publish_{};bool timeout_active_{false};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;rclcpp::Publisher<msg::LandingStatus>::SharedPtr pub_;
  rclcpp::Publisher<msg::HeightMap>::SharedPtr height_map_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};
}  // namespace landing_evaluator

int main(int argc,char ** argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<landing_evaluator::LandingEvaluatorNode>());rclcpp::shutdown();return 0;}
