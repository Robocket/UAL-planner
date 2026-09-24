#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

#include "livox_avia_driver/msg/custom_msg.hpp"

namespace livox_avia_driver
{

constexpr uint32_t kPointStep = 24U;

sensor_msgs::msg::PointField pointField(
  const std::string & name, uint32_t offset, uint8_t datatype)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1U;
  return field;
}

std::vector<sensor_msgs::msg::PointField> pointFields()
{
  using sensor_msgs::msg::PointField;
  return {
    pointField("x", 0U, PointField::FLOAT32),
    pointField("y", 4U, PointField::FLOAT32),
    pointField("z", 8U, PointField::FLOAT32),
    pointField("intensity", 12U, PointField::FLOAT32),
    pointField("offset_time", 16U, PointField::UINT32),
    pointField("tag", 20U, PointField::UINT8),
    pointField("line", 21U, PointField::UINT8),
  };
}

class CustomToPointCloud2Node : public rclcpp::Node
{
public:
  CustomToPointCloud2Node()
  : Node("livox_custom_to_pointcloud2")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/livox/lidar");
    output_topic_ = declare_parameter<std::string>("output_topic", "/livox/lidar/points");
    frame_id_ = declare_parameter<std::string>("frame_id", "");
    const auto qos_depth = declare_parameter<int>("qos_depth", 1);
    const auto output_reliability = declare_parameter<std::string>(
      "output_reliability", "reliable");

    if (input_topic_.empty() || output_topic_.empty()) {
      throw std::invalid_argument("input_topic and output_topic must not be empty");
    }
    if (input_topic_ == output_topic_) {
      throw std::invalid_argument(
              "CustomMsg input and PointCloud2 output must use different topic names");
    }
    if (qos_depth <= 0) {
      throw std::invalid_argument("qos_depth must be greater than zero");
    }
    if (output_reliability != "reliable" && output_reliability != "best_effort") {
      throw std::invalid_argument(
              "output_reliability must be reliable or best_effort");
    }

    auto input_qos = rclcpp::QoS(
      rclcpp::KeepLast(static_cast<size_t>(qos_depth)));
    input_qos.best_effort().durability_volatile();
    auto output_qos = rclcpp::QoS(
      rclcpp::KeepLast(static_cast<size_t>(qos_depth)));
    if (output_reliability == "reliable") {
      output_qos.reliable();
    } else {
      output_qos.best_effort();
    }
    output_qos.durability_volatile();
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, output_qos);
    subscription_ = create_subscription<msg::CustomMsg>(
      input_topic_, input_qos,
      [this](msg::CustomMsg::ConstSharedPtr message) {convertAndPublish(*message);});

    RCLCPP_INFO(
      get_logger(),
      "Converting Livox CustomMsg %s -> PointCloud2 %s (%s output)",
      input_topic_.c_str(), output_topic_.c_str(), output_reliability.c_str());
  }

private:
  void convertAndPublish(const msg::CustomMsg & input)
  {
    if (input.points.size() > std::numeric_limits<uint32_t>::max()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "CustomMsg contains too many points");
      return;
    }
    if (input.point_num != input.points.size()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "CustomMsg point_num=%u differs from points.size=%zu; using points.size",
        input.point_num, input.points.size());
    }

    auto output = std::make_unique<sensor_msgs::msg::PointCloud2>();
    output->header = input.header;
    if (!frame_id_.empty()) {
      output->header.frame_id = frame_id_;
    }
    output->height = 1U;
    output->width = static_cast<uint32_t>(input.points.size());
    output->fields = pointFields();
    output->is_bigendian = false;
    output->point_step = kPointStep;
    output->row_step = output->point_step * output->width;
    output->is_dense = false;
    output->data.resize(output->row_step);

    for (size_t index = 0; index < input.points.size(); ++index) {
      const auto & source = input.points[index];
      uint8_t * destination = output->data.data() + index * output->point_step;
      const float intensity = static_cast<float>(source.reflectivity);
      std::memcpy(destination + 0U, &source.x, sizeof(source.x));
      std::memcpy(destination + 4U, &source.y, sizeof(source.y));
      std::memcpy(destination + 8U, &source.z, sizeof(source.z));
      std::memcpy(destination + 12U, &intensity, sizeof(intensity));
      std::memcpy(
        destination + 16U, &source.offset_time, sizeof(source.offset_time));
      std::memcpy(destination + 20U, &source.tag, sizeof(source.tag));
      std::memcpy(destination + 21U, &source.line, sizeof(source.line));
    }

    publisher_->publish(std::move(output));
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<msg::CustomMsg>::SharedPtr subscription_;
};

}  // namespace livox_avia_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<livox_avia_driver::CustomToPointCloud2Node>());
  rclcpp::shutdown();
  return 0;
}
