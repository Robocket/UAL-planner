#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include <livox_avia_driver/CustomMsg.h>

namespace livox_avia_driver
{

constexpr uint32_t kPointStep = 24U;

sensor_msgs::PointField pointField(
  const std::string & name, uint32_t offset, uint8_t datatype)
{
  sensor_msgs::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1U;
  return field;
}

std::vector<sensor_msgs::PointField> pointFields()
{
  using sensor_msgs::PointField;
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

class CustomToPointCloud2Node
{
public:
  CustomToPointCloud2Node()
  : nh_(), pnh_("~")
  {
    pnh_.param<std::string>("input_topic", input_topic_, "/livox/lidar");
    pnh_.param<std::string>("output_topic", output_topic_, "/livox/lidar/points");
    pnh_.param<std::string>("frame_id", frame_id_, "");
    int queue_depth = 1;
    std::string output_reliability;
    pnh_.param("qos_depth", queue_depth, 1);
    pnh_.param<std::string>("output_reliability", output_reliability, "reliable");

    if (input_topic_.empty() || output_topic_.empty()) {
      throw std::invalid_argument("input_topic and output_topic must not be empty");
    }
    if (input_topic_ == output_topic_) {
      throw std::invalid_argument(
              "CustomMsg input and PointCloud2 output must use different topic names");
    }
    if (queue_depth <= 0) {
      throw std::invalid_argument("qos_depth must be greater than zero");
    }
    if (output_reliability != "reliable" && output_reliability != "best_effort") {
      throw std::invalid_argument(
              "output_reliability must be reliable or best_effort");
    }
    if (output_reliability == "best_effort") {
      ROS_WARN("ROS 1 has no DDS best-effort QoS; using the standard ROS TCP transport");
    }

    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, queue_depth);
    subscription_ = nh_.subscribe(
      input_topic_, queue_depth, &CustomToPointCloud2Node::convertAndPublish, this,
      ros::TransportHints().tcpNoDelay());

    ROS_INFO(
      "Converting Livox CustomMsg %s -> PointCloud2 %s",
      input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void convertAndPublish(const CustomMsg::ConstPtr & message)
  {
    const auto & input = *message;
    if (input.points.size() > std::numeric_limits<uint32_t>::max()) {
      ROS_ERROR_THROTTLE(5.0, "CustomMsg contains too many points");
      return;
    }
    if (input.point_num != input.points.size()) {
      ROS_WARN_THROTTLE(
        5.0,
        "CustomMsg point_num=%u differs from points.size=%zu; using points.size",
        input.point_num, input.points.size());
    }

    sensor_msgs::PointCloud2 output;
    output.header = input.header;
    if (!frame_id_.empty()) {
      output.header.frame_id = frame_id_;
    }
    output.height = 1U;
    output.width = static_cast<uint32_t>(input.points.size());
    output.fields = pointFields();
    output.is_bigendian = false;
    output.point_step = kPointStep;
    output.row_step = output.point_step * output.width;
    output.is_dense = false;
    output.data.resize(output.row_step);

    for (size_t index = 0; index < input.points.size(); ++index) {
      const auto & source = input.points[index];
      uint8_t * destination = output.data.data() + index * output.point_step;
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

    publisher_.publish(output);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  ros::Publisher publisher_;
  ros::Subscriber subscription_;
};

}  // namespace livox_avia_driver

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "livox_custom_to_pointcloud2");
  try {
    livox_avia_driver::CustomToPointCloud2Node node;
    ros::spin();
  } catch (const std::exception & exception) {
    ROS_FATAL("Livox adapter startup failed: %s", exception.what());
    return 1;
  }
  return 0;
}
