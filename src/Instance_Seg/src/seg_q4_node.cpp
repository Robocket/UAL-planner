#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <curl/curl.h>
#include <cv_bridge/cv_bridge.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "ins_seg/msg/seg_info.hpp"
#include "ins_seg/msg/segmentation_result.hpp"
#include "sam3.h"

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

namespace
{

double elapsed_ms(const Clock::time_point & begin, const Clock::time_point & end)
{
  return std::chrono::duration<double, std::milli>(end - begin).count();
}

diagnostic_msgs::msg::KeyValue diagnostic_value(
  const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

std::string default_model_path()
{
  const char * cache_home = std::getenv("XDG_CACHE_HOME");
  if (cache_home != nullptr && cache_home[0] != '\0') {
    return (fs::path(cache_home) / "ual_planner" / "models" /
      "sam3-q4_0.ggml").string();
  }
  const char * user_home = std::getenv("HOME");
  if (user_home == nullptr || user_home[0] == '\0') {
    throw std::runtime_error(
            "无法确定 Q4_0 缓存目录：HOME 和 XDG_CACHE_HOME 均未设置");
  }
  return (fs::path(user_home) / ".cache" / "ual_planner" / "models" /
    "sam3-q4_0.ggml").string();
}

}  // namespace

class Sam3Q4Segmentation final : public rclcpp::Node
{
public:
  Sam3Q4Segmentation()
  : Node("sam3_segmentation")
  {
    declare_parameter<std::string>("image_topic", "/camera/color/image_raw");
    declare_parameter<std::string>("result_topic", "/sam3/segmentation");
    declare_parameter<std::string>("annotated_topic", "/sam3/annotated_image");
    declare_parameter<std::string>("performance_topic", "/sam3/performance");
    declare_parameter<std::string>("text_prompt", "large ship");
    declare_parameter<double>("confidence_threshold", 0.5);
    declare_parameter<double>("min_mask_area_ratio", 0.0);
    declare_parameter<int>("max_instances", 20);
    declare_parameter<double>("tracking_iou_threshold", 0.3);
    declare_parameter<int>("max_track_age_frames", 5);
    declare_parameter<bool>("publish_annotated", true);
    declare_parameter<double>("overlay_alpha", 0.45);
    declare_parameter<int>("qos_depth", 1);
    declare_parameter<std::string>(
      "q4_model_url",
      "https://huggingface.co/PABannier/sam3.cpp/resolve/main/"
      "sam3-q4_0.ggml");
    declare_parameter<std::string>("q4_model_path", "");
    declare_parameter<int64_t>("q4_model_size_bytes", 706606590);
    declare_parameter<bool>("q4_auto_download", true);
    declare_parameter<int>("q4_num_threads", 8);
    declare_parameter<bool>("q4_use_gpu", false);
    declare_parameter<double>("q4_nms_threshold", 0.1);
    declare_parameter<double>("performance_warn_latency_ms", 1000.0);
    declare_parameter<double>("performance_ema_alpha", 0.2);

    image_topic_ = get_parameter("image_topic").as_string();
    text_prompt_ = get_parameter("text_prompt").as_string();
    confidence_threshold_ = get_parameter("confidence_threshold").as_double();
    min_mask_area_ratio_ = get_parameter("min_mask_area_ratio").as_double();
    max_instances_ = static_cast<int>(get_parameter("max_instances").as_int());
    tracking_iou_threshold_ = get_parameter("tracking_iou_threshold").as_double();
    max_track_age_frames_ = static_cast<int>(
      get_parameter("max_track_age_frames").as_int());
    publish_annotated_ = get_parameter("publish_annotated").as_bool();
    overlay_alpha_ = get_parameter("overlay_alpha").as_double();
    q4_nms_threshold_ = get_parameter("q4_nms_threshold").as_double();
    performance_warn_latency_ms_ =
      get_parameter("performance_warn_latency_ms").as_double();
    performance_ema_alpha_ = get_parameter("performance_ema_alpha").as_double();
    validate_parameters();

    std::string model_path = get_parameter("q4_model_path").as_string();
    if (model_path.empty()) {
      model_path = default_model_path();
    }
    ensure_model(model_path);

    sam3_params params;
    params.model_path = model_path;
    params.n_threads = static_cast<int>(get_parameter("q4_num_threads").as_int());
    params.use_gpu = get_parameter("q4_use_gpu").as_bool();
    RCLCPP_INFO(
      get_logger(), "正在加载 SAM 3 Q4_0: %s (threads=%d, gpu=%s)",
      model_path.c_str(), params.n_threads, params.use_gpu ? "true" : "false");
    const auto load_started = Clock::now();
    model_ = sam3_load_model(params);
    if (!model_) {
      throw std::runtime_error("sam3.cpp 无法加载 Q4_0 模型");
    }
    if (sam3_is_visual_only(*model_)) {
      throw std::runtime_error("配置的模型不含文本编码器，不能执行 large ship 检测");
    }
    state_ = sam3_create_state(*model_, params);
    if (!state_) {
      throw std::runtime_error("sam3.cpp 无法创建推理状态");
    }
    RCLCPP_INFO(
      get_logger(), "SAM 3 Q4_0 加载完成，耗时 %.1f s",
      elapsed_ms(load_started, Clock::now()) / 1000.0);

    const auto depth = static_cast<size_t>(get_parameter("qos_depth").as_int());
    result_pub_ = create_publisher<ins_seg::msg::SegmentationResult>(
      get_parameter("result_topic").as_string(), depth);
    annotated_pub_ = create_publisher<sensor_msgs::msg::Image>(
      get_parameter("annotated_topic").as_string(), depth);
    performance_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      get_parameter("performance_topic").as_string(), depth);
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS().keep_last(depth),
      std::bind(&Sam3Q4Segmentation::image_callback, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "SAM 3 Q4_0 节点已启动: topic=%s, prompt=\"%s\"",
      image_topic_.c_str(), text_prompt_.c_str());
  }

private:
  struct Track
  {
    int id;
    cv::Mat mask;
    int age;
  };

  struct DownloadProgress
  {
    rclcpp::Logger logger;
    int last_percent{-1};
  };

  void validate_parameters() const
  {
    if (text_prompt_.empty()) {
      throw std::invalid_argument("text_prompt 不能为空");
    }
    for (const auto & item : std::vector<std::pair<std::string, double>>{
        {"confidence_threshold", confidence_threshold_},
        {"min_mask_area_ratio", min_mask_area_ratio_},
        {"tracking_iou_threshold", tracking_iou_threshold_},
        {"overlay_alpha", overlay_alpha_},
        {"q4_nms_threshold", q4_nms_threshold_}})
    {
      if (item.second < 0.0 || item.second > 1.0) {
        throw std::invalid_argument(item.first + " 必须在 [0, 1] 范围内");
      }
    }
    if (max_instances_ <= 0 || max_track_age_frames_ < 0) {
      throw std::invalid_argument("实例数量和轨迹寿命参数无效");
    }
    if (get_parameter("qos_depth").as_int() <= 0 ||
      get_parameter("q4_num_threads").as_int() <= 0)
    {
      throw std::invalid_argument("qos_depth 和 q4_num_threads 必须大于 0");
    }
  }

  static size_t write_download(void * data, size_t size, size_t count, void * stream)
  {
    return std::fwrite(data, size, count, static_cast<FILE *>(stream));
  }

  static int download_progress(
    void * client, curl_off_t total, curl_off_t current,
    curl_off_t, curl_off_t)
  {
    if (total <= 0) {
      return 0;
    }
    auto * progress = static_cast<DownloadProgress *>(client);
    const int percent = static_cast<int>(100 * current / total);
    if (percent >= progress->last_percent + 10 || percent == 100) {
      progress->last_percent = percent;
      RCLCPP_INFO(progress->logger, "Q4_0 下载进度: %d%%", percent);
    }
    return 0;
  }

  void ensure_model(const std::string & model_path)
  {
    const auto expected_size = static_cast<uintmax_t>(
      get_parameter("q4_model_size_bytes").as_int());
    std::error_code error;
    if (fs::is_regular_file(model_path, error) &&
      fs::file_size(model_path, error) == expected_size)
    {
      RCLCPP_INFO(get_logger(), "使用已缓存的 SAM 3 Q4_0: %s", model_path.c_str());
      return;
    }
    if (!get_parameter("q4_auto_download").as_bool()) {
      throw std::runtime_error("Q4_0 模型不存在且 q4_auto_download=false: " + model_path);
    }
    fs::create_directories(fs::path(model_path).parent_path());
    const std::string temporary_path = model_path + ".part";
    FILE * output = std::fopen(temporary_path.c_str(), "wb");
    if (output == nullptr) {
      throw std::runtime_error("无法创建模型临时文件: " + temporary_path);
    }
    CURL * curl = curl_easy_init();
    if (curl == nullptr) {
      std::fclose(output);
      throw std::runtime_error("libcurl 初始化失败");
    }
    const auto url = get_parameter("q4_model_url").as_string();
    DownloadProgress progress{get_logger()};
    RCLCPP_INFO(
      get_logger(), "首次运行自动下载 SAM 3 Q4_0 (约 674 MiB): %s", url.c_str());
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl, CURLOPT_FAILONERROR, 1L);
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "UAL-planner/0.1");
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &write_download);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, output);
    curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 0L);
    curl_easy_setopt(curl, CURLOPT_XFERINFOFUNCTION, &download_progress);
    curl_easy_setopt(curl, CURLOPT_XFERINFODATA, &progress);
    const CURLcode result = curl_easy_perform(curl);
    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
    curl_easy_cleanup(curl);
    std::fclose(output);
    if (result != CURLE_OK) {
      fs::remove(temporary_path, error);
      throw std::runtime_error(
              "Q4_0 下载失败 (HTTP " + std::to_string(http_code) + "): " +
              curl_easy_strerror(result));
    }
    const auto actual_size = fs::file_size(temporary_path, error);
    if (error || actual_size != expected_size) {
      fs::remove(temporary_path, error);
      throw std::runtime_error(
              "Q4_0 文件大小校验失败，期望 " + std::to_string(expected_size) +
              "，实际 " + std::to_string(actual_size));
    }
    fs::rename(temporary_path, model_path);
    RCLCPP_INFO(get_logger(), "Q4_0 下载并校验完成: %s", model_path.c_str());
  }

  static double mask_iou(const cv::Mat & first, const cv::Mat & second)
  {
    if (first.size() != second.size()) {
      return 0.0;
    }
    cv::Mat intersection;
    cv::Mat union_mask;
    cv::bitwise_and(first, second, intersection);
    cv::bitwise_or(first, second, union_mask);
    const double union_pixels = cv::countNonZero(union_mask);
    return union_pixels > 0.0 ? cv::countNonZero(intersection) / union_pixels : 0.0;
  }

  std::vector<int> assign_track_ids(const std::vector<cv::Mat> & masks)
  {
    struct Candidate {double iou; size_t detection; size_t track;};
    std::vector<Candidate> candidates;
    for (size_t detection = 0; detection < masks.size(); ++detection) {
      for (size_t track = 0; track < tracks_.size(); ++track) {
        const double iou = mask_iou(masks[detection], tracks_[track].mask);
        if (iou >= tracking_iou_threshold_) {
          candidates.push_back({iou, detection, track});
        }
      }
    }
    std::sort(
      candidates.begin(), candidates.end(),
      [](const Candidate & lhs, const Candidate & rhs) {return lhs.iou > rhs.iou;});
    std::vector<int> assignments(masks.size(), -1);
    std::vector<bool> used_tracks(tracks_.size(), false);
    for (const auto & candidate : candidates) {
      if (assignments[candidate.detection] < 0 && !used_tracks[candidate.track]) {
        assignments[candidate.detection] = tracks_[candidate.track].id;
        used_tracks[candidate.track] = true;
      }
    }
    for (auto & assignment : assignments) {
      if (assignment < 0) {
        assignment = next_track_id_++;
      }
    }
    std::vector<Track> next_tracks;
    for (size_t i = 0; i < masks.size(); ++i) {
      next_tracks.push_back({assignments[i], masks[i].clone(), 0});
    }
    for (size_t i = 0; i < tracks_.size(); ++i) {
      if (!used_tracks[i] && tracks_[i].age < max_track_age_frames_) {
        auto old_track = tracks_[i];
        ++old_track.age;
        next_tracks.push_back(std::move(old_track));
      }
    }
    tracks_ = std::move(next_tracks);
    return assignments;
  }

  void publish_performance(
    size_t detection_count, double encode_ms, double segment_ms,
    double publish_ms, double total_ms)
  {
    if (latency_ema_ms_ <= 0.0) {
      latency_ema_ms_ = total_ms;
    } else {
      latency_ema_ms_ = performance_ema_alpha_ * total_ms +
        (1.0 - performance_ema_alpha_) * latency_ema_ms_;
    }
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "sam3_segmentation/performance";
    status.hardware_id = "CPU / sam3.cpp Q4_0";
    status.level = total_ms > performance_warn_latency_ms_ ?
      diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = status.level == diagnostic_msgs::msg::DiagnosticStatus::WARN ?
      "inference latency above configured limit" : "ok";
    status.values = {
      diagnostic_value("frame_index", std::to_string(processed_frames_)),
      diagnostic_value("detections", std::to_string(detection_count)),
      diagnostic_value("encode_ms", std::to_string(encode_ms)),
      diagnostic_value("segment_ms", std::to_string(segment_ms)),
      diagnostic_value("publish_ms", std::to_string(publish_ms)),
      diagnostic_value("total_ms", std::to_string(total_ms)),
      diagnostic_value("effective_fps", std::to_string(1000.0 / latency_ema_ms_)),
      diagnostic_value("quantization", "q4_0"),
      diagnostic_value("backend", "sam3.cpp")};
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now();
    message.status.push_back(std::move(status));
    performance_pub_->publish(message);
  }

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    const auto started = Clock::now();
    try {
      const auto cv_image = cv_bridge::toCvCopy(
        message, sensor_msgs::image_encodings::BGR8);
      cv::Mat rgb;
      cv::cvtColor(cv_image->image, rgb, cv::COLOR_BGR2RGB);
      sam3_image input;
      input.width = rgb.cols;
      input.height = rgb.rows;
      input.channels = 3;
      input.data.assign(rgb.datastart, rgb.dataend);

      const auto encode_started = Clock::now();
      if (!sam3_encode_image(*state_, *model_, input)) {
        throw std::runtime_error("sam3.cpp 图像编码失败");
      }
      const auto encode_done = Clock::now();
      sam3_pcs_params pcs;
      pcs.text_prompt = text_prompt_;
      pcs.score_threshold = static_cast<float>(confidence_threshold_);
      pcs.nms_threshold = static_cast<float>(q4_nms_threshold_);
      auto result = sam3_segment_pcs(*state_, *model_, pcs);
      const auto segment_done = Clock::now();
      std::sort(
        result.detections.begin(), result.detections.end(),
        [](const sam3_detection & lhs, const sam3_detection & rhs) {
          return lhs.score > rhs.score;
        });

      std::vector<cv::Mat> masks;
      std::vector<float> scores;
      const double image_area = static_cast<double>(rgb.rows * rgb.cols);
      for (const auto & detection : result.detections) {
        if (masks.size() >= static_cast<size_t>(max_instances_)) {
          break;
        }
        const auto & source = detection.mask;
        if (source.width <= 0 || source.height <= 0 ||
          source.data.size() != static_cast<size_t>(source.width * source.height))
        {
          continue;
        }
        cv::Mat mask(source.height, source.width, CV_8UC1,
          const_cast<uint8_t *>(source.data.data()));
        cv::Mat normalized;
        if (mask.size() != rgb.size()) {
          cv::resize(mask, normalized, rgb.size(), 0.0, 0.0, cv::INTER_NEAREST);
        } else {
          normalized = mask.clone();
        }
        cv::threshold(normalized, normalized, 0, 255, cv::THRESH_BINARY);
        if (cv::countNonZero(normalized) / image_area < min_mask_area_ratio_) {
          continue;
        }
        masks.push_back(std::move(normalized));
        scores.push_back(detection.score);
      }
      const auto track_ids = assign_track_ids(masks);

      cv::Mat id_map(rgb.rows, rgb.cols, CV_32SC1, cv::Scalar(0));
      cv::Mat annotated = cv_image->image.clone();
      ins_seg::msg::SegmentationResult output;
      output.header = message->header;
      for (int i = static_cast<int>(masks.size()) - 1; i >= 0; --i) {
        id_map.setTo(track_ids[static_cast<size_t>(i)], masks[static_cast<size_t>(i)]);
      }
      for (size_t i = 0; i < masks.size(); ++i) {
        ins_seg::msg::SegInfo info;
        info.track_id = track_ids[i];
        info.class_name = text_prompt_;
        info.confidence = scores[i];
        output.instances.push_back(info);
        if (publish_annotated_) {
          const cv::Scalar color(
            (37 * track_ids[i] + 53) % 205 + 50,
            (79 * track_ids[i] + 31) % 205 + 50,
            (131 * track_ids[i] + 17) % 205 + 50);
          cv::Mat color_layer(annotated.size(), annotated.type(), color);
          cv::Mat blended;
          cv::addWeighted(
            annotated, 1.0 - overlay_alpha_, color_layer, overlay_alpha_, 0.0, blended);
          blended.copyTo(annotated, masks[i]);
          std::vector<std::vector<cv::Point>> contours;
          cv::findContours(masks[i], contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
          cv::drawContours(annotated, contours, -1, color, 2);
        }
      }
      output.instance_id_map = *cv_bridge::CvImage(
        message->header, sensor_msgs::image_encodings::TYPE_32SC1, id_map).toImageMsg();
      if (publish_annotated_) {
        output.annotated_image = *cv_bridge::CvImage(
          message->header, sensor_msgs::image_encodings::BGR8, annotated).toImageMsg();
        annotated_pub_->publish(output.annotated_image);
      }
      result_pub_->publish(output);
      const auto publish_done = Clock::now();
      ++processed_frames_;
      const double total_ms = elapsed_ms(started, publish_done);
      publish_performance(
        masks.size(), elapsed_ms(encode_started, encode_done),
        elapsed_ms(encode_done, segment_done),
        elapsed_ms(segment_done, publish_done), total_ms);
      RCLCPP_INFO(
        get_logger(), "SAM 3 Q4_0 第 %zu 帧: %zu 个实例, total=%.0f ms "
        "(encode=%.0f, segment=%.0f), EMA FPS=%.3f",
        processed_frames_, masks.size(), total_ms,
        elapsed_ms(encode_started, encode_done), elapsed_ms(encode_done, segment_done),
        1000.0 / latency_ema_ms_);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(get_logger(), "SAM 3 Q4_0 图像处理失败: %s", exception.what());
    }
  }

  std::string image_topic_;
  std::string text_prompt_;
  double confidence_threshold_{0.5};
  double min_mask_area_ratio_{0.0};
  int max_instances_{20};
  double tracking_iou_threshold_{0.3};
  int max_track_age_frames_{5};
  bool publish_annotated_{true};
  double overlay_alpha_{0.45};
  double q4_nms_threshold_{0.1};
  double performance_warn_latency_ms_{1000.0};
  double performance_ema_alpha_{0.2};
  double latency_ema_ms_{0.0};
  size_t processed_frames_{0};
  int next_track_id_{1};
  std::vector<Track> tracks_;
  std::shared_ptr<sam3_model> model_;
  sam3_state_ptr state_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<ins_seg::msg::SegmentationResult>::SharedPtr result_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr performance_pub_;
};

int main(int argc, char ** argv)
{
  curl_global_init(CURL_GLOBAL_DEFAULT);
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Sam3Q4Segmentation>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("sam3_segmentation"), "%s", exception.what());
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  curl_global_cleanup();
  return 0;
}
