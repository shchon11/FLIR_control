#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

namespace
{

std::string NormalizeName(std::string value)
{
  std::string normalized;
  normalized.reserve(value.size());

  for (const unsigned char character : value) {
    if (std::isalnum(character) != 0) {
      normalized.push_back(static_cast<char>(std::tolower(character)));
    }
  }

  return normalized;
}

std::string TrimAscii(std::string value)
{
  const auto is_space = [](unsigned char character) {
      return std::isspace(character) != 0;
    };

  value.erase(
    value.begin(),
    std::find_if_not(value.begin(), value.end(), is_space));
  value.erase(
    std::find_if_not(value.rbegin(), value.rend(), is_space).base(),
    value.end());
  return value;
}

std::string StripMatchingQuotes(std::string value)
{
  if (value.size() >= 2U) {
    const bool is_double_quoted = value.front() == '"' && value.back() == '"';
    const bool is_single_quoted = value.front() == '\'' && value.back() == '\'';
    if (is_double_quoted || is_single_quoted) {
      return value.substr(1, value.size() - 2U);
    }
  }

  return value;
}

std::string EscapeYamlDoubleQuoted(std::string value)
{
  std::string escaped;
  escaped.reserve(value.size());
  for (const char character : value) {
    if (character == '\\' || character == '"') {
      escaped.push_back('\\');
    }
    escaped.push_back(character);
  }
  return escaped;
}

std::size_t CountLeadingSpaces(const std::string & value)
{
  return static_cast<std::size_t>(
    std::distance(
      value.begin(),
      std::find_if(value.begin(), value.end(), [](char character) {
        return character != ' ';
      })));
}

std::optional<std::string> MatchYamlMapKey(const std::string & line)
{
  const std::string trimmed = TrimAscii(line);
  if (trimmed.empty() || trimmed[0] == '#' || trimmed.back() != ':') {
    return std::nullopt;
  }

  return StripMatchingQuotes(TrimAscii(trimmed.substr(0, trimmed.size() - 1U)));
}

bool IsYamlMapKey(const std::string & line, const char * key)
{
  const auto map_key = MatchYamlMapKey(line);
  return map_key.has_value() && *map_key == key;
}

std::string FormatDouble(double value)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(10) << value;
  return stream.str();
}

std::string CurrentUtcTimestamp()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm utc_time{};
#if defined(_WIN32)
  gmtime_s(&utc_time, &now_time);
#else
  gmtime_r(&now_time, &utc_time);
#endif

  std::ostringstream stream;
  stream << std::put_time(&utc_time, "%Y-%m-%dT%H:%M:%SZ");
  return stream.str();
}

}  // namespace

namespace
{

constexpr auto kUiPollInterval = std::chrono::milliseconds(10);

}  // namespace

class FlirCameraCalibrationNode : public rclcpp::Node
{
public:
  FlirCameraCalibrationNode()
  : Node("flir_camera_calibration"),
    input_topic_(declare_parameter<std::string>("input_topic", "/image_rgb/compressed")),
    annotated_output_topic_(declare_parameter<std::string>(
        "annotated_output_topic",
        "/calibration/image_annotated/compressed")),
    output_yaml_path_(declare_parameter<std::string>(
        "output_yaml_path",
        "calibration/flir_camera_info.yaml")),
    camera_serial_(declare_parameter<std::string>("camera_serial", "")),
    camera_name_(declare_parameter<std::string>("camera_name", "")),
    frame_id_(declare_parameter<std::string>("frame_id", "")),
    sample_image_dir_(declare_parameter<std::string>(
        "sample_image_dir",
        "calibration/captures")),
    board_cols_(declare_parameter<int>("board_cols", 9)),
    board_rows_(declare_parameter<int>("board_rows", 6)),
    square_size_m_(declare_parameter<double>("square_size_m", 0.024)),
    min_calibration_frames_(declare_parameter<int>("min_calibration_frames", 15)),
    display_window_(declare_parameter<bool>("display_window", true)),
    window_name_(declare_parameter<std::string>("window_name", "FLIR Calibration")),
    preview_scale_(declare_parameter<double>("preview_scale", 1.0)),
    preview_max_width_(declare_parameter<int>("preview_max_width", 640)),
    preview_fast_check_(declare_parameter<bool>("preview_fast_check", true)),
    annotated_jpeg_quality_(declare_parameter<int>("annotated_jpeg_quality", 80)),
    input_qos_reliability_(declare_parameter<std::string>("input_qos_reliability", "best_effort")),
    input_qos_depth_(declare_parameter<int>("input_qos_depth", 10)),
    output_qos_reliability_(declare_parameter<std::string>("output_qos_reliability", "reliable")),
    output_qos_depth_(declare_parameter<int>("output_qos_depth", 10)),
    board_size_(board_cols_, board_rows_)
  {
    if (board_cols_ <= 0 || board_rows_ <= 0) {
      throw std::runtime_error("board_cols and board_rows must both be positive.");
    }

    if (square_size_m_ <= 0.0) {
      throw std::runtime_error("square_size_m must be positive.");
    }

    if (preview_scale_ <= 0.0) {
      throw std::runtime_error("preview_scale must be positive.");
    }

    base_object_points_ = BuildObjectPoints();

    annotated_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      annotated_output_topic_,
      BuildQoS(output_qos_reliability_, output_qos_depth_));

    image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      input_topic_,
      BuildQoS(input_qos_reliability_, input_qos_depth_),
      std::bind(&FlirCameraCalibrationNode::OnCompressedImage, this, std::placeholders::_1));

    if (display_window_) {
      cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    }
    keyboard_timer_ = create_wall_timer(
      kUiPollInterval,
      std::bind(&FlirCameraCalibrationNode::OnKeyboardTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "Calibration node listening on '%s'. Press space to capture when the chessboard is detected, "
      "'c' to calibrate, 'r' to reset samples, and 'q' to quit.",
      input_topic_.c_str());
  }

  ~FlirCameraCalibrationNode() override
  {
    if (display_window_) {
      try {
        cv::destroyWindow(window_name_);
      } catch (...) {
      }
    }
  }

private:
  struct LatestFrameState
  {
    bool has_frame = false;
    bool board_detected = false;
    std::uint64_t received_sequence = 0;
    std::uint64_t processed_sequence = 0;
    sensor_msgs::msg::CompressedImage::ConstSharedPtr message;
    sensor_msgs::msg::CompressedImage::_header_type header;
    cv::Mat annotated_preview_bgr;
    cv::Size image_size;
  };

  std::string NormalizeQoSReliability(const std::string & value) const
  {
    const std::string normalized = NormalizeName(value);
    if (normalized == "besteffort") {
      return "best_effort";
    }

    return "reliable";
  }

  rclcpp::QoS BuildQoS(const std::string & reliability, int depth) const
  {
    rclcpp::QoS qos{rclcpp::KeepLast(static_cast<std::size_t>(std::max(1, depth)))};
    qos.durability_volatile();

    if (NormalizeQoSReliability(reliability) == "best_effort") {
      qos.best_effort();
    } else {
      qos.reliable();
    }

    return qos;
  }

  std::vector<cv::Point3f> BuildObjectPoints() const
  {
    std::vector<cv::Point3f> object_points;
    object_points.reserve(static_cast<std::size_t>(board_cols_ * board_rows_));

    for (int row = 0; row < board_rows_; ++row) {
      for (int col = 0; col < board_cols_; ++col) {
        object_points.emplace_back(
          static_cast<float>(col * square_size_m_),
          static_cast<float>(row * square_size_m_),
          0.0F);
      }
    }

    return object_points;
  }

  void OnCompressedImage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(latest_frame_mutex_);
      latest_frame_.has_frame = true;
      latest_frame_.message = msg;
      latest_frame_.header = msg->header;
      ++latest_frame_.received_sequence;
    }
  }

  cv::Mat DecodeCompressedImage(const sensor_msgs::msg::CompressedImage & msg) const
  {
    if (msg.data.empty()) {
      return cv::Mat();
    }

    const cv::Mat encoded_buffer(
      1,
      static_cast<int>(msg.data.size()),
      CV_8UC1,
      const_cast<std::uint8_t *>(msg.data.data()));
    return cv::imdecode(encoded_buffer, cv::IMREAD_COLOR);
  }

  bool DetectChessboard(
    const cv::Mat & bgr_image,
    std::vector<cv::Point2f> & corners,
    bool fast_check) const
  {
    cv::Mat grayscale;
    cv::cvtColor(bgr_image, grayscale, cv::COLOR_BGR2GRAY);

    int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
    if (fast_check) {
      flags |= cv::CALIB_CB_FAST_CHECK;
    }
    const bool detected = cv::findChessboardCorners(grayscale, board_size_, corners, flags);
    if (!detected) {
      corners.clear();
      return false;
    }

    cv::cornerSubPix(
      grayscale,
      corners,
      cv::Size(11, 11),
      cv::Size(-1, -1),
      cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));

    return true;
  }

  cv::Mat PreparePreviewImage(const cv::Mat & bgr_image) const
  {
    if (preview_max_width_ <= 0 || bgr_image.cols <= preview_max_width_) {
      return bgr_image;
    }

    const double scale = static_cast<double>(preview_max_width_) / static_cast<double>(bgr_image.cols);
    cv::Mat preview;
    cv::resize(bgr_image, preview, cv::Size(), scale, scale, cv::INTER_AREA);
    return preview;
  }

  void DrawOverlay(cv::Mat & image, bool board_detected)
  {
    const std::size_t capture_count = captured_image_points_.size();
    const cv::Scalar status_color = board_detected ? cv::Scalar(0, 220, 0) : cv::Scalar(0, 0, 255);
    DrawOverlayText(
      image,
      board_detected ? "Board detected" : "Board not detected",
      0,
      status_color);
    DrawOverlayText(
      image,
      "Captured frames: " + std::to_string(capture_count),
      1,
      cv::Scalar(255, 255, 255));
    DrawOverlayText(
      image,
      "space:capture  c:calib  r:reset  q:quit",
      2,
      cv::Scalar(255, 255, 255));
  }

  void DrawOverlayText(
    cv::Mat & image,
    const std::string & text,
    int line_index,
    const cv::Scalar & color) const
  {
    constexpr int left_margin = 12;
    constexpr int top_margin = 22;
    constexpr int line_spacing = 24;
    constexpr double base_scale = 0.58;
    constexpr int foreground_thickness = 1;
    constexpr int shadow_thickness = 3;
    int baseline = 0;
    const cv::Size text_size = cv::getTextSize(
      text,
      cv::FONT_HERSHEY_SIMPLEX,
      base_scale,
      foreground_thickness,
      &baseline);
    const int available_width = std::max(1, image.cols - left_margin * 2);
    const double scale = text_size.width <= available_width ?
      base_scale :
      std::max(0.35, base_scale * static_cast<double>(available_width) / static_cast<double>(text_size.width));
    const cv::Point origin(left_margin, top_margin + line_index * line_spacing);
    cv::putText(
      image,
      text,
      origin,
      cv::FONT_HERSHEY_SIMPLEX,
      scale,
      cv::Scalar(0, 0, 0),
      shadow_thickness,
      cv::LINE_AA);
    cv::putText(
      image,
      text,
      origin,
      cv::FONT_HERSHEY_SIMPLEX,
      scale,
      color,
      foreground_thickness,
      cv::LINE_AA);
  }

  void PublishAnnotatedImage(
    const sensor_msgs::msg::CompressedImage::_header_type & header,
    const cv::Mat & annotated_bgr)
  {
    if (annotated_pub_->get_subscription_count() == 0U) {
      return;
    }

    std::vector<std::uint8_t> encoded;
    const std::vector<int> parameters = {
      cv::IMWRITE_JPEG_QUALITY,
      std::clamp(annotated_jpeg_quality_, 0, 100)
    };

    if (!cv::imencode(".jpg", annotated_bgr, encoded, parameters)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Failed to encode annotated calibration preview.");
      return;
    }

    sensor_msgs::msg::CompressedImage output;
    output.header = header;
    output.format = "jpeg";
    output.data = std::move(encoded);
    annotated_pub_->publish(std::move(output));
  }

  void OnKeyboardTimer()
  {
    ProcessLatestFrame();

    LatestFrameState snapshot;
    {
      std::lock_guard<std::mutex> lock(latest_frame_mutex_);
      snapshot = latest_frame_;
    }

    if (!display_window_) {
      return;
    }

    if (!snapshot.has_frame) {
      return;
    }

    cv::Mat display_image = snapshot.annotated_preview_bgr;
    if (display_image.empty()) {
      return;
    }

    if (std::abs(preview_scale_ - 1.0) > 1e-6) {
      cv::resize(
        snapshot.annotated_preview_bgr,
        display_image,
        cv::Size(),
        preview_scale_,
        preview_scale_,
      cv::INTER_LINEAR);
    }

    cv::imshow(window_name_, display_image);

    const int key = cv::waitKey(1) & 0xFF;
    if (key == 0xFF) {
      return;
    }

    switch (key) {
      case ' ':
        CaptureCurrentFrame();
        break;
      case 'c':
      case 'C':
        CalibrateAndSave();
        break;
      case 'r':
      case 'R':
        ResetCapturedFrames();
        break;
      case 'q':
      case 'Q':
      case 27:
        RCLCPP_INFO(get_logger(), "Calibration window requested shutdown.");
        rclcpp::shutdown();
        break;
      default:
        break;
    }
  }

  void ProcessLatestFrame()
  {
    sensor_msgs::msg::CompressedImage::ConstSharedPtr message;
    std::uint64_t sequence = 0;
    {
      std::lock_guard<std::mutex> lock(latest_frame_mutex_);
      if (latest_frame_.message == nullptr ||
        latest_frame_.processed_sequence == latest_frame_.received_sequence)
      {
        return;
      }
      message = latest_frame_.message;
      sequence = latest_frame_.received_sequence;
    }

    ProcessFrame(message, sequence);
  }

  void CaptureCurrentFrame()
  {
    sensor_msgs::msg::CompressedImage::ConstSharedPtr message;
    {
      std::lock_guard<std::mutex> lock(latest_frame_mutex_);
      message = latest_frame_.message;
    }

    if (message == nullptr) {
      RCLCPP_WARN(get_logger(), "No image has been received yet.");
      return;
    }

    const cv::Mat full_resolution_bgr = DecodeCompressedImage(*message);
    if (full_resolution_bgr.empty()) {
      RCLCPP_WARN(get_logger(), "Failed to decode the latest compressed image for capture.");
      return;
    }

    std::vector<cv::Point2f> full_resolution_corners;
    if (!DetectChessboard(full_resolution_bgr, full_resolution_corners, false)) {
      RCLCPP_WARN(get_logger(), "Chessboard not detected in the latest full-resolution frame. Capture skipped.");
      return;
    }

    const cv::Size image_size = full_resolution_bgr.size();

    if (calibration_image_size_.width == 0 || calibration_image_size_.height == 0) {
      calibration_image_size_ = image_size;
    } else if (calibration_image_size_ != image_size) {
      RCLCPP_WARN(
        get_logger(),
        "Image size changed from %dx%d to %dx%d. Capture skipped.",
        calibration_image_size_.width,
        calibration_image_size_.height,
        image_size.width,
        image_size.height);
      return;
    }

    captured_image_points_.push_back(full_resolution_corners);
    captured_object_points_.push_back(base_object_points_);

    if (!sample_image_dir_.empty()) {
      SaveCapturedImage(full_resolution_bgr, captured_image_points_.size());
    }

    RCLCPP_INFO(
      get_logger(),
      "Captured calibration frame %zu. Need at least %d frames before calibration.",
      captured_image_points_.size(),
      min_calibration_frames_);
  }

  void SaveCapturedImage(const cv::Mat & image, std::size_t capture_index)
  {
    try {
      const std::filesystem::path output_dir(sample_image_dir_);
      const std::filesystem::path serial_output_dir =
        camera_serial_.empty() ? output_dir : output_dir / camera_serial_;
      std::filesystem::create_directories(serial_output_dir);

      std::ostringstream filename;
      filename << "capture_" << std::setfill('0') << std::setw(3) << capture_index << ".jpg";
      const std::filesystem::path image_path = serial_output_dir / filename.str();
      cv::imwrite(image_path.string(), image);
    } catch (const std::exception & exception) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to save captured calibration image to '%s': %s",
        sample_image_dir_.c_str(),
        exception.what());
    }
  }

  void CalibrateAndSave()
  {
    if (captured_image_points_.size() < static_cast<std::size_t>(std::max(1, min_calibration_frames_))) {
      RCLCPP_WARN(
        get_logger(),
        "Need at least %d captured frames before calibration. Current count: %zu",
        min_calibration_frames_,
        captured_image_points_.size());
      return;
    }

    if (calibration_image_size_.width <= 0 || calibration_image_size_.height <= 0) {
      RCLCPP_WARN(get_logger(), "Calibration image size is invalid.");
      return;
    }

    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distortion_coefficients;
    std::vector<cv::Mat> rotation_vectors;
    std::vector<cv::Mat> translation_vectors;

    const double rms = cv::calibrateCamera(
      captured_object_points_,
      captured_image_points_,
      calibration_image_size_,
      camera_matrix,
      distortion_coefficients,
      rotation_vectors,
      translation_vectors);

    const double mean_error = ComputeMeanReprojectionError(
      camera_matrix,
      distortion_coefficients,
      rotation_vectors,
      translation_vectors);

    try {
      WriteCalibrationYaml(camera_matrix, distortion_coefficients, rms, mean_error);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        get_logger(),
        "Calibration succeeded but saving '%s' failed: %s",
        output_yaml_path_.c_str(),
        exception.what());
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Calibration finished with RMS %.6f and mean reprojection error %.6f. Saved to '%s'.",
      rms,
      mean_error,
      output_yaml_path_.c_str());
  }

  double ComputeMeanReprojectionError(
    const cv::Mat & camera_matrix,
    const cv::Mat & distortion_coefficients,
    const std::vector<cv::Mat> & rotation_vectors,
    const std::vector<cv::Mat> & translation_vectors) const
  {
    double total_squared_error = 0.0;
    std::size_t total_point_count = 0U;

    for (std::size_t index = 0; index < captured_object_points_.size(); ++index) {
      std::vector<cv::Point2f> projected_points;
      cv::projectPoints(
        captured_object_points_[index],
        rotation_vectors[index],
        translation_vectors[index],
        camera_matrix,
        distortion_coefficients,
        projected_points);

      const double error = cv::norm(captured_image_points_[index], projected_points, cv::NORM_L2);
      total_squared_error += error * error;
      total_point_count += projected_points.size();
    }

    if (total_point_count == 0U) {
      return 0.0;
    }

    return std::sqrt(total_squared_error / static_cast<double>(total_point_count));
  }

  std::vector<double> FlattenMatToDoubles(const cv::Mat & matrix) const
  {
    cv::Mat flattened = matrix.reshape(1, 1);
    cv::Mat as_double;
    flattened.convertTo(as_double, CV_64F);

    std::vector<double> values;
    values.reserve(as_double.total());

    for (int column = 0; column < as_double.cols; ++column) {
      values.push_back(as_double.at<double>(0, column));
    }

    return values;
  }

  std::string FormatYamlList(const std::vector<double> & values) const
  {
    std::ostringstream stream;
    stream << "[";
    for (std::size_t index = 0; index < values.size(); ++index) {
      if (index > 0U) {
        stream << ", ";
      }
      stream << FormatDouble(values[index]);
    }
    stream << "]";
    return stream.str();
  }

  void WriteLegacyCalibrationYaml(
    const cv::Mat & camera_matrix,
    const cv::Mat & distortion_coefficients,
    double rms,
    double mean_error) const
  {
    const std::filesystem::path output_path(output_yaml_path_);
    if (output_path.has_parent_path()) {
      std::filesystem::create_directories(output_path.parent_path());
    }

    const std::vector<double> k = FlattenMatToDoubles(camera_matrix);
    const std::vector<double> d = FlattenMatToDoubles(distortion_coefficients);
    const std::vector<double> r = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0
    };
    const std::vector<double> p = {
      camera_matrix.at<double>(0, 0), 0.0, camera_matrix.at<double>(0, 2), 0.0,
      0.0, camera_matrix.at<double>(1, 1), camera_matrix.at<double>(1, 2), 0.0,
      0.0, 0.0, 1.0, 0.0
    };

    std::ofstream stream(output_path.string());
    if (!stream.is_open()) {
      throw std::runtime_error("Failed to open calibration output file: " + output_yaml_path_);
    }

    stream << "flir_camera:\n";
    stream << "  ros__parameters:\n";
    stream << "    camera_info.distortion_model: \"plumb_bob\"\n";
    stream << "    camera_info.d: " << FormatYamlList(d) << "\n";
    stream << "    camera_info.k: " << FormatYamlList(k) << "\n";
    stream << "    camera_info.r: " << FormatYamlList(r) << "\n";
    stream << "    camera_info.p: " << FormatYamlList(p) << "\n";
    stream << "calibration:\n";
    stream << "  generated_at_utc: \"" << CurrentUtcTimestamp() << "\"\n";
    stream << "  image_width: " << calibration_image_size_.width << "\n";
    stream << "  image_height: " << calibration_image_size_.height << "\n";
    stream << "  board_cols: " << board_cols_ << "\n";
    stream << "  board_rows: " << board_rows_ << "\n";
    stream << "  square_size_m: " << FormatDouble(square_size_m_) << "\n";
    stream << "  captured_frames: " << captured_image_points_.size() << "\n";
    stream << "  rms_reprojection_error: " << FormatDouble(rms) << "\n";
    stream << "  mean_reprojection_error: " << FormatDouble(mean_error) << "\n";
  }

  std::vector<std::string> RenderSerialIndexedCalibrationEntry(
    const std::vector<double> & k,
    const std::vector<double> & d,
    const std::vector<double> & r,
    const std::vector<double> & p,
    double rms,
    double mean_error) const
  {
    std::vector<std::string> lines;
    const std::string camera_name = camera_name_.empty() ? camera_serial_ : camera_name_;

    lines.push_back("  \"" + EscapeYamlDoubleQuoted(camera_serial_) + "\":");
    lines.push_back("    camera_name: \"" + EscapeYamlDoubleQuoted(camera_name) + "\"");
    lines.push_back("    frame_id: \"" + EscapeYamlDoubleQuoted(frame_id_) + "\"");
    lines.push_back("    camera_info:");
    lines.push_back("      distortion_model: \"plumb_bob\"");
    lines.push_back("      d: " + FormatYamlList(d));
    lines.push_back("      k: " + FormatYamlList(k));
    lines.push_back("      r: " + FormatYamlList(r));
    lines.push_back("      p: " + FormatYamlList(p));
    lines.push_back("      binning_x: 0");
    lines.push_back("      binning_y: 0");
    lines.push_back("      roi:");
    lines.push_back("        x_offset: 0");
    lines.push_back("        y_offset: 0");
    lines.push_back("        height: 0");
    lines.push_back("        width: 0");
    lines.push_back("        do_rectify: false");
    lines.push_back("    calibration:");
    lines.push_back("      generated_at_utc: \"" + CurrentUtcTimestamp() + "\"");
    lines.push_back("      image_width: " + std::to_string(calibration_image_size_.width));
    lines.push_back("      image_height: " + std::to_string(calibration_image_size_.height));
    lines.push_back("      board_cols: " + std::to_string(board_cols_));
    lines.push_back("      board_rows: " + std::to_string(board_rows_));
    lines.push_back("      square_size_m: " + FormatDouble(square_size_m_));
    lines.push_back("      captured_frames: " + std::to_string(captured_image_points_.size()));
    lines.push_back("      rms_reprojection_error: " + FormatDouble(rms));
    lines.push_back("      mean_reprojection_error: " + FormatDouble(mean_error));
    return lines;
  }

  std::vector<std::string> ReadExistingYamlLines(const std::filesystem::path & path) const
  {
    std::vector<std::string> lines;
    std::ifstream input(path.string());
    if (!input.is_open()) {
      return lines;
    }

    std::string line;
    while (std::getline(input, line)) {
      lines.push_back(line);
    }
    return lines;
  }

  void WriteLines(const std::filesystem::path & output_path, const std::vector<std::string> & lines) const
  {
    std::ofstream stream(output_path.string());
    if (!stream.is_open()) {
      throw std::runtime_error("Failed to open calibration output file: " + output_yaml_path_);
    }

    for (const std::string & line : lines) {
      stream << line << "\n";
    }
  }

  void WriteSerialIndexedCalibrationYaml(
    const cv::Mat & camera_matrix,
    const cv::Mat & distortion_coefficients,
    double rms,
    double mean_error) const
  {
    const std::filesystem::path output_path(output_yaml_path_);
    if (output_path.has_parent_path()) {
      std::filesystem::create_directories(output_path.parent_path());
    }

    const std::vector<double> k = FlattenMatToDoubles(camera_matrix);
    const std::vector<double> d = FlattenMatToDoubles(distortion_coefficients);
    const std::vector<double> r = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0
    };
    const std::vector<double> p = {
      camera_matrix.at<double>(0, 0), 0.0, camera_matrix.at<double>(0, 2), 0.0,
      0.0, camera_matrix.at<double>(1, 1), camera_matrix.at<double>(1, 2), 0.0,
      0.0, 0.0, 1.0, 0.0
    };
    const std::vector<std::string> entry_lines =
      RenderSerialIndexedCalibrationEntry(k, d, r, p, rms, mean_error);

    std::vector<std::string> lines = ReadExistingYamlLines(output_path);
    const auto registry_iter = std::find_if(
      lines.begin(),
      lines.end(),
      [](const std::string & line) {
        return IsYamlMapKey(line, "camera_info_by_serial");
      });

    if (registry_iter == lines.end()) {
      std::vector<std::string> fresh_lines;
      fresh_lines.push_back("version: 2");
      fresh_lines.push_back("camera_info_by_serial:");
      fresh_lines.insert(fresh_lines.end(), entry_lines.begin(), entry_lines.end());
      WriteLines(output_path, fresh_lines);
      return;
    }

    const std::size_t registry_index =
      static_cast<std::size_t>(std::distance(lines.begin(), registry_iter));
    const std::size_t registry_indent = CountLeadingSpaces(lines[registry_index]);
    std::size_t registry_end = lines.size();
    std::optional<std::size_t> target_start;
    std::size_t target_end = lines.size();

    for (std::size_t index = registry_index + 1U; index < lines.size(); ++index) {
      const std::string trimmed = TrimAscii(lines[index]);
      if (trimmed.empty() || trimmed[0] == '#') {
        continue;
      }

      const std::size_t indent = CountLeadingSpaces(lines[index]);
      if (indent <= registry_indent) {
        registry_end = index;
        break;
      }

      if (indent != registry_indent + 2U) {
        continue;
      }

      const auto serial_key = MatchYamlMapKey(lines[index]);
      if (!serial_key.has_value() || *serial_key != camera_serial_) {
        continue;
      }

      target_start = index;
      target_end = registry_end;
      for (std::size_t end_index = index + 1U; end_index < lines.size(); ++end_index) {
        const std::string end_trimmed = TrimAscii(lines[end_index]);
        if (end_trimmed.empty() || end_trimmed[0] == '#') {
          continue;
        }

        const std::size_t end_indent = CountLeadingSpaces(lines[end_index]);
        if (end_indent <= registry_indent ||
          (end_indent == registry_indent + 2U && MatchYamlMapKey(lines[end_index]).has_value()))
        {
          target_end = end_index;
          break;
        }
      }
      break;
    }

    if (target_start.has_value()) {
      lines.erase(lines.begin() + static_cast<std::ptrdiff_t>(*target_start),
        lines.begin() + static_cast<std::ptrdiff_t>(target_end));
      lines.insert(lines.begin() + static_cast<std::ptrdiff_t>(*target_start),
        entry_lines.begin(), entry_lines.end());
    } else {
      lines.insert(lines.begin() + static_cast<std::ptrdiff_t>(registry_end),
        entry_lines.begin(), entry_lines.end());
    }

    WriteLines(output_path, lines);
  }

  void WriteCalibrationYaml(
    const cv::Mat & camera_matrix,
    const cv::Mat & distortion_coefficients,
    double rms,
    double mean_error) const
  {
    if (camera_serial_.empty()) {
      WriteLegacyCalibrationYaml(camera_matrix, distortion_coefficients, rms, mean_error);
      return;
    }

    WriteSerialIndexedCalibrationYaml(camera_matrix, distortion_coefficients, rms, mean_error);
  }

  void ResetCapturedFrames()
  {
    captured_image_points_.clear();
    captured_object_points_.clear();
    calibration_image_size_ = cv::Size();
    RCLCPP_INFO(get_logger(), "Captured calibration frames cleared.");
  }

  void ProcessFrame(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & message,
    std::uint64_t sequence)
  {
    if (message == nullptr) {
      return;
    }

    const cv::Mat full_resolution_bgr = DecodeCompressedImage(*message);
    if (full_resolution_bgr.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Failed to decode compressed image from '%s'.",
        input_topic_.c_str());
      return;
    }

    cv::Mat preview_bgr = PreparePreviewImage(full_resolution_bgr);
    std::vector<cv::Point2f> preview_corners;
    const bool board_detected = DetectChessboard(preview_bgr, preview_corners, preview_fast_check_);

    cv::Mat annotated_preview = preview_bgr.clone();
    if (board_detected) {
      cv::drawChessboardCorners(annotated_preview, board_size_, preview_corners, true);
    }
    DrawOverlay(annotated_preview, board_detected);
    PublishAnnotatedImage(message->header, annotated_preview);

    std::lock_guard<std::mutex> lock(latest_frame_mutex_);
    if (latest_frame_.received_sequence != sequence) {
      return;
    }
    latest_frame_.has_frame = true;
    latest_frame_.board_detected = board_detected;
    latest_frame_.header = message->header;
    latest_frame_.processed_sequence = sequence;
    latest_frame_.annotated_preview_bgr = annotated_preview;
    latest_frame_.image_size = full_resolution_bgr.size();
  }

  std::string input_topic_;
  std::string annotated_output_topic_;
  std::string output_yaml_path_;
  std::string camera_serial_;
  std::string camera_name_;
  std::string frame_id_;
  std::string sample_image_dir_;
  int board_cols_;
  int board_rows_;
  double square_size_m_;
  int min_calibration_frames_;
  bool display_window_;
  std::string window_name_;
  double preview_scale_;
  int preview_max_width_;
  bool preview_fast_check_;
  int annotated_jpeg_quality_;
  std::string input_qos_reliability_;
  int input_qos_depth_;
  std::string output_qos_reliability_;
  int output_qos_depth_;
  cv::Size board_size_;
  cv::Size calibration_image_size_;
  std::vector<cv::Point3f> base_object_points_;
  std::vector<std::vector<cv::Point2f>> captured_image_points_;
  std::vector<std::vector<cv::Point3f>> captured_object_points_;

  std::mutex latest_frame_mutex_;
  LatestFrameState latest_frame_;

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr annotated_pub_;
  rclcpp::TimerBase::SharedPtr keyboard_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FlirCameraCalibrationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
