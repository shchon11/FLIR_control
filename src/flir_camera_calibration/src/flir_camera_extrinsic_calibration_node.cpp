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
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

namespace
{

constexpr auto kUiPollInterval = std::chrono::milliseconds(30);

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

std::string StripLeadingSlash(std::string value)
{
  while (!value.empty() && value.front() == '/') {
    value.erase(value.begin());
  }
  return value;
}

std::string NamespacedTopic(const std::string & camera_namespace, const std::string & topic)
{
  const std::string normalized_namespace = StripLeadingSlash(camera_namespace);
  const std::string normalized_topic = StripLeadingSlash(topic);
  if (normalized_namespace.empty()) {
    return "/" + normalized_topic;
  }
  return "/" + normalized_namespace + "/" + normalized_topic;
}

struct Quaternion
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

double Dot(const Quaternion & lhs, const Quaternion & rhs)
{
  return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z + lhs.w * rhs.w;
}

Quaternion Normalized(Quaternion value)
{
  const double norm = std::sqrt(Dot(value, value));
  if (norm <= std::numeric_limits<double>::epsilon()) {
    return Quaternion{};
  }
  value.x /= norm;
  value.y /= norm;
  value.z /= norm;
  value.w /= norm;
  return value;
}

Quaternion QuaternionFromRotation(const cv::Matx33d & rotation)
{
  Quaternion q;
  const double trace = rotation(0, 0) + rotation(1, 1) + rotation(2, 2);
  if (trace > 0.0) {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    q.w = 0.25 * s;
    q.x = (rotation(2, 1) - rotation(1, 2)) / s;
    q.y = (rotation(0, 2) - rotation(2, 0)) / s;
    q.z = (rotation(1, 0) - rotation(0, 1)) / s;
  } else if (rotation(0, 0) > rotation(1, 1) && rotation(0, 0) > rotation(2, 2)) {
    const double s = std::sqrt(1.0 + rotation(0, 0) - rotation(1, 1) - rotation(2, 2)) * 2.0;
    q.w = (rotation(2, 1) - rotation(1, 2)) / s;
    q.x = 0.25 * s;
    q.y = (rotation(0, 1) + rotation(1, 0)) / s;
    q.z = (rotation(0, 2) + rotation(2, 0)) / s;
  } else if (rotation(1, 1) > rotation(2, 2)) {
    const double s = std::sqrt(1.0 + rotation(1, 1) - rotation(0, 0) - rotation(2, 2)) * 2.0;
    q.w = (rotation(0, 2) - rotation(2, 0)) / s;
    q.x = (rotation(0, 1) + rotation(1, 0)) / s;
    q.y = 0.25 * s;
    q.z = (rotation(1, 2) + rotation(2, 1)) / s;
  } else {
    const double s = std::sqrt(1.0 + rotation(2, 2) - rotation(0, 0) - rotation(1, 1)) * 2.0;
    q.w = (rotation(1, 0) - rotation(0, 1)) / s;
    q.x = (rotation(0, 2) + rotation(2, 0)) / s;
    q.y = (rotation(1, 2) + rotation(2, 1)) / s;
    q.z = 0.25 * s;
  }

  return Normalized(q);
}

cv::Matx33d MatxFromRodrigues(const cv::Mat & rotation_vector)
{
  cv::Mat rotation_matrix;
  cv::Rodrigues(rotation_vector, rotation_matrix);

  cv::Matx33d result;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      result(row, col) = rotation_matrix.at<double>(row, col);
    }
  }
  return result;
}

cv::Vec3d Vec3FromMat(const cv::Mat & vector)
{
  return cv::Vec3d(
    vector.at<double>(0, 0),
    vector.at<double>(1, 0),
    vector.at<double>(2, 0));
}

}  // namespace

class FlirCameraExtrinsicCalibrationNode : public rclcpp::Node
{
public:
  FlirCameraExtrinsicCalibrationNode()
  : Node("flir_camera_extrinsic_calibration"),
    camera_namespaces_(declare_parameter<std::vector<std::string>>(
        "camera_namespaces",
        std::vector<std::string>{"camera0", "camera1"})),
    camera_names_(declare_parameter<std::vector<std::string>>("camera_names", std::vector<std::string>{})),
    camera_serials_(declare_parameter<std::vector<std::string>>("camera_serials", std::vector<std::string>{})),
    camera_frame_ids_(declare_parameter<std::vector<std::string>>("camera_frame_ids", std::vector<std::string>{})),
    reference_camera_(declare_parameter<std::string>("reference_camera", "camera0")),
    reference_frame_(declare_parameter<std::string>("reference_frame", "flir_rig_frame")),
    image_topic_(declare_parameter<std::string>("image_topic", "image_rgb/compressed")),
    camera_info_topic_(declare_parameter<std::string>("camera_info_topic", "camera_info")),
    output_yaml_path_(declare_parameter<std::string>(
        "output_yaml_path",
        "calibration/flir_camera_extrinsics.yaml")),
    board_cols_(declare_parameter<int>("board_cols", 6)),
    board_rows_(declare_parameter<int>("board_rows", 5)),
    square_size_m_(declare_parameter<double>("square_size_m", 0.08)),
    min_observations_(declare_parameter<int>("min_observations", 5)),
    max_frame_age_ms_(declare_parameter<int>("max_frame_age_ms", 500)),
    display_window_(declare_parameter<bool>("display_window", true)),
    window_name_(declare_parameter<std::string>("window_name", "FLIR Extrinsic Calibration")),
    preview_max_width_(declare_parameter<int>("preview_max_width", 640)),
    preview_fast_check_(declare_parameter<bool>("preview_fast_check", true)),
    input_qos_reliability_(declare_parameter<std::string>("input_qos_reliability", "best_effort")),
    input_qos_depth_(declare_parameter<int>("input_qos_depth", 10)),
    camera_info_qos_reliability_(declare_parameter<std::string>("camera_info_qos_reliability", "reliable")),
    camera_info_qos_depth_(declare_parameter<int>("camera_info_qos_depth", 20)),
    board_size_(board_cols_, board_rows_)
  {
    if (camera_namespaces_.size() < 2U) {
      throw std::runtime_error("At least two camera_namespaces are required for extrinsic calibration.");
    }
    if (board_cols_ <= 0 || board_rows_ <= 0 || square_size_m_ <= 0.0) {
      throw std::runtime_error("board_cols, board_rows, and square_size_m must be positive.");
    }

    FillOptionalCameraMetadata();
    base_object_points_ = BuildObjectPoints();
    cameras_.reserve(camera_namespaces_.size());

    for (std::size_t index = 0; index < camera_namespaces_.size(); ++index) {
      CameraRuntime runtime;
      runtime.name = camera_names_[index];
      runtime.camera_namespace = camera_namespaces_[index];
      runtime.serial = camera_serials_[index];
      runtime.frame_id = camera_frame_ids_[index];
      cameras_.push_back(std::move(runtime));
    }

    reference_camera_index_ = FindReferenceCameraIndex();

    for (std::size_t index = 0; index < cameras_.size(); ++index) {
      CameraRuntime & camera = cameras_[index];
      camera.image_sub = create_subscription<sensor_msgs::msg::CompressedImage>(
        NamespacedTopic(camera.camera_namespace, image_topic_),
        BuildQoS(input_qos_reliability_, input_qos_depth_),
        [this, index](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
          OnCompressedImage(index, std::move(msg));
        });
      camera.camera_info_sub = create_subscription<sensor_msgs::msg::CameraInfo>(
        NamespacedTopic(camera.camera_namespace, camera_info_topic_),
        BuildQoS(camera_info_qos_reliability_, camera_info_qos_depth_),
        [this, index](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(cameras_mutex_);
          cameras_[index].camera_info = *msg;
          cameras_[index].has_camera_info = true;
        });

      RCLCPP_INFO(
        get_logger(),
        "Extrinsic calibration listening to %s and %s.",
        NamespacedTopic(camera.camera_namespace, image_topic_).c_str(),
        NamespacedTopic(camera.camera_namespace, camera_info_topic_).c_str());
    }

    if (display_window_) {
      cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    }

    timer_ = create_wall_timer(kUiPollInterval, std::bind(&FlirCameraExtrinsicCalibrationNode::OnTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "Extrinsic calibration uses manual capture by default. Press space to capture an observation, "
      "'c' to save, 'r' to reset, and 'q' to quit.");
  }

  ~FlirCameraExtrinsicCalibrationNode() override
  {
    if (display_window_) {
      try {
        cv::destroyWindow(window_name_);
      } catch (...) {
      }
    }
  }

private:
  struct CameraPose
  {
    cv::Matx33d rotation_cam_board{cv::Matx33d::eye()};
    cv::Vec3d translation_cam_board{0.0, 0.0, 0.0};
    double reprojection_error{0.0};
  };

  struct Observation
  {
    std::vector<CameraPose> poses;
  };

  struct CameraRuntime
  {
    std::string name;
    std::string camera_namespace;
    std::string serial;
    std::string frame_id;
    bool has_image{false};
    bool board_detected{false};
    bool has_camera_info{false};
    rclcpp::Time received_time;
    std::uint64_t received_sequence{0};
    std::uint64_t processed_sequence{0};
    std::vector<cv::Point2f> corners;
    sensor_msgs::msg::CompressedImage::ConstSharedPtr latest_message;
    cv::Mat latest_bgr;
    cv::Mat annotated_bgr;
    sensor_msgs::msg::CameraInfo camera_info;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;
  };

  struct ProcessedCameraFrame
  {
    bool has_image{false};
    bool board_detected{false};
    rclcpp::Time received_time;
    std::vector<cv::Point2f> corners;
    cv::Mat latest_bgr;
    cv::Mat annotated_bgr;
  };

  struct PendingFrame
  {
    std::size_t camera_index{0U};
    std::uint64_t sequence{0U};
    std::string camera_name;
    rclcpp::Time received_time;
    sensor_msgs::msg::CompressedImage::ConstSharedPtr message;
  };

  struct CaptureCameraSnapshot
  {
    std::string name;
    bool has_camera_info{false};
    rclcpp::Time received_time;
    sensor_msgs::msg::CameraInfo camera_info;
    sensor_msgs::msg::CompressedImage::ConstSharedPtr message;
  };

  void FillOptionalCameraMetadata()
  {
    const std::size_t count = camera_namespaces_.size();
    if (camera_names_.empty()) {
      camera_names_ = camera_namespaces_;
      for (std::string & name : camera_names_) {
        name = StripLeadingSlash(name);
      }
    }
    if (camera_serials_.empty()) {
      camera_serials_.assign(count, "");
    }
    if (camera_frame_ids_.empty()) {
      camera_frame_ids_.reserve(count);
      for (const std::string & name : camera_names_) {
        camera_frame_ids_.push_back(name + "_optical_frame");
      }
    }

    if (camera_names_.size() != count || camera_serials_.size() != count ||
      camera_frame_ids_.size() != count)
    {
      throw std::runtime_error(
        "camera_namespaces, camera_names, camera_serials, and camera_frame_ids must have matching lengths.");
    }
  }

  std::size_t FindReferenceCameraIndex() const
  {
    for (std::size_t index = 0; index < cameras_.size(); ++index) {
      if (cameras_[index].name == reference_camera_ ||
        cameras_[index].camera_namespace == reference_camera_ ||
        cameras_[index].serial == reference_camera_)
      {
        return index;
      }
    }
    throw std::runtime_error("reference_camera was not found in the configured cameras: " + reference_camera_);
  }

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
    bool fast_check = false) const
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

  void DrawFittedText(
    cv::Mat & image,
    const std::string & text,
    const cv::Point & origin,
    double base_scale,
    const cv::Scalar & color,
    int thickness) const
  {
    int baseline = 0;
    const cv::Size text_size = cv::getTextSize(
      text,
      cv::FONT_HERSHEY_SIMPLEX,
      base_scale,
      thickness,
      &baseline);
    const int available_width = std::max(1, image.cols - origin.x - 12);
    const double scale = text_size.width <= available_width ?
      base_scale :
      std::max(0.35, base_scale * static_cast<double>(available_width) / static_cast<double>(text_size.width));

    cv::putText(
      image,
      text,
      origin,
      cv::FONT_HERSHEY_SIMPLEX,
      scale,
      cv::Scalar(0, 0, 0),
      thickness + 2,
      cv::LINE_AA);
    cv::putText(
      image,
      text,
      origin,
      cv::FONT_HERSHEY_SIMPLEX,
      scale,
      color,
      thickness,
      cv::LINE_AA);
  }

  void OnCompressedImage(std::size_t index, sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(cameras_mutex_);
    CameraRuntime & camera = cameras_[index];
    camera.latest_message = std::move(msg);
    camera.received_time = now();
    ++camera.received_sequence;
  }

  ProcessedCameraFrame ProcessCameraPreview(
    const std::string & camera_name,
    const rclcpp::Time & received_time,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg) const
  {
    ProcessedCameraFrame processed;
    processed.received_time = received_time;
    processed.latest_bgr = DecodeCompressedImage(*msg);
    processed.has_image = !processed.latest_bgr.empty();

    if (!processed.has_image) {
      return processed;
    }

    cv::Mat preview_bgr = PreparePreviewImage(processed.latest_bgr);
    std::vector<cv::Point2f> preview_corners;
    processed.board_detected = DetectChessboard(preview_bgr, preview_corners, preview_fast_check_);
    processed.corners = preview_corners;

    if (processed.board_detected && preview_bgr.size() != processed.latest_bgr.size()) {
      const float scale_x = static_cast<float>(processed.latest_bgr.cols) / static_cast<float>(preview_bgr.cols);
      const float scale_y = static_cast<float>(processed.latest_bgr.rows) / static_cast<float>(preview_bgr.rows);
      for (cv::Point2f & corner : processed.corners) {
        corner.x *= scale_x;
        corner.y *= scale_y;
      }
    }

    processed.annotated_bgr = preview_bgr.clone();
    if (processed.board_detected) {
      cv::drawChessboardCorners(processed.annotated_bgr, board_size_, preview_corners, true);
    }

    const cv::Scalar status_color = processed.board_detected ? cv::Scalar(40, 220, 40) : cv::Scalar(40, 40, 230);
    DrawFittedText(
      processed.annotated_bgr,
      camera_name + (processed.board_detected ? " board" : " no board"),
      cv::Point(12, 28),
      0.58,
      status_color,
      1);
    return processed;
  }

  std::vector<PendingFrame> TakePendingFrames()
  {
    std::vector<PendingFrame> pending;
    std::lock_guard<std::mutex> lock(cameras_mutex_);
    for (std::size_t index = 0; index < cameras_.size(); ++index) {
      CameraRuntime & camera = cameras_[index];
      if (camera.latest_message == nullptr || camera.processed_sequence == camera.received_sequence) {
        continue;
      }

      pending.push_back(PendingFrame{
          index,
          camera.received_sequence,
          camera.name,
          camera.received_time,
          camera.latest_message});
    }
    return pending;
  }

  void ProcessPendingFrames()
  {
    const std::vector<PendingFrame> pending = TakePendingFrames();
    for (const PendingFrame & frame : pending) {
      const ProcessedCameraFrame processed =
        ProcessCameraPreview(frame.camera_name, frame.received_time, frame.message);

      if (!processed.has_image) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          5000,
          "Failed to decode image for %s.",
          frame.camera_name.c_str());
      }

      std::lock_guard<std::mutex> lock(cameras_mutex_);
      CameraRuntime & camera = cameras_[frame.camera_index];
      if (camera.received_sequence != frame.sequence) {
        continue;
      }

      camera.has_image = processed.has_image;
      camera.board_detected = processed.board_detected;
      camera.received_time = processed.received_time;
      camera.corners = processed.corners;
      camera.latest_bgr = processed.latest_bgr;
      camera.annotated_bgr = processed.annotated_bgr;
      camera.processed_sequence = frame.sequence;
    }
  }

  bool HasUsableCameraInfo(const sensor_msgs::msg::CameraInfo & camera_info) const
  {
    return std::abs(camera_info.k[0]) > 1e-9 && std::abs(camera_info.k[4]) > 1e-9;
  }

  bool CameraReady(const CameraRuntime & camera) const
  {
    if (!camera.has_image || !camera.board_detected || !camera.has_camera_info ||
      !HasUsableCameraInfo(camera.camera_info))
    {
      return false;
    }

    const auto age = now() - camera.received_time;
    return age.nanoseconds() <= static_cast<std::int64_t>(max_frame_age_ms_) * 1000000LL;
  }

  bool AllCamerasReady() const
  {
    return std::all_of(cameras_.begin(), cameras_.end(), [this](const CameraRuntime & camera) {
      return CameraReady(camera);
    });
  }

  std::vector<CaptureCameraSnapshot> SnapshotCamerasForCapture()
  {
    std::vector<CaptureCameraSnapshot> snapshots;
    std::lock_guard<std::mutex> lock(cameras_mutex_);
    snapshots.reserve(cameras_.size());
    for (const CameraRuntime & camera : cameras_) {
      snapshots.push_back(CaptureCameraSnapshot{
          camera.name,
          camera.has_camera_info,
          camera.received_time,
          camera.camera_info,
          camera.latest_message});
    }
    return snapshots;
  }

  cv::Mat CameraMatrix(const sensor_msgs::msg::CameraInfo & camera_info) const
  {
    cv::Mat matrix = cv::Mat::eye(3, 3, CV_64F);
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        matrix.at<double>(row, col) = camera_info.k[static_cast<std::size_t>(row * 3 + col)];
      }
    }
    return matrix;
  }

  cv::Mat DistortionCoefficients(const sensor_msgs::msg::CameraInfo & camera_info) const
  {
    cv::Mat coefficients(
      1,
      static_cast<int>(camera_info.d.size()),
      CV_64F,
      cv::Scalar(0.0));
    for (std::size_t index = 0; index < camera_info.d.size(); ++index) {
      coefficients.at<double>(0, static_cast<int>(index)) = camera_info.d[index];
    }
    return coefficients;
  }

  double ReprojectionError(
    const std::vector<cv::Point2f> & image_points,
    const cv::Mat & camera_matrix,
    const cv::Mat & distortion_coefficients,
    const cv::Mat & rotation_vector,
    const cv::Mat & translation_vector) const
  {
    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(
      base_object_points_,
      rotation_vector,
      translation_vector,
      camera_matrix,
      distortion_coefficients,
      projected_points);
    const double error = cv::norm(image_points, projected_points, cv::NORM_L2);
    return error / std::sqrt(static_cast<double>(std::max<std::size_t>(1U, projected_points.size())));
  }

  CameraPose SolveCameraBoardPose(const CameraRuntime & camera) const
  {
    const cv::Mat camera_matrix = CameraMatrix(camera.camera_info);
    const cv::Mat distortion_coefficients = DistortionCoefficients(camera.camera_info);
    cv::Mat rotation_vector;
    cv::Mat translation_vector;

    const bool solved = cv::solvePnP(
      base_object_points_,
      camera.corners,
      camera_matrix,
      distortion_coefficients,
      rotation_vector,
      translation_vector,
      false,
      cv::SOLVEPNP_ITERATIVE);

    if (!solved) {
      throw std::runtime_error("solvePnP failed for " + camera.name);
    }

    CameraPose pose;
    pose.rotation_cam_board = MatxFromRodrigues(rotation_vector);
    pose.translation_cam_board = Vec3FromMat(translation_vector);
    pose.reprojection_error = ReprojectionError(
      camera.corners,
      camera_matrix,
      distortion_coefficients,
      rotation_vector,
      translation_vector);
    return pose;
  }

  void CaptureObservation()
  {
    const std::vector<CaptureCameraSnapshot> snapshots = SnapshotCamerasForCapture();
    const rclcpp::Time capture_time = now();

    Observation observation;
    observation.poses.reserve(snapshots.size());
    for (const CaptureCameraSnapshot & snapshot : snapshots) {
      if (snapshot.message == nullptr) {
        RCLCPP_WARN(get_logger(), "Not capturing: %s has no image yet.", snapshot.name.c_str());
        return;
      }

      const auto age = capture_time - snapshot.received_time;
      if (age.nanoseconds() > static_cast<std::int64_t>(max_frame_age_ms_) * 1000000LL) {
        RCLCPP_WARN(
          get_logger(),
          "Not capturing: %s image is stale (%.3f s old).",
          snapshot.name.c_str(),
          static_cast<double>(age.nanoseconds()) / 1.0e9);
        return;
      }

      if (!snapshot.has_camera_info || !HasUsableCameraInfo(snapshot.camera_info)) {
        RCLCPP_WARN(get_logger(), "Not capturing: %s has no usable CameraInfo.", snapshot.name.c_str());
        return;
      }

      const cv::Mat full_resolution_bgr = DecodeCompressedImage(*snapshot.message);
      if (full_resolution_bgr.empty()) {
        RCLCPP_WARN(get_logger(), "Not capturing: failed to decode latest image for %s.", snapshot.name.c_str());
        return;
      }

      std::vector<cv::Point2f> full_resolution_corners;
      if (!DetectChessboard(full_resolution_bgr, full_resolution_corners, false)) {
        RCLCPP_WARN(
          get_logger(),
          "Not capturing: chessboard was not detected in latest full-resolution image for %s.",
          snapshot.name.c_str());
        return;
      }

      CameraRuntime solved_camera;
      solved_camera.name = snapshot.name;
      solved_camera.camera_info = snapshot.camera_info;
      solved_camera.corners = std::move(full_resolution_corners);
      observation.poses.push_back(SolveCameraBoardPose(solved_camera));
    }

    const double mean_error = std::accumulate(
      observation.poses.begin(),
      observation.poses.end(),
      0.0,
      [](double total, const CameraPose & pose) {
        return total + pose.reprojection_error;
      }) / static_cast<double>(observation.poses.size());

    observations_.push_back(std::move(observation));

    RCLCPP_INFO(
      get_logger(),
      "Captured extrinsic observation %zu/%d. Mean solvePnP reprojection error: %.4f px.",
      observations_.size(),
      min_observations_,
      mean_error);
  }

  struct RelativePose
  {
    cv::Matx33d rotation_ref_camera{cv::Matx33d::eye()};
    cv::Vec3d translation_ref_camera{0.0, 0.0, 0.0};
    double reprojection_error{0.0};
  };

  RelativePose RelativePoseForObservation(const Observation & observation, std::size_t camera_index) const
  {
    if (camera_index == reference_camera_index_) {
      return RelativePose{};
    }

    const CameraPose & reference_pose = observation.poses[reference_camera_index_];
    const CameraPose & camera_pose = observation.poses[camera_index];

    RelativePose relative;
    relative.rotation_ref_camera =
      reference_pose.rotation_cam_board * camera_pose.rotation_cam_board.t();
    relative.translation_ref_camera =
      reference_pose.translation_cam_board -
      relative.rotation_ref_camera * camera_pose.translation_cam_board;
    relative.reprojection_error =
      0.5 * (reference_pose.reprojection_error + camera_pose.reprojection_error);
    return relative;
  }

  RelativePose AverageRelativePose(std::size_t camera_index) const
  {
    if (camera_index == reference_camera_index_) {
      return RelativePose{};
    }

    cv::Vec3d translation_sum(0.0, 0.0, 0.0);
    Quaternion quaternion_sum;
    quaternion_sum.x = 0.0;
    quaternion_sum.y = 0.0;
    quaternion_sum.z = 0.0;
    quaternion_sum.w = 0.0;
    Quaternion first_quaternion;
    bool has_first_quaternion = false;
    double error_sum = 0.0;

    for (const Observation & observation : observations_) {
      const RelativePose relative = RelativePoseForObservation(observation, camera_index);
      translation_sum += relative.translation_ref_camera;
      Quaternion q = QuaternionFromRotation(relative.rotation_ref_camera);
      if (!has_first_quaternion) {
        first_quaternion = q;
        has_first_quaternion = true;
      } else if (Dot(q, first_quaternion) < 0.0) {
        q.x = -q.x;
        q.y = -q.y;
        q.z = -q.z;
        q.w = -q.w;
      }
      quaternion_sum.x += q.x;
      quaternion_sum.y += q.y;
      quaternion_sum.z += q.z;
      quaternion_sum.w += q.w;
      error_sum += relative.reprojection_error;
    }

    const double count = static_cast<double>(observations_.size());
    const Quaternion average_quaternion = Normalized(Quaternion{
        quaternion_sum.x / count,
        quaternion_sum.y / count,
        quaternion_sum.z / count,
        quaternion_sum.w / count});

    RelativePose average;
    average.translation_ref_camera = translation_sum * (1.0 / count);
    average.rotation_ref_camera = RotationFromQuaternion(average_quaternion);
    average.reprojection_error = error_sum / count;
    return average;
  }

  cv::Matx33d RotationFromQuaternion(const Quaternion & q) const
  {
    const Quaternion normalized = Normalized(q);
    const double xx = normalized.x * normalized.x;
    const double yy = normalized.y * normalized.y;
    const double zz = normalized.z * normalized.z;
    const double xy = normalized.x * normalized.y;
    const double xz = normalized.x * normalized.z;
    const double yz = normalized.y * normalized.z;
    const double wx = normalized.w * normalized.x;
    const double wy = normalized.w * normalized.y;
    const double wz = normalized.w * normalized.z;

    return cv::Matx33d(
      1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy),
      2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
      2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy));
  }

  void WriteExtrinsicsYaml()
  {
    if (observations_.size() < static_cast<std::size_t>(std::max(1, min_observations_))) {
      RCLCPP_WARN(
        get_logger(),
        "Need at least %d observations before saving extrinsics. Current count: %zu.",
        min_observations_,
        observations_.size());
      return;
    }

    const std::filesystem::path output_path(output_yaml_path_);
    if (output_path.has_parent_path()) {
      std::filesystem::create_directories(output_path.parent_path());
    }

    std::ofstream stream(output_path.string());
    if (!stream.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open extrinsic output file: %s", output_yaml_path_.c_str());
      return;
    }

    stream << "version: 1\n";
    stream << "reference_frame: \"" << EscapeYamlDoubleQuoted(reference_frame_) << "\"\n";
    stream << "extrinsics_by_serial:\n";

    for (std::size_t index = 0; index < cameras_.size(); ++index) {
      const CameraRuntime & camera = cameras_[index];
      const RelativePose average = AverageRelativePose(index);
      const Quaternion q = QuaternionFromRotation(average.rotation_ref_camera);
      const std::string serial_key = camera.serial.empty() ? camera.name : camera.serial;

      stream << "  \"" << EscapeYamlDoubleQuoted(serial_key) << "\":\n";
      stream << "    camera_name: \"" << EscapeYamlDoubleQuoted(camera.name) << "\"\n";
      stream << "    parent_frame: \"" << EscapeYamlDoubleQuoted(reference_frame_) << "\"\n";
      stream << "    child_frame: \"" << EscapeYamlDoubleQuoted(camera.frame_id) << "\"\n";
      stream << "    translation_xyz_m: ["
             << FormatDouble(average.translation_ref_camera[0]) << ", "
             << FormatDouble(average.translation_ref_camera[1]) << ", "
             << FormatDouble(average.translation_ref_camera[2]) << "]\n";
      stream << "    rotation_xyzw: ["
             << FormatDouble(q.x) << ", "
             << FormatDouble(q.y) << ", "
             << FormatDouble(q.z) << ", "
             << FormatDouble(q.w) << "]\n";
      stream << "    calibration:\n";
      stream << "      method: \"multi_camera_chessboard\"\n";
      stream << "      reference_camera: \"" << EscapeYamlDoubleQuoted(cameras_[reference_camera_index_].name) << "\"\n";
      stream << "      generated_at_utc: \"" << CurrentUtcTimestamp() << "\"\n";
      stream << "      board_cols: " << board_cols_ << "\n";
      stream << "      board_rows: " << board_rows_ << "\n";
      stream << "      square_size_m: " << FormatDouble(square_size_m_) << "\n";
      stream << "      observations: " << observations_.size() << "\n";
      stream << "      mean_pair_reprojection_error: " << FormatDouble(average.reprojection_error) << "\n";
    }

    RCLCPP_INFO(
      get_logger(),
      "Saved extrinsic calibration with %zu observations to '%s'.",
      observations_.size(),
      output_yaml_path_.c_str());
  }

  cv::Mat BuildPreview() const
  {
    std::vector<cv::Mat> previews;
    previews.reserve(cameras_.size());
    for (const CameraRuntime & camera : cameras_) {
      if (camera.annotated_bgr.empty()) {
        cv::Mat waiting = cv::Mat(360, 480, CV_8UC3, cv::Scalar(30, 30, 30));
        DrawFittedText(
          waiting,
          camera.name + " waiting",
          cv::Point(12, 28),
          0.58,
          cv::Scalar(220, 220, 220),
          1);
        previews.push_back(waiting);
      } else {
        cv::Mat resized;
        const double scale = 360.0 / static_cast<double>(camera.annotated_bgr.rows);
        cv::resize(camera.annotated_bgr, resized, cv::Size(), scale, scale);
        previews.push_back(resized);
      }
    }

    int max_height = 0;
    for (const cv::Mat & preview : previews) {
      max_height = std::max(max_height, preview.rows);
    }

    for (cv::Mat & preview : previews) {
      if (preview.rows == max_height) {
        continue;
      }
      cv::copyMakeBorder(
        preview,
        preview,
        0,
        max_height - preview.rows,
        0,
        0,
        cv::BORDER_CONSTANT,
        cv::Scalar(30, 30, 30));
    }

    cv::Mat combined;
    cv::hconcat(previews, combined);
    DrawFittedText(
      combined,
      "observations " + std::to_string(observations_.size()) + "/" + std::to_string(min_observations_) +
      "  space:capture  c:save  r:reset  q:quit",
      cv::Point(20, std::max(30, combined.rows - 20)),
      0.58,
      cv::Scalar(255, 255, 255),
      1);
    return combined;
  }

  void ResetObservations()
  {
    observations_.clear();
    RCLCPP_INFO(get_logger(), "Extrinsic observations cleared.");
  }

  void OnTimer()
  {
    ProcessPendingFrames();

    if (!display_window_) {
      return;
    }

    const cv::Mat preview = BuildPreview();
    if (!preview.empty()) {
      cv::imshow(window_name_, preview);
    }

    const int key = cv::waitKey(1);
    switch (key) {
      case ' ':
        CaptureObservation();
        break;
      case 'c':
      case 'C':
        WriteExtrinsicsYaml();
        break;
      case 'r':
      case 'R':
        ResetObservations();
        break;
      case 'q':
      case 'Q':
      case 27:
        RCLCPP_INFO(get_logger(), "Extrinsic calibration window requested shutdown.");
        rclcpp::shutdown();
        break;
      default:
        break;
    }
  }

  std::vector<std::string> camera_namespaces_;
  std::vector<std::string> camera_names_;
  std::vector<std::string> camera_serials_;
  std::vector<std::string> camera_frame_ids_;
  std::string reference_camera_;
  std::string reference_frame_;
  std::string image_topic_;
  std::string camera_info_topic_;
  std::string output_yaml_path_;
  int board_cols_;
  int board_rows_;
  double square_size_m_;
  int min_observations_;
  int max_frame_age_ms_;
  bool display_window_;
  std::string window_name_;
  int preview_max_width_;
  bool preview_fast_check_;
  std::string input_qos_reliability_;
  int input_qos_depth_;
  std::string camera_info_qos_reliability_;
  int camera_info_qos_depth_;
  cv::Size board_size_;
  std::vector<cv::Point3f> base_object_points_;
  std::vector<CameraRuntime> cameras_;
  std::mutex cameras_mutex_;
  std::size_t reference_camera_index_{0U};
  std::vector<Observation> observations_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FlirCameraExtrinsicCalibrationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
