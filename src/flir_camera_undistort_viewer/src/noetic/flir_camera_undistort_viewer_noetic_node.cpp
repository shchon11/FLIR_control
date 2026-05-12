#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>

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

std::string NormalizeCompressedFormat(const std::string & value)
{
  const std::string normalized = NormalizeName(value);
  if (normalized.find("png") != std::string::npos) {
    return "png";
  }
  return "jpeg";
}

bool HasUsableCalibration(const sensor_msgs::CameraInfo & camera_info)
{
  return std::abs(camera_info.K[0]) > 1e-9 && std::abs(camera_info.K[4]) > 1e-9;
}

cv::Mat CameraMatrix(const sensor_msgs::CameraInfo & camera_info)
{
  cv::Mat matrix(3, 3, CV_64F);
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      matrix.at<double>(row, col) = camera_info.K[static_cast<std::size_t>(row * 3 + col)];
    }
  }
  return matrix;
}

cv::Mat DistortionCoefficients(const sensor_msgs::CameraInfo & camera_info)
{
  cv::Mat coefficients(
    1,
    static_cast<int>(camera_info.D.size()),
    CV_64F,
    cv::Scalar(0.0));
  for (std::size_t index = 0; index < camera_info.D.size(); ++index) {
    coefficients.at<double>(0, static_cast<int>(index)) = camera_info.D[index];
  }
  return coefficients;
}

}  // namespace

class FlirCameraUndistortViewerNoetic
{
public:
  FlirCameraUndistortViewerNoetic()
  : private_nh_("~")
  {
    private_nh_.param<std::string>("input_topic", input_topic_, "/image_rgb/compressed");
    private_nh_.param<std::string>("camera_info_topic", camera_info_topic_, "/camera_info");
    private_nh_.param<std::string>("output_topic", output_topic_, "/image_rgb/undistorted/compressed");
    private_nh_.param<int>("output_jpeg_quality", output_jpeg_quality_, 80);
    private_nh_.param<int>("output_png_compression_level", output_png_compression_level_, 3);
    private_nh_.param<int>("input_queue_size", input_queue_size_, 10);
    private_nh_.param<int>("camera_info_queue_size", camera_info_queue_size_, 20);
    private_nh_.param<int>("output_queue_size", output_queue_size_, 10);

    image_sub_ = nh_.subscribe(input_topic_, std::max(1, input_queue_size_),
      &FlirCameraUndistortViewerNoetic::OnCompressedImage, this);
    camera_info_sub_ = nh_.subscribe(camera_info_topic_, std::max(1, camera_info_queue_size_),
      &FlirCameraUndistortViewerNoetic::OnCameraInfo, this);
    output_pub_ = nh_.advertise<sensor_msgs::CompressedImage>(output_topic_, std::max(1, output_queue_size_));

    ROS_INFO(
      "Noetic undistort viewer listening on image '%s' and camera_info '%s', publishing '%s'.",
      input_topic_.c_str(),
      camera_info_topic_.c_str(),
      output_topic_.c_str());
  }

private:
  void OnCameraInfo(const sensor_msgs::CameraInfoConstPtr & msg)
  {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    latest_camera_info_ = *msg;
    has_camera_info_ = true;
  }

  void OnCompressedImage(const sensor_msgs::CompressedImageConstPtr & msg)
  {
    sensor_msgs::CameraInfo camera_info;
    {
      std::lock_guard<std::mutex> lock(camera_info_mutex_);
      if (!has_camera_info_) {
        ROS_WARN_THROTTLE(5.0, "Waiting for CameraInfo on '%s'.", camera_info_topic_.c_str());
        return;
      }
      camera_info = latest_camera_info_;
    }

    if (!HasUsableCalibration(camera_info)) {
      ROS_WARN_THROTTLE(
        5.0,
        "CameraInfo on '%s' does not contain usable intrinsics yet.",
        camera_info_topic_.c_str());
      return;
    }

    const cv::Mat input_bgr = DecodeCompressedImage(*msg);
    if (input_bgr.empty()) {
      ROS_WARN_THROTTLE(5.0, "Failed to decode compressed image from '%s'.", input_topic_.c_str());
      return;
    }

    EnsureUndistortMaps(camera_info, input_bgr.size());

    cv::Mat undistorted_bgr;
    cv::remap(input_bgr, undistorted_bgr, map1_, map2_, cv::INTER_LINEAR);

    sensor_msgs::CompressedImage output;
    output.header = msg->header;
    if (!EncodeCompressedImage(msg->format, undistorted_bgr, output)) {
      ROS_WARN_THROTTLE(5.0, "Failed to encode undistorted image for '%s'.", output_topic_.c_str());
      return;
    }
    output_pub_.publish(output);
  }

  cv::Mat DecodeCompressedImage(const sensor_msgs::CompressedImage & msg) const
  {
    if (msg.data.empty()) {
      return cv::Mat();
    }
    const cv::Mat encoded(
      1,
      static_cast<int>(msg.data.size()),
      CV_8UC1,
      const_cast<std::uint8_t *>(msg.data.data()));
    return cv::imdecode(encoded, cv::IMREAD_COLOR);
  }

  bool EncodeCompressedImage(
    const std::string & input_format,
    const cv::Mat & image,
    sensor_msgs::CompressedImage & output) const
  {
    std::vector<std::uint8_t> encoded;
    const std::string format = NormalizeCompressedFormat(input_format);
    if (format == "png") {
      const std::vector<int> params = {
        cv::IMWRITE_PNG_COMPRESSION,
        std::clamp(output_png_compression_level_, 0, 9)
      };
      if (!cv::imencode(".png", image, encoded, params)) {
        return false;
      }
      output.format = "png";
    } else {
      const std::vector<int> params = {
        cv::IMWRITE_JPEG_QUALITY,
        std::clamp(output_jpeg_quality_, 0, 100)
      };
      if (!cv::imencode(".jpg", image, encoded, params)) {
        return false;
      }
      output.format = "jpeg";
    }

    output.data = std::move(encoded);
    return true;
  }

  void EnsureUndistortMaps(const sensor_msgs::CameraInfo & camera_info, const cv::Size & image_size)
  {
    const bool same_size = has_maps_ && image_size == map_size_;
    const bool same_k = camera_info.K == last_k_;
    const bool same_d = camera_info.D == last_d_;
    if (same_size && same_k && same_d) {
      return;
    }

    cv::initUndistortRectifyMap(
      CameraMatrix(camera_info),
      DistortionCoefficients(camera_info),
      cv::Mat::eye(3, 3, CV_64F),
      CameraMatrix(camera_info),
      image_size,
      CV_16SC2,
      map1_,
      map2_);

    map_size_ = image_size;
    last_k_ = camera_info.K;
    last_d_ = camera_info.D;
    has_maps_ = true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber image_sub_;
  ros::Subscriber camera_info_sub_;
  ros::Publisher output_pub_;
  std::mutex camera_info_mutex_;
  bool has_camera_info_{false};
  sensor_msgs::CameraInfo latest_camera_info_;
  cv::Mat map1_;
  cv::Mat map2_;
  cv::Size map_size_;
  sensor_msgs::CameraInfo::_K_type last_k_{};
  std::vector<double> last_d_;
  bool has_maps_{false};
  std::string input_topic_;
  std::string camera_info_topic_;
  std::string output_topic_;
  int output_jpeg_quality_{80};
  int output_png_compression_level_{3};
  int input_queue_size_{10};
  int camera_info_queue_size_{20};
  int output_queue_size_{10};
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "flir_camera_undistort_viewer");
  FlirCameraUndistortViewerNoetic node;
  ros::spin();
  return 0;
}
