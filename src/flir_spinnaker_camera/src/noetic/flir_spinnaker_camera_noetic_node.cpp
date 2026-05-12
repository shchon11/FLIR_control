#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <boost/array.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "flir_spinnaker_camera/FlirMetadata.h"

namespace
{

using Spinnaker::CameraList;
using Spinnaker::CameraPtr;
using Spinnaker::ColorProcessingAlgorithm;
using Spinnaker::ImageProcessor;
using Spinnaker::ImagePtr;
using Spinnaker::PixelFormatEnums;
using Spinnaker::SystemPtr;
using Spinnaker::GenApi::CBooleanPtr;
using Spinnaker::GenApi::CCommandPtr;
using Spinnaker::GenApi::CEnumEntryPtr;
using Spinnaker::GenApi::CEnumerationPtr;
using Spinnaker::GenApi::CFloatPtr;
using Spinnaker::GenApi::CIntegerPtr;
using Spinnaker::GenApi::CNodePtr;
using Spinnaker::GenApi::CStringPtr;
using Spinnaker::GenApi::INodeMap;
using Spinnaker::GenApi::IsAvailable;
using Spinnaker::GenApi::IsReadable;
using Spinnaker::GenApi::IsWritable;

std::string TrimAscii(std::string value)
{
  const auto is_space = [](unsigned char character) {
      return std::isspace(character) != 0;
    };

  value.erase(value.begin(), std::find_if_not(value.begin(), value.end(), is_space));
  value.erase(std::find_if_not(value.rbegin(), value.rend(), is_space).base(), value.end());
  return value;
}

std::string StripMatchingQuotes(std::string value)
{
  value = TrimAscii(std::move(value));
  if (value.size() >= 2U) {
    const bool double_quoted = value.front() == '"' && value.back() == '"';
    const bool single_quoted = value.front() == '\'' && value.back() == '\'';
    if (double_quoted || single_quoted) {
      return value.substr(1, value.size() - 2U);
    }
  }
  return value;
}

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

std::string RemoveYamlComment(const std::string & line)
{
  bool in_single_quote = false;
  bool in_double_quote = false;
  for (std::size_t index = 0; index < line.size(); ++index) {
    const char character = line[index];
    if (character == '\'' && !in_double_quote) {
      in_single_quote = !in_single_quote;
    } else if (character == '"' && !in_single_quote) {
      in_double_quote = !in_double_quote;
    } else if (character == '#' && !in_single_quote && !in_double_quote) {
      return line.substr(0, index);
    }
  }
  return line;
}

std::size_t CountLeadingSpaces(const std::string & value)
{
  return static_cast<std::size_t>(
    std::distance(value.begin(), std::find_if(value.begin(), value.end(), [](char character) {
      return character != ' ';
    })));
}

std::optional<std::string> MatchYamlMapKey(const std::string & line)
{
  const std::string trimmed = TrimAscii(RemoveYamlComment(line));
  if (trimmed.empty() || trimmed.front() == '#' || trimmed.back() != ':') {
    return std::nullopt;
  }
  return StripMatchingQuotes(trimmed.substr(0, trimmed.size() - 1U));
}

bool IsYamlMapKey(const std::string & line, const char * key)
{
  const auto map_key = MatchYamlMapKey(line);
  return map_key.has_value() && *map_key == key;
}

std::optional<std::string> MatchYamlScalarValue(const std::string & line, const char * key)
{
  const std::string trimmed = TrimAscii(RemoveYamlComment(line));
  const std::string prefix = std::string(key) + ":";
  if (trimmed.rfind(prefix, 0) != 0) {
    return std::nullopt;
  }
  return TrimAscii(trimmed.substr(prefix.size()));
}

std::vector<double> ParseYamlDoubleList(std::string value, const std::string & field_name)
{
  value = TrimAscii(std::move(value));
  if (value.size() < 2U || value.front() != '[' || value.back() != ']') {
    throw std::runtime_error("Expected YAML list for '" + field_name + "'.");
  }

  value = value.substr(1, value.size() - 2U);
  std::vector<double> result;
  std::stringstream stream(value);
  std::string token;
  while (std::getline(stream, token, ',')) {
    token = TrimAscii(token);
    if (!token.empty()) {
      result.push_back(std::stod(token));
    }
  }
  return result;
}

int ParseYamlNonNegativeInt(std::string value, const std::string & field_name)
{
  value = StripMatchingQuotes(std::move(value));
  const int parsed = std::stoi(value);
  if (parsed < 0) {
    throw std::runtime_error(field_name + " must be non-negative.");
  }
  return parsed;
}

template<std::size_t Size>
boost::array<double, Size> ToFixedArray(
  const std::vector<double> & values,
  const std::string & field_name)
{
  if (values.size() != Size) {
    throw std::runtime_error(field_name + " must contain " + std::to_string(Size) + " values.");
  }

  boost::array<double, Size> result{};
  std::copy(values.begin(), values.end(), result.begin());
  return result;
}

bool ExecuteCommandByName(INodeMap & node_map, const char * node_name)
{
  CCommandPtr command = node_map.GetNode(node_name);
  if (!IsWritable(command)) {
    return false;
  }
  command->Execute();
  return true;
}

bool SetEnumerationByName(INodeMap & node_map, const char * node_name, const std::string & entry_name)
{
  CEnumerationPtr enum_node = node_map.GetNode(node_name);
  if (!IsWritable(enum_node)) {
    return false;
  }

  CEnumEntryPtr entry = enum_node->GetEntryByName(entry_name.c_str());
  if (!IsAvailable(entry) || !IsReadable(entry)) {
    return false;
  }

  enum_node->SetIntValue(entry->GetValue(), true);
  return true;
}

bool SetIntegerByName(INodeMap & node_map, const char * node_name, std::int64_t value)
{
  CIntegerPtr int_node = node_map.GetNode(node_name);
  if (!IsWritable(int_node)) {
    return false;
  }
  int_node->SetValue(value, true);
  return true;
}

bool SetFloatByName(INodeMap & node_map, const char * node_name, double value)
{
  CFloatPtr float_node = node_map.GetNode(node_name);
  if (!IsWritable(float_node)) {
    return false;
  }
  float_node->SetValue(value, true);
  return true;
}

bool SetBooleanByName(INodeMap & node_map, const char * node_name, bool value)
{
  CBooleanPtr bool_node = node_map.GetNode(node_name);
  if (IsWritable(bool_node)) {
    bool_node->SetValue(value, true);
    return true;
  }

  CIntegerPtr int_node = node_map.GetNode(node_name);
  if (IsWritable(int_node)) {
    int_node->SetValue(value ? 1 : 0, true);
    return true;
  }

  return false;
}

std::string SafeNodeString(INodeMap & node_map, const char * node_name)
{
  try {
    CStringPtr string_node = node_map.GetNode(node_name);
    if (IsReadable(string_node)) {
      return string_node->GetValue().c_str();
    }
  } catch (...) {
  }
  return "";
}

std::string CleanNamespace(std::string value)
{
  value = StripMatchingQuotes(TrimAscii(std::move(value)));
  while (!value.empty() && value.front() == '/') {
    value.erase(value.begin());
  }
  while (!value.empty() && value.back() == '/') {
    value.pop_back();
  }
  return value;
}

std::string JoinTopic(const std::string & camera_namespace, const std::string & topic)
{
  const std::string clean_topic = CleanNamespace(topic);
  if (camera_namespace.empty()) {
    return clean_topic;
  }
  return camera_namespace + "/" + clean_topic;
}

struct RawOutputSpec
{
  std::string encoding;
  PixelFormatEnums converted_pixel_format{Spinnaker::PixelFormat_Mono8};
  bool requires_conversion{false};
};

std::optional<RawOutputSpec> RawOutputSpecForPixelFormat(PixelFormatEnums pixel_format)
{
  switch (pixel_format) {
    case Spinnaker::PixelFormat_Mono8:
      return RawOutputSpec{sensor_msgs::image_encodings::MONO8, Spinnaker::PixelFormat_Mono8, false};
    case Spinnaker::PixelFormat_Mono16:
      return RawOutputSpec{sensor_msgs::image_encodings::MONO16, Spinnaker::PixelFormat_Mono16, false};
    case Spinnaker::PixelFormat_BayerRG8:
      return RawOutputSpec{sensor_msgs::image_encodings::BAYER_RGGB8, Spinnaker::PixelFormat_BayerRG8, false};
    case Spinnaker::PixelFormat_BayerGB8:
      return RawOutputSpec{sensor_msgs::image_encodings::BAYER_GBRG8, Spinnaker::PixelFormat_BayerGB8, false};
    case Spinnaker::PixelFormat_BayerGR8:
      return RawOutputSpec{sensor_msgs::image_encodings::BAYER_GRBG8, Spinnaker::PixelFormat_BayerGR8, false};
    case Spinnaker::PixelFormat_BayerBG8:
      return RawOutputSpec{sensor_msgs::image_encodings::BAYER_BGGR8, Spinnaker::PixelFormat_BayerBG8, false};
    case Spinnaker::PixelFormat_BayerRG16:
      return RawOutputSpec{sensor_msgs::image_encodings::BAYER_RGGB16, Spinnaker::PixelFormat_BayerRG16, false};
    case Spinnaker::PixelFormat_BayerGB16:
      return RawOutputSpec{sensor_msgs::image_encodings::BAYER_GBRG16, Spinnaker::PixelFormat_BayerGB16, false};
    case Spinnaker::PixelFormat_BayerGR16:
      return RawOutputSpec{sensor_msgs::image_encodings::BAYER_GRBG16, Spinnaker::PixelFormat_BayerGR16, false};
    case Spinnaker::PixelFormat_BayerBG16:
      return RawOutputSpec{sensor_msgs::image_encodings::BAYER_BGGR16, Spinnaker::PixelFormat_BayerBG16, false};
    case Spinnaker::PixelFormat_RGB8:
    case Spinnaker::PixelFormat_RGB8Packed:
      return RawOutputSpec{sensor_msgs::image_encodings::RGB8, Spinnaker::PixelFormat_RGB8, false};
    case Spinnaker::PixelFormat_BGR8:
      return RawOutputSpec{sensor_msgs::image_encodings::BGR8, Spinnaker::PixelFormat_BGR8, false};
    case Spinnaker::PixelFormat_Mono10p:
    case Spinnaker::PixelFormat_Mono12p:
      return RawOutputSpec{sensor_msgs::image_encodings::MONO16, Spinnaker::PixelFormat_Mono16, true};
    case Spinnaker::PixelFormat_BayerRG10p:
    case Spinnaker::PixelFormat_BayerRG12p:
      return RawOutputSpec{sensor_msgs::image_encodings::BAYER_RGGB16, Spinnaker::PixelFormat_BayerRG16, true};
    case Spinnaker::PixelFormat_BayerGB10p:
    case Spinnaker::PixelFormat_BayerGB12p:
      return RawOutputSpec{sensor_msgs::image_encodings::BAYER_GBRG16, Spinnaker::PixelFormat_BayerGB16, true};
    case Spinnaker::PixelFormat_BayerGR10p:
    case Spinnaker::PixelFormat_BayerGR12p:
      return RawOutputSpec{sensor_msgs::image_encodings::BAYER_GRBG16, Spinnaker::PixelFormat_BayerGR16, true};
    case Spinnaker::PixelFormat_BayerBG10p:
    case Spinnaker::PixelFormat_BayerBG12p:
      return RawOutputSpec{sensor_msgs::image_encodings::BAYER_BGGR16, Spinnaker::PixelFormat_BayerBG16, true};
    default:
      return std::nullopt;
  }
}

ColorProcessingAlgorithm ColorProcessingFromName(const std::string & value)
{
  const std::string normalized = NormalizeName(value);
  if (normalized == "none") {
    return Spinnaker::NO_COLOR_PROCESSING;
  }
  if (normalized == "nearest" || normalized == "nearestneighbor") {
    return Spinnaker::NEAREST_NEIGHBOR;
  }
  if (normalized == "edge" || normalized == "edge_sensing") {
    return Spinnaker::EDGE_SENSING;
  }
  if (normalized == "hqlinear" || normalized == "highqualitylinear") {
    return Spinnaker::HQ_LINEAR;
  }
  return Spinnaker::HQ_LINEAR;
}

struct CameraConfig
{
  std::string name{"flir_camera"};
  std::string camera_namespace;
  std::string serial;
  int index{0};
  std::string frame_id{"flir_camera_optical_frame"};
  int acquisition_timeout_ms{3000};
  int publisher_queue_size{20};
  bool use_camera_timestamp_in_header{false};
  bool auto_pixel_format{false};
  std::string pixel_format{"BayerRG16"};
  std::string buffer_handling_mode{"NewestOnly"};
  std::string color_processing{"hq_linear"};
  bool publish_raw{false};
  bool publish_camera_info{true};
  bool publish_metadata{true};
  bool publish_rgb_compressed{true};
  std::string rgb_compression_format{"jpeg"};
  int rgb_jpeg_quality{80};
  int rgb_png_compression_level{0};
  std::string camera_info_yaml_path{"calibration/flir_camera_info.yaml"};
  sensor_msgs::CameraInfo camera_info;
  std::map<std::string, std::string> enum_controls;
  std::map<std::string, bool> bool_controls;
  std::map<std::string, double> numeric_controls;
};

void SetDefaultCameraInfo(CameraConfig & config)
{
  config.camera_info.header.frame_id = config.frame_id;
  config.camera_info.distortion_model = "plumb_bob";
  config.camera_info.D.assign(5, 0.0);
  config.camera_info.K.assign(0.0);
  config.camera_info.R.assign(0.0);
  config.camera_info.P.assign(0.0);
}

std::vector<std::string> ReadLines(const std::string & path)
{
  std::ifstream stream(path);
  if (!stream.is_open()) {
    return {};
  }

  std::vector<std::string> lines;
  std::string line;
  while (std::getline(stream, line)) {
    lines.push_back(line);
  }
  return lines;
}

bool ApplySerialCameraInfoYaml(CameraConfig & config, const std::vector<std::string> & lines)
{
  if (config.serial.empty()) {
    return false;
  }

  bool in_registry = false;
  bool in_target = false;
  std::size_t registry_indent = 0U;
  std::size_t serial_indent = 0U;
  bool saw_value = false;

  for (const std::string & raw_line : lines) {
    const std::string without_comment = RemoveYamlComment(raw_line);
    const std::string trimmed = TrimAscii(without_comment);
    if (trimmed.empty()) {
      continue;
    }

    const std::size_t indent = CountLeadingSpaces(without_comment);
    if (!in_registry) {
      if (IsYamlMapKey(raw_line, "camera_info_by_serial")) {
        in_registry = true;
        registry_indent = indent;
      }
      continue;
    }

    if (indent <= registry_indent && !IsYamlMapKey(raw_line, "camera_info_by_serial")) {
      break;
    }

    if (indent == registry_indent + 2U) {
      const auto serial_key = MatchYamlMapKey(raw_line);
      in_target = serial_key.has_value() && *serial_key == config.serial;
      serial_indent = indent;
      continue;
    }

    if (!in_target || indent <= serial_indent) {
      continue;
    }

    if (const auto value = MatchYamlScalarValue(raw_line, "frame_id")) {
      config.frame_id = StripMatchingQuotes(*value);
      config.camera_info.header.frame_id = config.frame_id;
      continue;
    }
    if (const auto value = MatchYamlScalarValue(raw_line, "distortion_model")) {
      config.camera_info.distortion_model = StripMatchingQuotes(*value);
      saw_value = true;
      continue;
    }
    if (const auto value = MatchYamlScalarValue(raw_line, "d")) {
      config.camera_info.D = ParseYamlDoubleList(*value, "camera_info.d");
      saw_value = true;
      continue;
    }
    if (const auto value = MatchYamlScalarValue(raw_line, "k")) {
      config.camera_info.K = ToFixedArray<9>(ParseYamlDoubleList(*value, "camera_info.k"), "camera_info.k");
      saw_value = true;
      continue;
    }
    if (const auto value = MatchYamlScalarValue(raw_line, "r")) {
      config.camera_info.R = ToFixedArray<9>(ParseYamlDoubleList(*value, "camera_info.r"), "camera_info.r");
      saw_value = true;
      continue;
    }
    if (const auto value = MatchYamlScalarValue(raw_line, "p")) {
      config.camera_info.P = ToFixedArray<12>(ParseYamlDoubleList(*value, "camera_info.p"), "camera_info.p");
      saw_value = true;
      continue;
    }
    if (const auto value = MatchYamlScalarValue(raw_line, "binning_x")) {
      config.camera_info.binning_x = static_cast<std::uint32_t>(
        ParseYamlNonNegativeInt(*value, "camera_info.binning_x"));
      continue;
    }
    if (const auto value = MatchYamlScalarValue(raw_line, "binning_y")) {
      config.camera_info.binning_y = static_cast<std::uint32_t>(
        ParseYamlNonNegativeInt(*value, "camera_info.binning_y"));
      continue;
    }
  }

  return saw_value;
}

void ApplyCameraInfoYaml(CameraConfig & config)
{
  if (config.camera_info_yaml_path.empty()) {
    return;
  }

  const std::vector<std::string> lines = ReadLines(config.camera_info_yaml_path);
  if (lines.empty()) {
    ROS_WARN("Camera info YAML '%s' was not found or empty.", config.camera_info_yaml_path.c_str());
    return;
  }

  const bool has_registry = std::any_of(lines.begin(), lines.end(), [](const std::string & line) {
    return IsYamlMapKey(line, "camera_info_by_serial");
  });

  if (has_registry) {
    if (!ApplySerialCameraInfoYaml(config, lines)) {
      ROS_WARN(
        "Camera info YAML '%s' does not contain usable calibration for serial '%s'.",
        config.camera_info_yaml_path.c_str(),
        config.serial.c_str());
    }
  }
}

std::string ValueFromCameraEntry(const std::map<std::string, std::string> & entry, const char * key)
{
  const auto iter = entry.find(key);
  return iter == entry.end() ? "" : iter->second;
}

std::vector<std::map<std::string, std::string>> ParseCameraInventory(const std::string & path)
{
  std::vector<std::map<std::string, std::string>> cameras;
  std::map<std::string, std::string> current;
  bool in_cameras = false;

  for (const std::string & raw_line : ReadLines(path)) {
    const std::string line = TrimAscii(RemoveYamlComment(raw_line));
    if (line.empty()) {
      continue;
    }
    if (line == "cameras:") {
      in_cameras = true;
      continue;
    }
    if (!in_cameras) {
      continue;
    }
    if (line.rfind("- ", 0) == 0) {
      if (!current.empty()) {
        cameras.push_back(current);
        current.clear();
      }
      const std::string item = TrimAscii(line.substr(2));
      const std::size_t colon = item.find(':');
      if (colon != std::string::npos) {
        current[TrimAscii(item.substr(0, colon))] = StripMatchingQuotes(item.substr(colon + 1U));
      }
      continue;
    }
    const std::size_t colon = line.find(':');
    if (!current.empty() && colon != std::string::npos) {
      current[TrimAscii(line.substr(0, colon))] = StripMatchingQuotes(line.substr(colon + 1U));
    }
  }

  if (!current.empty()) {
    cameras.push_back(current);
  }
  return cameras;
}

template<typename T>
void GetPrivateParam(const ros::NodeHandle & private_nh, const std::string & name, T & value)
{
  private_nh.param(name, value, value);
}

void LoadSharedParams(const ros::NodeHandle & private_nh, CameraConfig & config)
{
  GetPrivateParam(private_nh, "frame_id", config.frame_id);
  GetPrivateParam(private_nh, "camera_index", config.index);
  GetPrivateParam(private_nh, "acquisition_timeout_ms", config.acquisition_timeout_ms);
  GetPrivateParam(private_nh, "publisher_queue_size", config.publisher_queue_size);
  GetPrivateParam(private_nh, "use_camera_timestamp_in_header", config.use_camera_timestamp_in_header);
  GetPrivateParam(private_nh, "auto_pixel_format", config.auto_pixel_format);
  GetPrivateParam(private_nh, "pixel_format", config.pixel_format);
  GetPrivateParam(private_nh, "buffer_handling_mode", config.buffer_handling_mode);
  GetPrivateParam(private_nh, "color_processing", config.color_processing);
  GetPrivateParam(private_nh, "publish_raw", config.publish_raw);
  GetPrivateParam(private_nh, "publish_camera_info", config.publish_camera_info);
  GetPrivateParam(private_nh, "publish_metadata", config.publish_metadata);
  GetPrivateParam(private_nh, "publish_rgb_compressed", config.publish_rgb_compressed);
  GetPrivateParam(private_nh, "rgb_compression_format", config.rgb_compression_format);
  GetPrivateParam(private_nh, "rgb_jpeg_quality", config.rgb_jpeg_quality);
  GetPrivateParam(private_nh, "rgb_png_compression_level", config.rgb_png_compression_level);
  GetPrivateParam(private_nh, "camera_info_yaml_path", config.camera_info_yaml_path);
  GetPrivateParam(private_nh, "camera_info.yaml_path", config.camera_info_yaml_path);

  const auto read_enum_control = [&](const char * param_name, const char * node_name) {
      std::string value;
      if (private_nh.getParam(param_name, value)) {
        config.enum_controls[node_name] = value;
      }
    };
  const auto read_bool_control = [&](const char * param_name, const char * node_name) {
      bool value = false;
      if (private_nh.getParam(param_name, value)) {
        config.bool_controls[node_name] = value;
      }
    };
  const auto read_numeric_control = [&](const char * param_name, const char * node_name) {
      double value = 0.0;
      if (private_nh.getParam(param_name, value)) {
        config.numeric_controls[node_name] = value;
      }
    };

  read_bool_control("camera.AcquisitionFrameRateEnable", "AcquisitionFrameRateEnable");
  read_numeric_control("camera.AcquisitionFrameRate", "AcquisitionFrameRate");
  read_numeric_control("camera.FrameRateHz_Val", "FrameRateHz_Val");
  read_enum_control("camera.ExposureAuto", "ExposureAuto");
  read_numeric_control("camera.ExposureTime", "ExposureTime");
  read_numeric_control("camera.ExposureTime_FloatVal", "ExposureTime_FloatVal");
  read_enum_control("camera.GainAuto", "GainAuto");
  read_numeric_control("camera.Gain", "Gain");
  read_numeric_control("camera.GainDB_Val", "GainDB_Val");
  read_numeric_control("camera.Width", "Width");
  read_numeric_control("camera.Height", "Height");
  read_numeric_control("camera.DeviceLinkThroughputLimit", "DeviceLinkThroughputLimit");
}

std::vector<CameraConfig> LoadCameraConfigs(const ros::NodeHandle & private_nh)
{
  CameraConfig base;
  LoadSharedParams(private_nh, base);

  std::string cameras_file;
  private_nh.param<std::string>("cameras_file", cameras_file, "");

  if (cameras_file.empty()) {
    GetPrivateParam(private_nh, "camera_serial", base.serial);
    SetDefaultCameraInfo(base);
    ApplyCameraInfoYaml(base);
    return {base};
  }

  std::vector<CameraConfig> configs;
  const std::vector<std::map<std::string, std::string>> entries = ParseCameraInventory(cameras_file);
  for (std::size_t index = 0; index < entries.size(); ++index) {
    const auto & entry = entries[index];
    CameraConfig config = base;
    config.name = ValueFromCameraEntry(entry, "name");
    if (config.name.empty()) {
      config.name = "camera" + std::to_string(index);
    }
    config.camera_namespace = CleanNamespace(ValueFromCameraEntry(entry, "namespace"));
    if (config.camera_namespace.empty()) {
      config.camera_namespace = config.name;
    }
    config.serial = ValueFromCameraEntry(entry, "serial");
    config.frame_id = ValueFromCameraEntry(entry, "frame_id");
    if (config.frame_id.empty()) {
      config.frame_id = config.name + "_optical_frame";
    }
    config.index = static_cast<int>(index);
    SetDefaultCameraInfo(config);
    ApplyCameraInfoYaml(config);
    configs.push_back(std::move(config));
  }

  if (configs.empty()) {
    throw std::runtime_error("No cameras found in inventory: " + cameras_file);
  }
  return configs;
}

class CameraStreamer
{
public:
  CameraStreamer(const CameraConfig & config, SystemPtr system)
  : config_(config),
    system_(system),
    camera_nh_(ros::NodeHandle(), config_.camera_namespace)
  {
    image_processor_.SetColorProcessing(ColorProcessingFromName(config_.color_processing));
  }

  void Start(CameraList & camera_list)
  {
    camera_ = SelectCamera(camera_list);
    ConfigureCamera();
    Advertise();
    camera_->BeginAcquisition();
    running_ = true;
    acquisition_thread_ = std::thread(&CameraStreamer::AcquisitionLoop, this);
  }

  void Stop()
  {
    running_ = false;
    if (acquisition_thread_.joinable()) {
      acquisition_thread_.join();
    }
    if (camera_) {
      try {
        camera_->EndAcquisition();
      } catch (...) {
      }
      try {
        camera_->DeInit();
      } catch (...) {
      }
      camera_ = nullptr;
    }
  }

  ~CameraStreamer()
  {
    Stop();
  }

private:
  CameraPtr SelectCamera(CameraList & camera_list)
  {
    if (!config_.serial.empty()) {
      for (std::size_t index = 0; index < camera_list.GetSize(); ++index) {
        CameraPtr camera = camera_list.GetByIndex(static_cast<unsigned int>(index));
        INodeMap & tl_node_map = camera->GetTLDeviceNodeMap();
        if (SafeNodeString(tl_node_map, "DeviceSerialNumber") == config_.serial) {
          return camera;
        }
      }
      throw std::runtime_error("Requested camera_serial was not found: " + config_.serial);
    }

    if (config_.index < 0 || static_cast<std::size_t>(config_.index) >= camera_list.GetSize()) {
      throw std::runtime_error("camera_index is out of range.");
    }
    return camera_list.GetByIndex(static_cast<unsigned int>(config_.index));
  }

  void ConfigureCamera()
  {
    camera_->Init();
    INodeMap & node_map = camera_->GetNodeMap();
    INodeMap & stream_node_map = camera_->GetTLStreamNodeMap();

    if (!config_.buffer_handling_mode.empty()) {
      if (SetEnumerationByName(stream_node_map, "StreamBufferHandlingMode", config_.buffer_handling_mode)) {
        ROS_INFO("[%s] StreamBufferHandlingMode set to '%s'.", config_.name.c_str(), config_.buffer_handling_mode.c_str());
      } else {
        ROS_WARN("[%s] Could not set StreamBufferHandlingMode to '%s'.",
          config_.name.c_str(), config_.buffer_handling_mode.c_str());
      }
    }

    if (!config_.auto_pixel_format && !config_.pixel_format.empty()) {
      if (SetEnumerationByName(node_map, "PixelFormat", config_.pixel_format)) {
        ROS_INFO("[%s] PixelFormat set to '%s'.", config_.name.c_str(), config_.pixel_format.c_str());
      } else {
        ROS_WARN("[%s] Could not set PixelFormat to '%s'.", config_.name.c_str(), config_.pixel_format.c_str());
      }
    }

    ApplyStartupCameraControls(node_map);
    SetEnumerationByName(node_map, "AcquisitionMode", "Continuous");

    INodeMap & tl_node_map = camera_->GetTLDeviceNodeMap();
    ROS_INFO(
      "[%s] Using FLIR camera model='%s' serial='%s'.",
      config_.name.c_str(),
      SafeNodeString(tl_node_map, "DeviceModelName").c_str(),
      SafeNodeString(tl_node_map, "DeviceSerialNumber").c_str());
  }

  void ApplyStartupCameraControls(INodeMap & node_map)
  {
    for (const auto & item : config_.bool_controls) {
      if (!SetBooleanByName(node_map, item.first.c_str(), item.second)) {
        ROS_WARN("[%s] Could not set camera.%s.", config_.name.c_str(), item.first.c_str());
      }
    }

    for (const auto & item : config_.numeric_controls) {
      if (!SetFloatByName(node_map, item.first.c_str(), item.second) &&
        !SetIntegerByName(node_map, item.first.c_str(), static_cast<std::int64_t>(item.second)))
      {
        ROS_WARN("[%s] Could not set camera.%s.", config_.name.c_str(), item.first.c_str());
      }
    }

    for (const auto & item : config_.enum_controls) {
      if (!SetEnumerationByName(node_map, item.first.c_str(), item.second)) {
        ROS_WARN(
          "[%s] Could not set camera.%s to '%s'.",
          config_.name.c_str(),
          item.first.c_str(),
          item.second.c_str());
      }
    }
  }

  void Advertise()
  {
    const int queue_size = std::max(1, config_.publisher_queue_size);
    if (config_.publish_raw) {
      raw_pub_ = camera_nh_.advertise<sensor_msgs::Image>("image_raw", queue_size);
    }
    if (config_.publish_camera_info) {
      camera_info_pub_ = camera_nh_.advertise<sensor_msgs::CameraInfo>("camera_info", queue_size);
    }
    if (config_.publish_metadata) {
      metadata_pub_ = camera_nh_.advertise<flir_spinnaker_camera::FlirMetadata>("image_raw/metadata", queue_size);
    }
    if (config_.publish_rgb_compressed) {
      rgb_pub_ = camera_nh_.advertise<sensor_msgs::CompressedImage>("image_rgb/compressed", queue_size);
    }
  }

  sensor_msgs::Image BuildImageMessage(
    const ImagePtr & image,
    const std::string & encoding,
    const ros::Time & stamp) const
  {
    sensor_msgs::Image msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = config_.frame_id;
    msg.height = static_cast<std::uint32_t>(image->GetHeight());
    msg.width = static_cast<std::uint32_t>(image->GetWidth());
    msg.encoding = encoding;
    msg.is_bigendian = 0U;

    std::size_t step = image->GetStride();
    if (step == 0U) {
      const int bit_depth = sensor_msgs::image_encodings::bitDepth(encoding);
      const int channels = sensor_msgs::image_encodings::numChannels(encoding);
      step = static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(bit_depth / 8) *
        static_cast<std::size_t>(channels);
    }
    msg.step = static_cast<std::uint32_t>(step);
    const std::size_t expected_size = static_cast<std::size_t>(msg.step) * msg.height;
    if (image->GetImageSize() < expected_size) {
      throw std::runtime_error("Spinnaker image buffer is smaller than ROS image dimensions require.");
    }
    msg.data.resize(expected_size);
    std::memcpy(msg.data.data(), image->GetData(), expected_size);
    return msg;
  }

  sensor_msgs::CameraInfo BuildCameraInfoMessage(
    const ImagePtr & image,
    const ros::Time & stamp) const
  {
    sensor_msgs::CameraInfo msg = config_.camera_info;
    msg.header.stamp = stamp;
    msg.header.frame_id = config_.frame_id;
    msg.width = static_cast<std::uint32_t>(image->GetWidth());
    msg.height = static_cast<std::uint32_t>(image->GetHeight());
    return msg;
  }

  flir_spinnaker_camera::FlirMetadata BuildMetadataMessage(
    const ImagePtr & original_image,
    const ImagePtr & raw_image,
    const std::string & encoding,
    const ros::Time & stamp) const
  {
    flir_spinnaker_camera::FlirMetadata msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = config_.frame_id;
    msg.width = static_cast<std::uint32_t>(raw_image->GetWidth());
    msg.height = static_cast<std::uint32_t>(raw_image->GetHeight());
    msg.step = static_cast<std::uint32_t>(raw_image->GetStride());
    msg.encoding = encoding;
    msg.pixel_format = original_image->GetPixelFormatName().c_str();
    msg.camera_frame_id = original_image->GetFrameID();
    msg.camera_timestamp_ns = original_image->GetTimeStamp();
    return msg;
  }

  sensor_msgs::CompressedImage BuildCompressedImageMessage(
    const ImagePtr & rgb_image,
    const ros::Time & stamp) const
  {
    const int width = static_cast<int>(rgb_image->GetWidth());
    const int height = static_cast<int>(rgb_image->GetHeight());
    std::size_t step = rgb_image->GetStride();
    if (step == 0U) {
      step = static_cast<std::size_t>(width) * 3U;
    }

    cv::Mat rgb_view(height, width, CV_8UC3, rgb_image->GetData(), step);
    cv::Mat bgr_image;
    cv::cvtColor(rgb_view, bgr_image, cv::COLOR_RGB2BGR);

    const std::string format = NormalizeName(config_.rgb_compression_format) == "png" ? "png" : "jpeg";
    const std::string extension = format == "png" ? ".png" : ".jpg";
    const std::vector<int> params = format == "png" ?
      std::vector<int>{cv::IMWRITE_PNG_COMPRESSION, std::clamp(config_.rgb_png_compression_level, 0, 9)} :
      std::vector<int>{cv::IMWRITE_JPEG_QUALITY, std::clamp(config_.rgb_jpeg_quality, 0, 100)};

    sensor_msgs::CompressedImage msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = config_.frame_id;
    msg.format = format;
    if (!cv::imencode(extension, bgr_image, msg.data, params)) {
      throw std::runtime_error("OpenCV failed to encode RGB compressed image.");
    }
    return msg;
  }

  ros::Time ResolveHeaderStamp(const ImagePtr & image) const
  {
    if (!config_.use_camera_timestamp_in_header) {
      return ros::Time::now();
    }
    const std::uint64_t timestamp_ns = image->GetTimeStamp();
    if (timestamp_ns == 0U) {
      return ros::Time::now();
    }
    ros::Time stamp;
    stamp.fromNSec(timestamp_ns);
    return stamp;
  }

  void AcquisitionLoop()
  {
    while (ros::ok() && running_) {
      ImagePtr image;
      try {
        image = camera_->GetNextImage(config_.acquisition_timeout_ms);
        if (image->IsIncomplete()) {
          ROS_WARN_THROTTLE(
            5.0,
            "[%s] Incomplete image received. status=%d",
            config_.name.c_str(),
            static_cast<int>(image->GetImageStatus()));
          image->Release();
          continue;
        }

        const ros::Time stamp = ResolveHeaderStamp(image);
        ImagePtr raw_image = image;
        std::string raw_encoding;
        const auto raw_spec = RawOutputSpecForPixelFormat(image->GetPixelFormat());
        if (raw_spec.has_value()) {
          raw_encoding = raw_spec->encoding;
          if (raw_spec->requires_conversion) {
            raw_image = image_processor_.Convert(image, raw_spec->converted_pixel_format);
          }
        }

        if (raw_spec.has_value()) {
          if (config_.publish_raw) {
            raw_pub_.publish(BuildImageMessage(raw_image, raw_encoding, stamp));
          }
          if (config_.publish_camera_info) {
            camera_info_pub_.publish(BuildCameraInfoMessage(raw_image, stamp));
          }
          if (config_.publish_metadata) {
            metadata_pub_.publish(BuildMetadataMessage(image, raw_image, raw_encoding, stamp));
          }
        } else {
          ROS_WARN_THROTTLE(
            5.0,
            "[%s] Pixel format '%s' is not mapped to a ROS raw encoding.",
            config_.name.c_str(),
            image->GetPixelFormatName().c_str());
        }

        if (config_.publish_rgb_compressed) {
          ImagePtr rgb_image = image;
          if (image->GetPixelFormat() != Spinnaker::PixelFormat_RGB8 &&
            image->GetPixelFormat() != Spinnaker::PixelFormat_RGB8Packed)
          {
            rgb_image = image_processor_.Convert(image, Spinnaker::PixelFormat_RGB8);
          }
          rgb_pub_.publish(BuildCompressedImageMessage(rgb_image, stamp));
        }

        image->Release();
      } catch (const Spinnaker::Exception & exception) {
        if (image) {
          try {
            image->Release();
          } catch (...) {
          }
        }
        ROS_ERROR_THROTTLE(2.0, "[%s] Spinnaker acquisition error: %s", config_.name.c_str(), exception.what());
      } catch (const std::exception & exception) {
        if (image) {
          try {
            image->Release();
          } catch (...) {
          }
        }
        ROS_ERROR_THROTTLE(2.0, "[%s] Acquisition error: %s", config_.name.c_str(), exception.what());
      }
    }
  }

  CameraConfig config_;
  SystemPtr system_;
  ros::NodeHandle camera_nh_;
  CameraPtr camera_;
  ImageProcessor image_processor_;
  bool running_{false};
  std::thread acquisition_thread_;
  ros::Publisher raw_pub_;
  ros::Publisher camera_info_pub_;
  ros::Publisher metadata_pub_;
  ros::Publisher rgb_pub_;
};

}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "flir_spinnaker_camera");
  ros::NodeHandle private_nh("~");

  try {
    std::vector<CameraConfig> configs = LoadCameraConfigs(private_nh);
    SystemPtr system = Spinnaker::System::GetInstance();
    CameraList camera_list = system->GetCameras();

    if (camera_list.GetSize() == 0U) {
      throw std::runtime_error("No FLIR/Spinnaker cameras detected.");
    }

    std::vector<std::unique_ptr<CameraStreamer>> streamers;
    for (const CameraConfig & config : configs) {
      auto streamer = std::make_unique<CameraStreamer>(config, system);
      streamer->Start(camera_list);
      streamers.push_back(std::move(streamer));
    }

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();

    for (auto & streamer : streamers) {
      streamer->Stop();
    }
    camera_list.Clear();
    system->ReleaseInstance();
  } catch (const std::exception & exception) {
    ROS_FATAL("flir_spinnaker_camera failed: %s", exception.what());
    return 1;
  }

  return 0;
}
