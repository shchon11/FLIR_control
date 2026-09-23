#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <deque>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>
#include <exception>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "flir_spinnaker_camera/msg/flir_metadata.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

#ifdef FLIR_HAVE_GPU_JPEG
#include "gpu_jpeg_encoder.hpp"
#endif

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
using Spinnaker::GenApi::NodeList_t;
using Spinnaker::GenApi::StringList_t;

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

std::optional<std::string> MatchYamlScalarValue(const std::string & line, const char * key)
{
  const std::string trimmed = TrimAscii(line);
  if (trimmed.empty() || trimmed[0] == '#') {
    return std::nullopt;
  }

  const std::string prefix = std::string(key) + ":";
  if (trimmed.rfind(prefix, 0) != 0) {
    return std::nullopt;
  }

  return TrimAscii(trimmed.substr(prefix.size()));
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

std::vector<double> ParseYamlDoubleList(const std::string & raw_value, const char * field_name)
{
  const std::string trimmed = TrimAscii(raw_value);
  if (trimmed.size() < 2U || trimmed.front() != '[' || trimmed.back() != ']') {
    throw std::runtime_error(
      std::string("Expected a YAML list for '") + field_name + "'.");
  }

  const std::string body = TrimAscii(trimmed.substr(1, trimmed.size() - 2U));
  if (body.empty()) {
    return {};
  }

  std::vector<double> values;
  std::stringstream stream(body);
  std::string token;
  while (std::getline(stream, token, ',')) {
    token = TrimAscii(token);
    if (token.empty()) {
      throw std::runtime_error(
        std::string("Encountered an empty numeric value while parsing '") + field_name + "'.");
    }

    std::size_t consumed = 0U;
    const double parsed_value = std::stod(token, &consumed);
    if (consumed != token.size()) {
      throw std::runtime_error(
        std::string("Failed to fully parse numeric value '") + token + "' for '" + field_name + "'.");
    }

    values.push_back(parsed_value);
  }

  return values;
}

int ParseYamlNonNegativeInt(const std::string & raw_value, const char * field_name)
{
  const std::string trimmed = TrimAscii(raw_value);
  std::size_t consumed = 0U;
  const int parsed_value = std::stoi(trimmed, &consumed);
  if (consumed != trimmed.size()) {
    throw std::runtime_error(
      std::string("Failed to fully parse integer value '") + trimmed + "' for '" + field_name + "'.");
  }

  if (parsed_value < 0) {
    throw std::runtime_error(
      std::string("Expected a non-negative integer for '") + field_name + "'.");
  }

  return parsed_value;
}

bool ParseYamlBool(const std::string & raw_value, const char * field_name)
{
  const std::string normalized = NormalizeName(raw_value);
  if (normalized == "true" || normalized == "1" || normalized == "yes" || normalized == "on") {
    return true;
  }

  if (normalized == "false" || normalized == "0" || normalized == "no" || normalized == "off") {
    return false;
  }

  throw std::runtime_error(
    std::string("Failed to parse boolean value '") + raw_value + "' for '" + field_name + "'.");
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

bool IsHostBigEndian()
{
  constexpr std::uint16_t probe = 0x0102;
  return reinterpret_cast<const std::uint8_t *>(&probe)[0] == 0x01;
}

std::string SafeNodeString(INodeMap & node_map, const char * node_name)
{
  CStringPtr value_node = node_map.GetNode(node_name);
  if (!IsReadable(value_node)) {
    return "";
  }

  return value_node->GetValue().c_str();
}

bool SetEnumerationByName(INodeMap & node_map, const char * node_name, const std::string & entry_name)
{
  CEnumerationPtr enum_node = node_map.GetNode(node_name);
  if (!IsReadable(enum_node) || !IsWritable(enum_node)) {
    return false;
  }

  CEnumEntryPtr entry = enum_node->GetEntryByName(entry_name.c_str());
  if (!IsReadable(entry)) {
    return false;
  }

  enum_node->SetIntValue(entry->GetValue());
  return true;
}

bool SetBooleanByName(INodeMap & node_map, const char * node_name, bool value)
{
  CBooleanPtr bool_node = node_map.GetNode(node_name);
  if (!IsWritable(bool_node)) {
    return false;
  }

  bool_node->SetValue(value, true);
  return true;
}

bool EnumerationContains(INodeMap & node_map, const char * node_name, const std::string & entry_name)
{
  CEnumerationPtr enum_node = node_map.GetNode(node_name);
  if (!IsReadable(enum_node)) {
    return false;
  }

  CEnumEntryPtr entry = enum_node->GetEntryByName(entry_name.c_str());
  return IsReadable(entry);
}

ColorProcessingAlgorithm ParseColorProcessing(const std::string & value)
{
  const std::string normalized = NormalizeName(value);

  if (normalized == "nearestneighbor") {
    return Spinnaker::SPINNAKER_COLOR_PROCESSING_ALGORITHM_NEAREST_NEIGHBOR;
  }

  if (normalized == "nearestneighboravg") {
    return Spinnaker::SPINNAKER_COLOR_PROCESSING_ALGORITHM_NEAREST_NEIGHBOR_AVG;
  }

  if (normalized == "bilinear") {
    return Spinnaker::SPINNAKER_COLOR_PROCESSING_ALGORITHM_BILINEAR;
  }

  if (normalized == "edgesensing") {
    return Spinnaker::SPINNAKER_COLOR_PROCESSING_ALGORITHM_EDGE_SENSING;
  }

  if (normalized == "ipp") {
    return Spinnaker::SPINNAKER_COLOR_PROCESSING_ALGORITHM_IPP;
  }

  if (normalized == "directionalfilter") {
    return Spinnaker::SPINNAKER_COLOR_PROCESSING_ALGORITHM_DIRECTIONAL_FILTER;
  }

  if (normalized == "rigorous") {
    return Spinnaker::SPINNAKER_COLOR_PROCESSING_ALGORITHM_RIGOROUS;
  }

  if (normalized == "weighteddirectionalfilter") {
    return Spinnaker::SPINNAKER_COLOR_PROCESSING_ALGORITHM_WEIGHTED_DIRECTIONAL_FILTER;
  }

  return Spinnaker::SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR;
}

std::string NormalizePixelFormatParameter(std::string value)
{
  constexpr const char spinnaker_prefix[] = "Spinnaker::PixelFormat_";
  constexpr const char plain_prefix[] = "PixelFormat_";

  if (value.rfind(spinnaker_prefix, 0) == 0) {
    return value.substr(sizeof(spinnaker_prefix) - 1);
  }

  if (value.rfind(plain_prefix, 0) == 0) {
    return value.substr(sizeof(plain_prefix) - 1);
  }

  return value;
}

std::string NormalizeHardwareTriggerRole(const std::string & value)
{
  const std::string normalized = NormalizeName(value);
  if (normalized.empty() || normalized == "none" || normalized == "off" ||
    normalized == "disabled" || normalized == "disable")
  {
    return "none";
  }

  if (normalized == "master" || normalized == "bfsmaster") {
    return "master";
  }

  if (normalized == "slave" || normalized == "bfsslave") {
    return "slave";
  }

  throw std::runtime_error(
          "hardware_trigger.role must be one of none, master, or slave; got '" + value + "'.");
}

std::string NormalizePtpActionRole(const std::string & value)
{
  const std::string normalized = NormalizeName(value);
  if (normalized.empty() || normalized == "none" || normalized == "off" ||
    normalized == "disabled" || normalized == "disable")
  {
    return "none";
  }

  if (normalized == "receiver" || normalized == "receive" || normalized == "slave") {
    return "receiver";
  }

  if (normalized == "sender" || normalized == "send" || normalized == "master") {
    return "sender";
  }

  throw std::runtime_error(
          "ptp_action.role must be one of none, receiver, or sender; got '" + value + "'.");
}

// header.stamp source. Empty keeps the older boolean use_camera_timestamp_in_header.
std::string NormalizeTimestampMode(const std::string & value, bool legacy_use_camera_timestamp)
{
  const std::string normalized = NormalizeName(value);
  if (normalized.empty()) {
    return legacy_use_camera_timestamp ? "camera_first_frame" : "host";
  }
  if (normalized == "host") {
    return "host";
  }
  if (normalized == "cameralatched") {
    return "camera_latched";
  }
  if (normalized == "camerafirstframe") {
    return "camera_first_frame";
  }
  throw std::runtime_error(
          "timestamp.mode must be one of host, camera_latched, or camera_first_frame; got '" +
          value + "'.");
}

std::string NormalizeExposureLatch(const std::string & value)
{
  const std::string normalized = NormalizeName(value);
  if (normalized.empty() || normalized == "start") {
    return "start";
  }
  if (normalized == "end") {
    return "end";
  }
  throw std::runtime_error(
          "timestamp.exposure_latch must be start or end; got '" + value + "'.");
}

std::uint32_t ValidateUint32Parameter(std::int64_t value, const char * parameter_name)
{
  if (value < 0 || value > static_cast<std::int64_t>(std::numeric_limits<std::uint32_t>::max())) {
    throw std::runtime_error(
            std::string(parameter_name) + " must be between 0 and 4294967295.");
  }

  return static_cast<std::uint32_t>(value);
}

const char * ActionCommandStatusName(Spinnaker::ActionCommandStatus status)
{
  switch (status) {
    case Spinnaker::SPINNAKER_ACTION_COMMAND_STATUS_OK:
      return "OK";
    case Spinnaker::SPINNAKER_ACTION_COMMAND_STATUS_NO_REF_TIME:
      return "NO_REF_TIME";
    case Spinnaker::SPINNAKER_ACTION_COMMAND_STATUS_OVERFLOW:
      return "OVERFLOW";
    case Spinnaker::SPINNAKER_ACTION_COMMAND_STATUS_ACTION_LATE:
      return "ACTION_LATE";
    case Spinnaker::SPINNAKER_ACTION_COMMAND_STATUS_ERROR:
      return "ERROR";
  }

  return "UNKNOWN";
}

std::uint32_t ParseIpv4Address(std::string value, const char * parameter_name)
{
  value = TrimAscii(value);
  if (value.empty()) {
    throw std::runtime_error(std::string(parameter_name) + " must not be empty.");
  }

  std::array<unsigned long, 4> octets{};
  std::stringstream stream(value);
  std::string token;
  for (std::size_t index = 0; index < octets.size(); ++index) {
    if (!std::getline(stream, token, '.')) {
      throw std::runtime_error(
              std::string(parameter_name) + " must be an IPv4 address; got '" + value + "'.");
    }

    token = TrimAscii(token);
    if (token.empty()) {
      throw std::runtime_error(
              std::string(parameter_name) + " contains an empty IPv4 octet.");
    }

    std::size_t consumed = 0U;
    const unsigned long octet = std::stoul(token, &consumed);
    if (consumed != token.size() || octet > 255UL) {
      throw std::runtime_error(
              std::string(parameter_name) + " contains invalid IPv4 octet '" + token + "'.");
    }

    octets[index] = octet;
  }

  if (std::getline(stream, token, '.')) {
    throw std::runtime_error(
            std::string(parameter_name) + " must contain exactly four IPv4 octets; got '" + value + "'.");
  }

  return (static_cast<std::uint32_t>(octets[0]) << 24U) |
         (static_cast<std::uint32_t>(octets[1]) << 16U) |
         (static_cast<std::uint32_t>(octets[2]) << 8U) |
         static_cast<std::uint32_t>(octets[3]);
}

std::string FormatIpv4Address(std::uint32_t value)
{
  std::ostringstream stream;
  stream << ((value >> 24U) & 0xffU) << "."
         << ((value >> 16U) & 0xffU) << "."
         << ((value >> 8U) & 0xffU) << "."
         << (value & 0xffU);
  return stream.str();
}

bool IsLinkLocalIpv4(std::uint32_t value)
{
  return ((value >> 24U) & 0xffU) == 169U && ((value >> 16U) & 0xffU) == 254U;
}

std::vector<std::string> PixelFormatParameterCandidates(const std::string & value)
{
  const std::string normalized = NormalizePixelFormatParameter(value);
  std::vector<std::string> candidates;
  candidates.push_back(normalized);

  if (normalized == "RGB8") {
    candidates.push_back("RGB8Packed");
  } else if (normalized == "RGB8Packed") {
    candidates.push_back("RGB8");
  } else if (normalized == "BGR8Packed") {
    candidates.push_back("BGR8");
  } else if (normalized == "YUV422") {
    candidates.push_back("YUV422Packed");
  }

  return candidates;
}

struct RawOutputSpec
{
  PixelFormatEnums target_pixel_format;
  std::string encoding;
  bool requires_conversion;
};

struct PreparedRawImage
{
  ImagePtr image;
  std::string encoding;
};

template<std::size_t N>
std::array<double, N> ToFixedArray(
  const std::vector<double> & values,
  const char * parameter_name)
{
  if (values.size() != N) {
    throw std::runtime_error(
      std::string("Parameter '") + parameter_name + "' must contain exactly " +
      std::to_string(N) + " values.");
  }

  std::array<double, N> result{};
  std::copy(values.begin(), values.end(), result.begin());
  return result;
}

enum class ControlMapKind
{
  Camera,
  Stream,
  TlDevice
};

enum class ControlValueKind
{
  Boolean,
  Integer,
  Float,
  Enumeration,
  String
};

struct ControlBinding
{
  ControlMapKind map_kind;
  ControlValueKind value_kind;
  std::string node_name;
};

struct NamedStringParameter
{
  std::string name;
  std::string value;
};

struct NamedBoolParameter
{
  std::string name;
  bool value;
};

std::optional<RawOutputSpec> RawOutputSpecForPixelFormat(PixelFormatEnums pixel_format)
{
  switch (pixel_format) {
    case Spinnaker::PixelFormat_Mono8:
      return RawOutputSpec{Spinnaker::PixelFormat_Mono8, sensor_msgs::image_encodings::MONO8, false};
    case Spinnaker::PixelFormat_Mono16:
      return RawOutputSpec{Spinnaker::PixelFormat_Mono16, sensor_msgs::image_encodings::MONO16, false};
    case Spinnaker::PixelFormat_Mono10:
    case Spinnaker::PixelFormat_Mono10p:
    case Spinnaker::PixelFormat_Mono10Packed:
    case Spinnaker::PixelFormat_Mono12:
    case Spinnaker::PixelFormat_Mono12p:
    case Spinnaker::PixelFormat_Mono12Packed:
    case Spinnaker::PixelFormat_Mono14:
      return RawOutputSpec{Spinnaker::PixelFormat_Mono16, sensor_msgs::image_encodings::MONO16, true};
    case Spinnaker::PixelFormat_BayerRG8:
      return RawOutputSpec{Spinnaker::PixelFormat_BayerRG8, sensor_msgs::image_encodings::BAYER_RGGB8, false};
    case Spinnaker::PixelFormat_BayerBG8:
      return RawOutputSpec{Spinnaker::PixelFormat_BayerBG8, sensor_msgs::image_encodings::BAYER_BGGR8, false};
    case Spinnaker::PixelFormat_BayerGB8:
      return RawOutputSpec{Spinnaker::PixelFormat_BayerGB8, sensor_msgs::image_encodings::BAYER_GBRG8, false};
    case Spinnaker::PixelFormat_BayerGR8:
      return RawOutputSpec{Spinnaker::PixelFormat_BayerGR8, sensor_msgs::image_encodings::BAYER_GRBG8, false};
    case Spinnaker::PixelFormat_BayerRG16:
      return RawOutputSpec{Spinnaker::PixelFormat_BayerRG16, sensor_msgs::image_encodings::BAYER_RGGB16, false};
    case Spinnaker::PixelFormat_BayerBG16:
      return RawOutputSpec{Spinnaker::PixelFormat_BayerBG16, sensor_msgs::image_encodings::BAYER_BGGR16, false};
    case Spinnaker::PixelFormat_BayerGB16:
      return RawOutputSpec{Spinnaker::PixelFormat_BayerGB16, sensor_msgs::image_encodings::BAYER_GBRG16, false};
    case Spinnaker::PixelFormat_BayerGR16:
      return RawOutputSpec{Spinnaker::PixelFormat_BayerGR16, sensor_msgs::image_encodings::BAYER_GRBG16, false};
    case Spinnaker::PixelFormat_BayerRG10:
    case Spinnaker::PixelFormat_BayerRG10p:
    case Spinnaker::PixelFormat_BayerRG10Packed:
    case Spinnaker::PixelFormat_BayerRG12:
    case Spinnaker::PixelFormat_BayerRG12p:
    case Spinnaker::PixelFormat_BayerRG12Packed:
      return RawOutputSpec{Spinnaker::PixelFormat_BayerRG16, sensor_msgs::image_encodings::BAYER_RGGB16, true};
    case Spinnaker::PixelFormat_BayerBG10:
    case Spinnaker::PixelFormat_BayerBG10p:
    case Spinnaker::PixelFormat_BayerBG10Packed:
    case Spinnaker::PixelFormat_BayerBG12:
    case Spinnaker::PixelFormat_BayerBG12p:
    case Spinnaker::PixelFormat_BayerBG12Packed:
      return RawOutputSpec{Spinnaker::PixelFormat_BayerBG16, sensor_msgs::image_encodings::BAYER_BGGR16, true};
    case Spinnaker::PixelFormat_BayerGB10:
    case Spinnaker::PixelFormat_BayerGB10p:
    case Spinnaker::PixelFormat_BayerGB10Packed:
    case Spinnaker::PixelFormat_BayerGB12:
    case Spinnaker::PixelFormat_BayerGB12p:
    case Spinnaker::PixelFormat_BayerGB12Packed:
      return RawOutputSpec{Spinnaker::PixelFormat_BayerGB16, sensor_msgs::image_encodings::BAYER_GBRG16, true};
    case Spinnaker::PixelFormat_BayerGR10:
    case Spinnaker::PixelFormat_BayerGR10p:
    case Spinnaker::PixelFormat_BayerGR10Packed:
    case Spinnaker::PixelFormat_BayerGR12:
    case Spinnaker::PixelFormat_BayerGR12p:
    case Spinnaker::PixelFormat_BayerGR12Packed:
      return RawOutputSpec{Spinnaker::PixelFormat_BayerGR16, sensor_msgs::image_encodings::BAYER_GRBG16, true};
    case Spinnaker::PixelFormat_RGB8:
    case Spinnaker::PixelFormat_RGB8Packed:
      return RawOutputSpec{Spinnaker::PixelFormat_RGB8, sensor_msgs::image_encodings::RGB8, false};
    case Spinnaker::PixelFormat_BGR8:
      return RawOutputSpec{Spinnaker::PixelFormat_BGR8, sensor_msgs::image_encodings::BGR8, false};
    case Spinnaker::PixelFormat_YUV422Packed:
    case Spinnaker::PixelFormat_YUV422_8:
    case Spinnaker::PixelFormat_YUV422_8_UYVY:
      return RawOutputSpec{pixel_format, sensor_msgs::image_encodings::YUV422, false};
    default:
      return std::nullopt;
  }
}

std::vector<std::string> PreferredPixelFormats()
{
  return {
    "BayerRG8",
    "BayerBG8",
    "BayerGB8",
    "BayerGR8",
    "Mono8",
    "RGB8",
    "BGR8",
    "BayerRG16",
    "BayerBG16",
    "BayerGB16",
    "BayerGR16",
    "Mono16"
  };
}

}  // namespace

class FlirSpinnakerCameraNode : public rclcpp::Node
{
public:
  FlirSpinnakerCameraNode()
  : Node("flir_spinnaker_camera"),
    publish_raw_(declare_parameter<bool>("publish_raw", true)),
    publish_camera_info_(declare_parameter<bool>("publish_camera_info", true)),
    publish_metadata_(declare_parameter<bool>("publish_metadata", true)),
    publish_rgb_compressed_(declare_parameter<bool>("publish_rgb_compressed", true)),
    publisher_qos_reliability_(declare_parameter<std::string>("publisher_qos_reliability", "reliable")),
    publisher_qos_depth_(declare_parameter<int>("publisher_qos_depth", 20)),
    frame_id_(declare_parameter<std::string>("frame_id", "flir_camera_optical_frame")),
    camera_serial_(declare_parameter<std::string>("camera_serial", "")),
    camera_index_(declare_parameter<int>("camera_index", 0)),
    camera_init_max_attempts_(declare_parameter<int>("camera_init.max_attempts", 10)),
    camera_init_retry_delay_ms_(declare_parameter<int>("camera_init.retry_delay_ms", 2000)),
    acquisition_timeout_ms_(declare_parameter<int>("acquisition_timeout_ms", 1000)),
    use_camera_timestamp_in_header_(declare_parameter<bool>("use_camera_timestamp_in_header", false)),
    timestamp_mode_(NormalizeTimestampMode(
        declare_parameter<std::string>("timestamp.mode", ""), use_camera_timestamp_in_header_)),
    timestamp_exposure_latch_(NormalizeExposureLatch(
        declare_parameter<std::string>("timestamp.exposure_latch", "start"))),
    timestamp_trigger_grid_hz_(declare_parameter<double>("timestamp.trigger_grid_hz", 0.0)),
    timestamp_trigger_grid_offset_ns_(declare_parameter<std::int64_t>(
        "timestamp.trigger_grid_offset_ns", 0)),
    timestamp_grid_warn_ms_(declare_parameter<double>("timestamp.grid_warn_ms", 3.0)),
    timestamp_capture_offset_ns_(declare_parameter<std::int64_t>("timestamp.capture_offset_ns", 0)),
    timestamp_latch_interval_sec_(declare_parameter<double>("timestamp.latch_interval_sec", 2.0)),
    chunk_data_enable_(declare_parameter<bool>("chunk_data.enable", true)),
    stream_stats_interval_sec_(declare_parameter<double>("stream_stats.interval_sec", 30.0)),
    camera_info_yaml_path_(declare_parameter<std::string>("camera_info.yaml_path", "")),
    auto_pixel_format_(declare_parameter<bool>("auto_pixel_format", true)),
    pixel_format_(declare_parameter<std::string>("pixel_format", "")),
    buffer_handling_mode_(declare_parameter<std::string>("buffer_handling_mode", "OldestFirst")),
    hardware_trigger_role_(NormalizeHardwareTriggerRole(
        declare_parameter<std::string>("hardware_trigger.role", "none"))),
    hardware_trigger_master_output_line_(declare_parameter<std::string>(
        "hardware_trigger.master.output_line", "Line1")),
    hardware_trigger_master_line_source_(declare_parameter<std::string>(
        "hardware_trigger.master.line_source", "ExposureActive")),
    hardware_trigger_master_line_source_fallbacks_(declare_parameter<std::vector<std::string>>(
        "hardware_trigger.master.line_source_fallbacks",
        std::vector<std::string>{"FrameTriggerWait", "UserOutput0"})),
    hardware_trigger_master_enable_3v3_(declare_parameter<bool>(
        "hardware_trigger.master.enable_3v3", true)),
    hardware_trigger_master_require_3v3_(declare_parameter<bool>(
        "hardware_trigger.master.require_3v3", true)),
    hardware_trigger_master_3v3_line_(declare_parameter<std::string>(
        "hardware_trigger.master.line_3v3", "Line2")),
    hardware_trigger_master_3v3_enable_nodes_(declare_parameter<std::vector<std::string>>(
        "hardware_trigger.master.line_3v3_enable_nodes",
        std::vector<std::string>{"V3_3Enable", "Line3V3Enable", "LineVoltageEnable"})),
    hardware_trigger_slave_trigger_source_(declare_parameter<std::string>(
        "hardware_trigger.slave.trigger_source", "Line3")),
    hardware_trigger_slave_trigger_activation_(declare_parameter<std::string>(
        "hardware_trigger.slave.trigger_activation", "RisingEdge")),
    hardware_trigger_slave_trigger_overlap_(declare_parameter<std::string>(
        "hardware_trigger.slave.trigger_overlap", "ReadOut")),
    network_force_ip_enable_(declare_parameter<bool>("network.force_ip.enable", false)),
    network_force_ip_address_(declare_parameter<std::string>("network.force_ip.address", "")),
    network_force_ip_subnet_mask_(declare_parameter<std::string>(
        "network.force_ip.subnet_mask", "255.255.255.0")),
    network_force_ip_gateway_(declare_parameter<std::string>(
        "network.force_ip.gateway", "0.0.0.0")),
    network_force_ip_only_if_link_local_(declare_parameter<bool>(
        "network.force_ip.only_if_link_local", true)),
    network_force_ip_wait_after_ms_(declare_parameter<int>(
        "network.force_ip.wait_after_ms", 1500)),
    network_force_ip_rediscovery_timeout_ms_(declare_parameter<int>(
        "network.force_ip.rediscovery_timeout_ms", 5000)),
    ptp_enabled_(declare_parameter<bool>("ptp.enable", false)),
    ptp_mode_(declare_parameter<std::string>("ptp.mode", "SlaveOnly")),
    ptp_wait_for_sync_(declare_parameter<bool>("ptp.wait_for_sync", true)),
    ptp_require_sync_(declare_parameter<bool>("ptp.require_sync", true)),
    ptp_sync_timeout_ms_(declare_parameter<int>("ptp.sync_timeout_ms", 10000)),
    ptp_sync_poll_ms_(declare_parameter<int>("ptp.sync_poll_ms", 250)),
    ptp_accepted_statuses_(declare_parameter<std::vector<std::string>>(
        "ptp.accepted_statuses",
        std::vector<std::string>{"Slave"})),
    ptp_action_role_(NormalizePtpActionRole(
        declare_parameter<std::string>("ptp_action.role", "none"))),
    ptp_action_selector_(declare_parameter<std::string>("ptp_action.selector", "Action0")),
    ptp_action_trigger_selector_(declare_parameter<std::string>(
        "ptp_action.trigger_selector", "FrameStart")),
    ptp_action_trigger_source_(declare_parameter<std::string>(
        "ptp_action.trigger_source", "Action0")),
    ptp_action_trigger_activation_(declare_parameter<std::string>(
        "ptp_action.trigger_activation", "RisingEdge")),
    ptp_action_trigger_overlap_(declare_parameter<std::string>(
        "ptp_action.trigger_overlap", "ReadOut")),
    ptp_action_device_key_(ValidateUint32Parameter(
        declare_parameter<std::int64_t>("ptp_action.device_key", 1),
        "ptp_action.device_key")),
    ptp_action_group_key_(ValidateUint32Parameter(
        declare_parameter<std::int64_t>("ptp_action.group_key", 1),
        "ptp_action.group_key")),
    ptp_action_group_mask_(ValidateUint32Parameter(
        declare_parameter<std::int64_t>("ptp_action.group_mask", 4294967295LL),
        "ptp_action.group_mask")),
    ptp_action_rate_hz_(declare_parameter<double>("ptp_action.rate_hz", 10.0)),
    ptp_action_schedule_ahead_ms_(declare_parameter<double>(
        "ptp_action.schedule_ahead_ms", 100.0)),
    ptp_action_start_delay_ms_(declare_parameter<double>("ptp_action.start_delay_ms", 1000.0)),
    ptp_action_align_to_second_(declare_parameter<bool>("ptp_action.align_to_second", true)),
    ptp_action_request_ack_(declare_parameter<bool>("ptp_action.request_ack", false)),
    ptp_action_expected_ack_count_(declare_parameter<int>("ptp_action.expected_ack_count", 0)),
    ptp_action_log_interval_sec_(declare_parameter<double>("ptp_action.log_interval_sec", 5.0)),
    color_processing_(declare_parameter<std::string>("color_processing", "hq_linear")),
    rgb_compression_format_(declare_parameter<std::string>("rgb_compression_format", "jpeg")),
    rgb_jpeg_quality_(declare_parameter<int>("rgb_jpeg_quality", 90)),
    rgb_png_compression_level_(declare_parameter<int>("rgb_png_compression_level", 3)),
    // "gpu": demosaic + JPEG on the GPU (nvJPEG/NPP) instead of the acquisition
    // thread; falls back to the CPU path if the GPU is unavailable.
    rgb_encoder_(declare_parameter<std::string>("rgb_encoder", "cpu")),
    gpu_demosaic_(declare_parameter<bool>("gpu_demosaic", true)),
    gpu_device_(declare_parameter<int>("gpu_device", 0))
  {
    if (!publish_raw_ && !publish_camera_info_ && !publish_metadata_ && !publish_rgb_compressed_) {
      throw std::runtime_error(
        "At least one of publish_raw, publish_camera_info, publish_metadata, or publish_rgb_compressed must be true.");
    }

    if (ptp_action_role_ != "none") {
      if (hardware_trigger_role_ != "none") {
        throw std::runtime_error(
                "ptp_action.role cannot be combined with hardware_trigger.role. "
                "Use PTP action triggering or BFS GPIO triggering, not both.");
      }

      if (!ptp_enabled_) {
        ptp_enabled_ = true;
        set_parameter(rclcpp::Parameter("ptp.enable", true));
        RCLCPP_WARN(
          get_logger(),
          "ptp_action.role='%s' requires PTP. Enabling ptp.enable for this node.",
          ptp_action_role_.c_str());
      }
    }

    if (ptp_sync_timeout_ms_ < 0 || ptp_sync_poll_ms_ <= 0) {
      throw std::runtime_error("ptp.sync_timeout_ms must be non-negative and ptp.sync_poll_ms must be positive.");
    }

    if (network_force_ip_enable_) {
      if (network_force_ip_address_.empty()) {
        throw std::runtime_error(
                "network.force_ip.address must be set when network.force_ip.enable is true.");
      }
      if (network_force_ip_wait_after_ms_ < 0) {
        throw std::runtime_error("network.force_ip.wait_after_ms must be non-negative.");
      }
      if (network_force_ip_rediscovery_timeout_ms_ < 0) {
        throw std::runtime_error(
                "network.force_ip.rediscovery_timeout_ms must be non-negative.");
      }
    }

    if (ptp_action_role_ == "sender") {
      if (ptp_action_rate_hz_ <= 0.0) {
        throw std::runtime_error("ptp_action.rate_hz must be positive when ptp_action.role is sender.");
      }
      if (ptp_action_schedule_ahead_ms_ <= 0.0) {
        throw std::runtime_error(
                "ptp_action.schedule_ahead_ms must be positive when ptp_action.role is sender.");
      }
      if (ptp_action_start_delay_ms_ < 0.0 || ptp_action_log_interval_sec_ < 0.0) {
        throw std::runtime_error(
                "ptp_action.start_delay_ms and ptp_action.log_interval_sec must be non-negative.");
      }
      if (ptp_action_expected_ack_count_ < 0) {
        throw std::runtime_error("ptp_action.expected_ack_count must be non-negative.");
      }
    }

    const auto qos = BuildPublisherQoS();

    if (publish_raw_) {
      raw_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", qos);
    }

    if (publish_camera_info_) {
      camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", qos);
    }

    if (publish_metadata_) {
      metadata_pub_ = create_publisher<flir_spinnaker_camera::msg::FlirMetadata>(
        "image_raw/metadata",
        qos);
    }

    if (publish_rgb_compressed_) {
      rgb_compressed_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        "image_rgb/compressed",
        qos);
    }

    rgb_compression_format_ = NormalizeCompressionFormat(rgb_compression_format_);
    pixel_format_ = NormalizePixelFormatParameter(pixel_format_);
    image_processor_.SetColorProcessing(ParseColorProcessing(color_processing_));
    InitializeGpuEncoder();

    camera_info_distortion_model_ = declare_parameter<std::string>(
      "camera_info.distortion_model",
      "plumb_bob");
    camera_info_d_ = declare_parameter<std::vector<double>>(
      "camera_info.d",
      std::vector<double>{});
    camera_info_k_ = ToFixedArray<9>(
      declare_parameter<std::vector<double>>(
        "camera_info.k",
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      "camera_info.k");
    camera_info_r_ = ToFixedArray<9>(
      declare_parameter<std::vector<double>>(
        "camera_info.r",
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      "camera_info.r");
    camera_info_p_ = ToFixedArray<12>(
      declare_parameter<std::vector<double>>(
        "camera_info.p",
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      "camera_info.p");

    const int camera_info_binning_x = declare_parameter<int>("camera_info.binning_x", 0);
    const int camera_info_binning_y = declare_parameter<int>("camera_info.binning_y", 0);
    const int camera_info_roi_x_offset = declare_parameter<int>("camera_info.roi.x_offset", 0);
    const int camera_info_roi_y_offset = declare_parameter<int>("camera_info.roi.y_offset", 0);
    const int camera_info_roi_height = declare_parameter<int>("camera_info.roi.height", 0);
    const int camera_info_roi_width = declare_parameter<int>("camera_info.roi.width", 0);
    camera_info_roi_do_rectify_ = declare_parameter<bool>("camera_info.roi.do_rectify", false);

    if (camera_info_binning_x < 0 || camera_info_binning_y < 0 ||
      camera_info_roi_x_offset < 0 || camera_info_roi_y_offset < 0 ||
      camera_info_roi_height < 0 || camera_info_roi_width < 0)
    {
      throw std::runtime_error("camera_info binning and ROI parameters must be non-negative.");
    }

    camera_info_binning_x_ = static_cast<std::uint32_t>(camera_info_binning_x);
    camera_info_binning_y_ = static_cast<std::uint32_t>(camera_info_binning_y);
    camera_info_roi_x_offset_ = static_cast<std::uint32_t>(camera_info_roi_x_offset);
    camera_info_roi_y_offset_ = static_cast<std::uint32_t>(camera_info_roi_y_offset);
    camera_info_roi_height_ = static_cast<std::uint32_t>(camera_info_roi_height);
    camera_info_roi_width_ = static_cast<std::uint32_t>(camera_info_roi_width);

    if (!camera_info_yaml_path_.empty()) {
      ApplyCameraInfoYamlOverrides(camera_info_yaml_path_);
    }

    RCLCPP_INFO(
      get_logger(),
      "RGB compressed publish=%s format=%s qos_reliability=%s qos_depth=%d",
      publish_rgb_compressed_ ? "true" : "false",
      rgb_compression_format_.c_str(),
      NormalizeQoSReliability(publisher_qos_reliability_).c_str(),
      publisher_qos_depth_);

    try {
      InitializeCamera();
      running_.store(true);
      // Before the acquisition thread: the camera->host mapping must exist before the first frame is
      // stamped, or the first frames of a recording carry host time and the rest exposure time.
      StartClockSync();
      acquisition_thread_ = std::thread(&FlirSpinnakerCameraNode::AcquisitionLoop, this);
      StartPtpActionSender();
    } catch (...) {
      ShutdownCamera();
      throw;
    }
  }

  ~FlirSpinnakerCameraNode() override
  {
    ShutdownCamera();
  }

private:
  static std::string ControlPrefix(ControlMapKind map_kind)
  {
    switch (map_kind) {
      case ControlMapKind::Camera:
        return "camera";
      case ControlMapKind::Stream:
        return "stream";
      case ControlMapKind::TlDevice:
        return "tl_device";
    }

    return "camera";
  }

  static bool IsManagedControlNode(ControlMapKind map_kind, const std::string & node_name)
  {
    if (map_kind == ControlMapKind::Camera) {
      return node_name == "PixelFormat" || node_name == "AcquisitionMode";
    }

    if (map_kind == ControlMapKind::Stream) {
      return node_name == "StreamBufferHandlingMode";
    }

    return false;
  }

  static std::optional<ControlValueKind> ControlValueKindForNode(const CNodePtr & node)
  {
    switch (node->GetPrincipalInterfaceType()) {
      case Spinnaker::GenApi::intfIBoolean:
        return ControlValueKind::Boolean;
      case Spinnaker::GenApi::intfIInteger:
        return ControlValueKind::Integer;
      case Spinnaker::GenApi::intfIFloat:
        return ControlValueKind::Float;
      case Spinnaker::GenApi::intfIEnumeration:
        return ControlValueKind::Enumeration;
      case Spinnaker::GenApi::intfIString:
        return ControlValueKind::String;
      default:
        return std::nullopt;
    }
  }

  INodeMap & ResolveNodeMap(ControlMapKind map_kind) const
  {
    switch (map_kind) {
      case ControlMapKind::Camera:
        if (camera_node_map_ == nullptr) {
          throw std::runtime_error("Camera node map is not available.");
        }
        return *camera_node_map_;
      case ControlMapKind::Stream:
        if (stream_node_map_ == nullptr) {
          throw std::runtime_error("Stream node map is not available.");
        }
        return *stream_node_map_;
      case ControlMapKind::TlDevice:
        if (tl_device_node_map_ == nullptr) {
          throw std::runtime_error("Transport-layer device node map is not available.");
        }
        return *tl_device_node_map_;
    }

    throw std::runtime_error("Unexpected control map kind.");
  }

  std::string BuildControlParameterName(ControlMapKind map_kind, const std::string & node_name) const
  {
    return ControlPrefix(map_kind) + "." + node_name;
  }

  rcl_interfaces::msg::ParameterDescriptor BuildControlParameterDescriptor(
    const CNodePtr & node,
    ControlValueKind value_kind) const
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;

    std::ostringstream description;
    description << "GenICam node '" << node->GetName().c_str() << "'";

    const std::string display_name = node->GetDisplayName().c_str();
    if (!display_name.empty() && display_name != node->GetName().c_str()) {
      description << " (" << display_name << ")";
    }

    const std::string tooltip = node->GetToolTip().c_str();
    if (!tooltip.empty()) {
      description << ". " << tooltip;
    }

    if (value_kind == ControlValueKind::Enumeration) {
      CEnumerationPtr enum_node = static_cast<CEnumerationPtr>(node);
      if (IsReadable(enum_node)) {
        StringList_t symbolics;
        enum_node->GetSymbolics(symbolics);
        if (!symbolics.empty()) {
          description << " Valid values: ";
          for (std::size_t index = 0; index < symbolics.size(); ++index) {
            if (index > 0U) {
              description << ", ";
            }
            description << symbolics[index].c_str();
          }
          description << ".";
        }
      }
    }

    descriptor.description = description.str();
    return descriptor;
  }

  bool ShouldExposeControlNode(const CNodePtr & node, ControlMapKind map_kind) const
  {
    if (!node || !IsAvailable(node) || !IsReadable(node) || !IsWritable(node)) {
      return false;
    }

    if (node->GetVisibility() == Spinnaker::GenApi::Invisible) {
      return false;
    }

    const std::string node_name = node->GetName().c_str();
    if (node_name.empty() || IsManagedControlNode(map_kind, node_name)) {
      return false;
    }

    return ControlValueKindForNode(node).has_value();
  }

  void ApplyControlParameterValue(const ControlBinding & binding, const rclcpp::Parameter & parameter)
  {
    INodeMap & node_map = ResolveNodeMap(binding.map_kind);
    const char * node_name = binding.node_name.c_str();

    switch (binding.value_kind) {
      case ControlValueKind::Boolean:
      {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
          throw std::runtime_error("expected a bool value");
        }

        CBooleanPtr node = node_map.GetNode(node_name);
        if (!IsWritable(node)) {
          throw std::runtime_error("node is not writable in the current camera state");
        }

        node->SetValue(parameter.as_bool(), true);
        return;
      }
      case ControlValueKind::Integer:
      {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
          throw std::runtime_error("expected an integer value");
        }

        CIntegerPtr node = node_map.GetNode(node_name);
        if (!IsWritable(node)) {
          throw std::runtime_error("node is not writable in the current camera state");
        }

        node->SetValue(parameter.as_int(), true);
        return;
      }
      case ControlValueKind::Float:
      {
        double value = 0.0;
        if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
          value = parameter.as_double();
        } else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          value = static_cast<double>(parameter.as_int());
        } else {
          throw std::runtime_error("expected a floating-point value");
        }

        CFloatPtr node = node_map.GetNode(node_name);
        if (!IsWritable(node)) {
          throw std::runtime_error("node is not writable in the current camera state");
        }

        node->SetValue(value, true);
        return;
      }
      case ControlValueKind::Enumeration:
      {
        CEnumerationPtr node = node_map.GetNode(node_name);
        if (!IsWritable(node)) {
          throw std::runtime_error("node is not writable in the current camera state");
        }

        if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
          std::string value = parameter.as_string();
          if (binding.node_name == "PixelFormat") {
            value = NormalizePixelFormatParameter(value);
          }

          CEnumEntryPtr entry = node->GetEntryByName(value.c_str());
          if (!IsReadable(entry)) {
            throw std::runtime_error("unknown enum symbolic '" + value + "'");
          }

          node->SetIntValue(entry->GetValue(), true);
          return;
        }

        if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          node->SetIntValue(parameter.as_int(), true);
          return;
        }

        throw std::runtime_error("expected a string or integer enum value");
      }
      case ControlValueKind::String:
      {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
          throw std::runtime_error("expected a string value");
        }

        CStringPtr node = node_map.GetNode(node_name);
        if (!IsWritable(node)) {
          throw std::runtime_error("node is not writable in the current camera state");
        }

        node->SetValue(parameter.as_string().c_str(), true);
        return;
      }
    }

    throw std::runtime_error("unsupported control parameter type");
  }

  void RegisterControlParameter(
    const std::string & parameter_name,
    const ControlBinding & binding,
    const CNodePtr & node)
  {
    const auto descriptor = BuildControlParameterDescriptor(node, binding.value_kind);

    switch (binding.value_kind) {
      case ControlValueKind::Boolean:
      {
        CBooleanPtr value_node = ResolveNodeMap(binding.map_kind).GetNode(binding.node_name.c_str());
        const bool current = value_node->GetValue();
        const bool configured = declare_parameter<bool>(parameter_name, current, descriptor);
        control_bindings_.emplace(parameter_name, binding);
        if (configured != current) {
          pending_control_overrides_.emplace_back(parameter_name, configured);
        }
        return;
      }
      case ControlValueKind::Integer:
      {
        CIntegerPtr value_node = ResolveNodeMap(binding.map_kind).GetNode(binding.node_name.c_str());
        const std::int64_t current = value_node->GetValue();
        const std::int64_t configured = declare_parameter<std::int64_t>(parameter_name, current, descriptor);
        control_bindings_.emplace(parameter_name, binding);
        if (configured != current) {
          pending_control_overrides_.emplace_back(parameter_name, configured);
        }
        return;
      }
      case ControlValueKind::Float:
      {
        CFloatPtr value_node = ResolveNodeMap(binding.map_kind).GetNode(binding.node_name.c_str());
        const double current = value_node->GetValue();
        const double configured = declare_parameter<double>(parameter_name, current, descriptor);
        control_bindings_.emplace(parameter_name, binding);
        if (std::abs(configured - current) > 1e-12) {
          pending_control_overrides_.emplace_back(parameter_name, configured);
        }
        return;
      }
      case ControlValueKind::Enumeration:
      {
        CEnumerationPtr value_node = ResolveNodeMap(binding.map_kind).GetNode(binding.node_name.c_str());
        const std::string current = value_node->ToString().c_str();
        const std::string configured = declare_parameter<std::string>(parameter_name, current, descriptor);
        control_bindings_.emplace(parameter_name, binding);
        if (configured != current) {
          pending_control_overrides_.emplace_back(parameter_name, configured);
        }
        return;
      }
      case ControlValueKind::String:
      {
        CStringPtr value_node = ResolveNodeMap(binding.map_kind).GetNode(binding.node_name.c_str());
        const std::string current = value_node->GetValue().c_str();
        const std::string configured = declare_parameter<std::string>(parameter_name, current, descriptor);
        control_bindings_.emplace(parameter_name, binding);
        if (configured != current) {
          pending_control_overrides_.emplace_back(parameter_name, configured);
        }
        return;
      }
    }

    throw std::runtime_error("unsupported control parameter type");
  }

  std::size_t RegisterWritableControlParameters(
    INodeMap & node_map, ControlMapKind map_kind, bool log_summary = true)
  {
    NodeList_t nodes;
    node_map.GetNodes(nodes);

    std::size_t registered = 0U;
    for (const auto & raw_node : nodes) {
      try {
        CNodePtr node(raw_node);
        if (!ShouldExposeControlNode(node, map_kind)) {
          continue;
        }

        const auto value_kind = ControlValueKindForNode(node);
        if (!value_kind.has_value()) {
          continue;
        }

        const std::string node_name = node->GetName().c_str();
        const std::string parameter_name = BuildControlParameterName(map_kind, node_name);
        if (control_bindings_.find(parameter_name) != control_bindings_.end()) {
          continue;
        }

        RegisterControlParameter(parameter_name, ControlBinding{map_kind, *value_kind, node_name}, node);
        ++registered;
      } catch (const std::exception & exception) {
        RCLCPP_DEBUG(
          get_logger(),
          "Skipping a %s control parameter: %s",
          ControlPrefix(map_kind).c_str(),
          exception.what());
      }
    }

    if (log_summary) {
      RCLCPP_INFO(
        get_logger(),
        "Registered %zu writable %s control parameters.",
        registered,
        ControlPrefix(map_kind).c_str());
    }
    return registered;
  }

  // A node that is locked when the camera is opened is not registered, so its startup override
  // used to be dropped without a word: AcquisitionFrameRate while AcquisitionFrameRateEnable is
  // false, ExposureTime while ExposureAuto is not Off, Gamma while GammaEnable is false. Which of
  // them are locked depends on the state the camera kept from whatever ran it last (settings live
  // in camera RAM until a power cycle), so it hit some cameras and not others — on 2026-09-21 one
  // of fourteen came up without its 30 fps limit and took ~51 fps of the link. The overrides just
  // applied are what unlock them, so register what became writable and apply its overrides, until
  // nothing new shows up.
  void RegisterControlParametersUnlockedByOverrides()
  {
    for (int round = 0; round < 4; ++round) {
      const std::size_t added =
        RegisterWritableControlParameters(*camera_node_map_, ControlMapKind::Camera, false) +
        RegisterWritableControlParameters(*stream_node_map_, ControlMapKind::Stream, false) +
        RegisterWritableControlParameters(*tl_device_node_map_, ControlMapKind::TlDevice, false);
      if (added == 0U) {
        return;
      }
      RCLCPP_INFO(
        get_logger(),
        "Registered %zu control parameters unlocked by the startup overrides.",
        added);
      NormalizeFrameRateStartupOverrides();
      ApplyPendingControlOverrides();
    }
  }

  // Whatever is still unregistered never reached the camera. Say so loudly instead of starting a
  // camera that quietly ignores part of its configuration.
  void ReportIgnoredControlOverrides()
  {
    std::string ignored;
    std::size_t count = 0U;
    for (const auto & entry : get_node_parameters_interface()->get_parameter_overrides()) {
      const std::string & name = entry.first;
      std::optional<ControlMapKind> kind;
      for (const auto candidate : {ControlMapKind::Camera, ControlMapKind::Stream, ControlMapKind::TlDevice}) {
        if (name.rfind(ControlPrefix(candidate) + ".", 0) == 0) {
          kind = candidate;
        }
      }
      if (!kind.has_value() || control_bindings_.count(name) > 0U) {
        continue;
      }
      if (IsManagedControlNode(*kind, name.substr(ControlPrefix(*kind).size() + 1U))) {
        continue;
      }
      ignored += (count++ == 0U ? "" : ", ") + name;
    }
    if (count > 0U) {
      RCLCPP_ERROR(
        get_logger(),
        "%zu startup camera override(s) NOT applied — the GenICam node is locked, unavailable or "
        "absent in this camera state: %s",
        count,
        ignored.c_str());
    }
  }

  // Nodes the startup sequence writes on purpose after the camera.* overrides (PTP, hardware
  // trigger, PTP action, chunk data). Their final value is the node's decision, not a drift.
  static bool IsSetByStartupLogic(const std::string & node_name)
  {
    for (const char * prefix : {"Trigger", "Line", "GevIEEE1588", "Ptp", "Action", "UserOutput",
        "V3_3Enable", "Chunk", "Timestamp", "AcquisitionMode", "PixelFormat", "StreamBufferHandlingMode"})
    {
      if (node_name.rfind(prefix, 0) == 0) {
        return true;
      }
    }
    return false;
  }

  // What the camera holds for this binding, as text, when it differs from `wanted` beyond the
  // camera's own rounding (increment, or 0.5 % — AcquisitionFrameRate 30 is kept as 29.9952,
  // DeviceLinkThroughputLimit 75000000 as 75001631). nullopt when it matches or cannot be read.
  std::optional<std::string> ControlValueMismatch(
    const ControlBinding & binding, const rclcpp::Parameter & wanted)
  {
    try {
      INodeMap & node_map = ResolveNodeMap(binding.map_kind);
      CNodePtr node = node_map.GetNode(binding.node_name.c_str());
      if (!IsReadable(node)) {
        return std::nullopt;
      }
      switch (binding.value_kind) {
        case ControlValueKind::Boolean:
        {
          if (wanted.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
            return std::nullopt;
          }
          CBooleanPtr value_node = node_map.GetNode(binding.node_name.c_str());
          const bool current = value_node->GetValue();
          return current == wanted.as_bool() ? std::nullopt :
                 std::optional<std::string>(current ? "true" : "false");
        }
        case ControlValueKind::Integer:
        {
          if (wanted.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
            return std::nullopt;
          }
          CIntegerPtr value_node = node_map.GetNode(binding.node_name.c_str());
          const std::int64_t current = value_node->GetValue();
          std::int64_t increment = 1;
          try {
            increment = std::max<std::int64_t>(1, value_node->GetInc());
          } catch (const Spinnaker::Exception &) {
          }
          const double tolerance = std::max<double>(
            static_cast<double>(increment), 0.005 * std::abs(static_cast<double>(wanted.as_int())));
          return std::abs(static_cast<double>(current - wanted.as_int())) <= tolerance ?
                 std::nullopt : std::optional<std::string>(std::to_string(current));
        }
        case ControlValueKind::Float:
        {
          if (wanted.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
            return std::nullopt;
          }
          CFloatPtr value_node = node_map.GetNode(binding.node_name.c_str());
          const double current = value_node->GetValue();
          const double tolerance = std::max(1e-6, 0.005 * std::abs(wanted.as_double()));
          if (std::abs(current - wanted.as_double()) <= tolerance) {
            return std::nullopt;
          }
          std::ostringstream text;
          text << current;
          return text.str();
        }
        case ControlValueKind::Enumeration:
        {
          if (wanted.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
            return std::nullopt;
          }
          CEnumerationPtr value_node = node_map.GetNode(binding.node_name.c_str());
          const std::string current = value_node->ToString().c_str();
          return current == wanted.as_string() ? std::nullopt : std::optional<std::string>(current);
        }
        case ControlValueKind::String:
        {
          if (wanted.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
            return std::nullopt;
          }
          CStringPtr value_node = node_map.GetNode(binding.node_name.c_str());
          const std::string current = value_node->GetValue().c_str();
          return current == wanted.as_string() ? std::nullopt : std::optional<std::string>(current);
        }
      }
    } catch (const Spinnaker::Exception &) {
    }
    return std::nullopt;
  }

  // Read back every configured camera.* / stream.* / tl_device.* override once the whole startup
  // sequence has run, just before streaming (TL parameters lock while streaming). A value can be
  // written and still not stay: two GenICam nodes that are one register pair recompute each other,
  // and overrides of the same type go in name order. On 2026-09-22 DeviceLinkThroughputLimit
  // 75000000 went in before GevSCPD 0, the camera recomputed the limit to 125000000 on 13 of 14
  // cameras, and the simultaneous PTP-action bursts overflowed the switch — frames and GVCP
  // replies were lost and every camera came and went. Re-apply what drifted once, in priority
  // order, then report what still does not match.
  void VerifyControlOverridesTookEffect()
  {
    auto find_drift = [this]() {
        std::vector<std::pair<rclcpp::Parameter, std::string>> drift;
        for (const auto & entry : get_node_parameters_interface()->get_parameter_overrides()) {
          const std::string & name = entry.first;
          const auto binding = control_bindings_.find(name);
          if (binding == control_bindings_.end() || IsSetByStartupLogic(binding->second.node_name) ||
            IsManagedControlNode(binding->second.map_kind, binding->second.node_name))
          {
            continue;
          }
          rclcpp::Parameter wanted;
          if (!get_parameter(name, wanted)) {
            continue;
          }
          if (const auto current = ControlValueMismatch(binding->second, wanted)) {
            drift.emplace_back(wanted, *current);
          }
        }
        // Priority order like the startup pass, but the link limit last: GevSCPD and the packet size
        // recompute it, so whatever of those drifted has to be back in place before it.
        auto rank = [this](const rclcpp::Parameter & parameter) {
            const auto & binding = control_bindings_.at(parameter.get_name());
            return ControlOverridePriority(binding) * 2 +
                   (binding.node_name == "DeviceLinkThroughputLimit" ? 1 : 0);
          };
        std::stable_sort(
          drift.begin(), drift.end(),
          [&rank](const auto & lhs, const auto & rhs) {return rank(lhs.first) < rank(rhs.first);});
        return drift;
      };

    const auto drift = find_drift();
    if (drift.empty()) {
      return;
    }
    std::string fixed_text;
    for (const auto & [wanted, current] : drift) {
      try {
        ApplyControlParameterValue(control_bindings_.at(wanted.get_name()), wanted);
      } catch (const std::exception &) {
      }
      fixed_text += (fixed_text.empty() ? "" : ", ") + wanted.get_name() + " (" + current + " -> " +
        wanted.value_to_string() + ")";
    }

    const auto still = find_drift();
    std::string still_text;
    for (const auto & [wanted, current] : still) {
      still_text += (still_text.empty() ? "" : ", ") + wanted.get_name() + "=" +
        wanted.value_to_string() + " but camera has " + current;
    }
    if (still.size() < drift.size()) {
      RCLCPP_WARN(
        get_logger(),
        "Re-applied %zu startup camera setting(s) that another setting had overwritten: %s",
        drift.size() - still.size(),
        fixed_text.c_str());
    }
    if (!still.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "%zu startup camera setting(s) not in effect after startup — the camera changed or "
        "rejected them: %s",
        still.size(),
        still_text.c_str());
    }
  }

  static int ControlOverridePriority(const ControlBinding & binding)
  {
    switch (binding.value_kind) {
      case ControlValueKind::Enumeration:
        return 0;
      case ControlValueKind::Boolean:
        return 1;
      case ControlValueKind::Integer:
        return 2;
      case ControlValueKind::Float:
        return 3;
      case ControlValueKind::String:
        return 4;
    }

    return 10;
  }

  static bool ParameterNameMatches(
    const std::string & parameter_name,
    std::initializer_list<const char *> candidates)
  {
    for (const char * candidate : candidates) {
      if (parameter_name == candidate) {
        return true;
      }
    }

    return false;
  }

  std::optional<NamedStringParameter> FindStringParameterValue(
    std::initializer_list<const char *> parameter_names) const
  {
    for (const char * parameter_name : parameter_names) {
      rclcpp::Parameter parameter;
      if (!get_parameter(parameter_name, parameter)) {
        continue;
      }

      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
        continue;
      }

      return NamedStringParameter{parameter_name, parameter.as_string()};
    }

    return std::nullopt;
  }

  std::optional<NamedBoolParameter> FindBoolParameterValue(
    std::initializer_list<const char *> parameter_names) const
  {
    for (const char * parameter_name : parameter_names) {
      rclcpp::Parameter parameter;
      if (!get_parameter(parameter_name, parameter)) {
        continue;
      }

      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        continue;
      }

      return NamedBoolParameter{parameter_name, parameter.as_bool()};
    }

    return std::nullopt;
  }

  bool HasPendingControlOverride(std::initializer_list<const char *> parameter_names) const
  {
    return std::any_of(
      pending_control_overrides_.begin(),
      pending_control_overrides_.end(),
      [&](const rclcpp::Parameter & parameter) {
        return ParameterNameMatches(parameter.get_name(), parameter_names);
      });
  }

  void RemovePendingControlOverrides(std::initializer_list<const char *> parameter_names)
  {
    pending_control_overrides_.erase(
      std::remove_if(
        pending_control_overrides_.begin(),
        pending_control_overrides_.end(),
        [&](const rclcpp::Parameter & parameter) {
          return ParameterNameMatches(parameter.get_name(), parameter_names);
        }),
      pending_control_overrides_.end());
  }

  void NormalizeFrameRateStartupOverrides()
  {
    if (!HasPendingControlOverride({"camera.AcquisitionFrameRate", "camera.FrameRateHz_Val"})) {
      return;
    }

    const auto frame_rate_enable = FindBoolParameterValue(
      {"camera.AcquisitionFrameRateEnable", "camera.FrameRateEn_Val"});
    if (!frame_rate_enable.has_value() || frame_rate_enable->value) {
      return;
    }

    RCLCPP_WARN(
      get_logger(),
      "A manual frame-rate value is configured while '%s' is false. "
      "Enabling it because FLIR AcquisitionFrameRateEnable means manual frame-rate limiting, "
      "not frame-rate auto mode.",
      frame_rate_enable->name.c_str());

    set_parameter(rclcpp::Parameter(frame_rate_enable->name, true));
    RemovePendingControlOverrides({"camera.AcquisitionFrameRateEnable", "camera.FrameRateEn_Val"});
    pending_control_overrides_.emplace_back(frame_rate_enable->name, true);
  }

  std::optional<std::string> ControlOverrideHint(const rclcpp::Parameter & parameter) const
  {
    if (ParameterNameMatches(
        parameter.get_name(),
        {
          "camera.AcquisitionFrameRate",
          "camera.FrameRateHz_Val"}))
    {
      const auto frame_rate_enable = FindBoolParameterValue(
        {"camera.AcquisitionFrameRateEnable", "camera.FrameRateEn_Val"});
      if (frame_rate_enable.has_value() && !frame_rate_enable->value) {
        return "Set '" + frame_rate_enable->name + "' to true before applying a manual frame-rate "
               "override. FLIR AcquisitionFrameRateEnable means manual frame-rate limiting, "
               "not frame-rate auto mode.";
      }
    }

    if (ParameterNameMatches(
        parameter.get_name(),
        {
          "camera.ExposureTime",
          "camera.ExposureTime_FloatVal",
          "camera.ExposureTime_Val",
          "camera.ExposureTimeRaw_Val"}))
    {
      const auto exposure_auto = FindStringParameterValue({"camera.ExposureAuto", "camera.ExposureAuto_Val"});
      if (exposure_auto.has_value() && NormalizeName(exposure_auto->value) != "off") {
        return "Set '" + exposure_auto->name + "' to 'Off' before applying a manual exposure override, "
               "or remove '" + parameter.get_name() + "' from the startup parameters.";
      }
    }

    if (ParameterNameMatches(
        parameter.get_name(),
        {
          "camera.Gain",
          "camera.GainDB_Val",
          "camera.Gain_Val",
          "camera.GainRaw_Val"}))
    {
      const auto gain_auto = FindStringParameterValue({"camera.GainAuto", "camera.GainAuto_Val"});
      if (gain_auto.has_value() && NormalizeName(gain_auto->value) != "off") {
        return "Set '" + gain_auto->name + "' to 'Off' before applying a manual gain override, "
               "or remove '" + parameter.get_name() + "' from the startup parameters.";
      }
    }

    return std::nullopt;
  }

  std::string BuildControlOverrideFailureMessage(
    const std::string & prefix,
    const rclcpp::Parameter & parameter,
    const std::exception & exception) const
  {
    std::string message = prefix + "'" + parameter.get_name() + "': " + exception.what();
    const auto hint = ControlOverrideHint(parameter);
    if (hint.has_value()) {
      message += " ";
      message += *hint;
    }

    return message;
  }

  void ApplyPendingControlOverrides()
  {
    if (pending_control_overrides_.empty()) {
      return;
    }

    auto pending = pending_control_overrides_;
    pending_control_overrides_.clear();

    std::string last_error;
    const std::size_t max_passes = std::max<std::size_t>(1U, pending.size());

    for (std::size_t pass = 0; pass < max_passes && !pending.empty(); ++pass) {
      std::sort(
        pending.begin(),
        pending.end(),
        [this](const rclcpp::Parameter & lhs, const rclcpp::Parameter & rhs) {
          const auto & lhs_binding = control_bindings_.at(lhs.get_name());
          const auto & rhs_binding = control_bindings_.at(rhs.get_name());
          const int lhs_priority = ControlOverridePriority(lhs_binding);
          const int rhs_priority = ControlOverridePriority(rhs_binding);
          if (lhs_priority != rhs_priority) {
            return lhs_priority < rhs_priority;
          }
          return lhs.get_name() < rhs.get_name();
        });

      std::vector<rclcpp::Parameter> next_pass;
      std::size_t applied = 0U;

      for (const auto & parameter : pending) {
        try {
          ApplyControlParameterValue(control_bindings_.at(parameter.get_name()), parameter);
          ++applied;
        } catch (const std::exception & exception) {
          last_error = BuildControlOverrideFailureMessage(
            "Failed to apply startup override ",
            parameter,
            exception);
          next_pass.push_back(parameter);
        }
      }

      if (next_pass.empty()) {
        RCLCPP_INFO(
          get_logger(),
          "Applied %zu startup camera control overrides.",
          pending.size());
        return;
      }

      if (applied == 0U) {
        throw std::runtime_error(last_error);
      }

      pending = std::move(next_pass);
    }

    throw std::runtime_error(last_error.empty() ? "Failed to apply camera control overrides." : last_error);
  }

  void InitializeControlParameters()
  {
    RegisterWritableControlParameters(*camera_node_map_, ControlMapKind::Camera);
    RegisterWritableControlParameters(*stream_node_map_, ControlMapKind::Stream);
    RegisterWritableControlParameters(*tl_device_node_map_, ControlMapKind::TlDevice);
    NormalizeFrameRateStartupOverrides();
    ApplyPendingControlOverrides();
    RegisterControlParametersUnlockedByOverrides();
    ReportIgnoredControlOverrides();

    control_parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&FlirSpinnakerCameraNode::OnSetControlParameters, this, std::placeholders::_1));

    std::ostringstream stream;
    stream << "Camera control bridge ready. Example parameters:";
    bool appended = false;
    AppendFirstAvailableExample(
      stream,
      appended,
      {"camera.AcquisitionFrameRateEnable", "camera.FrameRateEn_Val"});
    AppendFirstAvailableExample(
      stream,
      appended,
      {"camera.AcquisitionFrameRate", "camera.FrameRateHz_Val"});
    AppendFirstAvailableExample(
      stream,
      appended,
      {"camera.ExposureAuto", "camera.ExposureAuto_Val"});
    AppendFirstAvailableExample(
      stream,
      appended,
      {
        "camera.ExposureTime",
        "camera.ExposureTime_FloatVal",
        "camera.ExposureTime_Val",
        "camera.ExposureTimeRaw_Val"});
    AppendFirstAvailableExample(
      stream,
      appended,
      {"camera.GainAuto", "camera.GainAuto_Val"});
    AppendFirstAvailableExample(
      stream,
      appended,
      {"camera.Gain", "camera.GainDB_Val", "camera.Gain_Val", "camera.GainRaw_Val"});

    if (!appended) {
      stream << " use 'ros2 param list /" << get_name() << "' to inspect available camera.*, stream.*, and tl_device.* parameters.";
    }

    RCLCPP_INFO(get_logger(), "%s", stream.str().c_str());
  }

  void AppendFirstAvailableExample(
    std::ostringstream & stream,
    bool & appended,
    std::initializer_list<const char *> parameter_names) const
  {
    for (const char * parameter_name : parameter_names) {
      if (control_bindings_.find(parameter_name) == control_bindings_.end()) {
        continue;
      }

      stream << (appended ? ", " : " ") << parameter_name;
      appended = true;
      return;
    }
  }

  rcl_interfaces::msg::SetParametersResult OnSetControlParameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & parameter : parameters) {
      const auto binding_it = control_bindings_.find(parameter.get_name());
      if (binding_it == control_bindings_.end()) {
        continue;
      }

      try {
        ApplyControlParameterValue(binding_it->second, parameter);
      } catch (const std::exception & exception) {
        result.successful = false;
        result.reason = BuildControlOverrideFailureMessage("Failed to set ", parameter, exception);
        return result;
      }
    }

    return result;
  }

  void InitializeCamera()
  {
    system_ = Spinnaker::System::GetInstance();
    camera_list_ = system_->GetCameras();

    const std::size_t camera_count = camera_list_.GetSize();
    if (camera_count == 0U) {
      throw std::runtime_error("No FLIR cameras detected by Spinnaker.");
    }

    for (std::size_t index = 0; index < camera_count; ++index) {
      CameraPtr candidate = camera_list_.GetByIndex(index);
      INodeMap & tl_node_map = candidate->GetTLDeviceNodeMap();

      const std::string serial = SafeNodeString(tl_node_map, "DeviceSerialNumber");
      const std::string vendor = SafeNodeString(tl_node_map, "DeviceVendorName");
      const std::string model = SafeNodeString(tl_node_map, "DeviceModelName");

      RCLCPP_INFO(
        get_logger(),
        "Detected camera[%zu]: vendor='%s' model='%s' serial='%s'",
        index,
        vendor.c_str(),
        model.c_str(),
        serial.c_str());
    }

    camera_ = SelectCamera();
    if (ApplyNetworkForceIpConfiguration(camera_)) {
      camera_ = nullptr;
      camera_ = WaitForSelectedCameraAfterForceIp();
    }

    // A GigE Vision camera grants Read/Write access to one controller at a time.
    // Every camera node enumerates the whole rig, so keeping the list alive holds
    // a device handle on the seven cameras this node does not own, and a sibling
    // node's Init() then fails with "Unable to set DeviceAccessStatus to
    // Read/Write" (-1005). camera_ keeps its own reference, so it stays valid.
    camera_list_.Clear();

    InitializeSelectedCamera();

    INodeMap & node_map = camera_->GetNodeMap();
    INodeMap & stream_node_map = camera_->GetTLStreamNodeMap();
    INodeMap & tl_node_map = camera_->GetTLDeviceNodeMap();
    camera_node_map_ = &node_map;
    stream_node_map_ = &stream_node_map;
    tl_device_node_map_ = &tl_node_map;

    ApplyBufferHandlingMode(stream_node_map);
    ApplyPixelFormat(node_map);
    SetContinuousAcquisition(node_map);
    InitializeControlParameters();
    ApplyPtpConfiguration(node_map);
    ApplyHardwareTriggerConfiguration(node_map);
    ApplyPtpActionConfiguration(node_map);
    ApplyChunkData(node_map);
    LogLinkSettings(node_map, tl_node_map);
    VerifyControlOverridesTookEffect();

    const std::string selected_serial = SafeNodeString(tl_node_map, "DeviceSerialNumber");
    const std::string selected_model = SafeNodeString(tl_node_map, "DeviceModelName");
    RCLCPP_INFO(
      get_logger(),
      "Using camera model='%s' serial='%s'",
      selected_model.c_str(),
      selected_serial.c_str());

    camera_->BeginAcquisition();
    acquisition_started_ = true;
    RCLCPP_INFO(get_logger(), "Camera acquisition started.");
  }

  // Init() failures split into transient ones worth retrying and hard ones that
  // need a human. Two codes clear on their own:
  //   -1005 another controller still holds the camera; clears when it lets go.
  //   -1010 register I/O write error ("Please try reconnecting the device").
  //         Every camera node enumerates the whole rig, so a simultaneous launch
  //         makes their Init() calls contend for each camera's single Read/Write
  //         slot and one or two hit this — SpinView, which opens cameras one at a
  //         time, never does. A retry (exactly what the message asks for) recovers
  //         it, so this no longer takes a healthy camera down on every launch.
  // Everything else — updater image mode (-1015), bad firmware, an unreachable
  // device — will not clear by re-Init, so retrying just buries the real message
  // under a stack of identical warnings; let those fail fast.
  static bool IsRetryableInitError(const std::exception & exception)
  {
    const auto * spinnaker_exception = dynamic_cast<const Spinnaker::Exception *>(&exception);
    if (spinnaker_exception == nullptr) {
      return false;
    }
    const int error = static_cast<int>(spinnaker_exception->GetError());
    return error == -1005 || error == -1010;
  }

  // Every camera node enumerates the whole rig, so their Init() calls contend for
  // the single Read/Write slot each camera grants. Retry instead of taking the
  // node down, which is what left a different camera dead on every launch.
  void InitializeSelectedCamera()
  {
    const int attempts = std::max(1, camera_init_max_attempts_);
    std::string last_error;

    for (int attempt = 1; attempt <= attempts; ++attempt) {
      try {
        camera_->Init();
        if (attempt > 1) {
          RCLCPP_INFO(get_logger(), "Camera Init succeeded on attempt %d/%d.", attempt, attempts);
        }
        return;
      } catch (const std::exception & exception) {
        last_error = exception.what();
        if (!IsRetryableInitError(exception)) {
          throw std::runtime_error("Camera Init failed: " + last_error);
        }
        if (attempt == attempts || !rclcpp::ok()) {
          break;
        }
        RCLCPP_WARN(
          get_logger(),
          "Camera Init attempt %d/%d failed: %s Retrying in %d ms.",
          attempt,
          attempts,
          last_error.c_str(),
          camera_init_retry_delay_ms_);
        std::this_thread::sleep_for(std::chrono::milliseconds(camera_init_retry_delay_ms_));
      }
    }

    throw std::runtime_error(
            "Camera Init failed after " + std::to_string(attempts) + " attempts: " + last_error);
  }

  CameraPtr SelectCamera()
  {
    CameraPtr selected = TrySelectCamera();
    if (selected) {
      return selected;
    }

    if (!camera_serial_.empty()) {
      throw std::runtime_error("Requested camera_serial was not found: " + camera_serial_);
    }

    throw std::runtime_error("camera_index is out of range.");
  }

  CameraPtr TrySelectCamera()
  {
    if (!camera_serial_.empty()) {
      for (std::size_t index = 0; index < camera_list_.GetSize(); ++index) {
        CameraPtr candidate = camera_list_.GetByIndex(index);
        INodeMap & tl_node_map = candidate->GetTLDeviceNodeMap();
        const std::string serial = SafeNodeString(tl_node_map, "DeviceSerialNumber");
        if (serial == camera_serial_) {
          return candidate;
        }
      }

      return nullptr;
    }

    if (camera_index_ < 0 || static_cast<std::size_t>(camera_index_) >= camera_list_.GetSize()) {
      return nullptr;
    }

    return camera_list_.GetByIndex(static_cast<unsigned int>(camera_index_));
  }

  CameraPtr WaitForSelectedCameraAfterForceIp()
  {
    const auto timeout = std::chrono::milliseconds(network_force_ip_rediscovery_timeout_ms_);
    const auto poll_period = std::chrono::milliseconds(250);
    const auto deadline = std::chrono::steady_clock::now() + timeout;

    do {
      camera_list_.Clear();
      camera_list_ = system_->GetCameras();
      if (CameraPtr selected = TrySelectCamera()) {
        return selected;
      }
      std::this_thread::sleep_for(poll_period);
    } while (std::chrono::steady_clock::now() < deadline);

    camera_list_.Clear();
    camera_list_ = system_->GetCameras();
    return SelectCamera();
  }

  bool ApplyNetworkForceIpConfiguration(CameraPtr selected_camera)
  {
    if (!network_force_ip_enable_) {
      return false;
    }

    INodeMap & tl_node_map = selected_camera->GetTLDeviceNodeMap();
    const std::uint32_t target_address =
      ParseIpv4Address(network_force_ip_address_, "network.force_ip.address");
    const std::uint32_t target_subnet_mask =
      ParseIpv4Address(network_force_ip_subnet_mask_, "network.force_ip.subnet_mask");
    const std::uint32_t target_gateway =
      ParseIpv4Address(network_force_ip_gateway_, "network.force_ip.gateway");

    const auto current_address = ReadIntegerNodeValue(tl_node_map, "GevDeviceIPAddress");
    if (current_address.has_value() && *current_address >= 0) {
      const auto current_ipv4 = static_cast<std::uint32_t>(*current_address);
      if (current_ipv4 == target_address) {
        RCLCPP_INFO(
          get_logger(),
          "Camera serial '%s' already has requested IP %s. Skipping ForceIP.",
          camera_serial_.c_str(),
          FormatIpv4Address(target_address).c_str());
        return false;
      }

      if (network_force_ip_only_if_link_local_ && !IsLinkLocalIpv4(current_ipv4)) {
        RCLCPP_INFO(
          get_logger(),
          "Camera serial '%s' current IP is %s and target is %s. Applying ForceIP because "
          "the current address does not match the requested camera inventory.",
          camera_serial_.c_str(),
          FormatIpv4Address(current_ipv4).c_str(),
          FormatIpv4Address(target_address).c_str());
      }
    }

    const auto wrong_subnet = ReadBooleanNodeValue(tl_node_map, "GevDeviceIsWrongSubnet");
    RCLCPP_WARN(
      get_logger(),
      "Applying ForceIP to camera serial '%s': current_ip=%s wrong_subnet=%s target=%s/%s gateway=%s.",
      camera_serial_.empty() ? "<index-selected>" : camera_serial_.c_str(),
      current_address.has_value() && *current_address >= 0 ?
      FormatIpv4Address(static_cast<std::uint32_t>(*current_address)).c_str() : "unknown",
      wrong_subnet.has_value() ? (*wrong_subnet ? "true" : "false") : "unknown",
      FormatIpv4Address(target_address).c_str(),
      FormatIpv4Address(target_subnet_mask).c_str(),
      FormatIpv4Address(target_gateway).c_str());

    if (!SetIntegerByName(tl_node_map, "GevDeviceForceIPAddress", target_address) ||
      !SetIntegerByName(tl_node_map, "GevDeviceForceSubnetMask", target_subnet_mask) ||
      !SetIntegerByName(tl_node_map, "GevDeviceForceGateway", target_gateway))
    {
      throw std::runtime_error(
              "Failed to write GevDeviceForceIPAddress/SubnetMask/Gateway on camera serial '" +
              (camera_serial_.empty() ? std::string("<index-selected>") : camera_serial_) + "'.");
    }

    if (!ExecuteCommandByName(tl_node_map, "GevDeviceForceIP")) {
      throw std::runtime_error(
              "Failed to execute GevDeviceForceIP on camera serial '" +
              (camera_serial_.empty() ? std::string("<index-selected>") : camera_serial_) + "'.");
    }

    if (network_force_ip_wait_after_ms_ > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(network_force_ip_wait_after_ms_));
    }

    RCLCPP_INFO(
      get_logger(),
      "ForceIP command sent to camera serial '%s'; refreshing camera list.",
      camera_serial_.empty() ? "<index-selected>" : camera_serial_.c_str());
    return true;
  }

  void ApplyBufferHandlingMode(INodeMap & stream_node_map)
  {
    if (buffer_handling_mode_.empty()) {
      return;
    }

    if (SetEnumerationByName(stream_node_map, "StreamBufferHandlingMode", buffer_handling_mode_)) {
      RCLCPP_INFO(
        get_logger(),
        "StreamBufferHandlingMode set to '%s'",
        buffer_handling_mode_.c_str());
      return;
    }

    RCLCPP_WARN(
      get_logger(),
      "Could not set StreamBufferHandlingMode to '%s'. Continuing with camera default.",
      buffer_handling_mode_.c_str());
  }

  void ApplyPixelFormat(INodeMap & node_map)
  {
    if (!pixel_format_.empty()) {
      for (const auto & candidate : PixelFormatParameterCandidates(pixel_format_)) {
        if (!SetEnumerationByName(node_map, "PixelFormat", candidate)) {
          continue;
        }

        if (candidate != pixel_format_) {
          RCLCPP_INFO(
            get_logger(),
            "PixelFormat alias '%s' resolved to '%s'",
            pixel_format_.c_str(),
            candidate.c_str());
          pixel_format_ = candidate;
        }

        RCLCPP_INFO(get_logger(), "PixelFormat set to '%s'", pixel_format_.c_str());
        return;
      }

      throw std::runtime_error("Failed to set PixelFormat to " + pixel_format_);
    }

    if (!auto_pixel_format_) {
      return;
    }

    for (const auto & candidate : PreferredPixelFormats()) {
      if (!EnumerationContains(node_map, "PixelFormat", candidate)) {
        continue;
      }

      if (SetEnumerationByName(node_map, "PixelFormat", candidate)) {
        RCLCPP_INFO(get_logger(), "PixelFormat auto-selected as '%s'", candidate.c_str());
        return;
      }
    }

    RCLCPP_WARN(
      get_logger(),
      "Could not auto-select a ROS-friendly PixelFormat. Raw publishing may be unavailable.");
  }

  void SetContinuousAcquisition(INodeMap & node_map)
  {
    if (!SetEnumerationByName(node_map, "AcquisitionMode", "Continuous")) {
      throw std::runtime_error("Failed to set AcquisitionMode to Continuous.");
    }
  }

  void RequireEnumeration(
    INodeMap & node_map,
    const char * node_name,
    const std::string & entry_name,
    const std::string & context)
  {
    if (!SetEnumerationByName(node_map, node_name, entry_name)) {
      throw std::runtime_error(
              context + ": failed to set " + node_name + " to '" + entry_name + "'.");
    }

    RCLCPP_INFO(get_logger(), "%s: %s='%s'", context.c_str(), node_name, entry_name.c_str());
  }

  bool TryEnumeration(
    INodeMap & node_map,
    const char * node_name,
    const std::string & entry_name,
    const std::string & context)
  {
    if (!SetEnumerationByName(node_map, node_name, entry_name)) {
      RCLCPP_DEBUG(
        get_logger(),
        "%s: %s='%s' is not available/writable.",
        context.c_str(),
        node_name,
        entry_name.c_str());
      return false;
    }

    RCLCPP_INFO(get_logger(), "%s: %s='%s'", context.c_str(), node_name, entry_name.c_str());
    return true;
  }

  void ConfigureActionSelector(INodeMap & node_map, const std::string & context)
  {
    if (ptp_action_selector_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "%s: ptp_action.selector is empty; leaving ActionSelector unchanged.",
        context.c_str());
      return;
    }

    if (TryEnumeration(node_map, "ActionSelector", ptp_action_selector_, context)) {
      return;
    }

    const auto current_action = ReadEnumerationNodeValue(node_map, "ActionSelector");
    if (current_action.has_value() && !current_action->empty()) {
      RCLCPP_WARN(
        get_logger(),
        "%s: ActionSelector could not be set to '%s'; using current camera selection '%s'.",
        context.c_str(),
        ptp_action_selector_.c_str(),
        current_action->c_str());
      return;
    }

    RCLCPP_WARN(
      get_logger(),
      "%s: ActionSelector is not writable/readable; assuming the camera exposes a single action signal.",
      context.c_str());
  }

  std::optional<std::int64_t> ReadIntegerNodeValue(
    INodeMap & node_map,
    const char * node_name) const
  {
    CIntegerPtr int_node = node_map.GetNode(node_name);
    if (!IsReadable(int_node)) {
      return std::nullopt;
    }

    return int_node->GetValue();
  }

  std::optional<bool> ReadBooleanNodeValue(
    INodeMap & node_map,
    const char * node_name) const
  {
    CBooleanPtr bool_node = node_map.GetNode(node_name);
    if (!IsReadable(bool_node)) {
      return std::nullopt;
    }

    return bool_node->GetValue();
  }

  std::optional<std::string> ReadEnumerationNodeValue(
    INodeMap & node_map,
    const char * node_name) const
  {
    CEnumerationPtr enum_node = node_map.GetNode(node_name);
    if (!IsReadable(enum_node)) {
      return std::nullopt;
    }

    return enum_node->ToString().c_str();
  }

  bool ExecuteCommandByName(INodeMap & node_map, const char * node_name) const
  {
    CCommandPtr command_node = node_map.GetNode(node_name);
    if (!IsWritable(command_node)) {
      return false;
    }

    command_node->Execute();
    return true;
  }

  void RequireInteger(
    INodeMap & node_map,
    const char * node_name,
    std::uint32_t value,
    const std::string & context)
  {
    CIntegerPtr int_node = node_map.GetNode(node_name);
    if (!IsWritable(int_node)) {
      throw std::runtime_error(context + ": failed to set " + node_name + ".");
    }

    int_node->SetValue(value, true);
    RCLCPP_INFO(get_logger(), "%s: %s=%u", context.c_str(), node_name, value);
  }

  bool SetIntegerByName(INodeMap & node_map, const char * node_name, std::uint32_t value) const
  {
    CIntegerPtr int_node = node_map.GetNode(node_name);
    if (!IsWritable(int_node)) {
      return false;
    }

    int_node->SetValue(value, true);
    return true;
  }

  bool PtpStatusIsAccepted(const std::string & status) const
  {
    const std::string normalized_status = NormalizeName(status);
    return std::any_of(
      ptp_accepted_statuses_.begin(),
      ptp_accepted_statuses_.end(),
      [&](const std::string & accepted_status) {
        return NormalizeName(accepted_status) == normalized_status;
      });
  }

  // Blackfly exposes the legacy GevIEEE1588* nodes; newer SFNC devices such as the
  // FLIR A50/A70 only expose Ptp* (PtpEnable, PtpStatus, ...).
  static const char * SelectPtpNodeName(
    INodeMap & node_map,
    const char * legacy_name,
    const char * sfnc_name)
  {
    return IsAvailable(node_map.GetNode(legacy_name)) ? legacy_name : sfnc_name;
  }

  void WaitForPtpSync(INodeMap & node_map)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(ptp_sync_timeout_ms_);
    std::string last_status = "unknown";
    const char * latch_node = SelectPtpNodeName(node_map, "GevIEEE1588DataSetLatch", "PtpDataSetLatch");
    const char * status_node = SelectPtpNodeName(node_map, "GevIEEE1588Status", "PtpStatus");
    const char * offset_node = SelectPtpNodeName(
      node_map, "GevIEEE1588OffsetFromMasterLatched", "PtpOffsetFromMaster");

    while (rclcpp::ok()) {
      ExecuteCommandByName(node_map, latch_node);

      if (const auto status = ReadEnumerationNodeValue(node_map, status_node)) {
        last_status = *status;
        if (PtpStatusIsAccepted(last_status)) {
          const auto offset_ns = ReadIntegerNodeValue(node_map, offset_node);
          if (offset_ns.has_value()) {
            RCLCPP_INFO(
              get_logger(),
              "PTP synchronized: %s='%s', offset_from_master=%ld ns.",
              status_node,
              last_status.c_str(),
              static_cast<long>(*offset_ns));
          } else {
            RCLCPP_INFO(
              get_logger(),
              "PTP synchronized: %s='%s'.",
              status_node,
              last_status.c_str());
          }
          return;
        }
      }

      if (std::chrono::steady_clock::now() >= deadline) {
        break;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(ptp_sync_poll_ms_));
    }

    std::ostringstream message;
    message << "PTP did not reach accepted status before timeout. Last " << status_node << "='"
            << last_status << "', accepted=[";
    for (std::size_t index = 0; index < ptp_accepted_statuses_.size(); ++index) {
      if (index > 0U) {
        message << ", ";
      }
      message << ptp_accepted_statuses_[index];
    }
    message << "].";

    if (ptp_require_sync_) {
      throw std::runtime_error(message.str());
    }

    RCLCPP_WARN(get_logger(), "%s Continuing because ptp.require_sync=false.", message.str().c_str());
  }

  void ApplyPtpConfiguration(INodeMap & node_map)
  {
    if (!ptp_enabled_) {
      return;
    }

    const std::string context = "PTP";
    const char * enable_node = SelectPtpNodeName(node_map, "GevIEEE1588", "PtpEnable");
    if (!SetBooleanByName(node_map, enable_node, true)) {
      throw std::runtime_error(context + ": failed to enable " + enable_node + ".");
    }
    RCLCPP_INFO(get_logger(), "%s: %s=true", context.c_str(), enable_node);

    if (!ptp_mode_.empty()) {
      const char * mode_node = SelectPtpNodeName(node_map, "GevIEEE1588Mode", "PtpMode");
      if (TryEnumeration(node_map, mode_node, ptp_mode_, context)) {
        RCLCPP_INFO(get_logger(), "%s: requested mode '%s'.", context.c_str(), ptp_mode_.c_str());
      } else {
        RCLCPP_WARN(
          get_logger(),
          "%s: %s='%s' is not writable/available. Continuing with camera default.",
          context.c_str(),
          mode_node,
          ptp_mode_.c_str());
      }
    }

    if (ptp_wait_for_sync_) {
      WaitForPtpSync(node_map);
    }
  }

  void ApplyHardwareTriggerConfiguration(INodeMap & node_map)
  {
    if (hardware_trigger_role_ == "none") {
      return;
    }

    if (hardware_trigger_role_ == "master") {
      ApplyBfsMasterHardwareTrigger(node_map);
      return;
    }

    if (hardware_trigger_role_ == "slave") {
      ApplyBfsSlaveHardwareTrigger(node_map);
      return;
    }

    throw std::runtime_error("Unsupported hardware_trigger.role: " + hardware_trigger_role_);
  }

  void ApplyBfsMasterHardwareTrigger(INodeMap & node_map)
  {
    const std::string context = "BFS hardware trigger master";
    RCLCPP_INFO(
      get_logger(),
      "Applying %s setup: output_line='%s', preferred_line_source='%s'.",
      context.c_str(),
      hardware_trigger_master_output_line_.c_str(),
      hardware_trigger_master_line_source_.c_str());

    RequireEnumeration(node_map, "AcquisitionMode", "Continuous", context);
    RequireEnumeration(node_map, "TriggerSelector", "FrameStart", context);
    RequireEnumeration(node_map, "TriggerMode", "Off", context);
    RequireEnumeration(node_map, "LineSelector", hardware_trigger_master_output_line_, context);
    RequireEnumeration(node_map, "LineMode", "Output", context);

    std::vector<std::string> line_source_candidates;
    if (!hardware_trigger_master_line_source_.empty()) {
      line_source_candidates.push_back(hardware_trigger_master_line_source_);
    }
    for (const auto & fallback : hardware_trigger_master_line_source_fallbacks_) {
      if (!fallback.empty() &&
        std::find(line_source_candidates.begin(), line_source_candidates.end(), fallback) ==
        line_source_candidates.end())
      {
        line_source_candidates.push_back(fallback);
      }
    }

    std::string applied_line_source;
    for (const auto & candidate : line_source_candidates) {
      if (TryEnumeration(node_map, "LineSource", candidate, context)) {
        applied_line_source = candidate;
        break;
      }
    }

    if (applied_line_source.empty()) {
      throw std::runtime_error(
              context + ": failed to set LineSource using configured candidates.");
    }

    if (hardware_trigger_master_enable_3v3_) {
      EnableBfsMasterLine3v3(node_map, context);
    }

    RCLCPP_INFO(
      get_logger(),
      "BFS hardware trigger master ready: output_line='%s', line_source='%s'.",
      hardware_trigger_master_output_line_.c_str(),
      applied_line_source.c_str());
  }

  void EnableBfsMasterLine3v3(INodeMap & node_map, const std::string & context)
  {
    RequireEnumeration(node_map, "LineSelector", hardware_trigger_master_3v3_line_, context);

    for (const auto & node_name : hardware_trigger_master_3v3_enable_nodes_) {
      if (node_name.empty()) {
        continue;
      }

      if (!SetBooleanByName(node_map, node_name.c_str(), true)) {
        RCLCPP_DEBUG(
          get_logger(),
          "%s: 3.3V node '%s' is not available/writable on %s.",
          context.c_str(),
          node_name.c_str(),
          hardware_trigger_master_3v3_line_.c_str());
        continue;
      }

      RCLCPP_INFO(
        get_logger(),
        "%s: enabled 3.3V on %s via %s.",
        context.c_str(),
        hardware_trigger_master_3v3_line_.c_str(),
        node_name.c_str());
      return;
    }

    const std::string message =
      context + ": failed to enable 3.3V on " + hardware_trigger_master_3v3_line_ +
      " using configured node candidates.";
    if (hardware_trigger_master_require_3v3_) {
      throw std::runtime_error(message);
    }

    RCLCPP_WARN(get_logger(), "%s Continuing because require_3v3=false.", message.c_str());
  }

  void ApplyBfsSlaveHardwareTrigger(INodeMap & node_map)
  {
    const std::string context = "BFS hardware trigger slave";
    RCLCPP_INFO(
      get_logger(),
      "Applying %s setup: trigger_source='%s'.",
      context.c_str(),
      hardware_trigger_slave_trigger_source_.c_str());

    RequireEnumeration(node_map, "AcquisitionMode", "Continuous", context);
    RequireEnumeration(node_map, "TriggerSelector", "FrameStart", context);
    RequireEnumeration(node_map, "TriggerMode", "Off", context);
    if (hardware_trigger_slave_trigger_source_.rfind("Line", 0) == 0) {
      RequireEnumeration(node_map, "LineSelector", hardware_trigger_slave_trigger_source_, context);
      RequireEnumeration(node_map, "LineMode", "Input", context);
    }
    RequireEnumeration(node_map, "TriggerSource", hardware_trigger_slave_trigger_source_, context);
    if (!hardware_trigger_slave_trigger_activation_.empty()) {
      RequireEnumeration(
        node_map,
        "TriggerActivation",
        hardware_trigger_slave_trigger_activation_,
        context);
    }
    if (!hardware_trigger_slave_trigger_overlap_.empty()) {
      RequireEnumeration(
        node_map,
        "TriggerOverlap",
        hardware_trigger_slave_trigger_overlap_,
        context);
    }
    RequireEnumeration(node_map, "TriggerMode", "On", context);

    RCLCPP_INFO(
      get_logger(),
      "BFS hardware trigger slave ready: trigger_source='%s'.",
      hardware_trigger_slave_trigger_source_.c_str());
  }

  void ApplyPtpActionConfiguration(INodeMap & node_map)
  {
    if (ptp_action_role_ == "none") {
      return;
    }

    const std::string context = "PTP action trigger " + ptp_action_role_;
    RCLCPP_INFO(
      get_logger(),
      "Applying %s setup: action='%s', device_key=%u, group_key=%u, group_mask=%u.",
      context.c_str(),
      ptp_action_selector_.c_str(),
      ptp_action_device_key_,
      ptp_action_group_key_,
      ptp_action_group_mask_);

    RequireEnumeration(node_map, "AcquisitionMode", "Continuous", context);
    ConfigureActionSelector(node_map, context);
    RequireInteger(node_map, "ActionDeviceKey", ptp_action_device_key_, context);
    RequireInteger(node_map, "ActionGroupKey", ptp_action_group_key_, context);
    RequireInteger(node_map, "ActionGroupMask", ptp_action_group_mask_, context);
    TryEnumeration(node_map, "ActionUnconditionalMode", "On", context);

    RequireEnumeration(node_map, "TriggerSelector", ptp_action_trigger_selector_, context);
    RequireEnumeration(node_map, "TriggerMode", "Off", context);
    RequireEnumeration(node_map, "TriggerSource", ptp_action_trigger_source_, context);
    if (!ptp_action_trigger_activation_.empty()) {
      if (!TryEnumeration(
        node_map,
        "TriggerActivation",
        ptp_action_trigger_activation_,
        context))
      {
        RCLCPP_WARN(
          get_logger(),
          "%s: TriggerActivation='%s' is not writable for trigger_source='%s'; using camera default.",
          context.c_str(),
          ptp_action_trigger_activation_.c_str(),
          ptp_action_trigger_source_.c_str());
      }
    }
    if (!ptp_action_trigger_overlap_.empty()) {
      if (!TryEnumeration(
        node_map,
        "TriggerOverlap",
        ptp_action_trigger_overlap_,
        context))
      {
        RCLCPP_WARN(
          get_logger(),
          "%s: TriggerOverlap='%s' is not writable for trigger_source='%s'; using camera default.",
          context.c_str(),
          ptp_action_trigger_overlap_.c_str(),
          ptp_action_trigger_source_.c_str());
      }
    }
    RequireEnumeration(node_map, "TriggerMode", "On", context);

    RCLCPP_INFO(
      get_logger(),
      "PTP action trigger receiver armed: trigger_source='%s'.",
      ptp_action_trigger_source_.c_str());
  }

  std::uint64_t ReadTimestampTickFrequency() const
  {
    if (camera_node_map_ == nullptr) {
      throw std::runtime_error("Camera node map is not available.");
    }

    if (const auto frequency = ReadIntegerNodeValue(*camera_node_map_, "GevTimestampTickFrequency")) {
      if (*frequency > 0) {
        return static_cast<std::uint64_t>(*frequency);
      }
    }

    RCLCPP_WARN(
      get_logger(),
      "Could not read GevTimestampTickFrequency. Assuming 1 GHz timestamp ticks.");
    return 1000000000ULL;
  }

  std::uint64_t MillisecondsToTimestampTicks(double milliseconds, std::uint64_t tick_frequency) const
  {
    const long double ticks =
      (static_cast<long double>(milliseconds) * static_cast<long double>(tick_frequency)) /
      1000.0L;
    if (ticks <= 0.0L) {
      return 1ULL;
    }

    const long double max_ticks = static_cast<long double>(std::numeric_limits<std::uint64_t>::max());
    if (ticks >= max_ticks) {
      return std::numeric_limits<std::uint64_t>::max();
    }

    return static_cast<std::uint64_t>(std::llround(ticks));
  }

  std::uint64_t ReadCameraTimestampTicks() const
  {
    if (camera_node_map_ == nullptr) {
      throw std::runtime_error("Camera node map is not available.");
    }

    if (!ExecuteCommandByName(*camera_node_map_, "TimestampLatch")) {
      RCLCPP_DEBUG(get_logger(), "TimestampLatch is not available/writable; reading Timestamp directly.");
    }

    if (const auto timestamp = ReadIntegerNodeValue(*camera_node_map_, "TimestampLatchValue")) {
      if (*timestamp >= 0) {
        return static_cast<std::uint64_t>(*timestamp);
      }
    }

    if (const auto timestamp = ReadIntegerNodeValue(*camera_node_map_, "Timestamp")) {
      if (*timestamp >= 0) {
        return static_cast<std::uint64_t>(*timestamp);
      }
    }

    throw std::runtime_error("Failed to read camera timestamp for scheduled PTP action command.");
  }

  void SendScheduledActionCommand(std::uint64_t action_time)
  {
    if (!system_) {
      throw std::runtime_error("Spinnaker system is not available.");
    }

    if (ptp_action_request_ack_ && ptp_action_expected_ack_count_ > 0) {
      unsigned int result_count = static_cast<unsigned int>(ptp_action_expected_ack_count_);
      const unsigned int expected_result_count = result_count;
      std::vector<Spinnaker::ActionCommandResult> results(result_count);
      system_->SendActionCommand(
        ptp_action_device_key_,
        ptp_action_group_key_,
        ptp_action_group_mask_,
        action_time,
        true,
        &result_count,
        results.data());

      if (result_count < expected_result_count) {
        throw std::runtime_error(
                "Scheduled action command received fewer acknowledgements than expected: " +
                std::to_string(result_count) + "/" + std::to_string(expected_result_count) + ".");
      }

      for (unsigned int index = 0; index < result_count; ++index) {
        if (results[index].Status == Spinnaker::SPINNAKER_ACTION_COMMAND_STATUS_OK) {
          continue;
        }

        std::ostringstream message;
        message << "Scheduled action command was not accepted by device 0x"
                << std::hex << results[index].DeviceAddress << std::dec
                << ": " << ActionCommandStatusName(results[index].Status)
                << " (" << static_cast<int>(results[index].Status) << ").";
        throw std::runtime_error(message.str());
      }
      return;
    }

    system_->SendActionCommand(
      ptp_action_device_key_,
      ptp_action_group_key_,
      ptp_action_group_mask_,
      action_time,
      false,
      nullptr,
      nullptr);
  }

  void PtpActionSenderLoop()
  {
    try {
      std::this_thread::sleep_for(
        std::chrono::duration<double, std::milli>(ptp_action_start_delay_ms_));

      const std::uint64_t tick_frequency = ReadTimestampTickFrequency();
      const std::uint64_t schedule_ahead_ticks =
        MillisecondsToTimestampTicks(ptp_action_schedule_ahead_ms_, tick_frequency);
      const std::uint64_t period_ticks =
        MillisecondsToTimestampTicks(1000.0 / ptp_action_rate_hz_, tick_frequency);
      const auto host_period =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(1.0 / ptp_action_rate_hz_));
      const auto log_interval =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(ptp_action_log_interval_sec_));

      // Trigger grid on PTP time (ptp_action.align_to_second): action n fires at exactly n / rate
      // seconds of PTP time, so every second starts on a shot (0, 1/30, 2/30 … s at 30 Hz) and the
      // shots of every run land on the same instants. A lidar phase-locked to the same PTP clock
      // (phase_lock_enable) then always faces the same direction when the cameras fire. Before, the
      // grid started at whatever camera_now + schedule_ahead was when the sender began, so the
      // lidar-to-camera relation changed from run to run. Slot times come from whole seconds plus
      // an integer fraction, so they do not drift the way repeated 33333333-tick steps did
      // (10 ns short per second). Only for whole-number rates; otherwise the old free grid.
      const double rate_rounded = std::round(ptp_action_rate_hz_);
      const bool aligned = ptp_action_align_to_second_ && rate_rounded >= 1.0 &&
        std::abs(ptp_action_rate_hz_ - rate_rounded) < 1e-9 && tick_frequency > 0U;
      const std::uint64_t rate_int = aligned ? static_cast<std::uint64_t>(rate_rounded) : 1U;
      auto slot_time = [&](std::uint64_t slot) {
          return (slot / rate_int) * tick_frequency +
                 ((slot % rate_int) * tick_frequency + rate_int / 2U) / rate_int;
        };
      auto first_slot_at_or_after = [&](std::uint64_t ticks) {
          std::uint64_t slot = (ticks / tick_frequency) * rate_int +
            ((ticks % tick_frequency) * rate_int + tick_frequency - 1U) / tick_frequency;
          while (slot_time(slot) < ticks) {
            ++slot;
          }
          return slot;
        };

      std::uint64_t next_action_time = 0U;
      std::uint64_t next_slot = 0U;
      std::uint64_t sent_count = 0U;
      auto next_send_time = std::chrono::steady_clock::now();
      auto next_log_time = std::chrono::steady_clock::now();

      RCLCPP_INFO(
        get_logger(),
        "PTP action sender started: rate=%.3f Hz, schedule_ahead=%.3f ms, grid=%s.",
        ptp_action_rate_hz_,
        ptp_action_schedule_ahead_ms_,
        aligned ? "aligned to PTP seconds (shot n at n/rate s)" : "free (starts at camera time + schedule_ahead)");

      std::uint64_t consecutive_failures = 0U;

      while (rclcpp::ok() && running_.load()) {
        // Keep a send failure inside the loop. This thread is the trigger source
        // for every camera in the rig, so letting one exception end it stops all
        // of them at once, and the only clue is a single line buried under the
        // -1011 timeouts that follow.
        try {
          const std::uint64_t camera_now = ReadCameraTimestampTicks();
          const std::uint64_t earliest_action_time =
            (std::numeric_limits<std::uint64_t>::max() - camera_now < schedule_ahead_ticks) ?
            std::numeric_limits<std::uint64_t>::max() :
            camera_now + schedule_ahead_ticks;

          const bool behind = next_action_time < earliest_action_time;
          const bool ahead = !behind && next_action_time - earliest_action_time > 5U * period_ticks;
          if (behind && !aligned) {
            next_action_time = earliest_action_time;
          } else if (behind) {
            next_slot = first_slot_at_or_after(earliest_action_time);
            next_action_time = slot_time(next_slot);
          } else if (ahead) {
            // The schedule is ahead of the camera clock — the clock stepped back (a PTP
            // correction, or one bad latch read that pushed the schedule forward). Only the
            // behind case used to re-anchor, so on 2026-09-22 (Orin GNSS grandmaster) the
            // schedule stayed 14.1 s in the future for good: each camera queued 10 actions,
            // fired them 14 s later as a 10-frame burst, and sent nothing in between.
            RCLCPP_WARN(
              get_logger(),
              "PTP action schedule was %.3f s ahead of the camera clock (the clock stepped back); "
              "re-anchoring to camera time + schedule_ahead.",
              static_cast<double>(next_action_time - earliest_action_time) /
              static_cast<double>(tick_frequency));
            if (aligned) {
              next_slot = first_slot_at_or_after(earliest_action_time);
              next_action_time = slot_time(next_slot);
            } else {
              next_action_time = earliest_action_time;
            }
          }

          SendScheduledActionCommand(next_action_time);
          ++sent_count;

          if (consecutive_failures > 0U) {
            RCLCPP_INFO(
              get_logger(),
              "PTP action sender recovered after %lu consecutive failures.",
              static_cast<unsigned long>(consecutive_failures));
            consecutive_failures = 0U;
          }

          const auto now_time = std::chrono::steady_clock::now();
          if (ptp_action_log_interval_sec_ > 0.0 && now_time >= next_log_time) {
            RCLCPP_INFO(
              get_logger(),
              "PTP action sender scheduled %lu commands; next_action_time=%lu ticks.",
              static_cast<unsigned long>(sent_count),
              static_cast<unsigned long>(next_action_time));
            next_log_time = now_time + log_interval;
          }

          if (aligned) {
            ++next_slot;
            next_action_time = slot_time(next_slot);
          } else {
            next_action_time =
              (std::numeric_limits<std::uint64_t>::max() - next_action_time < period_ticks) ?
              earliest_action_time :
              next_action_time + period_ticks;
          }
        } catch (const std::exception & exception) {
          ++consecutive_failures;
          RCLCPP_ERROR_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "PTP action sender error (%lu in a row): %s Recovering.",
            static_cast<unsigned long>(consecutive_failures),
            exception.what());

          RefreshSpinnakerInterfaces();
          // The camera clock is the only source of truth for the schedule, so
          // drop the stale target and re-derive it on the next pass.
          next_action_time = 0U;
          next_send_time = std::chrono::steady_clock::now();
        }

        next_send_time += host_period;
        const auto loop_done_time = std::chrono::steady_clock::now();
        if (loop_done_time < next_send_time) {
          std::this_thread::sleep_until(next_send_time);
        } else {
          next_send_time = loop_done_time;
        }
      }
    } catch (const std::exception & exception) {
      if (running_.load()) {
        RCLCPP_ERROR(get_logger(), "PTP action sender stopped after error: %s", exception.what());
        running_.store(false);
      }
    }
  }

  // Spinnaker reports -1002 ("Interface has been removed from the list and is no
  // longer valid") when its cached interface list goes stale, which is how a
  // camera or NIC dropping off the bus surfaces during a send. Refreshing the
  // list lets the next send resolve again.
  void RefreshSpinnakerInterfaces()
  {
    try {
      if (system_) {
        system_->UpdateInterfaceList();
      }
    } catch (const std::exception & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Could not refresh the Spinnaker interface list: %s",
        exception.what());
    }
  }

  void StartPtpActionSender()
  {
    if (ptp_action_role_ != "sender") {
      return;
    }

    ptp_action_thread_ = std::thread(&FlirSpinnakerCameraNode::PtpActionSenderLoop, this);
  }

  sensor_msgs::msg::Image BuildImageMessage(
    const ImagePtr & image,
    const std::string & encoding,
    const rclcpp::Time & stamp) const
  {
    sensor_msgs::msg::Image msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.height = static_cast<std::uint32_t>(image->GetHeight());
    msg.width = static_cast<std::uint32_t>(image->GetWidth());
    msg.encoding = encoding;
    msg.is_bigendian = IsHostBigEndian() ? 1U : 0U;

    std::size_t step = image->GetStride();
    if (step == 0U) {
      const int bits_per_channel = sensor_msgs::image_encodings::bitDepth(encoding);
      const int channels = sensor_msgs::image_encodings::numChannels(encoding);
      step = static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(bits_per_channel / 8) *
        static_cast<std::size_t>(channels);
    }

    msg.step = static_cast<std::uint32_t>(step);

    const std::size_t expected_size = static_cast<std::size_t>(msg.step) * static_cast<std::size_t>(msg.height);
    const std::size_t available_size = image->GetImageSize();

    if (available_size < expected_size) {
      throw std::runtime_error("Spinnaker image buffer is smaller than ROS image dimensions require.");
    }

    msg.data.resize(expected_size);
    std::memcpy(msg.data.data(), image->GetData(), expected_size);
    return msg;
  }

  std::optional<bool> ReadCameraBooleanNodeValue(std::initializer_list<const char *> node_names) const
  {
    if (camera_node_map_ == nullptr) {
      return std::nullopt;
    }

    for (const char * node_name : node_names) {
      try {
        CBooleanPtr bool_node = camera_node_map_->GetNode(node_name);
        if (IsReadable(bool_node)) {
          return bool_node->GetValue();
        }
      } catch (...) {
      }

      try {
        CIntegerPtr int_node = camera_node_map_->GetNode(node_name);
        if (IsReadable(int_node)) {
          return int_node->GetValue() != 0;
        }
      } catch (...) {
      }
    }

    return std::nullopt;
  }

  std::optional<double> ReadCameraNumericNodeValue(std::initializer_list<const char *> node_names) const
  {
    if (camera_node_map_ == nullptr) {
      return std::nullopt;
    }

    for (const char * node_name : node_names) {
      try {
        CFloatPtr float_node = camera_node_map_->GetNode(node_name);
        if (IsReadable(float_node)) {
          return float_node->GetValue();
        }
      } catch (...) {
      }

      try {
        CIntegerPtr int_node = camera_node_map_->GetNode(node_name);
        if (IsReadable(int_node)) {
          return static_cast<double>(int_node->GetValue());
        }
      } catch (...) {
      }
    }

    return std::nullopt;
  }

  std::optional<std::string> ReadCameraTextNodeValue(std::initializer_list<const char *> node_names) const
  {
    if (camera_node_map_ == nullptr) {
      return std::nullopt;
    }

    for (const char * node_name : node_names) {
      try {
        CEnumerationPtr enum_node = camera_node_map_->GetNode(node_name);
        if (IsReadable(enum_node)) {
          return enum_node->ToString().c_str();
        }
      } catch (...) {
      }

      try {
        CStringPtr string_node = camera_node_map_->GetNode(node_name);
        if (IsReadable(string_node)) {
          return string_node->GetValue().c_str();
        }
      } catch (...) {
      }
    }

    return std::nullopt;
  }

  PreparedRawImage PrepareRawImage(const ImagePtr & image) const
  {
    const auto raw_spec = RawOutputSpecForPixelFormat(image->GetPixelFormat());
    if (!raw_spec.has_value()) {
      throw std::runtime_error(
        "Pixel format '" + std::string(image->GetPixelFormatName().c_str()) + "' is not mapped to a ROS raw encoding.");
    }

    ImagePtr raw_image = image;
    if (raw_spec->requires_conversion) {
      raw_image = image_processor_.Convert(image, raw_spec->target_pixel_format);
    }

    return PreparedRawImage{raw_image, raw_spec->encoding};
  }

  std::string NormalizeCompressionFormat(const std::string & value) const
  {
    const std::string normalized = NormalizeName(value);
    if (normalized == "png") {
      return "png";
    }

    return "jpeg";
  }

  std::string NormalizeQoSReliability(const std::string & value) const
  {
    const std::string normalized = NormalizeName(value);
    if (normalized == "besteffort") {
      return "best_effort";
    }

    return "reliable";
  }

  rclcpp::QoS BuildPublisherQoS() const
  {
    const std::size_t depth = static_cast<std::size_t>(std::max(1, publisher_qos_depth_));
    rclcpp::QoS qos{rclcpp::KeepLast(depth)};
    qos.durability_volatile();

    if (NormalizeQoSReliability(publisher_qos_reliability_) == "best_effort") {
      qos.best_effort();
    } else {
      qos.reliable();
    }

    return qos;
  }

  void ApplyFlatCameraInfoYamlOverrides(
    const std::vector<std::string> & lines,
    const std::string & yaml_path)
  {
    bool saw_any_camera_info_value = false;
    for (const std::string & line : lines) {
      if (const auto value = MatchYamlScalarValue(line, "camera_info.distortion_model")) {
        camera_info_distortion_model_ = StripMatchingQuotes(*value);
        saw_any_camera_info_value = true;
        continue;
      }

      if (const auto value = MatchYamlScalarValue(line, "camera_info.d")) {
        camera_info_d_ = ParseYamlDoubleList(*value, "camera_info.d");
        saw_any_camera_info_value = true;
        continue;
      }

      if (const auto value = MatchYamlScalarValue(line, "camera_info.k")) {
        camera_info_k_ = ToFixedArray<9>(
          ParseYamlDoubleList(*value, "camera_info.k"),
          "camera_info.k");
        saw_any_camera_info_value = true;
        continue;
      }

      if (const auto value = MatchYamlScalarValue(line, "camera_info.r")) {
        camera_info_r_ = ToFixedArray<9>(
          ParseYamlDoubleList(*value, "camera_info.r"),
          "camera_info.r");
        saw_any_camera_info_value = true;
        continue;
      }

      if (const auto value = MatchYamlScalarValue(line, "camera_info.p")) {
        camera_info_p_ = ToFixedArray<12>(
          ParseYamlDoubleList(*value, "camera_info.p"),
          "camera_info.p");
        saw_any_camera_info_value = true;
        continue;
      }

      if (const auto value = MatchYamlScalarValue(line, "camera_info.binning_x")) {
        camera_info_binning_x_ = static_cast<std::uint32_t>(
          ParseYamlNonNegativeInt(*value, "camera_info.binning_x"));
        saw_any_camera_info_value = true;
        continue;
      }

      if (const auto value = MatchYamlScalarValue(line, "camera_info.binning_y")) {
        camera_info_binning_y_ = static_cast<std::uint32_t>(
          ParseYamlNonNegativeInt(*value, "camera_info.binning_y"));
        saw_any_camera_info_value = true;
        continue;
      }

      if (const auto value = MatchYamlScalarValue(line, "camera_info.roi.x_offset")) {
        camera_info_roi_x_offset_ = static_cast<std::uint32_t>(
          ParseYamlNonNegativeInt(*value, "camera_info.roi.x_offset"));
        saw_any_camera_info_value = true;
        continue;
      }

      if (const auto value = MatchYamlScalarValue(line, "camera_info.roi.y_offset")) {
        camera_info_roi_y_offset_ = static_cast<std::uint32_t>(
          ParseYamlNonNegativeInt(*value, "camera_info.roi.y_offset"));
        saw_any_camera_info_value = true;
        continue;
      }

      if (const auto value = MatchYamlScalarValue(line, "camera_info.roi.height")) {
        camera_info_roi_height_ = static_cast<std::uint32_t>(
          ParseYamlNonNegativeInt(*value, "camera_info.roi.height"));
        saw_any_camera_info_value = true;
        continue;
      }

      if (const auto value = MatchYamlScalarValue(line, "camera_info.roi.width")) {
        camera_info_roi_width_ = static_cast<std::uint32_t>(
          ParseYamlNonNegativeInt(*value, "camera_info.roi.width"));
        saw_any_camera_info_value = true;
        continue;
      }

      if (const auto value = MatchYamlScalarValue(line, "camera_info.roi.do_rectify")) {
        camera_info_roi_do_rectify_ = ParseYamlBool(*value, "camera_info.roi.do_rectify");
        saw_any_camera_info_value = true;
        continue;
      }
    }

    if (!saw_any_camera_info_value) {
      throw std::runtime_error(
        "Camera info YAML file did not contain any 'camera_info.*' entries: " + yaml_path);
    }

    RCLCPP_INFO(
      get_logger(),
      "Loaded camera_info calibration overrides from '%s'.",
      yaml_path.c_str());
  }

  void ApplySerialIndexedCameraInfoYamlOverrides(
    const std::vector<std::string> & lines,
    const std::string & yaml_path)
  {
    if (camera_serial_.empty()) {
      throw std::runtime_error(
        "camera_serial must be set when loading serial-indexed camera_info YAML: " + yaml_path);
    }

    bool in_registry = false;
    bool in_target_serial = false;
    bool in_camera_info = false;
    bool in_roi = false;
    bool saw_target_serial = false;
    bool saw_any_camera_info_value = false;
    std::size_t registry_indent = 0U;
    std::size_t serial_indent = 0U;
    std::size_t camera_info_indent = 0U;
    std::size_t roi_indent = 0U;

    for (const std::string & line : lines) {
      const std::string trimmed = TrimAscii(line);
      if (trimmed.empty() || trimmed[0] == '#') {
        continue;
      }

      const std::size_t indent = CountLeadingSpaces(line);

      if (!in_registry) {
        if (IsYamlMapKey(line, "camera_info_by_serial")) {
          in_registry = true;
          registry_indent = indent;
        }
        continue;
      }

      if (indent <= registry_indent && !IsYamlMapKey(line, "camera_info_by_serial")) {
        break;
      }

      if (indent == registry_indent + 2U) {
        if (const auto serial_key = MatchYamlMapKey(line)) {
          in_target_serial = *serial_key == camera_serial_;
          saw_target_serial = saw_target_serial || in_target_serial;
          serial_indent = indent;
          in_camera_info = false;
          in_roi = false;
          continue;
        }
      }

      if (!in_target_serial || indent <= serial_indent) {
        continue;
      }

      if (IsYamlMapKey(line, "camera_info")) {
        in_camera_info = true;
        camera_info_indent = indent;
        in_roi = false;
        continue;
      }

      if (!in_camera_info) {
        continue;
      }

      if (indent <= camera_info_indent) {
        in_camera_info = false;
        in_roi = false;
        continue;
      }

      if (in_roi && indent <= roi_indent) {
        in_roi = false;
      }

      if (IsYamlMapKey(line, "roi")) {
        in_roi = true;
        roi_indent = indent;
        continue;
      }

      if (in_roi) {
        if (const auto value = MatchYamlScalarValue(line, "x_offset")) {
          camera_info_roi_x_offset_ = static_cast<std::uint32_t>(
            ParseYamlNonNegativeInt(*value, "camera_info.roi.x_offset"));
          saw_any_camera_info_value = true;
          continue;
        }

        if (const auto value = MatchYamlScalarValue(line, "y_offset")) {
          camera_info_roi_y_offset_ = static_cast<std::uint32_t>(
            ParseYamlNonNegativeInt(*value, "camera_info.roi.y_offset"));
          saw_any_camera_info_value = true;
          continue;
        }

        if (const auto value = MatchYamlScalarValue(line, "height")) {
          camera_info_roi_height_ = static_cast<std::uint32_t>(
            ParseYamlNonNegativeInt(*value, "camera_info.roi.height"));
          saw_any_camera_info_value = true;
          continue;
        }

        if (const auto value = MatchYamlScalarValue(line, "width")) {
          camera_info_roi_width_ = static_cast<std::uint32_t>(
            ParseYamlNonNegativeInt(*value, "camera_info.roi.width"));
          saw_any_camera_info_value = true;
          continue;
        }

        if (const auto value = MatchYamlScalarValue(line, "do_rectify")) {
          camera_info_roi_do_rectify_ = ParseYamlBool(*value, "camera_info.roi.do_rectify");
          saw_any_camera_info_value = true;
          continue;
        }
      }

      if (const auto value = MatchYamlScalarValue(line, "distortion_model")) {
        camera_info_distortion_model_ = StripMatchingQuotes(*value);
        saw_any_camera_info_value = true;
        continue;
      }

      if (const auto value = MatchYamlScalarValue(line, "d")) {
        camera_info_d_ = ParseYamlDoubleList(*value, "camera_info.d");
        saw_any_camera_info_value = true;
        continue;
      }

      if (const auto value = MatchYamlScalarValue(line, "k")) {
        camera_info_k_ = ToFixedArray<9>(
          ParseYamlDoubleList(*value, "camera_info.k"),
          "camera_info.k");
        saw_any_camera_info_value = true;
        continue;
      }

      if (const auto value = MatchYamlScalarValue(line, "r")) {
        camera_info_r_ = ToFixedArray<9>(
          ParseYamlDoubleList(*value, "camera_info.r"),
          "camera_info.r");
        saw_any_camera_info_value = true;
        continue;
      }

      if (const auto value = MatchYamlScalarValue(line, "p")) {
        camera_info_p_ = ToFixedArray<12>(
          ParseYamlDoubleList(*value, "camera_info.p"),
          "camera_info.p");
        saw_any_camera_info_value = true;
        continue;
      }

      if (const auto value = MatchYamlScalarValue(line, "binning_x")) {
        camera_info_binning_x_ = static_cast<std::uint32_t>(
          ParseYamlNonNegativeInt(*value, "camera_info.binning_x"));
        saw_any_camera_info_value = true;
        continue;
      }

      if (const auto value = MatchYamlScalarValue(line, "binning_y")) {
        camera_info_binning_y_ = static_cast<std::uint32_t>(
          ParseYamlNonNegativeInt(*value, "camera_info.binning_y"));
        saw_any_camera_info_value = true;
        continue;
      }
    }

    if (!saw_target_serial) {
      RCLCPP_WARN(
        get_logger(),
        "Camera info YAML file '%s' does not contain serial '%s'. "
        "Using camera_info values from node parameters.",
        yaml_path.c_str(),
        camera_serial_.c_str());
      return;
    }

    if (!saw_any_camera_info_value) {
      RCLCPP_WARN(
        get_logger(),
        "Camera info YAML entry for serial '%s' in '%s' did not contain usable camera_info values. "
        "Using camera_info values from node parameters.",
        camera_serial_.c_str(),
        yaml_path.c_str());
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Loaded serial-indexed camera_info calibration for serial '%s' from '%s'.",
      camera_serial_.c_str(),
      yaml_path.c_str());
  }

  void ApplyCameraInfoYamlOverrides(const std::string & yaml_path)
  {
    std::ifstream stream(yaml_path);
    if (!stream.is_open()) {
      throw std::runtime_error("Failed to open camera_info YAML file: " + yaml_path);
    }

    std::vector<std::string> lines;
    std::string line;
    while (std::getline(stream, line)) {
      lines.push_back(line);
    }

    const bool has_serial_indexed_registry = std::any_of(
      lines.begin(),
      lines.end(),
      [](const std::string & candidate) {
        return IsYamlMapKey(candidate, "camera_info_by_serial");
      });

    if (has_serial_indexed_registry) {
      ApplySerialIndexedCameraInfoYamlOverrides(lines, yaml_path);
    } else {
      ApplyFlatCameraInfoYamlOverrides(lines, yaml_path);
    }
  }

  std::vector<int> CompressionParameters() const
  {
    if (rgb_compression_format_ == "png") {
      return {
        cv::IMWRITE_PNG_COMPRESSION,
        std::clamp(rgb_png_compression_level_, 0, 9)
      };
    }

    return {
      cv::IMWRITE_JPEG_QUALITY,
      std::clamp(rgb_jpeg_quality_, 0, 100)
    };
  }

  std::string CompressionExtension() const
  {
    return rgb_compression_format_ == "png" ? ".png" : ".jpg";
  }

  std::string CompressedFormatString() const
  {
    return rgb_compression_format_ == "png" ? "png" : "jpeg";
  }

  void InitializeGpuEncoder()
  {
    if (!publish_rgb_compressed_ || NormalizeName(rgb_encoder_) != "gpu") {
      return;
    }
    if (rgb_compression_format_ != "jpeg") {
      RCLCPP_WARN(get_logger(), "rgb_encoder=gpu only encodes JPEG; using the CPU for %s.",
        rgb_compression_format_.c_str());
      return;
    }
#ifdef FLIR_HAVE_GPU_JPEG
    auto encoder = std::make_unique<flir_gpu::GpuJpegEncoder>();
    std::string error;
    if (!encoder->Init(gpu_device_, std::clamp(rgb_jpeg_quality_, 1, 100), &error)) {
      RCLCPP_WARN(get_logger(), "GPU JPEG encoder unavailable (%s); using the CPU.", error.c_str());
      return;
    }
    RCLCPP_INFO(
      get_logger(), "RGB compressed on GPU %d: %s, demosaic on %s", gpu_device_,
      encoder->Describe().c_str(), gpu_demosaic_ ? "GPU (NPP)" : ("CPU (" + color_processing_ + ")").c_str());
    gpu_encoder_ = std::move(encoder);
#else
    RCLCPP_WARN(get_logger(),
      "rgb_encoder=gpu but this build has no GPU JPEG support (CUDA libraries not found at build "
      "time); using the CPU.");
#endif
  }

#ifdef FLIR_HAVE_GPU_JPEG
  static bool BayerPatternOf(Spinnaker::PixelFormatEnums format, flir_gpu::BayerPattern * pattern)
  {
    switch (format) {
      case Spinnaker::PixelFormat_BayerRG8: *pattern = flir_gpu::BayerPattern::kRGGB; return true;
      case Spinnaker::PixelFormat_BayerBG8: *pattern = flir_gpu::BayerPattern::kBGGR; return true;
      case Spinnaker::PixelFormat_BayerGR8: *pattern = flir_gpu::BayerPattern::kGRBG; return true;
      case Spinnaker::PixelFormat_BayerGB8: *pattern = flir_gpu::BayerPattern::kGBRG; return true;
      default: return false;
    }
  }
#endif

  // GPU path for image_rgb/compressed. Returns false when the frame must go the CPU
  // way (no GPU encoder, unsupported format, or a GPU error).
  bool TryGpuCompressed(
    const ImagePtr & image, const rclcpp::Time & stamp,
    sensor_msgs::msg::CompressedImage & msg)
  {
#ifdef FLIR_HAVE_GPU_JPEG
    if (!gpu_encoder_) {
      return false;
    }
    const int width = static_cast<int>(image->GetWidth());
    const int height = static_cast<int>(image->GetHeight());
    std::vector<std::uint8_t> jpeg;
    std::string error;
    bool ok = false;
    flir_gpu::BayerPattern pattern;
    if (gpu_demosaic_ && BayerPatternOf(image->GetPixelFormat(), &pattern)) {
      const int stride = image->GetStride() ? static_cast<int>(image->GetStride()) : width;
      ok = gpu_encoder_->EncodeBayer8(
        static_cast<const std::uint8_t *>(image->GetData()), width, height, stride, pattern, jpeg, &error);
    } else {
      ImagePtr rgb = image;
      if (image->GetPixelFormat() != Spinnaker::PixelFormat_RGB8 &&
        image->GetPixelFormat() != Spinnaker::PixelFormat_RGB8Packed)
      {
        rgb = image_processor_.Convert(image, Spinnaker::PixelFormat_RGB8);
      }
      const int stride = rgb->GetStride() ? static_cast<int>(rgb->GetStride()) : width * 3;
      ok = gpu_encoder_->EncodeRgb8(
        static_cast<const std::uint8_t *>(rgb->GetData()), width, height, stride, jpeg, &error);
    }
    if (!ok) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "GPU JPEG failed: %s", error.c_str());
      if (++gpu_failures_ >= 30) {
        RCLCPP_ERROR(get_logger(), "GPU JPEG failed %d times; switching to the CPU encoder.",
          gpu_failures_);
        gpu_encoder_.reset();
      }
      return false;
    }
    gpu_failures_ = 0;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.format = CompressedFormatString();
    msg.data = std::move(jpeg);
    return true;
#else
    (void)image;
    (void)stamp;
    (void)msg;
    return false;
#endif
  }

  sensor_msgs::msg::CompressedImage BuildCompressedImageMessage(
    const ImagePtr & rgb_image,
    const rclcpp::Time & stamp) const
  {
    if (rgb_image->GetPixelFormat() != Spinnaker::PixelFormat_RGB8 &&
      rgb_image->GetPixelFormat() != Spinnaker::PixelFormat_RGB8Packed)
    {
      throw std::runtime_error("Compressed RGB publish requires an RGB8 image.");
    }

    const int width = static_cast<int>(rgb_image->GetWidth());
    const int height = static_cast<int>(rgb_image->GetHeight());
    std::size_t step = rgb_image->GetStride();
    if (step == 0U) {
      step = static_cast<std::size_t>(width) * 3U;
    }

    cv::Mat rgb_view(
      height,
      width,
      CV_8UC3,
      rgb_image->GetData(),
      step);

    cv::Mat bgr_image;
    cv::cvtColor(rgb_view, bgr_image, cv::COLOR_RGB2BGR);

    std::vector<std::uint8_t> compressed_buffer;
    if (!cv::imencode(CompressionExtension(), bgr_image, compressed_buffer, CompressionParameters())) {
      throw std::runtime_error("OpenCV failed to encode RGB compressed image.");
    }

    sensor_msgs::msg::CompressedImage msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.format = CompressedFormatString();
    msg.data = std::move(compressed_buffer);
    return msg;
  }

  // What ReadFrameInfo found for one frame (chunk data, or the GVSP leader).
  // 카메라가 붙은 PC NIC 을 카메라 IP 로 찾는다 (같은 서브넷). 못 찾으면 빈 문자열.
  static std::string HostNicForCameraIp(std::uint32_t camera_ip)
  {
    struct ifaddrs * list = nullptr;
    if (getifaddrs(&list) != 0) {
      return "";
    }
    std::string found;
    for (struct ifaddrs * it = list; it != nullptr && found.empty(); it = it->ifa_next) {
      if (it->ifa_addr == nullptr || it->ifa_addr->sa_family != AF_INET || it->ifa_netmask == nullptr) {
        continue;
      }
      const std::uint32_t addr =
        ntohl(reinterpret_cast<struct sockaddr_in *>(it->ifa_addr)->sin_addr.s_addr);
      const std::uint32_t mask =
        ntohl(reinterpret_cast<struct sockaddr_in *>(it->ifa_netmask)->sin_addr.s_addr);
      if ((addr & mask) == (camera_ip & mask)) {
        found = it->ifa_name;
      }
    }
    freeifaddrs(list);
    return found;
  }

  static std::int64_t ReadSysNetValue(const std::string & nic, const char * file)
  {
    std::ifstream stream("/sys/class/net/" + nic + "/" + file);
    std::int64_t value = -1;
    if (stream >> value) {
      return value;
    }
    return -1;
  }

  // 링크 설정을 기동 로그에 한 줄로 남긴다. 2026-09-23 에 카메라가 프레임을 통째로 버리던 구간의 원인을
  // 사후에 못 가렸다 — MTU 가 1500 으로 돌아갔는지, 링크 제한이 얼마였는지가 어디에도 안 남아 있었다.
  void LogLinkSettings(INodeMap & node_map, INodeMap & tl_node_map)
  {
    link_throughput_limit_ = ReadIntegerNodeValue(node_map, "DeviceLinkThroughputLimit").value_or(-1);
    link_packet_size_ = ReadIntegerNodeValue(node_map, "GevSCPSPacketSize").value_or(-1);
    const auto ip = ReadIntegerNodeValue(tl_node_map, "GevDeviceIPAddress");
    if (ip && *ip > 0) {
      host_nic_ = HostNicForCameraIp(static_cast<std::uint32_t>(*ip));
    }
    std::int64_t speed = -1;
    if (!host_nic_.empty()) {
      host_mtu_ = ReadSysNetValue(host_nic_, "mtu");
      speed = ReadSysNetValue(host_nic_, "speed");
    }
    RCLCPP_INFO(
      get_logger(),
      "Link: throughput limit %.1f MB/s, packet %ld B; host NIC %s MTU %ld, %ld Mb/s.",
      link_throughput_limit_ > 0 ? static_cast<double>(link_throughput_limit_) / 1e6 : -1.0,
      static_cast<long>(link_packet_size_), host_nic_.empty() ? "?" : host_nic_.c_str(),
      static_cast<long>(host_mtu_), static_cast<long>(speed));
    if (host_mtu_ > 0 && link_packet_size_ > host_mtu_) {
      RCLCPP_ERROR(
        get_logger(),
        "GevSCPSPacketSize %ld is larger than the host NIC MTU %ld — the NIC drops every packet and no frame "
        "completes. Set the NIC to jumbo frames (MTU 9000) or lower the packet size.",
        static_cast<long>(link_packet_size_), static_cast<long>(host_mtu_));
    }
  }

  // 수신 상태 요약을 주기적으로 로그에 남긴다 (획득 스레드에서만 호출).
  //
  // 토픽으로 내보내지 않는다 — bag 에 토픽을 늘리지 않으려고 (2026-09-24 요청). 대신 형식을 고정해서
  // DM_clipGUI 의 진단기가 녹화 구간의 이 줄들을 모아 diagnostics.txt 에 넣는다. "Incomplete image" 경고는
  // 5초에 한 번만 찍혀 개수를 알 수 없었던 것을 대신한다.
  void ReportStreamStats(const rclcpp::Time & /*stamp*/)
  {
    if (stream_stats_interval_sec_ <= 0.0) {
      return;
    }
    const auto now_steady = std::chrono::steady_clock::now();
    if (next_stats_report_.time_since_epoch().count() == 0) {
      next_stats_report_ = now_steady + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(stream_stats_interval_sec_));
      return;
    }
    if (now_steady < next_stats_report_) {
      return;
    }
    next_stats_report_ = now_steady + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(stream_stats_interval_sec_));

    const auto stream_value = [this](const char * name) {
        return stream_node_map_ == nullptr ? static_cast<std::int64_t>(-1)
               : ReadIntegerNodeValue(*stream_node_map_, name).value_or(-1);
      };
    const std::int64_t missed = stream_value("StreamMissedPacketCount");
    const std::int64_t resend_req = stream_value("StreamPacketResendRequestCount");
    const std::int64_t resent = stream_value("StreamPacketResendReceivedCount");
    const std::int64_t lost_frames = stream_value("StreamLostFrameCount");
    const std::int64_t input_buffers = stream_value("StreamInputBufferCount");
    const bool trouble = window_incomplete_ > 0 || window_stale_ > 0;
    const auto text =
      "stream_stats: interval=%.0fs frames=%lu incomplete=%lu stale=%lu total_incomplete=%lu "
      "missed_packets=%ld resend_req=%ld resent=%ld stream_lost=%ld input_buffers=%ld "
      "limit_MBps=%.1f packet=%ld mtu=%ld nic=%s";
    if (trouble) {
      RCLCPP_WARN(
        get_logger(), text, stream_stats_interval_sec_,
        static_cast<unsigned long>(window_published_), static_cast<unsigned long>(window_incomplete_),
        static_cast<unsigned long>(window_stale_), static_cast<unsigned long>(incomplete_frames_),
        static_cast<long>(missed), static_cast<long>(resend_req), static_cast<long>(resent),
        static_cast<long>(lost_frames), static_cast<long>(input_buffers),
        link_throughput_limit_ > 0 ? static_cast<double>(link_throughput_limit_) / 1e6 : -1.0,
        static_cast<long>(link_packet_size_), static_cast<long>(host_mtu_),
        host_nic_.empty() ? "?" : host_nic_.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(), text, stream_stats_interval_sec_,
        static_cast<unsigned long>(window_published_), static_cast<unsigned long>(window_incomplete_),
        static_cast<unsigned long>(window_stale_), static_cast<unsigned long>(incomplete_frames_),
        static_cast<long>(missed), static_cast<long>(resend_req), static_cast<long>(resent),
        static_cast<long>(lost_frames), static_cast<long>(input_buffers),
        link_throughput_limit_ > 0 ? static_cast<double>(link_throughput_limit_) / 1e6 : -1.0,
        static_cast<long>(link_packet_size_), static_cast<long>(host_mtu_),
        host_nic_.empty() ? "?" : host_nic_.c_str());
    }
    window_published_ = window_incomplete_ = window_stale_ = 0;
  }

  struct FrameInfo
  {
    std::uint64_t frame_id{0};
    std::uint64_t timestamp_ns{0};
    std::int64_t exposure_ns{-1};   // this frame's own ExposureTime (chunk), -1 = unknown
    bool stale{false};              // did not advance: an older frame's leader values, not this frame's
    std::uint64_t skipped_before{0};  // incomplete frames dropped since the previous published frame
  };

  rclcpp::Time ResolveHeaderStamp(
    const FrameInfo & info,
    const rclcpp::Time & fallback_stamp)
  {
    if (timestamp_mode_ == "camera_latched") {
      return ResolveLatchedStamp(info, fallback_stamp);
    }
    if (timestamp_mode_ != "camera_first_frame" || camera_timestamp_header_disabled_due_to_instability_) {
      return fallback_stamp;
    }
    if (info.stale) {
      return fallback_stamp;          // an old frame's timestamp (see ReadFrameInfo) — not this frame's
    }

    const std::uint64_t camera_timestamp_ns = info.timestamp_ns;
    if (camera_timestamp_ns == 0U ||
      camera_timestamp_ns > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
    {
      return fallback_stamp;
    }

    const std::int64_t camera_timestamp = static_cast<std::int64_t>(camera_timestamp_ns);
    const std::int64_t fallback_ns = fallback_stamp.nanoseconds();

    if (!camera_timestamp_alignment_initialized_) {
      camera_timestamp_alignment_initialized_ = true;
      header_stamp_offset_ns_ = fallback_ns - camera_timestamp;
    } else if (camera_timestamp_ns < last_camera_timestamp_ns_) {
      camera_timestamp_header_disabled_due_to_instability_ = true;
      RCLCPP_WARN(
        get_logger(),
        "Camera timestamp moved backwards. Falling back to host receive time for topic headers. "
        "The original device timestamp remains in image_raw/metadata.camera_timestamp_ns.");
      return fallback_stamp;
    }

    last_camera_timestamp_ns_ = camera_timestamp_ns;
    return rclcpp::Time(
      header_stamp_offset_ns_ + camera_timestamp,
      get_clock()->get_clock_type());
  }

  // ---------- header.stamp = exposure start, mapped from the camera clock ----------
  //
  // Without PTP on the cameras, Image::GetTimeStamp() is each camera's own counter since
  // power-on. timestamp.mode=camera_latched moves it onto the host clock: every
  // timestamp.latch_interval_sec a TimestampLatch command is bracketed by two host clock
  // reads, which pins one counter value to a host time within half the GVCP round trip
  // (well under a millisecond). A line through recent samples gives offset and drift.
  //
  // Per frame, the exposure start (the camera timestamp, minus ExposureTime when the camera
  // latches at exposure end) is mapped onto the host clock. When the trigger pulses sit on a
  // fixed grid of the host clock (a PPS-locked N Hz pulse train and a host clock disciplined
  // to the same GNSS, e.g. via PTP), timestamp.trigger_grid_hz snaps that estimate to the
  // nearest grid instant — the trigger edge itself, identical for every camera. The distance
  // to the grid is logged; if it is not ~0 the grid assumption does not hold.

  struct ClockSample
  {
    std::int64_t camera_ns;
    std::int64_t host_ns;
    std::int64_t rtt_ns;
  };

  struct ClockMapping
  {
    std::int64_t camera_ref_ns{0};
    std::int64_t host_ref_ns{0};
    long double slope{1.0L};
    bool valid{false};
  };

  static std::int64_t MapCameraToHost(const ClockMapping & mapping, std::int64_t camera_ns)
  {
    return mapping.host_ref_ns +
           static_cast<std::int64_t>(std::llround(
             static_cast<double>(mapping.slope * static_cast<long double>(camera_ns - mapping.camera_ref_ns))));
  }

  static ClockMapping FitClockMapping(const std::deque<ClockSample> & samples)
  {
    ClockMapping mapping;
    if (samples.empty()) {
      return mapping;
    }
    std::int64_t min_rtt = std::numeric_limits<std::int64_t>::max();
    for (const auto & sample : samples) {
      min_rtt = std::min(min_rtt, sample.rtt_ns);
    }
    // Samples whose round trip was much slower than the best one pin the latch less tightly.
    std::vector<const ClockSample *> good;
    for (const auto & sample : samples) {
      if (sample.rtt_ns <= 2 * min_rtt + 200000) {
        good.push_back(&sample);
      }
    }
    const ClockSample & anchor = *good.back();
    mapping.camera_ref_ns = anchor.camera_ns;
    mapping.host_ref_ns = anchor.host_ns;
    mapping.valid = true;
    if (good.size() < 3 || good.back()->camera_ns - good.front()->camera_ns < 1000000000LL) {
      return mapping;
    }
    long double sx = 0.0L, sy = 0.0L, sxx = 0.0L, sxy = 0.0L;
    const long double n = static_cast<long double>(good.size());
    for (const ClockSample * sample : good) {
      const long double x = static_cast<long double>(sample->camera_ns - anchor.camera_ns);
      const long double y = static_cast<long double>(sample->host_ns - anchor.host_ns);
      sx += x;
      sy += y;
      sxx += x * x;
      sxy += x * y;
    }
    const long double denominator = n * sxx - sx * sx;
    if (denominator <= 0.0L) {
      return mapping;
    }
    const long double slope = (n * sxy - sx * sy) / denominator;
    if (std::fabs(static_cast<double>(slope - 1.0L)) > 1e-3) {
      return mapping;          // oscillators differ by ppm, not per mille — a bad fit, keep offset only
    }
    mapping.slope = slope;
    mapping.host_ref_ns += static_cast<std::int64_t>(std::llround(static_cast<double>((sy - slope * sx) / n)));
    return mapping;
  }

  // ---------- per-frame info: chunk data instead of the GVSP leader ----------
  //
  // Image::GetFrameID() / GetTimeStamp() come from the frame's GVSP leader packet. When the leader is lost
  // but the payload arrives, the frame is complete (IsIncomplete() is false) and the pixels are new, yet
  // the recycled buffer keeps the frame ID and timestamp of the frame that used it one buffer ring
  // earlier: stream.StreamBufferCountManual = 32 -> 32 frames, ~1 s at 30 Hz. Seen on the rig on the
  // cameras that were losing packets. Chunk data rides inside the payload, so the FrameID / Timestamp /
  // ExposureTime read from it belong to the pixels they arrived with.
  void ApplyChunkData(INodeMap & node_map)
  {
    chunk_timestamp_ = chunk_frame_id_ = chunk_exposure_ = false;
    if (!chunk_data_enable_) {
      return;
    }
    try {
      CBooleanPtr mode = node_map.GetNode("ChunkModeActive");
      CEnumerationPtr selector = node_map.GetNode("ChunkSelector");
      if (!IsWritable(mode) || !IsWritable(selector)) {
        RCLCPP_WARN(
          get_logger(),
          "Chunk data is not writable on this camera: frame ID / timestamp come from the GVSP leader "
          "(an old frame's values after a lost leader).");
        return;
      }
      const auto offered = [&](const char * name) {
          return IsReadable(CEnumEntryPtr(selector->GetEntryByName(name)));
        };
      if (!offered("Timestamp") && !offered("FrameID") && !offered("ExposureTime")) {
        RCLCPP_INFO(
          get_logger(), "The camera offers no Timestamp/FrameID/ExposureTime chunk; chunk data stays off.");
        return;
      }
      mode->SetValue(true);
      const auto enable_chunk = [&](const char * name) {
          CEnumEntryPtr entry = selector->GetEntryByName(name);
          if (!IsReadable(entry)) {
            return false;
          }
          selector->SetIntValue(entry->GetValue());
          CBooleanPtr enable = node_map.GetNode("ChunkEnable");
          if (IsWritable(enable)) {
            enable->SetValue(true);
            return true;
          }
          return IsReadable(enable) && enable->GetValue();
        };
      chunk_timestamp_ = enable_chunk("Timestamp");
      chunk_frame_id_ = enable_chunk("FrameID");
      chunk_exposure_ = enable_chunk("ExposureTime");
      RCLCPP_INFO(
        get_logger(), "Chunk data: Timestamp=%s FrameID=%s ExposureTime=%s",
        chunk_timestamp_ ? "on" : "n/a", chunk_frame_id_ ? "on" : "n/a", chunk_exposure_ ? "on" : "n/a");
    } catch (const Spinnaker::Exception & exception) {
      RCLCPP_WARN(get_logger(), "Could not enable chunk data (%s); using the GVSP leader.", exception.what());
    }
  }

  // Called on the acquisition thread only.
  FrameInfo ReadFrameInfo(const ImagePtr & image)
  {
    FrameInfo info;
    info.frame_id = image->GetFrameID();
    info.timestamp_ns = image->GetTimeStamp();
    info.skipped_before = skipped_since_last_frame_;
    skipped_since_last_frame_ = 0U;
    if (chunk_timestamp_ || chunk_frame_id_ || chunk_exposure_) {
      try {
        const Spinnaker::ChunkData chunk = image->GetChunkData();
        if (chunk_timestamp_) {
          const std::int64_t timestamp = chunk.GetTimestamp();
          // First frame: the chunk counter must be the same clock as the leader's (same units and
          // epoch) — otherwise mixing them breaks the camera->host mapping. Stop using it if not.
          if (!chunk_timestamp_checked_ && timestamp > 0) {
            chunk_timestamp_checked_ = true;
            const long double gap = std::fabs(
              static_cast<long double>(timestamp) - static_cast<long double>(info.timestamp_ns));
            if (gap > 1.0e8L) {
              chunk_timestamp_ = false;
              RCLCPP_WARN(
                get_logger(), "Chunk Timestamp (%lld) is not on the leader timestamp's clock (%llu); "
                "using the leader timestamp.", static_cast<long long>(timestamp),
                static_cast<unsigned long long>(info.timestamp_ns));
            }
          }
          if (chunk_timestamp_ && timestamp > 0) {
            info.timestamp_ns = static_cast<std::uint64_t>(timestamp);
          }
        }
        if (chunk_frame_id_) {
          const std::int64_t frame_id = chunk.GetFrameID();
          if (frame_id >= 0) {
            info.frame_id = static_cast<std::uint64_t>(frame_id);
          }
        }
        if (chunk_exposure_) {
          const double exposure_us = chunk.GetExposureTime();
          if (std::isfinite(exposure_us) && exposure_us > 0.0) {
            info.exposure_ns = static_cast<std::int64_t>(std::llround(exposure_us * 1000.0));
          }
        }
      } catch (const Spinnaker::Exception &) {
      }
    }

    if (last_frame_timestamp_ns_ != 0U && info.timestamp_ns <= last_frame_timestamp_ns_) {
      info.stale = true;
      ++stale_frames_;
      ++window_stale_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "Frame info did not advance (frame_id %llu, timestamp %.3f s behind the previous frame; %llu such "
        "frames so far). The frame's GVSP leader was probably lost while its payload arrived, and the "
        "buffer kept an older frame's info. %s",
        static_cast<unsigned long long>(info.frame_id),
        static_cast<double>(last_frame_timestamp_ns_ - info.timestamp_ns) / 1e9,
        static_cast<unsigned long long>(stale_frames_),
        chunk_timestamp_ ? "" : "Chunk data would carry the right values (chunk_data.enable).");
      return info;
    }
    if (last_frame_timestamp_ns_ != 0U) {
      // Trigger period = median of recent intervals between good frames (a dropped frame is one 2x interval).
      frame_intervals_ns_.push_back(static_cast<std::int64_t>(info.timestamp_ns - last_frame_timestamp_ns_));
      while (frame_intervals_ns_.size() > 31U) {
        frame_intervals_ns_.pop_front();
      }
      std::vector<std::int64_t> sorted(frame_intervals_ns_.begin(), frame_intervals_ns_.end());
      std::nth_element(sorted.begin(), sorted.begin() + sorted.size() / 2, sorted.end());
      frame_period_ns_ = sorted[sorted.size() / 2];
    }
    last_frame_timestamp_ns_ = info.timestamp_ns;
    return info;
  }

  void StartClockSync()
  {
    if (timestamp_mode_ != "camera_latched") {
      return;
    }
    if (timestamp_latch_interval_sec_ <= 0.0) {
      throw std::runtime_error("timestamp.latch_interval_sec must be > 0.");
    }
    if (timestamp_trigger_grid_hz_ < 0.0) {
      throw std::runtime_error("timestamp.trigger_grid_hz must be >= 0 (0 = no snapping).");
    }
    next_grid_report_ = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    try {
      clock_tick_frequency_ = ReadTimestampTickFrequency();
    } catch (const std::exception & exception) {
      RCLCPP_WARN(get_logger(), "%s Assuming 1 GHz timestamp ticks.", exception.what());
    }
    // A short burst so the mapping starts from the fastest of several round trips.
    int latched = 0;
    std::string last_error;
    for (int i = 0; i < 5; ++i) {
      try {
        SampleCameraClock(clock_tick_frequency_);
        ++latched;
      } catch (const std::exception & exception) {
        last_error = exception.what();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (latched == 0) {
      RCLCPP_ERROR(
        get_logger(),
        "timestamp.mode=camera_latched: the camera clock could not be latched (%s). header.stamp is host "
        "receive time until a latch succeeds.", last_error.c_str());
    }
    RCLCPP_INFO(
      get_logger(),
      "header.stamp = exposure start from the camera clock (timestamp.mode=camera_latched): "
      "exposure_latch=%s, capture_offset=%.3f ms, trigger grid %s, latch every %.1f s.",
      timestamp_exposure_latch_.c_str(),
      static_cast<double>(timestamp_capture_offset_ns_) / 1e6,
      timestamp_trigger_grid_hz_ > 0.0 ?
      (std::to_string(timestamp_trigger_grid_hz_) + " Hz").c_str() : "off",
      timestamp_latch_interval_sec_);
    clock_sync_thread_ = std::thread(&FlirSpinnakerCameraNode::ClockSyncLoop, this);
  }

  // StartClockSync already took the first samples; this keeps the mapping following the drift.
  void ClockSyncLoop()
  {
    std::uint64_t failures = 0U;
    while (rclcpp::ok() && running_.load()) {
      const auto until = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(timestamp_latch_interval_sec_));
      while (running_.load() && std::chrono::steady_clock::now() < until) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      if (!running_.load()) {
        break;
      }
      try {
        SampleCameraClock(clock_tick_frequency_);
        failures = 0U;
      } catch (const std::exception & exception) {
        ++failures;
        if (failures == 1U || failures % 10U == 0U) {
          RCLCPP_WARN(
            get_logger(), "Camera clock latch failed (%lu in a row): %s",
            static_cast<unsigned long>(failures), exception.what());
        }
      }
    }
  }

  void SampleCameraClock(std::uint64_t tick_frequency)
  {
    if (camera_node_map_ == nullptr) {
      throw std::runtime_error("camera node map is not available");
    }
    const std::int64_t before = now().nanoseconds();
    if (!ExecuteCommandByName(*camera_node_map_, "TimestampLatch")) {
      throw std::runtime_error("TimestampLatch is not available/writable");
    }
    const std::int64_t after = now().nanoseconds();
    const auto ticks = ReadIntegerNodeValue(*camera_node_map_, "TimestampLatchValue");
    if (!ticks || *ticks < 0) {
      throw std::runtime_error("TimestampLatchValue is not readable");
    }
    const std::int64_t camera_ns = tick_frequency == 1000000000ULL ? *ticks :
      static_cast<std::int64_t>(std::llround(
        static_cast<long double>(*ticks) * 1.0e9L / static_cast<long double>(tick_frequency)));
    const ClockSample sample{camera_ns, before + (after - before) / 2, after - before};

    // ExposureTime for the exposure-end correction. Read here, not per frame: a register read per
    // frame is a GigE round trip per frame per camera. Under auto exposure it can lag by one interval.
    if (const auto exposure_us =
      ReadCameraNumericNodeValue({"ExposureTime", "ExposureTime_FloatVal", "ExposureTime_Val"}))
    {
      if (std::isfinite(*exposure_us) && *exposure_us >= 0.0) {
        exposure_ns_.store(static_cast<std::int64_t>(std::llround(*exposure_us * 1000.0)));
      }
    }

    std::lock_guard<std::mutex> lock(clock_mutex_);
    if (clock_mapping_.valid && sample.rtt_ns < 5000000) {
      // The host clock stepped (ptp4l/phc2sys correcting it): the old samples no longer fit. A real step
      // persists, a slow latch reply (the A70 in particular) does not — so one outlier is dropped and only
      // a second sample off by the same amount restarts the mapping. Resetting on every outlier made the
      // A70 stamps jump by several ms every few minutes (2026-09-22: 4 resets per A70 in ~40 min).
      const std::int64_t jump = sample.host_ns - MapCameraToHost(clock_mapping_, sample.camera_ns);
      if (std::llabs(jump) > 5000000) {
        if (!pending_clock_jump_ || std::llabs(jump - pending_clock_jump_ns_) > 2000000) {
          pending_clock_jump_ = true;
          pending_clock_jump_ns_ = jump;
          return;
        }
        RCLCPP_WARN(
          get_logger(),
          "Host clock moved %.3f ms against the camera clock (two samples in a row — clock step). "
          "Restarting the camera->host timestamp mapping.", static_cast<double>(jump) / 1e6);
        clock_samples_.clear();
      }
      pending_clock_jump_ = false;
    }
    clock_samples_.push_back(sample);
    while (clock_samples_.size() > 16U) {
      clock_samples_.pop_front();
    }
    clock_mapping_ = FitClockMapping(clock_samples_);
  }

  rclcpp::Time ResolveLatchedStamp(const FrameInfo & info, const rclcpp::Time & fallback_stamp)
  {
    const std::uint64_t camera_timestamp_ns = info.timestamp_ns;
    ClockMapping mapping;
    {
      std::lock_guard<std::mutex> lock(clock_mutex_);
      mapping = clock_mapping_;
    }
    if (!mapping.valid || camera_timestamp_ns == 0U ||
      camera_timestamp_ns > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "timestamp.mode=camera_latched: no camera clock sample yet; header.stamp is host receive time.");
      return fallback_stamp;
    }

    std::int64_t exposure_start_ns = static_cast<std::int64_t>(camera_timestamp_ns);
    if (info.stale) {
      // This frame carries an older frame's timestamp (ReadFrameInfo). Its pixels are new: it is the trigger
      // after the previous published frame, plus any incomplete frames dropped in between. (Counting
      // triggers from the host arrival spacing instead mis-counts whenever OldestFirst dequeues a backlog.)
      if (last_exposure_start_ns_ == 0 || frame_period_ns_ <= 0) {
        return fallback_stamp;
      }
      const std::int64_t steps = 1 + static_cast<std::int64_t>(info.skipped_before);
      exposure_start_ns = last_exposure_start_ns_ + steps * frame_period_ns_;
    } else if (timestamp_exposure_latch_ == "end") {
      // The frame's own exposure from its chunk; else the clock thread's periodic register read.
      const std::int64_t exposure_ns = info.exposure_ns >= 0 ? info.exposure_ns : exposure_ns_.load();
      if (exposure_ns < 0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "timestamp.exposure_latch=end but ExposureTime is not known yet; header.stamp is host receive time.");
        return fallback_stamp;
      }
      exposure_start_ns -= exposure_ns;
    }
    if (!info.stale) {
      // A fixed delay between the moment the frame represents and the camera's timestamp — for a
      // microbolometer (no exposure) the detector time constant plus in-camera processing. Measured.
      exposure_start_ns -= timestamp_capture_offset_ns_;
    }
    last_exposure_start_ns_ = exposure_start_ns;

    std::int64_t stamp_ns = MapCameraToHost(mapping, exposure_start_ns);
    if (timestamp_trigger_grid_hz_ > 0.0) {
      const long double period = 1.0e9L / static_cast<long double>(timestamp_trigger_grid_hz_);
      const long double slot = std::round(
        static_cast<long double>(stamp_ns - timestamp_trigger_grid_offset_ns_) / period);
      const std::int64_t snapped = timestamp_trigger_grid_offset_ns_ +
        static_cast<std::int64_t>(std::llround(slot * period));
      TrackGridResidual(stamp_ns - snapped);
      stamp_ns = snapped;
    }

    if (last_header_stamp_ns_ != 0 && stamp_ns <= last_header_stamp_ns_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "header.stamp did not advance (%.3f ms vs the previous frame): the camera->host mapping or "
        "the trigger grid is off.", static_cast<double>(stamp_ns - last_header_stamp_ns_) / 1e6);
    }
    last_header_stamp_ns_ = stamp_ns;
    return rclcpp::Time(stamp_ns, get_clock()->get_clock_type());
  }

  // Called on the acquisition thread only.
  void TrackGridResidual(std::int64_t residual_ns)
  {
    grid_residuals_ns_.push_back(residual_ns);
    const auto now_steady = std::chrono::steady_clock::now();
    if (now_steady < next_grid_report_ || grid_residuals_ns_.empty()) {
      return;
    }
    next_grid_report_ = now_steady + std::chrono::seconds(30);
    std::vector<std::int64_t> sorted = grid_residuals_ns_;
    std::sort(sorted.begin(), sorted.end());
    const double median_ms = static_cast<double>(sorted[sorted.size() / 2]) / 1e6;
    const double worst_ms = static_cast<double>(
      std::max(std::llabs(sorted.front()), std::llabs(sorted.back()))) / 1e6;
    if (std::fabs(median_ms) > timestamp_grid_warn_ms_) {
      RCLCPP_WARN(
        get_logger(),
        "header.stamp is snapped to the %.3f Hz trigger grid, but frames land %.3f ms from it "
        "(median of %zu, worst %.3f ms). The grid assumption does not hold: the trigger pulses are not "
        "on that grid of the host clock (not PPS-locked, or offset -> timestamp.trigger_grid_offset_ns), "
        "the host clock is not synced to the same GNSS, or timestamp.exposure_latch is wrong.",
        timestamp_trigger_grid_hz_, median_ms, sorted.size(), worst_ms);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "header.stamp on the %.3f Hz trigger grid: frames land %.3f ms from it (median of %zu, worst %.3f ms).",
        timestamp_trigger_grid_hz_, median_ms, sorted.size(), worst_ms);
    }
    grid_residuals_ns_.clear();
  }

  flir_spinnaker_camera::msg::FlirMetadata BuildMetadataMessage(
    const ImagePtr & original_image,
    const PreparedRawImage & raw_image,
    const FrameInfo & info,
    const rclcpp::Time & stamp) const
  {
    flir_spinnaker_camera::msg::FlirMetadata msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.width = static_cast<std::uint32_t>(raw_image.image->GetWidth());
    msg.height = static_cast<std::uint32_t>(raw_image.image->GetHeight());
    msg.step = static_cast<std::uint32_t>(raw_image.image->GetStride());
    msg.encoding = raw_image.encoding;
    msg.pixel_format = original_image->GetPixelFormatName().c_str();
    // Chunk values when the camera sends them (right even after a lost leader), else the leader's.
    msg.camera_frame_id = info.frame_id;
    msg.camera_timestamp_ns = info.timestamp_ns;
    msg.acquisition_frame_rate_enable =
      ReadCameraBooleanNodeValue({"AcquisitionFrameRateEnable"}).value_or(false);
    msg.acquisition_frame_rate_hz =
      ReadCameraNumericNodeValue({"AcquisitionFrameRate", "FrameRateHz_Val"}).value_or(std::nan(""));
    msg.exposure_auto = ReadCameraTextNodeValue({"ExposureAuto"}).value_or("");
    // The frame's own exposure when chunk data carries it; otherwise the register as of now.
    msg.exposure_time_us = info.exposure_ns >= 0 ? static_cast<double>(info.exposure_ns) / 1000.0 :
      ReadCameraNumericNodeValue({"ExposureTime", "ExposureTime_FloatVal", "ExposureTime_Val"}).value_or(std::nan(""));
    msg.gain_auto = ReadCameraTextNodeValue({"GainAuto"}).value_or("");
    msg.gain_db = ReadCameraNumericNodeValue({"Gain", "GainDB_Val", "Gain_Val"}).value_or(std::nan(""));
    msg.black_level = ReadCameraNumericNodeValue({"BlackLevel", "BlackLevel_Val"}).value_or(std::nan(""));
    msg.gamma_enable = ReadCameraBooleanNodeValue({"GammaEnable", "GammaEnable_Val"}).value_or(false);
    msg.gamma = ReadCameraNumericNodeValue({"Gamma", "Gamma_FloatVal", "Gamma_Val"}).value_or(std::nan(""));
    msg.balance_white_auto = ReadCameraTextNodeValue({"BalanceWhiteAuto"}).value_or("");
    return msg;
  }

  sensor_msgs::msg::CameraInfo BuildCameraInfoMessage(
    const PreparedRawImage & raw_image,
    const rclcpp::Time & stamp) const
  {
    sensor_msgs::msg::CameraInfo msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.width = static_cast<std::uint32_t>(raw_image.image->GetWidth());
    msg.height = static_cast<std::uint32_t>(raw_image.image->GetHeight());
    msg.distortion_model = camera_info_distortion_model_;
    msg.d = camera_info_d_;
    std::copy(camera_info_k_.begin(), camera_info_k_.end(), msg.k.begin());
    std::copy(camera_info_r_.begin(), camera_info_r_.end(), msg.r.begin());
    std::copy(camera_info_p_.begin(), camera_info_p_.end(), msg.p.begin());
    msg.binning_x = camera_info_binning_x_;
    msg.binning_y = camera_info_binning_y_;
    msg.roi.x_offset = camera_info_roi_x_offset_;
    msg.roi.y_offset = camera_info_roi_y_offset_;
    msg.roi.height = camera_info_roi_height_;
    msg.roi.width = camera_info_roi_width_;
    msg.roi.do_rectify = camera_info_roi_do_rectify_;
    return msg;
  }

  bool IsFatalAcquisitionException(const Spinnaker::Exception & exception) const
  {
    const std::string message = exception.what();
    return message.find("Stream has been aborted") != std::string::npos;
  }

  void AcquisitionLoop()
  {
    while (rclcpp::ok() && running_.load()) {
      try {
        ImagePtr image = camera_->GetNextImage(acquisition_timeout_ms_);

        if (image->IsIncomplete()) {
          RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Incomplete image received. status=%d",
            static_cast<int>(image->GetImageStatus()));
          ++skipped_since_last_frame_;       // a trigger that produced no published frame
          ++incomplete_frames_;
          ++window_incomplete_;
          image->Release();
          continue;
        }

        const rclcpp::Time host_stamp = now();
        const FrameInfo info = ReadFrameInfo(image);
        const rclcpp::Time stamp = ResolveHeaderStamp(info, host_stamp);

        if (publish_raw_ || publish_camera_info_ || publish_metadata_) {
          const auto raw_spec = RawOutputSpecForPixelFormat(image->GetPixelFormat());
          if (!raw_spec.has_value()) {
            const std::string pixel_format_name = image->GetPixelFormatName().c_str();
            RCLCPP_WARN_THROTTLE(
              get_logger(),
              *get_clock(),
              5000,
              "Skipping image_raw publish because pixel format '%s' is not mapped to a ROS encoding.",
              pixel_format_name.c_str());
          } else {
            const PreparedRawImage raw_image = PrepareRawImage(image);

            if (publish_raw_) {
              raw_pub_->publish(BuildImageMessage(raw_image.image, raw_image.encoding, stamp));
            }

            if (publish_camera_info_) {
              camera_info_pub_->publish(BuildCameraInfoMessage(raw_image, stamp));
            }

            if (publish_metadata_) {
              metadata_pub_->publish(BuildMetadataMessage(image, raw_image, info, stamp));
            }
          }
        }

        sensor_msgs::msg::CompressedImage gpu_msg;
        if (publish_rgb_compressed_ && TryGpuCompressed(image, stamp, gpu_msg)) {
          rgb_compressed_pub_->publish(std::move(gpu_msg));
        } else if (publish_rgb_compressed_) {
          ImagePtr rgb_image = image;
          if (image->GetPixelFormat() != Spinnaker::PixelFormat_RGB8 &&
            image->GetPixelFormat() != Spinnaker::PixelFormat_RGB8Packed)
          {
            rgb_image = image_processor_.Convert(image, Spinnaker::PixelFormat_RGB8);
          }

          if (publish_rgb_compressed_) {
            rgb_compressed_pub_->publish(BuildCompressedImageMessage(rgb_image, stamp));
          }
        }

        ++published_frames_;
        ++window_published_;
        ReportStreamStats(stamp);
        image->Release();
      } catch (const Spinnaker::Exception & exception) {
        if (!running_.load()) {
          break;
        }

        if (IsFatalAcquisitionException(exception)) {
          RCLCPP_ERROR(
            get_logger(),
            "Stopping acquisition after fatal Spinnaker stream error: %s",
            exception.what());
          running_.store(false);
          break;
        }

        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          3000,
          "Spinnaker acquisition warning: %s",
          exception.what());
      } catch (const std::exception & exception) {
        if (!running_.load()) {
          break;
        }

        RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          3000,
          "Image publish error: %s",
          exception.what());
      }
    }
  }

  void ShutdownCamera() noexcept
  {
    running_.store(false);

    if (acquisition_thread_.joinable()) {
      acquisition_thread_.join();
    }

    if (ptp_action_thread_.joinable()) {
      ptp_action_thread_.join();
    }

    if (clock_sync_thread_.joinable()) {
      clock_sync_thread_.join();
    }

    if (camera_) {
      try {
        if (acquisition_started_) {
          camera_->EndAcquisition();
          acquisition_started_ = false;
        }
      } catch (const Spinnaker::Exception & exception) {
        RCLCPP_WARN(get_logger(), "EndAcquisition failed during shutdown: %s", exception.what());
      }

      try {
        camera_->DeInit();
      } catch (const Spinnaker::Exception & exception) {
        RCLCPP_WARN(get_logger(), "Camera DeInit failed during shutdown: %s", exception.what());
      }

      camera_ = nullptr;
    }

    if (camera_list_.GetSize() > 0U) {
      camera_list_.Clear();
    }

    if (system_) {
      system_->ReleaseInstance();
      system_ = nullptr;
    }

    camera_node_map_ = nullptr;
    stream_node_map_ = nullptr;
    tl_device_node_map_ = nullptr;
    control_bindings_.clear();
    pending_control_overrides_.clear();
    control_parameter_callback_handle_.reset();
  }

  bool publish_raw_;
  bool publish_camera_info_;
  bool publish_metadata_;
  bool publish_rgb_compressed_;
  std::string publisher_qos_reliability_;
  int publisher_qos_depth_;
  std::string frame_id_;
  std::string camera_serial_;
  int camera_index_;
  int camera_init_max_attempts_;
  int camera_init_retry_delay_ms_;
  int acquisition_timeout_ms_;
  bool use_camera_timestamp_in_header_;
  std::string timestamp_mode_;
  std::string timestamp_exposure_latch_;
  double timestamp_trigger_grid_hz_;
  std::int64_t timestamp_trigger_grid_offset_ns_;
  double timestamp_grid_warn_ms_;
  std::int64_t timestamp_capture_offset_ns_;
  double timestamp_latch_interval_sec_;
  bool chunk_data_enable_;
  double stream_stats_interval_sec_;
  std::string camera_info_yaml_path_;
  bool auto_pixel_format_;
  std::string pixel_format_;
  std::string buffer_handling_mode_;
  std::string hardware_trigger_role_;
  std::string hardware_trigger_master_output_line_;
  std::string hardware_trigger_master_line_source_;
  std::vector<std::string> hardware_trigger_master_line_source_fallbacks_;
  bool hardware_trigger_master_enable_3v3_;
  bool hardware_trigger_master_require_3v3_;
  std::string hardware_trigger_master_3v3_line_;
  std::vector<std::string> hardware_trigger_master_3v3_enable_nodes_;
  std::string hardware_trigger_slave_trigger_source_;
  std::string hardware_trigger_slave_trigger_activation_;
  std::string hardware_trigger_slave_trigger_overlap_;
  bool network_force_ip_enable_;
  std::string network_force_ip_address_;
  std::string network_force_ip_subnet_mask_;
  std::string network_force_ip_gateway_;
  bool network_force_ip_only_if_link_local_;
  int network_force_ip_wait_after_ms_;
  int network_force_ip_rediscovery_timeout_ms_;
  bool ptp_enabled_;
  std::string ptp_mode_;
  bool ptp_wait_for_sync_;
  bool ptp_require_sync_;
  int ptp_sync_timeout_ms_;
  int ptp_sync_poll_ms_;
  std::vector<std::string> ptp_accepted_statuses_;
  std::string ptp_action_role_;
  std::string ptp_action_selector_;
  std::string ptp_action_trigger_selector_;
  std::string ptp_action_trigger_source_;
  std::string ptp_action_trigger_activation_;
  std::string ptp_action_trigger_overlap_;
  std::uint32_t ptp_action_device_key_;
  std::uint32_t ptp_action_group_key_;
  std::uint32_t ptp_action_group_mask_;
  double ptp_action_rate_hz_;
  double ptp_action_schedule_ahead_ms_;
  double ptp_action_start_delay_ms_;
  bool ptp_action_align_to_second_;
  bool ptp_action_request_ack_;
  int ptp_action_expected_ack_count_;
  double ptp_action_log_interval_sec_;
  std::string color_processing_;
  std::string rgb_compression_format_;
  int rgb_jpeg_quality_;
  int rgb_png_compression_level_;
  std::string rgb_encoder_;
  bool gpu_demosaic_;
  int gpu_device_;
  int gpu_failures_ = 0;
#ifdef FLIR_HAVE_GPU_JPEG
  std::unique_ptr<flir_gpu::GpuJpegEncoder> gpu_encoder_;
#endif
  bool camera_timestamp_alignment_initialized_ = false;
  bool camera_timestamp_header_disabled_due_to_instability_ = false;
  std::uint64_t last_camera_timestamp_ns_ = 0U;
  std::int64_t header_stamp_offset_ns_ = 0;
  // timestamp.mode=camera_latched — samples/mapping shared with the clock sync thread
  std::mutex clock_mutex_;
  std::deque<ClockSample> clock_samples_;
  ClockMapping clock_mapping_;
  std::atomic<std::int64_t> exposure_ns_{-1};
  std::uint64_t clock_tick_frequency_{1000000000ULL};
  bool pending_clock_jump_{false};          // clock sync thread only (under clock_mutex_)
  std::int64_t pending_clock_jump_ns_{0};
  // chunk data / frame info (acquisition thread)
  bool chunk_timestamp_{false};
  bool chunk_frame_id_{false};
  bool chunk_exposure_{false};
  bool chunk_timestamp_checked_{false};
  std::uint64_t last_frame_timestamp_ns_{0};
  std::uint64_t stale_frames_{0};
  // 수신 상태 집계 (획득 스레드에서만 센다)
  std::uint64_t published_frames_{0};
  std::uint64_t incomplete_frames_{0};
  std::uint64_t window_published_{0};
  std::uint64_t window_incomplete_{0};
  std::uint64_t window_stale_{0};
  std::chrono::steady_clock::time_point next_stats_report_{};
  std::int64_t link_throughput_limit_{-1};
  std::int64_t link_packet_size_{-1};
  std::int64_t host_mtu_{-1};
  std::string host_nic_;
  std::deque<std::int64_t> frame_intervals_ns_;
  std::int64_t frame_period_ns_{0};
  std::int64_t last_exposure_start_ns_{0};
  std::uint64_t skipped_since_last_frame_{0};
  std::int64_t last_header_stamp_ns_ = 0;
  std::vector<std::int64_t> grid_residuals_ns_;
  std::chrono::steady_clock::time_point next_grid_report_{};
  std::string camera_info_distortion_model_;
  std::vector<double> camera_info_d_;
  std::array<double, 9> camera_info_k_{};
  std::array<double, 9> camera_info_r_{};
  std::array<double, 12> camera_info_p_{};
  std::uint32_t camera_info_binning_x_{0};
  std::uint32_t camera_info_binning_y_{0};
  std::uint32_t camera_info_roi_x_offset_{0};
  std::uint32_t camera_info_roi_y_offset_{0};
  std::uint32_t camera_info_roi_height_{0};
  std::uint32_t camera_info_roi_width_{0};
  bool camera_info_roi_do_rectify_{false};

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<flir_spinnaker_camera::msg::FlirMetadata>::SharedPtr metadata_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr rgb_compressed_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr control_parameter_callback_handle_;

  SystemPtr system_;
  CameraList camera_list_;
  CameraPtr camera_;
  ImageProcessor image_processor_;
  INodeMap * camera_node_map_{nullptr};
  INodeMap * stream_node_map_{nullptr};
  INodeMap * tl_device_node_map_{nullptr};
  std::unordered_map<std::string, ControlBinding> control_bindings_;
  std::vector<rclcpp::Parameter> pending_control_overrides_;

  std::atomic<bool> running_{false};
  bool acquisition_started_{false};
  std::thread acquisition_thread_;
  std::thread ptp_action_thread_;
  std::thread clock_sync_thread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  int exit_code = 0;

  try {
    auto node = std::make_shared<FlirSpinnakerCameraNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exception) {
    std::cerr << "flir_spinnaker_camera_node failed: " << exception.what() << std::endl;
    exit_code = 1;
  }

  rclcpp::shutdown();
  return exit_code;
}
