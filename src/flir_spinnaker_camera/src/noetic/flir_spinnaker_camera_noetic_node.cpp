#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
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
#include "flir_spinnaker_camera/SetCameraControl.h"

namespace
{

using Spinnaker::CameraList;
using Spinnaker::CameraPtr;
using Spinnaker::ColorProcessingAlgorithm;
using Spinnaker::ImageProcessor;
using Spinnaker::ImagePtr;
using Spinnaker::InterfaceList;
using Spinnaker::InterfacePtr;
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

std::atomic_bool g_shutdown_due_to_fatal_error{false};

std::string TrimAscii(std::string value)
{
  const auto is_space = [](unsigned char character) {
      return std::isspace(character) != 0;
    };

  value.erase(value.begin(), std::find_if_not(value.begin(), value.end(), is_space));
  value.erase(std::find_if_not(value.rbegin(), value.rend(), is_space).base(), value.end());
  return value;
}

bool IsFatalSpinnakerException(const Spinnaker::Exception & exception)
{
  const int error = static_cast<int>(exception.GetError());
  return exception == Spinnaker::SPINNAKER_ERR_IO ||
    exception == Spinnaker::SPINNAKER_ERR_ABORT ||
    exception == Spinnaker::SPINNAKER_ERR_INVALID_HANDLE ||
    exception == Spinnaker::SPINNAKER_ERR_NOT_AVAILABLE ||
    error == -1024;
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

std::vector<std::string> ParseYamlStringList(std::string value)
{
  value = TrimAscii(std::move(value));
  if (value.size() < 2U || value.front() != '[' || value.back() != ']') {
    return {};
  }

  value = value.substr(1, value.size() - 2U);
  std::vector<std::string> result;
  std::stringstream stream(value);
  std::string token;
  while (std::getline(stream, token, ',')) {
    token = StripMatchingQuotes(token);
    if (!token.empty()) {
      result.push_back(token);
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

std::uint32_t ParseYamlUint32(std::string value, const std::string & field_name)
{
  value = StripMatchingQuotes(std::move(value));
  std::size_t consumed = 0U;
  const unsigned long long parsed = std::stoull(value, &consumed, 0);
  if (consumed != value.size() || parsed > std::numeric_limits<std::uint32_t>::max()) {
    throw std::runtime_error(field_name + " must be a uint32 value.");
  }
  return static_cast<std::uint32_t>(parsed);
}

double ParseYamlDouble(std::string value, const std::string & field_name)
{
  value = StripMatchingQuotes(std::move(value));
  std::size_t consumed = 0U;
  const double parsed = std::stod(value, &consumed);
  if (consumed != value.size()) {
    throw std::runtime_error(field_name + " must be a floating-point value.");
  }
  return parsed;
}

bool ParseYamlBool(std::string value, const std::string & field_name)
{
  value = NormalizeName(StripMatchingQuotes(std::move(value)));
  if (value == "true" || value == "1" || value == "yes" || value == "on") {
    return true;
  }
  if (value == "false" || value == "0" || value == "no" || value == "off") {
    return false;
  }
  throw std::runtime_error("Expected boolean value for '" + field_name + "'.");
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

bool SetStringByName(INodeMap & node_map, const char * node_name, const std::string & value)
{
  CStringPtr string_node = node_map.GetNode(node_name);
  if (!IsWritable(string_node)) {
    return false;
  }
  string_node->SetValue(value.c_str(), true);
  return true;
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

std::optional<std::string> ReadEnumerationByName(INodeMap & node_map, const char * node_name)
{
  try {
    CEnumerationPtr enum_node = node_map.GetNode(node_name);
    if (IsReadable(enum_node)) {
      return std::string(enum_node->ToString().c_str());
    }
  } catch (...) {
  }
  return std::nullopt;
}

std::optional<std::int64_t> ReadIntegerByName(INodeMap & node_map, const char * node_name)
{
  try {
    CIntegerPtr int_node = node_map.GetNode(node_name);
    if (IsReadable(int_node)) {
      return int_node->GetValue();
    }
  } catch (...) {
  }
  return std::nullopt;
}

std::optional<double> ReadFloatByName(INodeMap & node_map, const char * node_name)
{
  try {
    CFloatPtr float_node = node_map.GetNode(node_name);
    if (IsReadable(float_node)) {
      return float_node->GetValue();
    }
  } catch (...) {
  }
  return std::nullopt;
}

std::optional<bool> ReadBooleanByName(INodeMap & node_map, const char * node_name)
{
  try {
    CBooleanPtr bool_node = node_map.GetNode(node_name);
    if (IsReadable(bool_node)) {
      return bool_node->GetValue();
    }
  } catch (...) {
  }
  return std::nullopt;
}

std::optional<std::string> ReadStringByName(INodeMap & node_map, const char * node_name)
{
  try {
    CStringPtr string_node = node_map.GetNode(node_name);
    if (IsReadable(string_node)) {
      return std::string(string_node->GetValue().c_str());
    }
  } catch (...) {
  }
  return std::nullopt;
}

std::string ReadNodeValueText(INodeMap & node_map, const std::string & node_name)
{
  if (const auto value = ReadEnumerationByName(node_map, node_name.c_str())) {
    return *value;
  }
  if (const auto value = ReadBooleanByName(node_map, node_name.c_str())) {
    return *value ? "true" : "false";
  }
  if (const auto value = ReadIntegerByName(node_map, node_name.c_str())) {
    return std::to_string(*value);
  }
  if (const auto value = ReadFloatByName(node_map, node_name.c_str())) {
    std::ostringstream stream;
    stream << std::setprecision(12) << *value;
    return stream.str();
  }
  if (const auto value = ReadStringByName(node_map, node_name.c_str())) {
    return *value;
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

std::uint32_t ParseIpv4Address(std::string value, const std::string & field_name)
{
  value = TrimAscii(StripMatchingQuotes(std::move(value)));
  if (value.empty()) {
    throw std::runtime_error(field_name + " must not be empty.");
  }

  std::array<unsigned long, 4> octets{};
  std::stringstream stream(value);
  std::string token;
  for (std::size_t index = 0; index < octets.size(); ++index) {
    if (!std::getline(stream, token, '.')) {
      throw std::runtime_error(field_name + " must be an IPv4 address; got '" + value + "'.");
    }

    token = TrimAscii(token);
    if (token.empty()) {
      throw std::runtime_error(field_name + " contains an empty IPv4 octet.");
    }

    std::size_t consumed = 0U;
    const unsigned long octet = std::stoul(token, &consumed);
    if (consumed != token.size() || octet > 255UL) {
      throw std::runtime_error(field_name + " contains invalid IPv4 octet '" + token + "'.");
    }
    octets[index] = octet;
  }

  if (std::getline(stream, token, '.')) {
    throw std::runtime_error(field_name + " must contain exactly four IPv4 octets; got '" + value + "'.");
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
    return Spinnaker::SPINNAKER_COLOR_PROCESSING_ALGORITHM_NONE;
  }
  if (normalized == "nearest" || normalized == "nearestneighbor") {
    return Spinnaker::SPINNAKER_COLOR_PROCESSING_ALGORITHM_NEAREST_NEIGHBOR;
  }
  if (normalized == "edge" || normalized == "edge_sensing") {
    return Spinnaker::SPINNAKER_COLOR_PROCESSING_ALGORITHM_EDGE_SENSING;
  }
  if (normalized == "hqlinear" || normalized == "highqualitylinear") {
    return Spinnaker::SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR;
  }
  return Spinnaker::SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR;
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
  throw std::runtime_error("hardware_trigger.role must be one of none, master, or slave; got '" + value + "'.");
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
  throw std::runtime_error("ptp_action.role must be one of none, receiver, or sender; got '" + value + "'.");
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

struct ControlMaps
{
  std::map<std::string, std::string> enum_controls;
  std::map<std::string, bool> bool_controls;
  std::map<std::string, double> numeric_controls;
};

struct CameraConfig
{
  std::string name{"flir_camera"};
  std::string camera_namespace;
  std::string camera_config_yaml_path;
  std::map<std::string, std::string> config_yaml_params;
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
  bool force_ip_enable{true};
  std::string force_ip_address;
  std::string force_ip_subnet_mask{"255.255.255.0"};
  std::string force_ip_gateway{"0.0.0.0"};
  int force_ip_wait_after_ms{1500};
  int force_ip_rediscovery_timeout_ms{5000};
  int force_ip_max_attempts{3};
  std::string hardware_trigger_role{"none"};
  std::string hardware_trigger_master_output_line{"Line1"};
  std::string hardware_trigger_master_line_source{"ExposureActive"};
  std::vector<std::string> hardware_trigger_master_line_source_fallbacks{"FrameTriggerWait", "UserOutput0"};
  bool hardware_trigger_master_enable_3v3{true};
  bool hardware_trigger_master_require_3v3{true};
  std::string hardware_trigger_master_3v3_line{"Line2"};
  std::vector<std::string> hardware_trigger_master_3v3_enable_nodes{"V3_3Enable", "Line3V3Enable", "LineVoltageEnable"};
  std::string hardware_trigger_slave_trigger_source{"Line3"};
  std::string hardware_trigger_slave_trigger_activation{"RisingEdge"};
  std::string hardware_trigger_slave_trigger_overlap{"ReadOut"};
  bool ptp_enabled{false};
  std::string ptp_mode{"SlaveOnly"};
  bool ptp_wait_for_sync{true};
  bool ptp_require_sync{true};
  int ptp_sync_timeout_ms{10000};
  int ptp_sync_poll_ms{250};
  std::vector<std::string> ptp_accepted_statuses{"Slave"};
  bool ptp_action_enable{true};
  std::string ptp_action_role{"none"};
  std::string ptp_action_selector{"Action0"};
  std::string ptp_action_trigger_selector{"FrameStart"};
  std::string ptp_action_trigger_source{"Action0"};
  std::string ptp_action_trigger_activation{"RisingEdge"};
  std::string ptp_action_trigger_overlap{"ReadOut"};
  std::uint32_t ptp_action_device_key{1U};
  std::uint32_t ptp_action_group_key{1U};
  std::uint32_t ptp_action_group_mask{4294967295U};
  double ptp_action_rate_hz{20.0};
  double ptp_action_schedule_ahead_ms{100.0};
  double ptp_action_start_delay_ms{1000.0};
  bool ptp_action_request_ack{false};
  int ptp_action_expected_ack_count{0};
  double ptp_action_log_interval_sec{5.0};
  sensor_msgs::CameraInfo camera_info;
  ControlMaps camera_controls;
  ControlMaps stream_controls;
  ControlMaps tl_device_controls;
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
    if (config.serial.empty()) {
      return;
    }
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

std::map<std::string, std::string> ParseRosParametersYaml(const std::string & path)
{
  std::map<std::string, std::string> values;
  if (path.empty()) {
    return values;
  }

  bool in_ros_parameters = false;
  std::size_t parameter_indent = 0U;
  for (const std::string & raw_line : ReadLines(path)) {
    const std::string without_comment = RemoveYamlComment(raw_line);
    const std::string line = TrimAscii(without_comment);
    if (line.empty()) {
      continue;
    }

    const std::size_t indent = CountLeadingSpaces(without_comment);
    if (!in_ros_parameters) {
      if (IsYamlMapKey(raw_line, "ros__parameters")) {
        in_ros_parameters = true;
        parameter_indent = indent;
      }
      continue;
    }

    if (indent <= parameter_indent && !IsYamlMapKey(raw_line, "ros__parameters")) {
      break;
    }

    const std::size_t separator = line.find(':');
    if (separator == std::string::npos) {
      continue;
    }

    const std::string key = TrimAscii(line.substr(0, separator));
    const std::string value = TrimAscii(line.substr(separator + 1U));
    if (!key.empty() && !value.empty()) {
      values[StripMatchingQuotes(key)] = value;
    }
  }
  return values;
}

std::optional<std::string> FindYamlValue(
  const std::map<std::string, std::string> & values,
  const char * key)
{
  const auto iter = values.find(key);
  if (iter == values.end() || iter->second.empty()) {
    return std::nullopt;
  }
  return iter->second;
}

bool LooksLikeYamlBool(const std::string & value)
{
  const std::string trimmed = TrimAscii(value);
  if (!trimmed.empty() && (trimmed.front() == '"' || trimmed.front() == '\'')) {
    return false;
  }
  const std::string normalized = NormalizeName(trimmed);
  return normalized == "true" || normalized == "false" || normalized == "yes" ||
         normalized == "no" || normalized == "on" || normalized == "off" ||
         normalized == "1" || normalized == "0";
}

bool TryParseYamlDouble(std::string value, double & parsed)
{
  value = TrimAscii(std::move(value));
  if (!value.empty() && (value.front() == '"' || value.front() == '\'')) {
    return false;
  }
  if (value.empty()) {
    return false;
  }

  std::size_t consumed = 0U;
  try {
    parsed = std::stod(value, &consumed);
  } catch (...) {
    return false;
  }
  return consumed == value.size();
}

void AddControlFromYamlValue(ControlMaps & controls, const std::string & node_name, const std::string & raw_value)
{
  if (node_name.empty() || raw_value.empty() || TrimAscii(raw_value).front() == '[') {
    return;
  }

  if (LooksLikeYamlBool(raw_value)) {
    controls.bool_controls[node_name] = ParseYamlBool(raw_value, node_name);
    controls.enum_controls.erase(node_name);
    controls.numeric_controls.erase(node_name);
    return;
  }

  double numeric_value = 0.0;
  if (TryParseYamlDouble(raw_value, numeric_value)) {
    controls.numeric_controls[node_name] = numeric_value;
    controls.enum_controls.erase(node_name);
    controls.bool_controls.erase(node_name);
    return;
  }

  controls.enum_controls[node_name] = StripMatchingQuotes(raw_value);
  controls.bool_controls.erase(node_name);
  controls.numeric_controls.erase(node_name);
}

void ApplyKnownYamlParameters(CameraConfig & config, const std::map<std::string, std::string> & values)
{
  if (const auto value = FindYamlValue(values, "frame_id")) {
    config.frame_id = StripMatchingQuotes(*value);
  }
  if (const auto value = FindYamlValue(values, "acquisition_timeout_ms")) {
    config.acquisition_timeout_ms = ParseYamlNonNegativeInt(*value, "acquisition_timeout_ms");
  }
  if (const auto value = FindYamlValue(values, "publisher_qos_depth")) {
    config.publisher_queue_size = ParseYamlNonNegativeInt(*value, "publisher_qos_depth");
  }
  if (const auto value = FindYamlValue(values, "publisher_queue_size")) {
    config.publisher_queue_size = ParseYamlNonNegativeInt(*value, "publisher_queue_size");
  }
  if (const auto value = FindYamlValue(values, "use_camera_timestamp_in_header")) {
    config.use_camera_timestamp_in_header = ParseYamlBool(*value, "use_camera_timestamp_in_header");
  }
  if (const auto value = FindYamlValue(values, "auto_pixel_format")) {
    config.auto_pixel_format = ParseYamlBool(*value, "auto_pixel_format");
  }
  if (const auto value = FindYamlValue(values, "pixel_format")) {
    config.pixel_format = StripMatchingQuotes(*value);
  }
  if (const auto value = FindYamlValue(values, "buffer_handling_mode")) {
    config.buffer_handling_mode = StripMatchingQuotes(*value);
  }
  if (const auto value = FindYamlValue(values, "color_processing")) {
    config.color_processing = StripMatchingQuotes(*value);
  }
  if (const auto value = FindYamlValue(values, "publish_raw")) {
    config.publish_raw = ParseYamlBool(*value, "publish_raw");
  }
  if (const auto value = FindYamlValue(values, "publish_camera_info")) {
    config.publish_camera_info = ParseYamlBool(*value, "publish_camera_info");
  }
  if (const auto value = FindYamlValue(values, "publish_metadata")) {
    config.publish_metadata = ParseYamlBool(*value, "publish_metadata");
  }
  if (const auto value = FindYamlValue(values, "publish_rgb_compressed")) {
    config.publish_rgb_compressed = ParseYamlBool(*value, "publish_rgb_compressed");
  }
  if (const auto value = FindYamlValue(values, "rgb_compression_format")) {
    config.rgb_compression_format = StripMatchingQuotes(*value);
  }
  if (const auto value = FindYamlValue(values, "rgb_jpeg_quality")) {
    config.rgb_jpeg_quality = ParseYamlNonNegativeInt(*value, "rgb_jpeg_quality");
  }
  if (const auto value = FindYamlValue(values, "rgb_png_compression_level")) {
    config.rgb_png_compression_level = ParseYamlNonNegativeInt(*value, "rgb_png_compression_level");
  }
  if (const auto value = FindYamlValue(values, "camera_info.yaml_path")) {
    config.camera_info_yaml_path = StripMatchingQuotes(*value);
  }
  if (const auto value = FindYamlValue(values, "camera_info_yaml_path")) {
    config.camera_info_yaml_path = StripMatchingQuotes(*value);
  }

  if (const auto value = FindYamlValue(values, "network.force_ip.enable")) {
    config.force_ip_enable = ParseYamlBool(*value, "network.force_ip.enable");
  }
  if (const auto value = FindYamlValue(values, "network.force_ip.address")) {
    config.force_ip_address = StripMatchingQuotes(*value);
  }
  if (const auto value = FindYamlValue(values, "network.force_ip.subnet_mask")) {
    config.force_ip_subnet_mask = StripMatchingQuotes(*value);
  }
  if (const auto value = FindYamlValue(values, "network.force_ip.gateway")) {
    config.force_ip_gateway = StripMatchingQuotes(*value);
  }
  if (const auto value = FindYamlValue(values, "network.force_ip.wait_after_ms")) {
    config.force_ip_wait_after_ms = ParseYamlNonNegativeInt(*value, "network.force_ip.wait_after_ms");
  }
  if (const auto value = FindYamlValue(values, "network.force_ip.rediscovery_timeout_ms")) {
    config.force_ip_rediscovery_timeout_ms =
      ParseYamlNonNegativeInt(*value, "network.force_ip.rediscovery_timeout_ms");
  }

  if (const auto value = FindYamlValue(values, "hardware_trigger.role")) {
    config.hardware_trigger_role = NormalizeHardwareTriggerRole(StripMatchingQuotes(*value));
  }
  if (const auto value = FindYamlValue(values, "hardware_trigger.master.output_line")) {
    config.hardware_trigger_master_output_line = StripMatchingQuotes(*value);
  }
  if (const auto value = FindYamlValue(values, "hardware_trigger.master.line_source")) {
    config.hardware_trigger_master_line_source = StripMatchingQuotes(*value);
  }
  if (const auto value = FindYamlValue(values, "hardware_trigger.master.line_source_fallbacks")) {
    config.hardware_trigger_master_line_source_fallbacks = ParseYamlStringList(*value);
  }
  if (const auto value = FindYamlValue(values, "hardware_trigger.master.enable_3v3")) {
    config.hardware_trigger_master_enable_3v3 = ParseYamlBool(*value, "hardware_trigger.master.enable_3v3");
  }
  if (const auto value = FindYamlValue(values, "hardware_trigger.master.require_3v3")) {
    config.hardware_trigger_master_require_3v3 = ParseYamlBool(*value, "hardware_trigger.master.require_3v3");
  }
  if (const auto value = FindYamlValue(values, "hardware_trigger.master.line_3v3")) {
    config.hardware_trigger_master_3v3_line = StripMatchingQuotes(*value);
  }
  if (const auto value = FindYamlValue(values, "hardware_trigger.master.line_3v3_enable_nodes")) {
    config.hardware_trigger_master_3v3_enable_nodes = ParseYamlStringList(*value);
  }
  if (const auto value = FindYamlValue(values, "hardware_trigger.slave.trigger_source")) {
    config.hardware_trigger_slave_trigger_source = StripMatchingQuotes(*value);
  }
  if (const auto value = FindYamlValue(values, "hardware_trigger.slave.trigger_activation")) {
    config.hardware_trigger_slave_trigger_activation = StripMatchingQuotes(*value);
  }
  if (const auto value = FindYamlValue(values, "hardware_trigger.slave.trigger_overlap")) {
    config.hardware_trigger_slave_trigger_overlap = StripMatchingQuotes(*value);
  }

  if (const auto value = FindYamlValue(values, "ptp.enable")) {
    config.ptp_enabled = ParseYamlBool(*value, "ptp.enable");
  }
  if (const auto value = FindYamlValue(values, "ptp.mode")) {
    config.ptp_mode = StripMatchingQuotes(*value);
  }
  if (const auto value = FindYamlValue(values, "ptp.wait_for_sync")) {
    config.ptp_wait_for_sync = ParseYamlBool(*value, "ptp.wait_for_sync");
  }
  if (const auto value = FindYamlValue(values, "ptp.require_sync")) {
    config.ptp_require_sync = ParseYamlBool(*value, "ptp.require_sync");
  }
  if (const auto value = FindYamlValue(values, "ptp.sync_timeout_ms")) {
    config.ptp_sync_timeout_ms = ParseYamlNonNegativeInt(*value, "ptp.sync_timeout_ms");
  }
  if (const auto value = FindYamlValue(values, "ptp.sync_poll_ms")) {
    config.ptp_sync_poll_ms = ParseYamlNonNegativeInt(*value, "ptp.sync_poll_ms");
  }
  if (const auto value = FindYamlValue(values, "ptp.accepted_statuses")) {
    config.ptp_accepted_statuses = ParseYamlStringList(*value);
  }

  if (const auto value = FindYamlValue(values, "ptp_action.enable")) {
    config.ptp_action_enable = ParseYamlBool(*value, "ptp_action.enable");
  }
  if (const auto value = FindYamlValue(values, "ptp_action.role")) {
    config.ptp_action_role = NormalizePtpActionRole(StripMatchingQuotes(*value));
  }
  if (const auto value = FindYamlValue(values, "ptp_action.selector")) {
    config.ptp_action_selector = StripMatchingQuotes(*value);
  }
  if (const auto value = FindYamlValue(values, "ptp_action.trigger_selector")) {
    config.ptp_action_trigger_selector = StripMatchingQuotes(*value);
  }
  if (const auto value = FindYamlValue(values, "ptp_action.trigger_source")) {
    config.ptp_action_trigger_source = StripMatchingQuotes(*value);
  }
  if (const auto value = FindYamlValue(values, "ptp_action.trigger_activation")) {
    config.ptp_action_trigger_activation = StripMatchingQuotes(*value);
  }
  if (const auto value = FindYamlValue(values, "ptp_action.trigger_overlap")) {
    config.ptp_action_trigger_overlap = StripMatchingQuotes(*value);
  }
  if (const auto value = FindYamlValue(values, "ptp_action.device_key")) {
    config.ptp_action_device_key = ParseYamlUint32(*value, "ptp_action.device_key");
  }
  if (const auto value = FindYamlValue(values, "ptp_action.group_key")) {
    config.ptp_action_group_key = ParseYamlUint32(*value, "ptp_action.group_key");
  }
  if (const auto value = FindYamlValue(values, "ptp_action.group_mask")) {
    config.ptp_action_group_mask = ParseYamlUint32(*value, "ptp_action.group_mask");
  }
  if (const auto value = FindYamlValue(values, "ptp_action.rate_hz")) {
    config.ptp_action_rate_hz = ParseYamlDouble(*value, "ptp_action.rate_hz");
  }
  if (const auto value = FindYamlValue(values, "ptp_action.schedule_ahead_ms")) {
    config.ptp_action_schedule_ahead_ms = ParseYamlDouble(*value, "ptp_action.schedule_ahead_ms");
  }
  if (const auto value = FindYamlValue(values, "ptp_action.start_delay_ms")) {
    config.ptp_action_start_delay_ms = ParseYamlDouble(*value, "ptp_action.start_delay_ms");
  }
  if (const auto value = FindYamlValue(values, "ptp_action.request_ack")) {
    config.ptp_action_request_ack = ParseYamlBool(*value, "ptp_action.request_ack");
  }
  if (const auto value = FindYamlValue(values, "ptp_action.expected_ack_count")) {
    config.ptp_action_expected_ack_count = ParseYamlNonNegativeInt(*value, "ptp_action.expected_ack_count");
  }
  if (const auto value = FindYamlValue(values, "ptp_action.log_interval_sec")) {
    config.ptp_action_log_interval_sec = ParseYamlDouble(*value, "ptp_action.log_interval_sec");
  }

  for (const auto & item : values) {
    if (item.first.rfind("camera.", 0) == 0) {
      AddControlFromYamlValue(config.camera_controls, item.first.substr(7), item.second);
    } else if (item.first.rfind("stream.", 0) == 0) {
      AddControlFromYamlValue(config.stream_controls, item.first.substr(7), item.second);
    } else if (item.first.rfind("tl_device.", 0) == 0) {
      AddControlFromYamlValue(config.tl_device_controls, item.first.substr(10), item.second);
    }
  }
}

void ApplyCameraInfoParameters(CameraConfig & config, const std::map<std::string, std::string> & values)
{
  if (const auto value = FindYamlValue(values, "camera_info.distortion_model")) {
    config.camera_info.distortion_model = StripMatchingQuotes(*value);
  }
  if (const auto value = FindYamlValue(values, "camera_info.d")) {
    config.camera_info.D = ParseYamlDoubleList(*value, "camera_info.d");
  }
  if (const auto value = FindYamlValue(values, "camera_info.k")) {
    config.camera_info.K = ToFixedArray<9>(ParseYamlDoubleList(*value, "camera_info.k"), "camera_info.k");
  }
  if (const auto value = FindYamlValue(values, "camera_info.r")) {
    config.camera_info.R = ToFixedArray<9>(ParseYamlDoubleList(*value, "camera_info.r"), "camera_info.r");
  }
  if (const auto value = FindYamlValue(values, "camera_info.p")) {
    config.camera_info.P = ToFixedArray<12>(ParseYamlDoubleList(*value, "camera_info.p"), "camera_info.p");
  }
  if (const auto value = FindYamlValue(values, "camera_info.binning_x")) {
    config.camera_info.binning_x = static_cast<std::uint32_t>(
      ParseYamlNonNegativeInt(*value, "camera_info.binning_x"));
  }
  if (const auto value = FindYamlValue(values, "camera_info.binning_y")) {
    config.camera_info.binning_y = static_cast<std::uint32_t>(
      ParseYamlNonNegativeInt(*value, "camera_info.binning_y"));
  }
  if (const auto value = FindYamlValue(values, "camera_info.roi.x_offset")) {
    config.camera_info.roi.x_offset = static_cast<std::uint32_t>(
      ParseYamlNonNegativeInt(*value, "camera_info.roi.x_offset"));
  }
  if (const auto value = FindYamlValue(values, "camera_info.roi.y_offset")) {
    config.camera_info.roi.y_offset = static_cast<std::uint32_t>(
      ParseYamlNonNegativeInt(*value, "camera_info.roi.y_offset"));
  }
  if (const auto value = FindYamlValue(values, "camera_info.roi.height")) {
    config.camera_info.roi.height = static_cast<std::uint32_t>(
      ParseYamlNonNegativeInt(*value, "camera_info.roi.height"));
  }
  if (const auto value = FindYamlValue(values, "camera_info.roi.width")) {
    config.camera_info.roi.width = static_cast<std::uint32_t>(
      ParseYamlNonNegativeInt(*value, "camera_info.roi.width"));
  }
  if (const auto value = FindYamlValue(values, "camera_info.roi.do_rectify")) {
    config.camera_info.roi.do_rectify = ParseYamlBool(*value, "camera_info.roi.do_rectify");
  }
}

template<typename T>
void GetPrivateParam(const ros::NodeHandle & private_nh, const std::string & name, T & value)
{
  private_nh.param(name, value, value);
}

void GetPrivateParam(const ros::NodeHandle & private_nh, const std::string & name, std::string & value)
{
  XmlRpc::XmlRpcValue raw_value;
  if (!private_nh.getParam(name, raw_value)) {
    return;
  }

  if (raw_value.getType() == XmlRpc::XmlRpcValue::TypeString) {
    value = static_cast<std::string>(raw_value);
  } else if (raw_value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    value = std::to_string(static_cast<int>(raw_value));
  } else if (raw_value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    std::ostringstream stream;
    stream << static_cast<double>(raw_value);
    value = stream.str();
  } else if (raw_value.getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
    value = static_cast<bool>(raw_value) ? "true" : "false";
  }
}

void LoadSharedParams(const ros::NodeHandle & private_nh, CameraConfig & config)
{
  GetPrivateParam(private_nh, "camera_config_yaml_path", config.camera_config_yaml_path);
  config.config_yaml_params = ParseRosParametersYaml(config.camera_config_yaml_path);
  ApplyKnownYamlParameters(config, config.config_yaml_params);

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
  GetPrivateParam(private_nh, "force_ip_enable", config.force_ip_enable);
  GetPrivateParam(private_nh, "force_ip_address", config.force_ip_address);
  GetPrivateParam(private_nh, "force_ip_subnet_mask", config.force_ip_subnet_mask);
  GetPrivateParam(private_nh, "force_ip_gateway", config.force_ip_gateway);
  GetPrivateParam(private_nh, "force_ip_wait_after_ms", config.force_ip_wait_after_ms);
  GetPrivateParam(private_nh, "force_ip_rediscovery_timeout_ms", config.force_ip_rediscovery_timeout_ms);
  GetPrivateParam(private_nh, "force_ip_max_attempts", config.force_ip_max_attempts);
  GetPrivateParam(private_nh, "hardware_trigger_role", config.hardware_trigger_role);
  GetPrivateParam(private_nh, "ptp_enable", config.ptp_enabled);
  GetPrivateParam(private_nh, "ptp_wait_for_sync", config.ptp_wait_for_sync);
  GetPrivateParam(private_nh, "ptp_require_sync", config.ptp_require_sync);
  GetPrivateParam(private_nh, "ptp_sync_timeout_ms", config.ptp_sync_timeout_ms);
  GetPrivateParam(private_nh, "ptp_action_enable", config.ptp_action_enable);
  GetPrivateParam(private_nh, "ptp_action_role", config.ptp_action_role);
  GetPrivateParam(private_nh, "ptp_action_rate_hz", config.ptp_action_rate_hz);
  GetPrivateParam(private_nh, "ptp_action_schedule_ahead_ms", config.ptp_action_schedule_ahead_ms);
  GetPrivateParam(private_nh, "ptp_action_request_ack", config.ptp_action_request_ack);
  GetPrivateParam(private_nh, "ptp_action_expected_ack_count", config.ptp_action_expected_ack_count);

  const auto read_enum_control = [&](const char * param_name, const char * node_name) {
      std::string value;
      if (private_nh.getParam(param_name, value)) {
        config.camera_controls.enum_controls[node_name] = value;
      }
    };
  const auto read_bool_control = [&](const char * param_name, const char * node_name) {
      bool value = false;
      if (private_nh.getParam(param_name, value)) {
        config.camera_controls.bool_controls[node_name] = value;
      }
    };
  const auto read_numeric_control = [&](const char * param_name, const char * node_name) {
      double value = 0.0;
      if (private_nh.getParam(param_name, value)) {
        config.camera_controls.numeric_controls[node_name] = value;
      }
    };

  read_bool_control("camera/AcquisitionFrameRateEnable", "AcquisitionFrameRateEnable");
  read_numeric_control("camera/AcquisitionFrameRate", "AcquisitionFrameRate");
  read_numeric_control("camera/FrameRateHz_Val", "FrameRateHz_Val");
  read_enum_control("camera/ExposureAuto", "ExposureAuto");
  read_numeric_control("camera/ExposureTime", "ExposureTime");
  read_numeric_control("camera/ExposureTime_FloatVal", "ExposureTime_FloatVal");
  read_enum_control("camera/GainAuto", "GainAuto");
  read_numeric_control("camera/Gain", "Gain");
  read_numeric_control("camera/GainDB_Val", "GainDB_Val");
  read_numeric_control("camera/Width", "Width");
  read_numeric_control("camera/Height", "Height");
  read_numeric_control("camera/DeviceLinkThroughputLimit", "DeviceLinkThroughputLimit");

  config.hardware_trigger_role = NormalizeHardwareTriggerRole(config.hardware_trigger_role);
  config.ptp_action_role = NormalizePtpActionRole(config.ptp_action_role);
  if (config.ptp_action_enable && config.ptp_action_role != "none") {
    config.ptp_enabled = true;
  }
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
    ApplyCameraInfoParameters(base, base.config_yaml_params);
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
    if (const std::string value = ValueFromCameraEntry(entry, "hardware_trigger_role"); !value.empty()) {
      config.hardware_trigger_role = NormalizeHardwareTriggerRole(value);
    }
    if (const std::string value = ValueFromCameraEntry(entry, "ptp_action_role"); !value.empty()) {
      config.ptp_action_role = NormalizePtpActionRole(value);
      if (config.ptp_action_enable && config.ptp_action_role != "none") {
        config.ptp_enabled = true;
      }
    }
    if (const std::string value = ValueFromCameraEntry(entry, "force_ip_enable"); !value.empty()) {
      config.force_ip_enable = ParseYamlBool(value, "force_ip_enable");
    }
    if (const std::string value = ValueFromCameraEntry(entry, "force_ip_address"); !value.empty()) {
      config.force_ip_address = value;
    }
    if (const std::string value = ValueFromCameraEntry(entry, "force_ip_subnet_mask"); !value.empty()) {
      config.force_ip_subnet_mask = value;
    }
    if (const std::string value = ValueFromCameraEntry(entry, "force_ip_gateway"); !value.empty()) {
      config.force_ip_gateway = value;
    }
    if (const std::string value = ValueFromCameraEntry(entry, "force_ip_wait_after_ms"); !value.empty()) {
      config.force_ip_wait_after_ms = ParseYamlNonNegativeInt(value, "force_ip_wait_after_ms");
    }
    if (const std::string value = ValueFromCameraEntry(entry, "force_ip_rediscovery_timeout_ms"); !value.empty()) {
      config.force_ip_rediscovery_timeout_ms = ParseYamlNonNegativeInt(value, "force_ip_rediscovery_timeout_ms");
    }
    if (const std::string value = ValueFromCameraEntry(entry, "force_ip_max_attempts"); !value.empty()) {
      config.force_ip_max_attempts = ParseYamlNonNegativeInt(value, "force_ip_max_attempts");
    }
    config.index = static_cast<int>(index);
    SetDefaultCameraInfo(config);
    ApplyCameraInfoParameters(config, config.config_yaml_params);
    ApplyCameraInfoYaml(config);
    configs.push_back(std::move(config));
  }

  if (configs.empty()) {
    throw std::runtime_error("No cameras found in inventory: " + cameras_file);
  }
  return configs;
}

CameraPtr TryFindCameraBySerial(CameraList & camera_list, const std::string & serial)
{
  if (serial.empty()) {
    return nullptr;
  }

  for (std::size_t index = 0; index < camera_list.GetSize(); ++index) {
    CameraPtr camera = camera_list.GetByIndex(static_cast<unsigned int>(index));
    INodeMap & tl_node_map = camera->GetTLDeviceNodeMap();
    if (SafeNodeString(tl_node_map, "DeviceSerialNumber") == serial) {
      return camera;
    }
  }
  return nullptr;
}

CameraPtr TryFindCameraForConfig(CameraList & camera_list, const CameraConfig & config)
{
  if (!config.serial.empty()) {
    return TryFindCameraBySerial(camera_list, config.serial);
  }

  if (config.index < 0 || static_cast<std::size_t>(config.index) >= camera_list.GetSize()) {
    return nullptr;
  }
  return camera_list.GetByIndex(static_cast<unsigned int>(config.index));
}

CameraPtr WaitForCameraForConfig(
  SystemPtr system,
  CameraList & camera_list,
  const CameraConfig & config,
  int timeout_ms)
{
  const auto poll_period = std::chrono::milliseconds(250);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  do {
    camera_list.Clear();
    camera_list = system->GetCameras();
    if (CameraPtr camera = TryFindCameraForConfig(camera_list, config)) {
      return camera;
    }
    std::this_thread::sleep_for(poll_period);
  } while (std::chrono::steady_clock::now() < deadline);

  camera_list.Clear();
  camera_list = system->GetCameras();
  return TryFindCameraForConfig(camera_list, config);
}

bool ExecuteCameraForceIp(
  CameraPtr camera,
  std::uint32_t target_address,
  std::uint32_t target_subnet_mask,
  std::uint32_t target_gateway)
{
  INodeMap & tl_node_map = camera->GetTLDeviceNodeMap();
  return SetIntegerByName(tl_node_map, "GevDeviceForceIPAddress", target_address) &&
         SetIntegerByName(tl_node_map, "GevDeviceForceSubnetMask", target_subnet_mask) &&
         SetIntegerByName(tl_node_map, "GevDeviceForceGateway", target_gateway) &&
         ExecuteCommandByName(tl_node_map, "GevDeviceForceIP");
}

bool ExecuteInterfaceForceIpForSerial(
  SystemPtr system,
  const std::string & serial,
  std::uint32_t target_address,
  std::uint32_t target_subnet_mask,
  std::uint32_t target_gateway)
{
  InterfaceList interface_list = system->GetInterfaces();
  bool executed = false;

  try {
    for (std::size_t interface_index = 0; interface_index < interface_list.GetSize(); ++interface_index) {
      InterfacePtr gev_interface = interface_list.GetByIndex(static_cast<unsigned int>(interface_index));
      INodeMap & interface_node_map = gev_interface->GetTLNodeMap();
      CIntegerPtr selector = interface_node_map.GetNode("DeviceSelector");
      if (!IsReadable(selector) || !IsWritable(selector)) {
        continue;
      }

      const std::int64_t original_selector = selector->GetValue();
      const std::int64_t min_selector = selector->GetMin();
      const std::int64_t max_selector = selector->GetMax();
      try {
        for (std::int64_t selector_index = max_selector; selector_index >= min_selector; --selector_index) {
          selector->SetValue(selector_index);
          if (SafeNodeString(interface_node_map, "DeviceSerialNumber") != serial) {
            continue;
          }

          if (!SetIntegerByName(interface_node_map, "GevDeviceForceIPAddress", target_address) ||
            !SetIntegerByName(interface_node_map, "GevDeviceForceSubnetMask", target_subnet_mask) ||
            !SetIntegerByName(interface_node_map, "GevDeviceForceGateway", target_gateway) ||
            !ExecuteCommandByName(interface_node_map, "GevDeviceForceIP"))
          {
            throw std::runtime_error("Failed to execute interface ForceIP for serial '" + serial + "'.");
          }

          ROS_INFO("Interface ForceIP command executed for serial '%s' on interface index %zu.",
            serial.c_str(), interface_index);
          executed = true;
          break;
        }
        selector->SetValue(original_selector);
      } catch (...) {
        try {
          selector->SetValue(original_selector);
        } catch (...) {
        }
        throw;
      }

      if (executed) {
        break;
      }
    }

    interface_list.Clear();
  } catch (...) {
    interface_list.Clear();
    throw;
  }

  return executed;
}

CameraPtr WaitForCameraAtTargetIp(
  SystemPtr system,
  CameraList & camera_list,
  const CameraConfig & config,
  const std::string & serial,
  std::uint32_t target_address,
  int timeout_ms)
{
  const auto poll_period = std::chrono::milliseconds(250);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  std::string last_state = "not detected";

  do {
    camera_list.Clear();
    camera_list = system->GetCameras();
    CameraPtr camera = serial.empty() ?
      TryFindCameraForConfig(camera_list, config) :
      TryFindCameraBySerial(camera_list, serial);

    if (camera) {
      INodeMap & tl_node_map = camera->GetTLDeviceNodeMap();
      const auto current_address = ReadIntegerByName(tl_node_map, "GevDeviceIPAddress");
      const auto wrong_subnet = ReadBooleanByName(tl_node_map, "GevDeviceIsWrongSubnet");
      const bool has_target_ip = current_address.has_value() && *current_address >= 0 &&
                                 static_cast<std::uint32_t>(*current_address) == target_address;
      const bool is_wrong_subnet = wrong_subnet.value_or(false);
      last_state = "ip=" +
        (current_address.has_value() && *current_address >= 0 ?
        FormatIpv4Address(static_cast<std::uint32_t>(*current_address)) : std::string("unknown")) +
        " wrong_subnet=" + (wrong_subnet.has_value() ? (is_wrong_subnet ? "true" : "false") : "unknown");
      if (has_target_ip && !is_wrong_subnet) {
        return camera;
      }
    }
    std::this_thread::sleep_for(poll_period);
  } while (std::chrono::steady_clock::now() < deadline);

  throw std::runtime_error(
    "Camera '" + config.name + "' did not settle at target IP " +
    FormatIpv4Address(target_address) + " before timeout; last_state=" + last_state + ".");
}

void ApplyForceIpForConfig(
  SystemPtr system,
  CameraList & camera_list,
  const CameraConfig & config)
{
  if (!config.force_ip_enable || config.force_ip_address.empty()) {
    return;
  }
  if (config.force_ip_wait_after_ms < 0) {
    throw std::runtime_error("force_ip_wait_after_ms must be non-negative.");
  }
  if (config.force_ip_rediscovery_timeout_ms < 0) {
    throw std::runtime_error("force_ip_rediscovery_timeout_ms must be non-negative.");
  }
  if (config.force_ip_max_attempts <= 0) {
    throw std::runtime_error("force_ip_max_attempts must be positive.");
  }

  const std::uint32_t target_address = ParseIpv4Address(config.force_ip_address, "force_ip_address");
  const std::uint32_t target_subnet_mask = ParseIpv4Address(config.force_ip_subnet_mask, "force_ip_subnet_mask");
  const std::uint32_t target_gateway = ParseIpv4Address(config.force_ip_gateway, "force_ip_gateway");

  bool verified = false;
  std::string last_error;
  for (int attempt = 1; attempt <= config.force_ip_max_attempts; ++attempt) {
    CameraPtr camera = WaitForCameraForConfig(
      system,
      camera_list,
      config,
      config.force_ip_rediscovery_timeout_ms);
    if (!camera) {
      last_error = "Could not find camera '" + config.name + "' while applying ForceIP.";
      continue;
    }

    INodeMap & tl_node_map = camera->GetTLDeviceNodeMap();
    const std::string serial = config.serial.empty() ?
      SafeNodeString(tl_node_map, "DeviceSerialNumber") :
      config.serial;
    const auto current_address = ReadIntegerByName(tl_node_map, "GevDeviceIPAddress");
    const auto wrong_subnet = ReadBooleanByName(tl_node_map, "GevDeviceIsWrongSubnet");

    if (current_address.has_value() && *current_address >= 0 &&
      static_cast<std::uint32_t>(*current_address) == target_address &&
      !wrong_subnet.value_or(false))
    {
      ROS_INFO("[%s] ForceIP already correct: serial='%s' ip=%s.",
        config.name.c_str(),
        serial.empty() ? "<unknown>" : serial.c_str(),
        FormatIpv4Address(target_address).c_str());
      verified = true;
      break;
    }

    ROS_WARN(
      "[%s] Applying ForceIP attempt %d/%d: serial='%s' current_ip=%s wrong_subnet=%s target=%s/%s gateway=%s.",
      config.name.c_str(),
      attempt,
      config.force_ip_max_attempts,
      serial.empty() ? "<unknown>" : serial.c_str(),
      current_address.has_value() && *current_address >= 0 ?
      FormatIpv4Address(static_cast<std::uint32_t>(*current_address)).c_str() : "unknown",
      wrong_subnet.has_value() ? (*wrong_subnet ? "true" : "false") : "unknown",
      FormatIpv4Address(target_address).c_str(),
      FormatIpv4Address(target_subnet_mask).c_str(),
      FormatIpv4Address(target_gateway).c_str());

    bool executed = false;
    if (!serial.empty()) {
      try {
        executed = ExecuteInterfaceForceIpForSerial(
          system,
          serial,
          target_address,
          target_subnet_mask,
          target_gateway);
      } catch (const std::exception & exception) {
        last_error = exception.what();
        ROS_WARN("[%s] Interface ForceIP failed for serial '%s': %s",
          config.name.c_str(), serial.c_str(), exception.what());
      }
    }

    if (!executed) {
      try {
        executed = ExecuteCameraForceIp(camera, target_address, target_subnet_mask, target_gateway);
      } catch (const std::exception & exception) {
        last_error = exception.what();
      }
    }

    camera = nullptr;
    camera_list.Clear();

    if (!executed) {
      if (last_error.empty()) {
        last_error = "Failed to execute ForceIP for camera '" + config.name + "'.";
      }
      continue;
    }

    if (config.force_ip_wait_after_ms > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(config.force_ip_wait_after_ms));
    }

    try {
      (void)WaitForCameraAtTargetIp(
        system,
        camera_list,
        config,
        serial,
        target_address,
        config.force_ip_rediscovery_timeout_ms);
      ROS_INFO("[%s] ForceIP verified: serial='%s' ip=%s.",
        config.name.c_str(),
        serial.empty() ? "<unknown>" : serial.c_str(),
        FormatIpv4Address(target_address).c_str());
      verified = true;
      break;
    } catch (const std::exception & exception) {
      last_error = exception.what();
      ROS_WARN("[%s] ForceIP verification failed attempt %d/%d: %s",
        config.name.c_str(), attempt, config.force_ip_max_attempts, exception.what());
    }
  }

  if (!verified) {
    throw std::runtime_error(
      last_error.empty() ? "ForceIP did not verify for camera '" + config.name + "'." : last_error);
  }
}

void ApplyForceIpForConfigs(
  SystemPtr system,
  CameraList & camera_list,
  const std::vector<CameraConfig> & configs)
{
  for (const CameraConfig & config : configs) {
    ApplyForceIpForConfig(system, camera_list, config);
  }
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
    if (config_.serial.empty()) {
      config_.serial = SafeNodeString(camera_->GetTLDeviceNodeMap(), "DeviceSerialNumber");
      ApplyCameraInfoYaml(config_);
    }
    ConfigureCamera();
    Advertise();
    ROS_INFO(
      "[%s] Advertised topics: raw=%s camera_info=%s metadata=%s rgb_compressed=%s.",
      config_.name.c_str(),
      config_.publish_raw ? "true" : "false",
      config_.publish_camera_info ? "true" : "false",
      config_.publish_metadata ? "true" : "false",
      config_.publish_rgb_compressed ? "true" : "false");
    ROS_INFO("[%s] Starting acquisition.", config_.name.c_str());
    camera_->BeginAcquisition();
    running_.store(true);
    acquisition_thread_ = std::thread(&CameraStreamer::AcquisitionLoop, this);
    ROS_INFO("[%s] Acquisition thread started.", config_.name.c_str());
  }

  void StartPtpActionSender()
  {
    if (!config_.ptp_action_enable || config_.ptp_action_role != "sender") {
      return;
    }
    ptp_action_thread_ = std::thread(&CameraStreamer::PtpActionSenderLoop, this);
  }

  void Stop()
  {
    running_.store(false);
    if (ptp_action_thread_.joinable()) {
      ptp_action_thread_.join();
    }
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
	  bool SetCameraControlService(
	    flir_spinnaker_camera::SetCameraControl::Request & request,
	    flir_spinnaker_camera::SetCameraControl::Response & response)
	  {
	    try {
	      std::lock_guard<std::mutex> lock(control_mutex_);
	      if (!camera_) {
	        response.success = false;
	        response.message = "Camera is not initialized.";
	        return true;
	      }

	      std::string scope = NormalizeName(request.scope);
	      if (scope.empty()) {
	        scope = "camera";
	      }
	      std::string node_name = TrimAscii(request.name);
	      std::string value = StripMatchingQuotes(TrimAscii(request.value));

	      INodeMap * node_map = nullptr;
	      if (scope == "camera" || scope == "node" || scope == "genicam") {
	        node_map = &camera_->GetNodeMap();
	      } else if (scope == "stream" || scope == "tlstream") {
	        node_map = &camera_->GetTLStreamNodeMap();
	      } else if (scope == "tldevice" || scope == "device") {
	        node_map = &camera_->GetTLDeviceNodeMap();
	      } else {
	        response.success = false;
	        response.message = "Unknown scope '" + request.scope + "'. Use camera, stream, or tl_device.";
	        return true;
	      }

	      if (node_name.empty()) {
	        response.success = false;
	        response.message = "Control name is empty.";
	        return true;
	      }

	      const std::string normalized_value = NormalizeName(value);
	      bool applied = false;
	      std::string applied_type;

	      if (normalized_value == "execute" || normalized_value == "run") {
	        applied = ExecuteCommandByName(*node_map, node_name.c_str());
	        applied_type = "command";
	      }

	      if (!applied &&
	        (normalized_value == "true" || normalized_value == "false" ||
	        normalized_value == "on" || normalized_value == "off" ||
	        normalized_value == "yes" || normalized_value == "no"))
	      {
	        const bool bool_value =
	          normalized_value == "true" || normalized_value == "on" || normalized_value == "yes";
	        applied = SetBooleanByName(*node_map, node_name.c_str(), bool_value);
	        applied_type = "bool";
	      }

	      if (!applied) {
	        try {
	          std::size_t consumed = 0U;
	          const double numeric_value = std::stod(value, &consumed);
	          if (consumed == value.size()) {
	            applied = SetFloatByName(*node_map, node_name.c_str(), numeric_value);
	            applied_type = "float";
	            if (!applied && std::isfinite(numeric_value)) {
	              const double rounded = std::round(numeric_value);
	              if (std::abs(numeric_value - rounded) < 1e-9 &&
	                rounded >= static_cast<double>(std::numeric_limits<std::int64_t>::min()) &&
	                rounded <= static_cast<double>(std::numeric_limits<std::int64_t>::max()))
	              {
	                applied = SetIntegerByName(
	                  *node_map,
	                  node_name.c_str(),
	                  static_cast<std::int64_t>(rounded));
	                applied_type = "integer";
	              }
	            }
	          }
	        } catch (...) {
	        }
	      }

	      if (!applied) {
	        applied = SetEnumerationByName(*node_map, node_name.c_str(), value);
	        applied_type = "enum";
	      }
	      if (!applied) {
	        applied = SetStringByName(*node_map, node_name.c_str(), value);
	        applied_type = "string";
	      }

	      response.success = applied;
	      response.applied_type = applied ? applied_type : "";
	      response.current_value = ReadNodeValueText(*node_map, node_name);
	      response.message = applied ?
	        "Set " + scope + "." + node_name + " to '" + value + "'." :
	        "Control '" + scope + "." + node_name + "' is not writable or value '" + value + "' is invalid.";

	      if (applied) {
	        ROS_INFO(
	          "[%s] Runtime control: %s.%s='%s' applied as %s; current='%s'.",
	          config_.name.c_str(),
	          scope.c_str(),
	          node_name.c_str(),
	          value.c_str(),
	          applied_type.c_str(),
	          response.current_value.c_str());
	      } else {
	        ROS_WARN(
	          "[%s] Runtime control failed: %s",
	          config_.name.c_str(),
	          response.message.c_str());
	      }
	    } catch (const std::exception & exception) {
	      response.success = false;
	      response.message = exception.what();
	    }
	    return true;
	  }

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
    INodeMap & tl_node_map = camera_->GetTLDeviceNodeMap();

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

    ApplyStartupControls(node_map, config_.camera_controls, "camera");
    ApplyStartupControls(stream_node_map, config_.stream_controls, "stream");
    ApplyStartupControls(tl_node_map, config_.tl_device_controls, "tl_device");
    SetEnumerationByName(node_map, "AcquisitionMode", "Continuous");
    ApplyPtpConfiguration(node_map);
    ApplyHardwareTriggerConfiguration(node_map);
    ApplyPtpActionConfiguration(node_map);
    LogGigETransportSettings(node_map);

    ROS_INFO(
      "[%s] Using FLIR camera model='%s' serial='%s'.",
      config_.name.c_str(),
      SafeNodeString(tl_node_map, "DeviceModelName").c_str(),
      SafeNodeString(tl_node_map, "DeviceSerialNumber").c_str());
  }

  void LogGigETransportSettings(INodeMap & node_map)
  {
    const auto packet_size = ReadIntegerByName(node_map, "GevSCPSPacketSize");
    const auto packet_delay = ReadIntegerByName(node_map, "GevSCPD");
    const auto frame_delay = ReadIntegerByName(node_map, "GevSCFTD");
    const auto throughput_limit = ReadIntegerByName(node_map, "DeviceLinkThroughputLimit");
    if (packet_size || packet_delay || frame_delay || throughput_limit) {
      const std::string packet_size_text = packet_size ? std::to_string(*packet_size) : "n/a";
      const std::string packet_delay_text = packet_delay ? std::to_string(*packet_delay) : "n/a";
      const std::string frame_delay_text = frame_delay ? std::to_string(*frame_delay) : "n/a";
      const std::string throughput_limit_text =
        throughput_limit ? std::to_string(*throughput_limit) : "n/a";
      ROS_INFO(
        "[%s] GigE transport: GevSCPSPacketSize=%s GevSCPD=%s GevSCFTD=%s DeviceLinkThroughputLimit=%s.",
        config_.name.c_str(),
        packet_size_text.c_str(),
        packet_delay_text.c_str(),
        frame_delay_text.c_str(),
        throughput_limit_text.c_str());
    }
  }

  void ApplyStartupControls(INodeMap & node_map, const ControlMaps & controls, const char * prefix)
  {
    for (const auto & item : controls.bool_controls) {
      bool applied = false;
      try {
        applied = SetBooleanByName(node_map, item.first.c_str(), item.second);
      } catch (const std::exception & exception) {
        ROS_WARN("[%s] Could not set %s.%s: %s", config_.name.c_str(), prefix, item.first.c_str(), exception.what());
        continue;
      }
      if (!applied) {
        ROS_WARN("[%s] Could not set %s.%s.", config_.name.c_str(), prefix, item.first.c_str());
      }
    }

    for (const auto & item : controls.numeric_controls) {
      bool applied = false;
      try {
        applied = SetFloatByName(node_map, item.first.c_str(), item.second) ||
                  SetIntegerByName(node_map, item.first.c_str(), static_cast<std::int64_t>(item.second));
      } catch (const std::exception & exception) {
        ROS_WARN("[%s] Could not set %s.%s: %s", config_.name.c_str(), prefix, item.first.c_str(), exception.what());
        continue;
      }
      if (!applied) {
        ROS_WARN("[%s] Could not set %s.%s.", config_.name.c_str(), prefix, item.first.c_str());
      }
    }

    for (const auto & item : controls.enum_controls) {
      bool applied = false;
      try {
        applied = SetEnumerationByName(node_map, item.first.c_str(), item.second) ||
                  SetStringByName(node_map, item.first.c_str(), item.second);
      } catch (const std::exception & exception) {
        ROS_WARN(
          "[%s] Could not set %s.%s to '%s': %s",
          config_.name.c_str(),
          prefix,
          item.first.c_str(),
          item.second.c_str(),
          exception.what());
        continue;
      }
      if (!applied) {
        ROS_WARN(
          "[%s] Could not set %s.%s to '%s'.",
          config_.name.c_str(),
          prefix,
          item.first.c_str(),
          item.second.c_str());
      }
    }
  }

  void RequireEnumeration(
    INodeMap & node_map,
    const char * node_name,
    const std::string & entry_name,
    const std::string & context)
  {
    if (!SetEnumerationByName(node_map, node_name, entry_name)) {
      throw std::runtime_error(context + ": failed to set " + node_name + " to '" + entry_name + "'.");
    }
    ROS_INFO("[%s] %s: %s='%s'", config_.name.c_str(), context.c_str(), node_name, entry_name.c_str());
  }

  bool TryEnumeration(
    INodeMap & node_map,
    const char * node_name,
    const std::string & entry_name,
    const std::string & context)
  {
    if (!SetEnumerationByName(node_map, node_name, entry_name)) {
      return false;
    }
    ROS_INFO("[%s] %s: %s='%s'", config_.name.c_str(), context.c_str(), node_name, entry_name.c_str());
    return true;
  }

  void RequireInteger(
    INodeMap & node_map,
    const char * node_name,
    std::uint32_t value,
    const std::string & context)
  {
    if (!SetIntegerByName(node_map, node_name, static_cast<std::int64_t>(value))) {
      throw std::runtime_error(context + ": failed to set " + node_name + ".");
    }
    ROS_INFO("[%s] %s: %s=%u", config_.name.c_str(), context.c_str(), node_name, value);
  }

  bool PtpStatusIsAccepted(const std::string & status) const
  {
    const std::string normalized_status = NormalizeName(status);
    return std::any_of(
      config_.ptp_accepted_statuses.begin(),
      config_.ptp_accepted_statuses.end(),
      [&](const std::string & accepted_status) {
        return NormalizeName(accepted_status) == normalized_status;
      });
  }

  void WaitForPtpSync(INodeMap & node_map)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(config_.ptp_sync_timeout_ms);
    std::string last_status = "unknown";

    while (ros::ok()) {
      ExecuteCommandByName(node_map, "GevIEEE1588DataSetLatch");
      if (const auto status = ReadEnumerationByName(node_map, "GevIEEE1588Status")) {
        last_status = *status;
        if (PtpStatusIsAccepted(last_status)) {
          const auto offset_ns = ReadIntegerByName(node_map, "GevIEEE1588OffsetFromMasterLatched");
          if (offset_ns.has_value()) {
            ROS_INFO(
              "[%s] PTP synchronized: GevIEEE1588Status='%s', offset_from_master=%ld ns.",
              config_.name.c_str(),
              last_status.c_str(),
              static_cast<long>(*offset_ns));
          } else {
            ROS_INFO("[%s] PTP synchronized: GevIEEE1588Status='%s'.", config_.name.c_str(), last_status.c_str());
          }
          return;
        }
      }

      if (std::chrono::steady_clock::now() >= deadline) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(std::max(1, config_.ptp_sync_poll_ms)));
    }

    std::ostringstream message;
    message << "PTP did not reach accepted status before timeout. Last GevIEEE1588Status='"
            << last_status << "'.";
    if (config_.ptp_require_sync) {
      throw std::runtime_error(message.str());
    }
    ROS_WARN("[%s] %s Continuing because ptp.require_sync=false.", config_.name.c_str(), message.str().c_str());
  }

  void ApplyPtpConfiguration(INodeMap & node_map)
  {
    if (!config_.ptp_enabled) {
      return;
    }

    const std::string context = "PTP";
    if (!SetBooleanByName(node_map, "GevIEEE1588", true)) {
      throw std::runtime_error(context + ": failed to enable GevIEEE1588.");
    }
    ROS_INFO("[%s] %s: GevIEEE1588=true", config_.name.c_str(), context.c_str());

    if (!config_.ptp_mode.empty()) {
      if (!TryEnumeration(node_map, "GevIEEE1588Mode", config_.ptp_mode, context)) {
        ROS_WARN(
          "[%s] %s: GevIEEE1588Mode='%s' is not writable/available. Continuing with camera default.",
          config_.name.c_str(),
          context.c_str(),
          config_.ptp_mode.c_str());
      }
    }

    if (config_.ptp_wait_for_sync) {
      WaitForPtpSync(node_map);
    }
  }

  void ApplyHardwareTriggerConfiguration(INodeMap & node_map)
  {
    if (config_.hardware_trigger_role == "none") {
      return;
    }
    if (config_.hardware_trigger_role == "master") {
      ApplyBfsMasterHardwareTrigger(node_map);
      return;
    }
    if (config_.hardware_trigger_role == "slave") {
      ApplyBfsSlaveHardwareTrigger(node_map);
      return;
    }
    throw std::runtime_error("Unsupported hardware_trigger.role: " + config_.hardware_trigger_role);
  }

  void ApplyBfsMasterHardwareTrigger(INodeMap & node_map)
  {
    const std::string context = "BFS hardware trigger master";
    RequireEnumeration(node_map, "AcquisitionMode", "Continuous", context);
    RequireEnumeration(node_map, "TriggerSelector", "FrameStart", context);
    RequireEnumeration(node_map, "TriggerMode", "Off", context);
    RequireEnumeration(node_map, "LineSelector", config_.hardware_trigger_master_output_line, context);
    RequireEnumeration(node_map, "LineMode", "Output", context);

    std::vector<std::string> line_source_candidates;
    if (!config_.hardware_trigger_master_line_source.empty()) {
      line_source_candidates.push_back(config_.hardware_trigger_master_line_source);
    }
    for (const auto & fallback : config_.hardware_trigger_master_line_source_fallbacks) {
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
      throw std::runtime_error(context + ": failed to set LineSource.");
    }

    if (config_.hardware_trigger_master_enable_3v3) {
      EnableBfsMasterLine3v3(node_map, context);
    }
    ROS_INFO(
      "[%s] BFS hardware trigger master ready: output_line='%s', line_source='%s'.",
      config_.name.c_str(),
      config_.hardware_trigger_master_output_line.c_str(),
      applied_line_source.c_str());
  }

  void EnableBfsMasterLine3v3(INodeMap & node_map, const std::string & context)
  {
    RequireEnumeration(node_map, "LineSelector", config_.hardware_trigger_master_3v3_line, context);
    for (const auto & node_name : config_.hardware_trigger_master_3v3_enable_nodes) {
      if (!node_name.empty() && SetBooleanByName(node_map, node_name.c_str(), true)) {
        ROS_INFO(
          "[%s] %s: enabled 3.3V on %s via %s.",
          config_.name.c_str(),
          context.c_str(),
          config_.hardware_trigger_master_3v3_line.c_str(),
          node_name.c_str());
        return;
      }
    }

    const std::string message =
      context + ": failed to enable 3.3V on " + config_.hardware_trigger_master_3v3_line + ".";
    if (config_.hardware_trigger_master_require_3v3) {
      throw std::runtime_error(message);
    }
    ROS_WARN("[%s] %s Continuing because require_3v3=false.", config_.name.c_str(), message.c_str());
  }

  void ApplyBfsSlaveHardwareTrigger(INodeMap & node_map)
  {
    const std::string context = "BFS hardware trigger slave";
    RequireEnumeration(node_map, "AcquisitionMode", "Continuous", context);
    RequireEnumeration(node_map, "TriggerSelector", "FrameStart", context);
    RequireEnumeration(node_map, "TriggerMode", "Off", context);
    if (config_.hardware_trigger_slave_trigger_source.rfind("Line", 0) == 0) {
      RequireEnumeration(node_map, "LineSelector", config_.hardware_trigger_slave_trigger_source, context);
      RequireEnumeration(node_map, "LineMode", "Input", context);
    }
    RequireEnumeration(node_map, "TriggerSource", config_.hardware_trigger_slave_trigger_source, context);
    if (!config_.hardware_trigger_slave_trigger_activation.empty()) {
      RequireEnumeration(node_map, "TriggerActivation", config_.hardware_trigger_slave_trigger_activation, context);
    }
    if (!config_.hardware_trigger_slave_trigger_overlap.empty()) {
      RequireEnumeration(node_map, "TriggerOverlap", config_.hardware_trigger_slave_trigger_overlap, context);
    }
    RequireEnumeration(node_map, "TriggerMode", "On", context);
    ROS_INFO(
      "[%s] BFS hardware trigger slave ready: trigger_source='%s'.",
      config_.name.c_str(),
      config_.hardware_trigger_slave_trigger_source.c_str());
  }

  void ConfigureActionSelector(INodeMap & node_map, const std::string & context)
  {
    if (config_.ptp_action_selector.empty()) {
      ROS_WARN("[%s] %s: ptp_action.selector is empty; leaving ActionSelector unchanged.",
        config_.name.c_str(), context.c_str());
      return;
    }
    if (TryEnumeration(node_map, "ActionSelector", config_.ptp_action_selector, context)) {
      return;
    }
    if (const auto current_action = ReadEnumerationByName(node_map, "ActionSelector")) {
      ROS_WARN(
        "[%s] %s: ActionSelector could not be set to '%s'; using current camera selection '%s'.",
        config_.name.c_str(),
        context.c_str(),
        config_.ptp_action_selector.c_str(),
        current_action->c_str());
      return;
    }
    ROS_WARN(
      "[%s] %s: ActionSelector is not writable/readable; assuming a single action signal.",
      config_.name.c_str(),
      context.c_str());
  }

  void ApplyPtpActionConfiguration(INodeMap & node_map)
  {
    if (!config_.ptp_action_enable || config_.ptp_action_role == "none") {
      return;
    }

    const std::string context = "PTP action trigger " + config_.ptp_action_role;
    RequireEnumeration(node_map, "AcquisitionMode", "Continuous", context);
    ConfigureActionSelector(node_map, context);
    RequireInteger(node_map, "ActionDeviceKey", config_.ptp_action_device_key, context);
    RequireInteger(node_map, "ActionGroupKey", config_.ptp_action_group_key, context);
    RequireInteger(node_map, "ActionGroupMask", config_.ptp_action_group_mask, context);
    TryEnumeration(node_map, "ActionUnconditionalMode", "On", context);

    RequireEnumeration(node_map, "TriggerSelector", config_.ptp_action_trigger_selector, context);
    RequireEnumeration(node_map, "TriggerMode", "Off", context);
    RequireEnumeration(node_map, "TriggerSource", config_.ptp_action_trigger_source, context);
    if (!config_.ptp_action_trigger_activation.empty()) {
      if (!TryEnumeration(node_map, "TriggerActivation", config_.ptp_action_trigger_activation, context)) {
        ROS_WARN("[%s] %s: TriggerActivation='%s' is not writable; using camera default.",
          config_.name.c_str(), context.c_str(), config_.ptp_action_trigger_activation.c_str());
      }
    }
    if (!config_.ptp_action_trigger_overlap.empty()) {
      if (!TryEnumeration(node_map, "TriggerOverlap", config_.ptp_action_trigger_overlap, context)) {
        ROS_WARN("[%s] %s: TriggerOverlap='%s' is not writable; using camera default.",
          config_.name.c_str(), context.c_str(), config_.ptp_action_trigger_overlap.c_str());
      }
    }
    RequireEnumeration(node_map, "TriggerMode", "On", context);
    ROS_INFO("[%s] PTP action trigger receiver armed: trigger_source='%s'.",
      config_.name.c_str(), config_.ptp_action_trigger_source.c_str());
  }

  std::uint64_t ReadTimestampTickFrequency()
  {
    INodeMap & node_map = camera_->GetNodeMap();
    if (const auto frequency = ReadIntegerByName(node_map, "GevTimestampTickFrequency")) {
      if (*frequency > 0) {
        return static_cast<std::uint64_t>(*frequency);
      }
    }
    ROS_WARN("[%s] Could not read GevTimestampTickFrequency. Assuming 1 GHz.", config_.name.c_str());
    return 1000000000ULL;
  }

  std::uint64_t MillisecondsToTimestampTicks(double milliseconds, std::uint64_t tick_frequency) const
  {
    const long double ticks =
      (static_cast<long double>(milliseconds) * static_cast<long double>(tick_frequency)) / 1000.0L;
    if (ticks <= 0.0L) {
      return 1ULL;
    }
    const long double max_ticks = static_cast<long double>(std::numeric_limits<std::uint64_t>::max());
    if (ticks >= max_ticks) {
      return std::numeric_limits<std::uint64_t>::max();
    }
    return static_cast<std::uint64_t>(std::llround(ticks));
  }

  std::uint64_t ReadCameraTimestampTicks()
  {
    INodeMap & node_map = camera_->GetNodeMap();
    if (const auto timestamp = ReadIntegerByName(node_map, "Timestamp")) {
      if (*timestamp >= 0) {
        return static_cast<std::uint64_t>(*timestamp);
      }
    }
    if (!ExecuteCommandByName(node_map, "TimestampLatch")) {
      ROS_DEBUG("[%s] TimestampLatch is not available/writable.", config_.name.c_str());
    }
    if (const auto timestamp = ReadIntegerByName(node_map, "TimestampLatchValue")) {
      if (*timestamp >= 0) {
        return static_cast<std::uint64_t>(*timestamp);
      }
    }
    throw std::runtime_error("Failed to read camera timestamp for scheduled PTP action command.");
  }

  std::uint64_t EstimateCameraTimestampTicks(
    std::uint64_t origin_ticks,
    const std::chrono::steady_clock::time_point & origin_host_time,
    std::uint64_t tick_frequency,
    const std::chrono::steady_clock::time_point & now_host_time) const
  {
    const long double elapsed_sec =
      std::chrono::duration<long double>(now_host_time - origin_host_time).count();
    if (elapsed_sec <= 0.0L) {
      return origin_ticks;
    }

    const long double elapsed_ticks =
      elapsed_sec * static_cast<long double>(tick_frequency);
    const long double max_ticks = static_cast<long double>(std::numeric_limits<std::uint64_t>::max());
    if (elapsed_ticks >= max_ticks - static_cast<long double>(origin_ticks)) {
      return std::numeric_limits<std::uint64_t>::max();
    }
    return origin_ticks + static_cast<std::uint64_t>(elapsed_ticks + 0.5L);
  }

  void SendScheduledActionCommand(std::uint64_t action_time)
  {
    if (config_.ptp_action_request_ack && config_.ptp_action_expected_ack_count > 0) {
      unsigned int result_count = static_cast<unsigned int>(config_.ptp_action_expected_ack_count);
      const unsigned int expected_result_count = result_count;
      std::vector<Spinnaker::ActionCommandResult> results(result_count);
      system_->SendActionCommand(
        config_.ptp_action_device_key,
        config_.ptp_action_group_key,
        config_.ptp_action_group_mask,
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
      config_.ptp_action_device_key,
      config_.ptp_action_group_key,
      config_.ptp_action_group_mask,
      action_time,
      false,
      nullptr,
      nullptr);
  }

  double ResolvePtpActionRateHz()
  {
    if (config_.ptp_action_rate_hz > 0.0) {
      return config_.ptp_action_rate_hz;
    }

    INodeMap & node_map = camera_->GetNodeMap();
    for (const char * node_name : {"AcquisitionResultingFrameRate", "ResultingFrameRate", "AcquisitionFrameRate"}) {
      if (const auto value = ReadFloatByName(node_map, node_name)) {
        if (*value > 0.0) {
          const double selected = *value * 0.95;
          ROS_WARN(
            "[%s] ptp_action.rate_hz<=0; using %.3f Hz from %s with 0.95 safety margin.",
            config_.name.c_str(),
            selected,
            node_name);
          return selected;
        }
      }
    }

    throw std::runtime_error(
      "ptp_action.rate_hz<=0 but no readable resulting/acquisition frame-rate node was available.");
  }

  void PtpActionSenderLoop()
  {
    try {
      std::this_thread::sleep_for(
        std::chrono::duration<double, std::milli>(config_.ptp_action_start_delay_ms));

      const double action_rate_hz = ResolvePtpActionRateHz();

      const std::uint64_t tick_frequency = ReadTimestampTickFrequency();
      const auto timestamp_origin_host_time = std::chrono::steady_clock::now();
      const std::uint64_t timestamp_origin_ticks = ReadCameraTimestampTicks();
      const std::uint64_t schedule_ahead_ticks =
        MillisecondsToTimestampTicks(config_.ptp_action_schedule_ahead_ms, tick_frequency);
      const std::uint64_t period_ticks =
        MillisecondsToTimestampTicks(1000.0 / action_rate_hz, tick_frequency);
      const auto host_period =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(1.0 / action_rate_hz));
      const auto log_interval =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(config_.ptp_action_log_interval_sec));

      std::uint64_t next_action_time = 0U;
      std::uint64_t sent_count = 0U;
      auto next_send_time = std::chrono::steady_clock::now();
      auto next_log_time = std::chrono::steady_clock::now();

      ROS_INFO("[%s] PTP action sender started: rate=%.3f Hz, schedule_ahead=%.3f ms.",
        config_.name.c_str(), action_rate_hz, config_.ptp_action_schedule_ahead_ms);

      while (ros::ok() && running_.load()) {
        try {
          const auto now_time = std::chrono::steady_clock::now();
          const std::uint64_t camera_now = EstimateCameraTimestampTicks(
            timestamp_origin_ticks,
            timestamp_origin_host_time,
            tick_frequency,
            now_time);
          const std::uint64_t earliest_action_time =
            (std::numeric_limits<std::uint64_t>::max() - camera_now < schedule_ahead_ticks) ?
            std::numeric_limits<std::uint64_t>::max() :
            camera_now + schedule_ahead_ticks;

          if (next_action_time < earliest_action_time) {
            next_action_time = earliest_action_time;
          }

          SendScheduledActionCommand(next_action_time);
          ++sent_count;

          if (config_.ptp_action_log_interval_sec > 0.0 && now_time >= next_log_time) {
            ROS_INFO(
              "[%s] PTP action sender scheduled %lu commands; next_action_time=%lu ticks.",
              config_.name.c_str(),
              static_cast<unsigned long>(sent_count),
              static_cast<unsigned long>(next_action_time));
            next_log_time = now_time + log_interval;
          }

          next_action_time =
            (std::numeric_limits<std::uint64_t>::max() - next_action_time < period_ticks) ?
            earliest_action_time :
            next_action_time + period_ticks;
        } catch (const Spinnaker::Exception & exception) {
          if (IsFatalSpinnakerException(exception)) {
            ROS_ERROR(
              "[%s] Fatal PTP action sender Spinnaker error; stopping node: %s",
              config_.name.c_str(),
              exception.what());
            running_.store(false);
            g_shutdown_due_to_fatal_error.store(true);
            ros::requestShutdown();
            break;
          }
          ROS_WARN_THROTTLE(
            2.0,
            "[%s] PTP action send failed; keeping sender alive and retrying: %s",
            config_.name.c_str(),
            exception.what());
          next_action_time = 0U;
        } catch (const std::exception & exception) {
          ROS_WARN_THROTTLE(
            2.0,
            "[%s] PTP action send failed; keeping sender alive and retrying: %s",
            config_.name.c_str(),
            exception.what());
          next_action_time = 0U;
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
        ROS_ERROR("[%s] PTP action sender stopped after error: %s", config_.name.c_str(), exception.what());
        running_.store(false);
        g_shutdown_due_to_fatal_error.store(true);
        ros::requestShutdown();
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
	    control_srv_ = camera_nh_.advertiseService(
	      "set_camera_control",
	      &CameraStreamer::SetCameraControlService,
	      this);
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
    while (ros::ok() && running_.load()) {
      ImagePtr image;
      try {
        image = camera_->GetNextImage(config_.acquisition_timeout_ms);
        if (image->IsIncomplete()) {
          const auto status = image->GetImageStatus();
          ROS_WARN_THROTTLE(
            5.0,
            "[%s] Incomplete image received. status=%d (%s)",
            config_.name.c_str(),
            static_cast<int>(status),
            Spinnaker::Image::GetImageStatusDescription(status));
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
        if (IsFatalSpinnakerException(exception)) {
          ROS_ERROR(
            "[%s] Fatal Spinnaker acquisition error; stopping node: %s",
            config_.name.c_str(),
            exception.what());
          running_.store(false);
          g_shutdown_due_to_fatal_error.store(true);
          ros::requestShutdown();
          break;
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
  std::atomic_bool running_{false};
  std::thread acquisition_thread_;
  std::thread ptp_action_thread_;
  ros::Publisher raw_pub_;
	  ros::Publisher camera_info_pub_;
	  ros::Publisher metadata_pub_;
	  ros::Publisher rgb_pub_;
	  ros::ServiceServer control_srv_;
	  std::mutex control_mutex_;
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

    ApplyForceIpForConfigs(system, camera_list, configs);
    camera_list.Clear();
    camera_list = system->GetCameras();
    if (camera_list.GetSize() == 0U) {
      throw std::runtime_error("No FLIR/Spinnaker cameras detected after ForceIP.");
    }

    std::vector<std::unique_ptr<CameraStreamer>> streamers;
    for (const CameraConfig & config : configs) {
      auto streamer = std::make_unique<CameraStreamer>(config, system);
      streamer->Start(camera_list);
      streamers.push_back(std::move(streamer));
    }
    for (auto & streamer : streamers) {
      streamer->StartPtpActionSender();
    }

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();

    for (auto & streamer : streamers) {
      streamer->Stop();
    }
    camera_list.Clear();
    system->ReleaseInstance();
    if (g_shutdown_due_to_fatal_error.load()) {
      return 1;
    }
  } catch (const std::exception & exception) {
    ROS_FATAL("flir_spinnaker_camera failed: %s", exception.what());
    return 1;
  }

  return 0;
}
