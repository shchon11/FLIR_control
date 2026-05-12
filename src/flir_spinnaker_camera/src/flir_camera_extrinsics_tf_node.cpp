#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace
{

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

std::string RemoveYamlComment(const std::string & value)
{
  bool in_single_quote = false;
  bool in_double_quote = false;

  for (std::size_t index = 0; index < value.size(); ++index) {
    const char character = value[index];
    if (character == '\'' && !in_double_quote) {
      in_single_quote = !in_single_quote;
      continue;
    }
    if (character == '"' && !in_single_quote) {
      in_double_quote = !in_double_quote;
      continue;
    }
    if (character == '#' && !in_single_quote && !in_double_quote) {
      return value.substr(0, index);
    }
  }

  return value;
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

std::optional<std::string> MatchYamlScalarValue(const std::string & line, const char * key)
{
  const std::string trimmed = TrimAscii(RemoveYamlComment(line));
  if (trimmed.empty()) {
    return std::nullopt;
  }

  const std::string prefix = std::string(key) + ":";
  if (trimmed.rfind(prefix, 0) != 0) {
    return std::nullopt;
  }

  return TrimAscii(trimmed.substr(prefix.size()));
}

std::optional<std::string> MatchYamlMapKey(const std::string & line)
{
  const std::string trimmed = TrimAscii(RemoveYamlComment(line));
  if (trimmed.empty() || trimmed.back() != ':') {
    return std::nullopt;
  }

  return StripMatchingQuotes(TrimAscii(trimmed.substr(0, trimmed.size() - 1U)));
}

bool IsYamlMapKey(const std::string & line, const char * key)
{
  const auto map_key = MatchYamlMapKey(line);
  return map_key.has_value() && *map_key == key;
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
              std::string("Encountered an empty numeric value while parsing '") +
              field_name + "'.");
    }

    std::size_t consumed = 0U;
    const double parsed_value = std::stod(token, &consumed);
    if (consumed != token.size()) {
      throw std::runtime_error(
              std::string("Failed to fully parse numeric value '") + token +
              "' for '" + field_name + "'.");
    }

    values.push_back(parsed_value);
  }

  return values;
}

template<std::size_t N>
std::array<double, N> ToFixedArray(
  const std::vector<double> & values,
  const char * field_name)
{
  if (values.size() != N) {
    throw std::runtime_error(
            std::string("'") + field_name + "' must contain exactly " +
            std::to_string(N) + " values.");
  }

  std::array<double, N> result{};
  std::copy(values.begin(), values.end(), result.begin());
  return result;
}

struct ExtrinsicTransform
{
  std::string serial;
  std::string camera_name;
  std::string parent_frame;
  std::string child_frame;
  std::array<double, 3> translation_xyz_m{};
  std::array<double, 4> rotation_xyzw{};
  bool has_translation{false};
  bool has_rotation{false};
};

bool ContainsString(const std::vector<std::string> & values, const std::string & target)
{
  return std::find(values.begin(), values.end(), target) != values.end();
}

}  // namespace

class FlirCameraExtrinsicsTfNode : public rclcpp::Node
{
public:
  FlirCameraExtrinsicsTfNode()
  : Node("flir_camera_extrinsics_tf"),
    extrinsics_yaml_path_(declare_parameter<std::string>(
        "extrinsics_yaml_path",
        "calibration/flir_camera_extrinsics.yaml")),
    camera_serials_(declare_parameter<std::vector<std::string>>(
        "camera_serials",
        std::vector<std::string>{})),
    frame_prefix_(declare_parameter<std::string>("frame_prefix", "")),
    normalize_quaternion_(declare_parameter<bool>("normalize_quaternion", true))
  {
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    PublishExtrinsics();
  }

private:
  std::vector<ExtrinsicTransform> LoadExtrinsicsYaml(const std::string & yaml_path) const
  {
    std::ifstream stream(yaml_path);
    if (!stream.is_open()) {
      throw std::runtime_error("Failed to open extrinsics YAML file: " + yaml_path);
    }

    std::vector<std::string> lines;
    std::string line;
    while (std::getline(stream, line)) {
      lines.push_back(line);
    }

    std::string reference_frame;
    std::vector<ExtrinsicTransform> extrinsics;
    std::optional<ExtrinsicTransform> current;
    bool in_extrinsics = false;
    std::size_t extrinsics_indent = 0U;
    std::size_t serial_indent = 0U;

    const auto finish_current = [&]() {
        if (current.has_value()) {
          extrinsics.push_back(*current);
          current.reset();
        }
      };

    for (const std::string & raw_line : lines) {
      const std::string without_comment = RemoveYamlComment(raw_line);
      const std::string trimmed = TrimAscii(without_comment);
      if (trimmed.empty()) {
        continue;
      }

      if (const auto value = MatchYamlScalarValue(raw_line, "reference_frame")) {
        reference_frame = StripMatchingQuotes(*value);
        continue;
      }

      const std::size_t indent = CountLeadingSpaces(without_comment);

      if (!in_extrinsics) {
        if (IsYamlMapKey(raw_line, "extrinsics_by_serial")) {
          in_extrinsics = true;
          extrinsics_indent = indent;
        }
        continue;
      }

      if (indent <= extrinsics_indent && !IsYamlMapKey(raw_line, "extrinsics_by_serial")) {
        break;
      }

      if (indent == extrinsics_indent + 2U) {
        if (const auto serial = MatchYamlMapKey(raw_line)) {
          finish_current();
          current = ExtrinsicTransform{};
          current->serial = *serial;
          serial_indent = indent;
          continue;
        }
      }

      if (!current.has_value() || indent <= serial_indent) {
        continue;
      }

      if (const auto value = MatchYamlScalarValue(raw_line, "camera_name")) {
        current->camera_name = StripMatchingQuotes(*value);
        continue;
      }

      if (const auto value = MatchYamlScalarValue(raw_line, "parent_frame")) {
        current->parent_frame = StripMatchingQuotes(*value);
        continue;
      }

      if (const auto value = MatchYamlScalarValue(raw_line, "child_frame")) {
        current->child_frame = StripMatchingQuotes(*value);
        continue;
      }

      if (const auto value = MatchYamlScalarValue(raw_line, "translation_xyz_m")) {
        current->translation_xyz_m = ToFixedArray<3>(
          ParseYamlDoubleList(*value, "translation_xyz_m"),
          "translation_xyz_m");
        current->has_translation = true;
        continue;
      }

      if (const auto value = MatchYamlScalarValue(raw_line, "rotation_xyzw")) {
        current->rotation_xyzw = ToFixedArray<4>(
          ParseYamlDoubleList(*value, "rotation_xyzw"),
          "rotation_xyzw");
        current->has_rotation = true;
        continue;
      }
    }

    finish_current();

    for (auto & extrinsic : extrinsics) {
      if (extrinsic.parent_frame.empty()) {
        extrinsic.parent_frame = reference_frame;
      }

      ValidateExtrinsic(extrinsic);
      NormalizeQuaternion(extrinsic);
    }

    if (extrinsics.empty()) {
      throw std::runtime_error(
              "Extrinsics YAML did not contain any entries under extrinsics_by_serial: " +
              yaml_path);
    }

    return extrinsics;
  }

  void ValidateExtrinsic(const ExtrinsicTransform & extrinsic) const
  {
    const std::string label = extrinsic.serial.empty() ? extrinsic.camera_name : extrinsic.serial;
    if (extrinsic.parent_frame.empty()) {
      throw std::runtime_error("Extrinsic '" + label + "' is missing parent_frame.");
    }
    if (extrinsic.child_frame.empty()) {
      throw std::runtime_error("Extrinsic '" + label + "' is missing child_frame.");
    }
    if (!extrinsic.has_translation) {
      throw std::runtime_error("Extrinsic '" + label + "' is missing translation_xyz_m.");
    }
    if (!extrinsic.has_rotation) {
      throw std::runtime_error("Extrinsic '" + label + "' is missing rotation_xyzw.");
    }
  }

  void NormalizeQuaternion(ExtrinsicTransform & extrinsic) const
  {
    const double x = extrinsic.rotation_xyzw[0];
    const double y = extrinsic.rotation_xyzw[1];
    const double z = extrinsic.rotation_xyzw[2];
    const double w = extrinsic.rotation_xyzw[3];
    const double norm = std::sqrt(x * x + y * y + z * z + w * w);

    if (norm < 1e-12) {
      const std::string label = extrinsic.serial.empty() ? extrinsic.camera_name : extrinsic.serial;
      throw std::runtime_error("Extrinsic '" + label + "' has a zero-length quaternion.");
    }

    if (!normalize_quaternion_ || std::abs(norm - 1.0) <= 1e-9) {
      return;
    }

    for (double & value : extrinsic.rotation_xyzw) {
      value /= norm;
    }
  }

  std::string FrameName(const std::string & frame) const
  {
    if (frame_prefix_.empty()) {
      return frame;
    }

    return frame_prefix_ + frame;
  }

  bool ShouldPublish(const ExtrinsicTransform & extrinsic) const
  {
    if (camera_serials_.empty()) {
      return true;
    }

    return ContainsString(camera_serials_, extrinsic.serial) ||
           ContainsString(camera_serials_, extrinsic.camera_name) ||
           ContainsString(camera_serials_, extrinsic.child_frame);
  }

  geometry_msgs::msg::TransformStamped BuildTransformMessage(
    const ExtrinsicTransform & extrinsic) const
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = FrameName(extrinsic.parent_frame);
    transform.child_frame_id = FrameName(extrinsic.child_frame);
    transform.transform.translation.x = extrinsic.translation_xyz_m[0];
    transform.transform.translation.y = extrinsic.translation_xyz_m[1];
    transform.transform.translation.z = extrinsic.translation_xyz_m[2];
    transform.transform.rotation.x = extrinsic.rotation_xyzw[0];
    transform.transform.rotation.y = extrinsic.rotation_xyzw[1];
    transform.transform.rotation.z = extrinsic.rotation_xyzw[2];
    transform.transform.rotation.w = extrinsic.rotation_xyzw[3];
    return transform;
  }

  void PublishExtrinsics()
  {
    const std::vector<ExtrinsicTransform> extrinsics = LoadExtrinsicsYaml(extrinsics_yaml_path_);
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.reserve(extrinsics.size());

    for (const auto & extrinsic : extrinsics) {
      if (!ShouldPublish(extrinsic)) {
        continue;
      }

      transforms.push_back(BuildTransformMessage(extrinsic));
    }

    if (transforms.empty()) {
      throw std::runtime_error(
              "No extrinsics matched camera_serials filter in: " + extrinsics_yaml_path_);
    }

    static_broadcaster_->sendTransform(transforms);
    RCLCPP_INFO(
      get_logger(),
      "Published %zu static FLIR camera extrinsic transforms from '%s'.",
      transforms.size(),
      extrinsics_yaml_path_.c_str());
  }

  std::string extrinsics_yaml_path_;
  std::vector<std::string> camera_serials_;
  std::string frame_prefix_;
  bool normalize_quaternion_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  int exit_code = 0;

  try {
    auto node = std::make_shared<FlirCameraExtrinsicsTfNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exception) {
    std::cerr << "flir_camera_extrinsics_tf_node failed: " << exception.what() << std::endl;
    exit_code = 1;
  }

  rclcpp::shutdown();
  return exit_code;
}
