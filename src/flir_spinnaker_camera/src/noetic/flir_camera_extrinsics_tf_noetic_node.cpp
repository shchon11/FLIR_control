#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace
{

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
  if (trimmed.empty() || trimmed.back() != ':') {
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

template<std::size_t Size>
std::array<double, Size> ToFixedArray(
  const std::vector<double> & values,
  const std::string & field_name)
{
  if (values.size() != Size) {
    throw std::runtime_error(field_name + " must contain " + std::to_string(Size) + " values.");
  }
  std::array<double, Size> result{};
  std::copy(values.begin(), values.end(), result.begin());
  return result;
}

bool ContainsString(const std::vector<std::string> & values, const std::string & needle)
{
  return std::find(values.begin(), values.end(), needle) != values.end();
}

struct ExtrinsicTransform
{
  std::string serial;
  std::string camera_name;
  std::string parent_frame;
  std::string child_frame;
  std::array<double, 3> translation_xyz_m{0.0, 0.0, 0.0};
  std::array<double, 4> rotation_xyzw{0.0, 0.0, 0.0, 1.0};
  bool has_translation{false};
  bool has_rotation{false};
};

class ExtrinsicsTfNoetic
{
public:
  ExtrinsicsTfNoetic()
  : private_nh_("~")
  {
    private_nh_.param<std::string>(
      "extrinsics_yaml_path",
      extrinsics_yaml_path_,
      "calibration/flir_camera_extrinsics.yaml");
    private_nh_.param<std::string>("frame_prefix", frame_prefix_, "");
    private_nh_.getParam("camera_serials", camera_serials_);
  }

  void Publish()
  {
    const std::vector<ExtrinsicTransform> extrinsics = LoadExtrinsicsYaml();
    std::vector<geometry_msgs::TransformStamped> transforms;
    transforms.reserve(extrinsics.size());

    for (const auto & extrinsic : extrinsics) {
      if (!ShouldPublish(extrinsic)) {
        continue;
      }
      transforms.push_back(BuildTransform(extrinsic));
    }

    if (transforms.empty()) {
      throw std::runtime_error("No extrinsics matched camera_serials filter in: " + extrinsics_yaml_path_);
    }

    broadcaster_.sendTransform(transforms);
    ROS_INFO(
      "Published %zu static FLIR camera extrinsic transforms from '%s'.",
      transforms.size(),
      extrinsics_yaml_path_.c_str());
  }

private:
  std::vector<std::string> ReadLines() const
  {
    std::ifstream stream(extrinsics_yaml_path_);
    if (!stream.is_open()) {
      throw std::runtime_error("Failed to open extrinsics YAML file: " + extrinsics_yaml_path_);
    }
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(stream, line)) {
      lines.push_back(line);
    }
    return lines;
  }

  std::vector<ExtrinsicTransform> LoadExtrinsicsYaml() const
  {
    const std::vector<std::string> lines = ReadLines();
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
      } else if (const auto value = MatchYamlScalarValue(raw_line, "parent_frame")) {
        current->parent_frame = StripMatchingQuotes(*value);
      } else if (const auto value = MatchYamlScalarValue(raw_line, "child_frame")) {
        current->child_frame = StripMatchingQuotes(*value);
      } else if (const auto value = MatchYamlScalarValue(raw_line, "translation_xyz_m")) {
        current->translation_xyz_m = ToFixedArray<3>(
          ParseYamlDoubleList(*value, "translation_xyz_m"),
          "translation_xyz_m");
        current->has_translation = true;
      } else if (const auto value = MatchYamlScalarValue(raw_line, "rotation_xyzw")) {
        current->rotation_xyzw = ToFixedArray<4>(
          ParseYamlDoubleList(*value, "rotation_xyzw"),
          "rotation_xyzw");
        current->has_rotation = true;
      }
    }
    finish_current();

    for (auto & extrinsic : extrinsics) {
      if (extrinsic.parent_frame.empty()) {
        extrinsic.parent_frame = reference_frame;
      }
      ValidateAndNormalize(extrinsic);
    }
    if (extrinsics.empty()) {
      throw std::runtime_error(
        "Extrinsics YAML did not contain any entries under extrinsics_by_serial: " +
        extrinsics_yaml_path_);
    }
    return extrinsics;
  }

  void ValidateAndNormalize(ExtrinsicTransform & extrinsic) const
  {
    const std::string label = extrinsic.serial.empty() ? extrinsic.camera_name : extrinsic.serial;
    if (extrinsic.parent_frame.empty() || extrinsic.child_frame.empty() ||
      !extrinsic.has_translation || !extrinsic.has_rotation)
    {
      throw std::runtime_error("Extrinsic '" + label + "' is incomplete.");
    }

    const double norm = std::sqrt(
      extrinsic.rotation_xyzw[0] * extrinsic.rotation_xyzw[0] +
      extrinsic.rotation_xyzw[1] * extrinsic.rotation_xyzw[1] +
      extrinsic.rotation_xyzw[2] * extrinsic.rotation_xyzw[2] +
      extrinsic.rotation_xyzw[3] * extrinsic.rotation_xyzw[3]);
    if (norm < 1e-12) {
      throw std::runtime_error("Extrinsic '" + label + "' has a zero-length quaternion.");
    }
    for (double & value : extrinsic.rotation_xyzw) {
      value /= norm;
    }
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

  std::string FrameName(const std::string & frame) const
  {
    return frame_prefix_.empty() ? frame : frame_prefix_ + frame;
  }

  geometry_msgs::TransformStamped BuildTransform(const ExtrinsicTransform & extrinsic) const
  {
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
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

  ros::NodeHandle private_nh_;
  tf2_ros::StaticTransformBroadcaster broadcaster_;
  std::string extrinsics_yaml_path_;
  std::vector<std::string> camera_serials_;
  std::string frame_prefix_;
};

}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "flir_camera_extrinsics_tf");
  try {
    ExtrinsicsTfNoetic node;
    node.Publish();
    ros::spin();
  } catch (const std::exception & exception) {
    ROS_FATAL("flir_camera_extrinsics_tf failed: %s", exception.what());
    return 1;
  }
  return 0;
}
