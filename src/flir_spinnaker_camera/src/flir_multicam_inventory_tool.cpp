#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

namespace
{

using Spinnaker::CameraList;
using Spinnaker::CameraPtr;
using Spinnaker::InterfaceList;
using Spinnaker::InterfacePtr;
using Spinnaker::SystemPtr;
using Spinnaker::GenApi::CBooleanPtr;
using Spinnaker::GenApi::CCommandPtr;
using Spinnaker::GenApi::CIntegerPtr;
using Spinnaker::GenApi::CStringPtr;
using Spinnaker::GenApi::INodeMap;
using Spinnaker::GenApi::IsReadable;
using Spinnaker::GenApi::IsWritable;

struct CameraEntry
{
  std::map<std::string, std::string> values;
  bool detected{false};
};

struct DetectedCamera
{
  std::string serial;
  std::string vendor;
  std::string model;
  std::optional<std::uint32_t> ip_address;
};

struct Options
{
  std::filesystem::path output_path{"src/flir_spinnaker_camera/config/multicam_cameras.yaml"};
  std::string force_ip_base;
  std::string force_ip_subnet_mask{"255.255.255.0"};
  std::string force_ip_gateway{"0.0.0.0"};
  std::string new_ptp_action_role;
  std::string new_hardware_trigger_role;
  bool apply_force_ip{false};
  int force_ip_wait_after_ms{1500};
  int force_ip_rediscovery_timeout_ms{5000};
  int force_ip_max_attempts{3};
  bool first_camera_ptp_sender{false};
  bool drop_missing{false};
  bool dry_run{false};
};

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

std::string QuoteYamlString(const std::string & value)
{
  std::string quoted = "\"";
  for (const char character : value) {
    if (character == '\\' || character == '"') {
      quoted.push_back('\\');
    }
    quoted.push_back(character);
  }
  quoted.push_back('"');
  return quoted;
}

bool LooksLikeBool(const std::string & value)
{
  std::string normalized;
  normalized.reserve(value.size());
  for (const unsigned char character : value) {
    if (std::isalnum(character) != 0) {
      normalized.push_back(static_cast<char>(std::tolower(character)));
    }
  }

  return normalized == "true" || normalized == "false";
}

bool LooksLikeInteger(const std::string & value)
{
  if (value.empty()) {
    return false;
  }

  std::size_t offset = 0U;
  if (value.front() == '-' || value.front() == '+') {
    offset = 1U;
  }

  return offset < value.size() &&
         std::all_of(value.begin() + static_cast<std::ptrdiff_t>(offset), value.end(), [](unsigned char character) {
           return std::isdigit(character) != 0;
         });
}

std::string NormalizeRole(std::string value)
{
  value = TrimAscii(value);
  std::string normalized;
  normalized.reserve(value.size());
  for (const unsigned char character : value) {
    if (character == '_' || character == '-') {
      continue;
    }
    normalized.push_back(static_cast<char>(std::tolower(character)));
  }
  return normalized;
}

std::string FormatYamlScalar(const std::string & value)
{
  if (LooksLikeBool(value) || LooksLikeInteger(value)) {
    return value;
  }

  return QuoteYamlString(value);
}

std::string FormatYamlScalarForKey(const std::string & key, const std::string & value)
{
  if (key == "serial") {
    return QuoteYamlString(value);
  }

  return FormatYamlScalar(value);
}

void ApplyKeyValue(CameraEntry & camera, const std::string & text)
{
  const std::string without_comment = RemoveYamlComment(text);
  const std::size_t separator = without_comment.find(':');
  if (separator == std::string::npos) {
    return;
  }

  const std::string key = TrimAscii(without_comment.substr(0, separator));
  std::string value = TrimAscii(without_comment.substr(separator + 1U));
  if (key.empty() || value.empty()) {
    return;
  }

  camera.values[key] = StripMatchingQuotes(value);
}

std::vector<CameraEntry> ParseExistingCameras(const std::filesystem::path & path)
{
  std::ifstream stream(path);
  if (!stream.is_open()) {
    return {};
  }

  std::vector<CameraEntry> cameras;
  std::optional<CameraEntry> current;
  bool in_cameras = false;

  std::string raw_line;
  while (std::getline(stream, raw_line)) {
    const std::string stripped = TrimAscii(RemoveYamlComment(raw_line));
    if (stripped.empty()) {
      continue;
    }

    if (stripped == "cameras:") {
      in_cameras = true;
      continue;
    }

    if (!in_cameras) {
      continue;
    }

    if (stripped.rfind("- ", 0) == 0) {
      if (current.has_value()) {
        cameras.push_back(*current);
      }
      current = CameraEntry{};
      ApplyKeyValue(*current, stripped.substr(2U));
      continue;
    }

    if (current.has_value()) {
      ApplyKeyValue(*current, stripped);
    }
  }

  if (current.has_value()) {
    cameras.push_back(*current);
  }

  return cameras;
}

std::optional<std::int64_t> ReadInteger(INodeMap & node_map, const char * node_name)
{
  CIntegerPtr node = node_map.GetNode(node_name);
  if (!IsReadable(node)) {
    return std::nullopt;
  }

  return node->GetValue();
}

std::optional<bool> ReadBoolean(INodeMap & node_map, const char * node_name)
{
  CBooleanPtr node = node_map.GetNode(node_name);
  if (!IsReadable(node)) {
    return std::nullopt;
  }

  return node->GetValue();
}

std::string ReadString(INodeMap & node_map, const char * node_name)
{
  CStringPtr node = node_map.GetNode(node_name);
  if (!IsReadable(node)) {
    return "";
  }

  return node->GetValue().c_str();
}

bool SetInteger(INodeMap & node_map, const char * node_name, std::int64_t value)
{
  CIntegerPtr node = node_map.GetNode(node_name);
  if (!IsWritable(node)) {
    return false;
  }

  node->SetValue(value);
  return true;
}

bool ExecuteCommand(INodeMap & node_map, const char * node_name)
{
  CCommandPtr node = node_map.GetNode(node_name);
  if (!IsWritable(node)) {
    return false;
  }

  node->Execute();
  return true;
}

bool ExecuteAutoForceIpForSerial(SystemPtr system, const std::string & serial)
{
  InterfaceList interface_list = system->GetInterfaces();
  bool executed = false;

  try {
    for (std::size_t interface_index = 0; interface_index < interface_list.GetSize(); ++interface_index) {
      InterfacePtr gev_interface = interface_list.GetByIndex(interface_index);
      INodeMap & interface_node_map = gev_interface->GetTLNodeMap();
      CIntegerPtr selector = interface_node_map.GetNode("DeviceSelector");
      CCommandPtr auto_force_ip = interface_node_map.GetNode("GevDeviceAutoForceIP");
      if (!IsReadable(selector) || !IsWritable(selector) || !IsWritable(auto_force_ip)) {
        continue;
      }

      const std::int64_t original_selector = selector->GetValue();
      const std::int64_t min_selector = selector->GetMin();
      const std::int64_t max_selector = selector->GetMax();
      for (std::int64_t index = max_selector; index >= min_selector; --index) {
        selector->SetValue(index);
        if (ReadString(interface_node_map, "DeviceSerialNumber") != serial) {
          continue;
        }

        std::cout << "Executing AutoForceIP fallback for serial " << serial
                  << " on interface index " << interface_index << "\n";
        auto_force_ip->Execute();
        executed = true;
        break;
      }
      selector->SetValue(original_selector);

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
      InterfacePtr gev_interface = interface_list.GetByIndex(interface_index);
      INodeMap & interface_node_map = gev_interface->GetTLNodeMap();
      CIntegerPtr selector = interface_node_map.GetNode("DeviceSelector");
      if (!IsReadable(selector) || !IsWritable(selector)) {
        continue;
      }

      const std::int64_t original_selector = selector->GetValue();
      const std::int64_t min_selector = selector->GetMin();
      const std::int64_t max_selector = selector->GetMax();
      for (std::int64_t index = max_selector; index >= min_selector; --index) {
        selector->SetValue(index);
        if (ReadString(interface_node_map, "DeviceSerialNumber") != serial) {
          continue;
        }

        if (!SetInteger(interface_node_map, "GevDeviceForceIPAddress", target_address) ||
          !SetInteger(interface_node_map, "GevDeviceForceSubnetMask", target_subnet_mask) ||
          !SetInteger(interface_node_map, "GevDeviceForceGateway", target_gateway) ||
          !ExecuteCommand(interface_node_map, "GevDeviceForceIP"))
        {
          selector->SetValue(original_selector);
          interface_list.Clear();
          throw std::runtime_error("Failed to execute interface ForceIP for serial '" + serial + "'.");
        }

        std::cout << "Interface ForceIP command executed for serial " << serial
                  << " on interface index " << interface_index << "\n";
        executed = true;
        break;
      }
      selector->SetValue(original_selector);

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

std::vector<DetectedCamera> DetectCameras()
{
  SystemPtr system = Spinnaker::System::GetInstance();
  CameraList camera_list = system->GetCameras();

  std::vector<DetectedCamera> cameras;
  cameras.reserve(camera_list.GetSize());

  for (std::size_t index = 0; index < camera_list.GetSize(); ++index) {
    CameraPtr camera = camera_list.GetByIndex(index);
    INodeMap & tl_node_map = camera->GetTLDeviceNodeMap();

    DetectedCamera detected;
    detected.serial = ReadString(tl_node_map, "DeviceSerialNumber");
    detected.vendor = ReadString(tl_node_map, "DeviceVendorName");
    detected.model = ReadString(tl_node_map, "DeviceModelName");
    const auto ip_address = ReadInteger(tl_node_map, "GevDeviceIPAddress");
    if (ip_address.has_value() && *ip_address >= 0) {
      detected.ip_address = static_cast<std::uint32_t>(*ip_address);
    }

    if (!detected.serial.empty()) {
      cameras.push_back(detected);
    }
  }

  camera_list.Clear();
  system->ReleaseInstance();
  return cameras;
}

std::uint32_t ParseIpv4Address(std::string value, const char * option_name)
{
  value = TrimAscii(value);
  if (value.empty()) {
    throw std::runtime_error(std::string(option_name) + " must not be empty.");
  }

  std::array<unsigned long, 4> octets{};
  std::stringstream stream(value);
  std::string token;

  for (std::size_t index = 0; index < octets.size(); ++index) {
    if (!std::getline(stream, token, '.')) {
      throw std::runtime_error(std::string(option_name) + " must be an IPv4 address.");
    }

    token = TrimAscii(token);
    if (token.empty()) {
      throw std::runtime_error(std::string(option_name) + " contains an empty IPv4 octet.");
    }

    std::size_t consumed = 0U;
    const unsigned long octet = std::stoul(token, &consumed);
    if (consumed != token.size() || octet > 255UL) {
      throw std::runtime_error(std::string(option_name) + " contains an invalid IPv4 octet.");
    }
    octets[index] = octet;
  }

  if (std::getline(stream, token, '.')) {
    throw std::runtime_error(std::string(option_name) + " must contain exactly four IPv4 octets.");
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

std::string NextCameraName(const std::set<std::string> & used_names)
{
  for (std::size_t index = 0; index < 10000U; ++index) {
    const std::string candidate = "camera" + std::to_string(index);
    if (used_names.find(candidate) == used_names.end()) {
      return candidate;
    }
  }

  throw std::runtime_error("Could not allocate a camera name.");
}

void FillDefaultCameraFields(CameraEntry & entry, std::set<std::string> & used_names)
{
  auto name_it = entry.values.find("name");
  if (name_it == entry.values.end() || name_it->second.empty()) {
    const std::string name = NextCameraName(used_names);
    entry.values["name"] = name;
    used_names.insert(name);
  } else {
    used_names.insert(name_it->second);
  }

  const std::string name = entry.values["name"];
  if (entry.values.find("namespace") == entry.values.end() || entry.values["namespace"].empty()) {
    entry.values["namespace"] = name;
  }
  if (entry.values.find("frame_id") == entry.values.end() || entry.values["frame_id"].empty()) {
    entry.values["frame_id"] = name + "_optical_frame";
  }
}

std::vector<CameraEntry> MergeDetectedCameras(
  std::vector<CameraEntry> existing,
  const std::vector<DetectedCamera> & detected,
  const Options & options)
{
  std::unordered_set<std::string> detected_serials;
  for (const auto & camera : detected) {
    detected_serials.insert(camera.serial);
  }

  std::set<std::string> used_names;
  std::unordered_set<std::string> included_serials;
  std::vector<CameraEntry> merged;

  for (auto & entry : existing) {
    const auto serial_it = entry.values.find("serial");
    const std::string serial = serial_it == entry.values.end() ? "" : serial_it->second;
    const bool is_detected = detected_serials.find(serial) != detected_serials.end();
    if (!options.drop_missing || is_detected) {
      entry.detected = is_detected;
      FillDefaultCameraFields(entry, used_names);
      if (!serial.empty()) {
        included_serials.insert(serial);
      }
      merged.push_back(entry);
    }
  }

  for (const auto & camera : detected) {
    if (included_serials.find(camera.serial) != included_serials.end()) {
      continue;
    }

    CameraEntry entry;
    const std::string name = NextCameraName(used_names);
    entry.values["name"] = name;
    entry.values["serial"] = camera.serial;
    FillDefaultCameraFields(entry, used_names);
    if (!options.new_hardware_trigger_role.empty()) {
      entry.values["hardware_trigger_role"] = options.new_hardware_trigger_role;
    }
    if (!options.new_ptp_action_role.empty()) {
      entry.values["ptp_action_role"] = options.new_ptp_action_role;
    }
    entry.detected = true;
    merged.push_back(entry);
  }

  return merged;
}

void AssignForceIpAddresses(std::vector<CameraEntry> & cameras, const Options & options)
{
  if (options.force_ip_base.empty()) {
    return;
  }

  std::set<std::uint32_t> used_addresses;
  for (const auto & camera : cameras) {
    const auto ip_it = camera.values.find("force_ip_address");
    if (ip_it == camera.values.end() || ip_it->second.empty()) {
      continue;
    }
    used_addresses.insert(ParseIpv4Address(ip_it->second, "force_ip_address"));
  }

  std::uint32_t next_address = ParseIpv4Address(options.force_ip_base, "--force-ip-base");
  for (auto & camera : cameras) {
    if (camera.values.find("force_ip_address") == camera.values.end() ||
      camera.values["force_ip_address"].empty())
    {
      while (used_addresses.find(next_address) != used_addresses.end()) {
        ++next_address;
      }
      camera.values["force_ip_address"] = FormatIpv4Address(next_address);
      used_addresses.insert(next_address);
      ++next_address;
    }

    if (camera.values.find("force_ip_subnet_mask") == camera.values.end()) {
      camera.values["force_ip_subnet_mask"] = options.force_ip_subnet_mask;
    }
    if (camera.values.find("force_ip_gateway") == camera.values.end()) {
      camera.values["force_ip_gateway"] = options.force_ip_gateway;
    }
  }
}

void EnsurePtpSender(std::vector<CameraEntry> & cameras, bool first_camera_ptp_sender)
{
  if (!first_camera_ptp_sender || cameras.empty()) {
    return;
  }

  const auto is_sender = [](const CameraEntry & camera) {
      const auto role_it = camera.values.find("ptp_action_role");
      if (role_it == camera.values.end()) {
        return false;
      }
      const std::string role = NormalizeRole(role_it->second);
      return role == "sender" || role == "send" || role == "master";
    };

  if (std::any_of(cameras.begin(), cameras.end(), is_sender)) {
    return;
  }

  cameras.front().values["ptp_action_role"] = "sender";
}

std::string BuildYaml(const std::vector<CameraEntry> & cameras)
{
  const std::vector<std::string> preferred_order = {
    "name",
    "serial",
    "namespace",
    "frame_id",
    "hardware_trigger_role",
    "ptp_action_role",
    "force_ip_address",
    "force_ip_subnet_mask",
    "force_ip_gateway",
    "force_ip_only_if_link_local",
    "force_ip_wait_after_ms",
    "force_ip_rediscovery_timeout_ms"
  };

  std::ostringstream stream;
  stream << "flir_multicam:\n";
  stream << "  ros__parameters:\n";
  stream << "    cameras:\n";

  for (const auto & camera : cameras) {
    stream << "      - name: "
           << FormatYamlScalarForKey("name", camera.values.at("name")) << "\n";

    std::set<std::string> written{"name"};
    for (const auto & key : preferred_order) {
      if (written.find(key) != written.end()) {
        continue;
      }
      const auto value_it = camera.values.find(key);
      if (value_it == camera.values.end()) {
        continue;
      }
      stream << "        " << key << ": "
             << FormatYamlScalarForKey(key, value_it->second) << "\n";
      written.insert(key);
    }

    for (const auto & [key, value] : camera.values) {
      if (written.find(key) != written.end()) {
        continue;
      }
      stream << "        " << key << ": " << FormatYamlScalarForKey(key, value) << "\n";
    }
  }

  return stream.str();
}

void WriteFileIfChanged(const std::filesystem::path & path, const std::string & content)
{
  {
    std::ifstream existing(path);
    if (existing.is_open()) {
      std::stringstream buffer;
      buffer << existing.rdbuf();
      if (buffer.str() == content) {
        std::cout << "No changes needed: " << path << "\n";
        return;
      }
    }
  }

  if (path.has_parent_path()) {
    std::filesystem::create_directories(path.parent_path());
  }

  std::ofstream output(path);
  if (!output.is_open()) {
    throw std::runtime_error("Failed to open output YAML for writing: " + path.string());
  }
  output << content;
  std::cout << "Updated camera inventory: " << path << "\n";
}

CameraPtr TryFindCameraBySerial(CameraList & camera_list, const std::string & serial)
{
  for (std::size_t index = 0; index < camera_list.GetSize(); ++index) {
    CameraPtr camera = camera_list.GetByIndex(index);
    INodeMap & tl_node_map = camera->GetTLDeviceNodeMap();
    if (ReadString(tl_node_map, "DeviceSerialNumber") == serial) {
      return camera;
    }
  }

  return nullptr;
}

CameraPtr WaitForCameraBySerial(
  SystemPtr system,
  CameraList & camera_list,
  const std::string & serial,
  int timeout_ms)
{
  const auto poll_period = std::chrono::milliseconds(250);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  do {
    camera_list.Clear();
    camera_list = system->GetCameras();
    if (CameraPtr camera = TryFindCameraBySerial(camera_list, serial)) {
      return camera;
    }
    std::this_thread::sleep_for(poll_period);
  } while (std::chrono::steady_clock::now() < deadline);

  camera_list.Clear();
  camera_list = system->GetCameras();
  return TryFindCameraBySerial(camera_list, serial);
}

CameraPtr WaitForCameraAtTargetIp(
  SystemPtr system,
  CameraList & camera_list,
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
    if (CameraPtr camera = TryFindCameraBySerial(camera_list, serial)) {
      INodeMap & tl_node_map = camera->GetTLDeviceNodeMap();
      const auto current_address = ReadInteger(tl_node_map, "GevDeviceIPAddress");
      const auto wrong_subnet = ReadBoolean(tl_node_map, "GevDeviceIsWrongSubnet");
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
          "Camera serial '" + serial + "' did not settle at target IP " +
          FormatIpv4Address(target_address) + " before timeout; last_state=" + last_state + ".");
}

void ApplyForceIpFromInventory(const std::vector<CameraEntry> & cameras, const Options & options)
{
  if (!options.apply_force_ip || options.dry_run) {
    return;
  }

  SystemPtr system = Spinnaker::System::GetInstance();
  CameraList camera_list;

  try {
    for (const auto & camera_entry : cameras) {
      const auto serial_it = camera_entry.values.find("serial");
      const auto address_it = camera_entry.values.find("force_ip_address");
      if (serial_it == camera_entry.values.end() || serial_it->second.empty() ||
        address_it == camera_entry.values.end() || address_it->second.empty())
      {
        continue;
      }

      const std::string & serial = serial_it->second;
      const std::uint32_t target_address =
        ParseIpv4Address(address_it->second, "force_ip_address");
      const std::uint32_t target_subnet_mask = ParseIpv4Address(
        camera_entry.values.count("force_ip_subnet_mask") ?
        camera_entry.values.at("force_ip_subnet_mask") : options.force_ip_subnet_mask,
        "force_ip_subnet_mask");
      const std::uint32_t target_gateway = ParseIpv4Address(
        camera_entry.values.count("force_ip_gateway") ?
        camera_entry.values.at("force_ip_gateway") : options.force_ip_gateway,
        "force_ip_gateway");

      bool verified = false;
      std::string last_error;
      for (int attempt = 1; attempt <= options.force_ip_max_attempts; ++attempt) {
        CameraPtr camera = WaitForCameraBySerial(
          system,
          camera_list,
          serial,
          options.force_ip_rediscovery_timeout_ms);
        if (!camera) {
          last_error = "Could not find camera serial '" + serial + "' while applying ForceIP.";
          continue;
        }

        INodeMap & tl_node_map = camera->GetTLDeviceNodeMap();
        const auto current_address = ReadInteger(tl_node_map, "GevDeviceIPAddress");
        const auto wrong_subnet = ReadBoolean(tl_node_map, "GevDeviceIsWrongSubnet");
        if (current_address.has_value() && *current_address >= 0 &&
          static_cast<std::uint32_t>(*current_address) == target_address &&
          !wrong_subnet.value_or(false))
        {
          std::cout << "ForceIP already correct for serial " << serial << ": "
                    << FormatIpv4Address(target_address) << "\n";
          verified = true;
          break;
        }

        std::cout << "Applying ForceIP serial " << serial
                  << " attempt " << attempt << "/" << options.force_ip_max_attempts
                  << ": current_ip="
                  << (current_address.has_value() && *current_address >= 0 ?
          FormatIpv4Address(static_cast<std::uint32_t>(*current_address)) : "unknown")
                  << " wrong_subnet="
                  << (wrong_subnet.has_value() ? (*wrong_subnet ? "true" : "false") : "unknown")
                  << " target=" << FormatIpv4Address(target_address)
                  << "/" << FormatIpv4Address(target_subnet_mask)
                  << " gateway=" << FormatIpv4Address(target_gateway) << "\n";

        if (wrong_subnet.value_or(false)) {
          try {
            if (ExecuteAutoForceIpForSerial(system, serial) && options.force_ip_wait_after_ms > 0) {
              std::this_thread::sleep_for(
                std::chrono::milliseconds(options.force_ip_wait_after_ms));
            }
            camera_list.Clear();
          } catch (const std::exception & auto_force_exception) {
            std::cout << "AutoForceIP failed for serial " << serial
                      << ": " << auto_force_exception.what() << "\n";
          }
        }

        if (!ExecuteInterfaceForceIpForSerial(
            system,
            serial,
            target_address,
            target_subnet_mask,
            target_gateway))
        {
          throw std::runtime_error(
                  "Could not find camera serial '" + serial + "' for interface ForceIP.");
        }

        camera = nullptr;
        camera_list.Clear();

        if (options.force_ip_wait_after_ms > 0) {
          std::this_thread::sleep_for(
            std::chrono::milliseconds(options.force_ip_wait_after_ms));
        }

        try {
          CameraPtr rediscovered = WaitForCameraAtTargetIp(
            system,
            camera_list,
            serial,
            target_address,
            options.force_ip_rediscovery_timeout_ms);
          (void)rediscovered;
          std::cout << "ForceIP verified for serial " << serial << ": "
                    << FormatIpv4Address(target_address) << "\n";
          verified = true;
          break;
        } catch (const std::exception & exception) {
          last_error = exception.what();
          std::cout << "ForceIP verification failed for serial " << serial
                    << " attempt " << attempt << "/" << options.force_ip_max_attempts
                    << ": " << last_error << "\n";
          if (attempt < options.force_ip_max_attempts) {
            try {
              if (ExecuteAutoForceIpForSerial(system, serial)) {
                camera_list.Clear();
                if (options.force_ip_wait_after_ms > 0) {
                  std::this_thread::sleep_for(
                    std::chrono::milliseconds(options.force_ip_wait_after_ms));
                }
              }
            } catch (const std::exception & auto_force_exception) {
              std::cout << "AutoForceIP fallback failed for serial " << serial
                        << ": " << auto_force_exception.what() << "\n";
            }
          }
        }
      }

      if (!verified) {
        throw std::runtime_error(last_error.empty() ?
                "ForceIP did not verify for serial '" + serial + "'." : last_error);
      }
    }

    camera_list.Clear();
    system->ReleaseInstance();
  } catch (...) {
    camera_list.Clear();
    system->ReleaseInstance();
    throw;
  }
}

Options ParseOptions(int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    const auto next_value = [&]() -> std::string {
        if (index + 1 >= argc) {
          throw std::runtime_error("Missing value for " + argument);
        }
        ++index;
        return argv[index];
      };

    if (argument == "--output" || argument == "--cameras-file") {
      options.output_path = next_value();
    } else if (argument == "--force-ip-base") {
      options.force_ip_base = next_value();
    } else if (argument == "--force-ip-subnet-mask") {
      options.force_ip_subnet_mask = next_value();
    } else if (argument == "--force-ip-gateway") {
      options.force_ip_gateway = next_value();
    } else if (argument == "--apply-force-ip") {
      options.apply_force_ip = true;
    } else if (argument == "--force-ip-wait-after-ms") {
      options.force_ip_wait_after_ms = std::stoi(next_value());
    } else if (argument == "--force-ip-rediscovery-timeout-ms") {
      options.force_ip_rediscovery_timeout_ms = std::stoi(next_value());
    } else if (argument == "--force-ip-max-attempts") {
      options.force_ip_max_attempts = std::stoi(next_value());
    } else if (argument == "--new-ptp-action-role") {
      options.new_ptp_action_role = next_value();
    } else if (argument == "--new-hardware-trigger-role") {
      options.new_hardware_trigger_role = next_value();
    } else if (argument == "--first-camera-ptp-sender") {
      options.first_camera_ptp_sender = true;
    } else if (argument == "--drop-missing") {
      options.drop_missing = true;
    } else if (argument == "--dry-run") {
      options.dry_run = true;
    } else if (argument == "--help" || argument == "-h") {
      std::cout
        << "Usage: flir_multicam_inventory_tool [options]\n"
        << "  --output PATH                         YAML to update\n"
        << "  --force-ip-base IPV4                  Assign sequential force_ip_address values\n"
        << "  --force-ip-subnet-mask IPV4           Subnet mask for assigned ForceIP entries\n"
        << "  --force-ip-gateway IPV4               Gateway for assigned ForceIP entries\n"
        << "  --apply-force-ip                      Apply YAML ForceIP entries sequentially\n"
        << "  --force-ip-wait-after-ms MS           Delay after each ForceIP command\n"
        << "  --force-ip-rediscovery-timeout-ms MS   Max wait for camera rediscovery\n"
        << "  --force-ip-max-attempts N             Max ForceIP command attempts per camera\n"
        << "  --new-ptp-action-role ROLE            ptp_action_role for newly discovered cameras\n"
        << "  --new-hardware-trigger-role ROLE      hardware_trigger_role for newly discovered cameras\n"
        << "  --first-camera-ptp-sender             Ensure first camera has ptp_action_role=sender\n"
        << "  --drop-missing                        Remove entries not currently detected\n"
        << "  --dry-run                             Print YAML instead of writing\n";
      std::exit(0);
    } else {
      throw std::runtime_error("Unknown argument: " + argument);
    }
  }

  if (options.force_ip_wait_after_ms < 0) {
    throw std::runtime_error("--force-ip-wait-after-ms must be non-negative.");
  }
  if (options.force_ip_rediscovery_timeout_ms < 0) {
    throw std::runtime_error("--force-ip-rediscovery-timeout-ms must be non-negative.");
  }
  if (options.force_ip_max_attempts <= 0) {
    throw std::runtime_error("--force-ip-max-attempts must be positive.");
  }

  return options;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::cout.setf(std::ios::unitbuf);

  try {
    const Options options = ParseOptions(argc, argv);
    const std::vector<DetectedCamera> detected = DetectCameras();
    if (detected.empty()) {
      throw std::runtime_error("No FLIR cameras detected by Spinnaker.");
    }

    std::vector<CameraEntry> cameras = MergeDetectedCameras(
      ParseExistingCameras(options.output_path),
      detected,
      options);
    AssignForceIpAddresses(cameras, options);
    EnsurePtpSender(cameras, options.first_camera_ptp_sender);

    const std::string yaml = BuildYaml(cameras);
    if (options.dry_run) {
      std::cout << yaml;
    } else {
      WriteFileIfChanged(options.output_path, yaml);
    }
    ApplyForceIpFromInventory(cameras, options);
  } catch (const std::exception & exception) {
    std::cerr << "flir_multicam_inventory_tool failed: " << exception.what() << std::endl;
    return 1;
  }

  return 0;
}
