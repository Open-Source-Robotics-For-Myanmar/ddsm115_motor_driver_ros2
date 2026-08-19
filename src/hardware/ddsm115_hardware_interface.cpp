#include "dsm115_motor_driver_ros2/hardware/ddsm115_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_set>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ddsm115_motor_driver_ros2
{
namespace
{
const rclcpp::Logger kLogger = rclcpp::get_logger("ddsm115_hardware_interface");

int parse_int_parameter(
  const hardware_interface::ComponentInfo & joint,
  const std::string & name)
{
  const auto iter = joint.parameters.find(name);
  if (iter == joint.parameters.end()) {
    throw std::runtime_error("joint " + joint.name + " is missing parameter " + name);
  }
  return std::stoi(iter->second);
}

int parse_direction_parameter(const hardware_interface::ComponentInfo & joint)
{
  const auto iter = joint.parameters.find("direction");
  if (iter == joint.parameters.end() || iter->second == "forward") {
    return 1;
  }
  if (iter->second == "backward") {
    return -1;
  }
  throw std::runtime_error(
          "joint " + joint.name + " parameter direction must be 'forward' or 'backward'");
}

double parse_double_parameter(
  const hardware_interface::HardwareInfo & info,
  const std::string & name,
  double default_value)
{
  const auto iter = info.hardware_parameters.find(name);
  if (iter == info.hardware_parameters.end()) {
    return default_value;
  }
  return std::stod(iter->second);
}
}  // namespace

hardware_interface::CallbackReturn Ddsm115HardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto port_iter = info_.hardware_parameters.find("port_name");
  if (port_iter != info_.hardware_parameters.end()) {
    port_name_ = port_iter->second;
  }

  try {
    command_deadband_ = parse_double_parameter(info_, "command_deadband", command_deadband_);
    command_change_epsilon_ =
      parse_double_parameter(info_, "command_change_epsilon", command_change_epsilon_);

    if (!parse_wheel_parameters() || !validate_joint_interfaces()) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(kLogger, "Failed to initialize DDSM115 hardware: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
Ddsm115HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(wheels_.size() * 2);

  for (auto & wheel : wheels_) {
    state_interfaces.emplace_back(
      wheel.joint_name, hardware_interface::HW_IF_POSITION, &wheel.position);
    state_interfaces.emplace_back(
      wheel.joint_name, hardware_interface::HW_IF_VELOCITY, &wheel.velocity);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
Ddsm115HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(wheels_.size());

  for (auto & wheel : wheels_) {
    command_interfaces.emplace_back(
      wheel.joint_name, hardware_interface::HW_IF_VELOCITY, &wheel.command);
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn Ddsm115HardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  communicator_ = std::make_unique<ddsm115::Communicator>(port_name_);
  if (communicator_->state() != ddsm115::State::normal) {
    RCLCPP_ERROR(kLogger, "Failed to open DDSM115 serial port %s", port_name_.c_str());
    communicator_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & wheel : wheels_) {
    if (!communicator_->set_wheel_mode(wheel.id, ddsm115::Mode::velocity_loop)) {
      RCLCPP_ERROR(kLogger, "Failed to set velocity mode for wheel id %d", wheel.id);
      communicator_.reset();
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(kLogger, "Configured DDSM115 hardware on %s", port_name_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ddsm115HardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  for (auto & wheel : wheels_) {
    wheel.command = 0.0;
    wheel.last_command = std::numeric_limits<double>::quiet_NaN();
    wheel.has_encoder_position = false;
  }
  stop_all_wheels();
  RCLCPP_INFO(kLogger, "Activated DDSM115 hardware");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ddsm115HardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  stop_all_wheels();
  RCLCPP_INFO(kLogger, "Deactivated DDSM115 hardware");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ddsm115HardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  stop_all_wheels();
  if (communicator_) {
    communicator_->disconnect();
    communicator_.reset();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Ddsm115HardwareInterface::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!communicator_) {
    return hardware_interface::return_type::ERROR;
  }

  // The DDSM115 returns its encoder position only in response to a drive packet.
  // Re-send the current command on every read so manual wheel movement is observed
  // even when the command itself has not changed.
  for (auto & wheel : wheels_) {
    const double command = std::abs(wheel.command) < command_deadband_ ? 0.0 : wheel.command;
    const auto response = communicator_->set_wheel_rpm(
      wheel.id, velocity_to_rpm(command) * wheel.direction);
    if (response.result == ddsm115::State::normal) {
      update_wheel_state(wheel, response);
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Ddsm115HardwareInterface::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!communicator_) {
    return hardware_interface::return_type::ERROR;
  }

  for (auto & wheel : wheels_) {
    const double command = std::abs(wheel.command) < command_deadband_ ? 0.0 : wheel.command;
    if (!std::isnan(wheel.last_command) &&
      std::abs(command - wheel.last_command) < command_change_epsilon_)
    {
      continue;
    }

    const auto response = communicator_->set_wheel_rpm(
      wheel.id,
      velocity_to_rpm(command) * wheel.direction);
    if (response.result == ddsm115::State::normal) {
      update_wheel_state(wheel, response);
    } else {
      wheel.velocity = command;
    }
    wheel.last_command = command;
  }

  return hardware_interface::return_type::OK;
}

double Ddsm115HardwareInterface::velocity_to_rpm(double velocity)
{
  return velocity / (M_PI * 2.0) * 60.0;
}

double Ddsm115HardwareInterface::rpm_to_velocity(double rpm)
{
  return rpm / 60.0 * (M_PI * 2.0);
}

void Ddsm115HardwareInterface::update_wheel_state(
  Wheel & wheel, const ddsm115::DriveResponse & response)
{
  const double encoder_position = response.position;
  if (wheel.has_encoder_position) {
    // Encoder position is one revolution (0 to 2 pi), so unwrap it into a
    // continuous joint position while retaining the configured joint direction.
    wheel.position += std::remainder(
      encoder_position - wheel.last_encoder_position, 2.0 * M_PI) * wheel.direction;
  } else {
    wheel.has_encoder_position = true;
  }

  wheel.last_encoder_position = encoder_position;
  wheel.velocity = rpm_to_velocity(response.velocity) * wheel.direction;
}

bool Ddsm115HardwareInterface::parse_wheel_parameters()
{
  wheels_.clear();
  wheels_.reserve(info_.joints.size());

  std::unordered_set<int> ids;
  for (const auto & joint : info_.joints) {
    Wheel wheel;
    wheel.joint_name = joint.name;
    wheel.id = parse_int_parameter(joint, "id");
    wheel.direction = parse_direction_parameter(joint);

    if (wheel.id <= 0 || wheel.id > 255) {
      RCLCPP_ERROR(kLogger, "Wheel id for joint %s must be in the range 1..255", joint.name.c_str());
      return false;
    }
    if (!ids.insert(wheel.id).second) {
      RCLCPP_ERROR(kLogger, "Wheel id %d is used more than once", wheel.id);
      return false;
    }

    wheels_.push_back(std::move(wheel));
  }

  if (wheels_.empty()) {
    RCLCPP_ERROR(kLogger, "No wheel joints were provided to the DDSM115 hardware interface");
    return false;
  }

  return true;
}

bool Ddsm115HardwareInterface::validate_joint_interfaces() const
{
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(
        kLogger, "Joint %s must have exactly one velocity command interface", joint.name.c_str());
      return false;
    }

    const bool has_position_state = std::any_of(
      joint.state_interfaces.begin(),
      joint.state_interfaces.end(),
      [](const auto & interface) {
        return interface.name == hardware_interface::HW_IF_POSITION;
      });
    const bool has_velocity_state = std::any_of(
      joint.state_interfaces.begin(),
      joint.state_interfaces.end(),
      [](const auto & interface) {
        return interface.name == hardware_interface::HW_IF_VELOCITY;
      });

    if (!has_position_state || !has_velocity_state) {
      RCLCPP_ERROR(
        kLogger, "Joint %s must have position and velocity state interfaces", joint.name.c_str());
      return false;
    }
  }

  return true;
}

void Ddsm115HardwareInterface::stop_all_wheels()
{
  if (!communicator_) {
    return;
  }

  for (auto & wheel : wheels_) {
    communicator_->set_wheel_rpm(wheel.id, 0.0);
    wheel.command = 0.0;
    wheel.last_command = 0.0;
  }
}

}  // namespace ddsm115_motor_driver_ros2

PLUGINLIB_EXPORT_CLASS(
  ddsm115_motor_driver_ros2::Ddsm115HardwareInterface,
  hardware_interface::SystemInterface)
