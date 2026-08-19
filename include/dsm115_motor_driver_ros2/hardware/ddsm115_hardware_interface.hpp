#ifndef DSM115_MOTOR_DRIVER_ROS2_HARDWARE_DDSM115_HARDWARE_INTERFACE_HPP_
#define DSM115_MOTOR_DRIVER_ROS2_HARDWARE_DDSM115_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "dsm115_motor_driver_ros2/ddsm115_communicator.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace ddsm115_motor_driver_ros2
{

class Ddsm115HardwareInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  struct Wheel
  {
    std::string joint_name;
    int id{0};
    int direction{1};
    double position{0.0};
    double velocity{0.0};
    double command{0.0};
    double last_command{0.0};
    double last_encoder_position{0.0};
    bool has_encoder_position{false};
  };

  static double velocity_to_rpm(double velocity);
  static double rpm_to_velocity(double rpm);
  static void update_wheel_state(Wheel & wheel, const ddsm115::DriveResponse & response);

  bool parse_wheel_parameters();
  bool validate_joint_interfaces() const;
  void stop_all_wheels();

  std::string port_name_{"/dev/ttyUSB0"};
  double command_deadband_{0.001};
  double command_change_epsilon_{0.001};
  std::vector<Wheel> wheels_;
  std::unique_ptr<ddsm115::Communicator> communicator_;
};

}  // namespace ddsm115_motor_driver_ros2

#endif  // DSM115_MOTOR_DRIVER_ROS2_HARDWARE_DDSM115_HARDWARE_INTERFACE_HPP_
