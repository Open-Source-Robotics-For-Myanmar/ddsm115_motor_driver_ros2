#ifndef DSM115_MOTOR_DRIVER_ROS2_DDSM115_COMMUNICATOR_HPP_
#define DSM115_MOTOR_DRIVER_ROS2_DDSM115_COMMUNICATOR_HPP_

#include <cstdint>
#include <mutex>
#include <string>

namespace ddsm115
{

enum class State : uint8_t
{
  normal = 0x01,
  failed = 0x02,
};

enum class Mode : uint8_t
{
  current_loop = 0x01,
  velocity_loop = 0x02,
  position_loop = 0x03,
};

struct DriveResponse
{
  double current{0.0};
  double velocity{0.0};
  double position{0.0};
  uint16_t raw_counts{0};
  State result{State::failed};
};

class Communicator
{
public:
  explicit Communicator(const std::string & port_name);
  ~Communicator();

  Communicator(const Communicator &) = delete;
  Communicator & operator=(const Communicator &) = delete;

  void disconnect();
  bool set_wheel_mode(int wheel_id, Mode mode);
  DriveResponse set_wheel_rpm(int wheel_id, double rpm);
  State state() const;

private:
  static constexpr uint8_t kCommandDriveMotor = 0x64;
  static constexpr uint8_t kCommandSwitchMode = 0xA0;

  static uint8_t maxim_crc8(const uint8_t * data, unsigned int size);

  std::string port_name_;
  int port_fd_{-1};
  mutable std::mutex port_mutex_;
  State state_{State::failed};
};

}  // namespace ddsm115

#endif  // DSM115_MOTOR_DRIVER_ROS2_DDSM115_COMMUNICATOR_HPP_
