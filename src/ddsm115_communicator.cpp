#include "dsm115_motor_driver_ros2/ddsm115_communicator.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

namespace ddsm115
{
namespace
{
const rclcpp::Logger kLogger = rclcpp::get_logger("ddsm115_communicator");
}

Communicator::Communicator(const std::string & port_name)
: port_name_(port_name), state_(State::normal)
{
  termios tty{};

  RCLCPP_INFO(kLogger, "Opening serial port %s", port_name_.c_str());
  port_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY);
  if (port_fd_ < 0) {
    RCLCPP_ERROR(kLogger, "Unable to open port %s: %s", port_name_.c_str(), strerror(errno));
    state_ = State::failed;
    return;
  }

  if (tcgetattr(port_fd_, &tty) != 0) {
    RCLCPP_ERROR(kLogger, "Unable to get attributes for port %s: %s", port_name_.c_str(), strerror(errno));
    state_ = State::failed;
    return;
  }

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag = 0;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;

  tty.c_cc[VTIME] = 1;
  tty.c_cc[VMIN] = 0;

  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  if (tcsetattr(port_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(kLogger, "Unable to set attributes for port %s: %s", port_name_.c_str(), strerror(errno));
    state_ = State::failed;
    return;
  }

  tcflush(port_fd_, TCIOFLUSH);
}

Communicator::~Communicator()
{
  disconnect();
}

void Communicator::disconnect()
{
  std::lock_guard<std::mutex> lock(port_mutex_);
  if (port_fd_ >= 0) {
    close(port_fd_);
    port_fd_ = -1;
  }
}

bool Communicator::set_wheel_mode(int wheel_id, Mode mode)
{
  if (port_fd_ < 0) {
    return false;
  }

  uint8_t mode_cmd[] = {
    static_cast<uint8_t>(wheel_id),
    kCommandSwitchMode,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    static_cast<uint8_t>(mode),
  };

  std::lock_guard<std::mutex> lock(port_mutex_);
  const ssize_t written = write(port_fd_, mode_cmd, sizeof(mode_cmd));
  return written == static_cast<ssize_t>(sizeof(mode_cmd));
}

DriveResponse Communicator::set_wheel_rpm(int wheel_id, double rpm)
{
  DriveResponse result;
  if (port_fd_ < 0) {
    return result;
  }

  const auto rpm_value = static_cast<int16_t>(rpm);
  uint8_t drive_cmd[] = {
    static_cast<uint8_t>(wheel_id),
    kCommandDriveMotor,
    static_cast<uint8_t>((rpm_value >> 8) & 0xFF),
    static_cast<uint8_t>(rpm_value & 0xFF),
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
  };
  uint8_t drive_response[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  drive_cmd[9] = maxim_crc8(drive_cmd, 9);

  ssize_t num_bytes = 0;
  int total_num_bytes = 0;
  {
    std::lock_guard<std::mutex> lock(port_mutex_);
    const ssize_t written = write(port_fd_, drive_cmd, sizeof(drive_cmd));
    if (written != static_cast<ssize_t>(sizeof(drive_cmd))) {
      RCLCPP_WARN(kLogger, "Failed to write full command for wheel id %d", wheel_id);
      return result;
    }

    for (size_t i = 0; i < sizeof(drive_response); ++i) {
      num_bytes = read(port_fd_, &drive_response[i], 1);
      if (num_bytes <= 0) {
        break;
      }
      total_num_bytes += static_cast<int>(num_bytes);
    }
  }

  if (num_bytes < 0) {
    RCLCPP_ERROR(kLogger, "Error reading DDSM115 response for wheel id %d", wheel_id);
    return result;
  }

  if (total_num_bytes < 10) {
    return result;
  }

  if (drive_response[0] != static_cast<uint8_t>(wheel_id)) {
    RCLCPP_WARN(
      kLogger, "Received response for wheel %u instead of %d", drive_response[0], wheel_id);
    return result;
  }

  if (drive_response[9] != maxim_crc8(drive_response, 9)) {
    RCLCPP_ERROR(kLogger, "CRC error in response from wheel id %d", wheel_id);
    return result;
  }

  const auto drive_current =
    static_cast<int16_t>((static_cast<uint16_t>(drive_response[2]) << 8) | drive_response[3]);
  const auto drive_velocity =
    static_cast<int16_t>((static_cast<uint16_t>(drive_response[5]) << 8) | drive_response[4]);
  const auto drive_position =
    static_cast<uint16_t>((static_cast<uint16_t>(drive_response[6]) << 8) | drive_response[7]);

  result.velocity = static_cast<double>(drive_velocity);
  result.position = static_cast<double>(drive_position) * (2.0 * M_PI / 32767.0);
  result.current = static_cast<double>(drive_current) * (8.0 / 32767.0);
  result.result = State::normal;
  return result;
}

State Communicator::state() const
{
  return state_;
}

uint8_t Communicator::maxim_crc8(const uint8_t * data, const unsigned int size)
{
  uint8_t crc = 0;
  for (unsigned int i = 0; i < size; ++i) {
    uint8_t inbyte = data[i];
    for (unsigned char j = 0; j < 8; ++j) {
      const uint8_t mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix) {
        crc ^= 0x8C;
      }
      inbyte >>= 1;
    }
  }
  return crc;
}

}  // namespace ddsm115
