// Free-running threaded ROS 2 driver for the WitMotion HWT605-485 IMU.
//
// A dedicated thread polls the device back-to-back (no fixed-rate timer) and
// publishes each reply as it arrives, so throughput is bounded only by the
// device's round-trip, not by timer quantization.
//
// Leaner read: 12 registers 0x34..0x3F (accel, gyro, mag, angle) -- temperature
// is dropped. Reply frame is 50 03 18 <24 data> <crc16> = 29 bytes.
//
// Register map (verified on a live unit):
//   0x34-0x36 accel X/Y/Z  raw/32768*16 g   (physically clips at +/-1.5 g)
//   0x37-0x39 gyro  X/Y/Z  raw/32768*2000 deg/s
//   0x3A-0x3C mag   X/Y/Z  absent on this 6-axis unit (reads 0)
//   0x3D-0x3F angle R/P/Y  raw/32768*180 deg
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <linux/serial.h>
#include <sys/ioctl.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

namespace {
constexpr double kG = 9.80665;
constexpr size_t kFrameLen = 29;   // 3 header + 24 data + 2 crc

uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b)
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  }
  return crc;
}

speed_t baud_const(int baud) {
  switch (baud) {
    case 9600: return B9600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    default: return B230400;
  }
}

int16_t be16(const uint8_t* p) {
  return static_cast<int16_t>((p[0] << 8) | p[1]);
}
}  // namespace

class Hwt605Fast : public rclcpp::Node {
 public:
  Hwt605Fast() : Node("hwt605_node") {
    port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    int baud = declare_parameter<int>("baud", 230400);
    addr_ = static_cast<uint8_t>(declare_parameter<int>("address", 0x50));
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");

    if (!openPort(baud)) {
      RCLCPP_FATAL(get_logger(), "Failed to open %s", port_.c_str());
      throw std::runtime_error("serial open failed");
    }

    uint8_t req[6] = {addr_, 0x03, 0x00, 0x34, 0x00, 0x0C};  // 12 registers
    std::memcpy(req_, req, 6);
    uint16_t c = crc16(req, 6);
    req_[6] = c & 0xFF;
    req_[7] = (c >> 8) & 0xFF;

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

    RCLCPP_INFO(get_logger(), "HWT605 (fast/threaded) on %s @ %d baud",
                port_.c_str(), baud);
    running_ = true;
    worker_ = std::thread(&Hwt605Fast::loop, this);
  }

  ~Hwt605Fast() override {
    running_ = false;
    if (worker_.joinable()) worker_.join();
    if (fd_ >= 0) close(fd_);
  }

 private:
  bool openPort(int baud) {
    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) return false;
    termios tio{};
    if (tcgetattr(fd_, &tio) != 0) return false;
    cfmakeraw(&tio);
    cfsetispeed(&tio, baud_const(baud));
    cfsetospeed(&tio, baud_const(baud));
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1;  // 0.1 s cap; a full reply returns well before this
    if (tcsetattr(fd_, TCSANOW, &tio) != 0) return false;
    serial_struct ss{};
    if (ioctl(fd_, TIOCGSERIAL, &ss) == 0) {
      ss.flags |= ASYNC_LOW_LATENCY;
      ioctl(fd_, TIOCSSERIAL, &ss);
    }
    tcflush(fd_, TCIOFLUSH);
    return true;
  }

  // Block until exactly kFrameLen bytes of a valid frame are assembled, or a
  // deadline passes. Returns true and fills out[] on success.
  bool readFrame(uint8_t* out) {
    auto deadline = std::chrono::steady_clock::now() + 50ms;
    while (std::chrono::steady_clock::now() < deadline && running_) {
      uint8_t tmp[64];
      ssize_t n = read(fd_, tmp, sizeof(tmp));
      if (n > 0) buf_.insert(buf_.end(), tmp, tmp + n);

      // scan for a complete valid frame
      size_t i = 0;
      while (i + kFrameLen <= buf_.size()) {
        if (buf_[i] == addr_ && buf_[i + 1] == 0x03 && buf_[i + 2] == 0x18) {
          uint16_t c = crc16(&buf_[i], kFrameLen - 2);
          uint16_t got = buf_[i + kFrameLen - 2] | (buf_[i + kFrameLen - 1] << 8);
          if (c == got) {
            std::memcpy(out, &buf_[i], kFrameLen);
            buf_.erase(buf_.begin(), buf_.begin() + i + kFrameLen);
            return true;
          }
        }
        ++i;
      }
      if (buf_.size() > 4 * kFrameLen)
        buf_.erase(buf_.begin(), buf_.end() - 2);  // keep a small tail
    }
    return false;
  }

  void loop() {
    uint8_t frame[kFrameLen];
    int fails = 0;
    while (running_ && rclcpp::ok()) {
      ssize_t w = write(fd_, req_, sizeof(req_));
      (void)w;
      if (readFrame(frame)) {
        publish(frame);
        fails = 0;
      } else {
        if (++fails % 100 == 1)
          RCLCPP_WARN(get_logger(), "no frame (fails=%d)", fails);
        tcflush(fd_, TCIFLUSH);
        buf_.clear();
      }
    }
  }

  void publish(const uint8_t* f) {
    const uint8_t* d = f + 3;  // 24 data bytes = 12 registers
    double ax = be16(d + 0) / 32768.0 * 16.0 * kG;
    double ay = be16(d + 2) / 32768.0 * 16.0 * kG;
    double az = be16(d + 4) / 32768.0 * 16.0 * kG;
    double gx = be16(d + 6) / 32768.0 * 2000.0 * M_PI / 180.0;
    double gy = be16(d + 8) / 32768.0 * 2000.0 * M_PI / 180.0;
    double gz = be16(d + 10) / 32768.0 * 2000.0 * M_PI / 180.0;
    double roll = be16(d + 18) / 32768.0 * M_PI;   // /32768*180*pi/180
    double pitch = be16(d + 20) / 32768.0 * M_PI;
    double yaw = be16(d + 22) / 32768.0 * M_PI;

    double cr = cos(roll / 2), sr = sin(roll / 2);
    double cp = cos(pitch / 2), sp = sin(pitch / 2);
    double cy = cos(yaw / 2), sy = sin(yaw / 2);

    sensor_msgs::msg::Imu m;
    m.header.stamp = this->now();
    m.header.frame_id = frame_id_;
    m.orientation.x = sr * cp * cy - cr * sp * sy;
    m.orientation.y = cr * sp * cy + sr * cp * sy;
    m.orientation.z = cr * cp * sy - sr * sp * cy;
    m.orientation.w = cr * cp * cy + sr * sp * sy;
    m.angular_velocity.x = gx;
    m.angular_velocity.y = gy;
    m.angular_velocity.z = gz;
    m.linear_acceleration.x = ax;
    m.linear_acceleration.y = ay;
    m.linear_acceleration.z = az;
    m.orientation_covariance = {0.0004, 0, 0, 0, 0.0004, 0, 0, 0, 0.02};
    m.angular_velocity_covariance = {0.0004, 0, 0, 0, 0.0004, 0, 0, 0, 0.0004};
    m.linear_acceleration_covariance = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};
    imu_pub_->publish(m);
  }

  int fd_ = -1;
  std::string port_, frame_id_;
  uint8_t addr_ = 0x50;
  uint8_t req_[8];
  std::vector<uint8_t> buf_;
  std::atomic<bool> running_{false};
  std::thread worker_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Hwt605Fast>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
