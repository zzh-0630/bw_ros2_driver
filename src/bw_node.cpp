#include <fcntl.h>
#include <termios.h>
#include <tf2/LinearMath/Quaternion.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <string>
#include <vector>

#include "bw_ros2_driver/bcd_utils.hpp"
#include "bw_ros2_driver/bws_parser.hpp"
#include "bw_ros2_driver/f3_parser.hpp"
#include "bw_ros2_driver/serial_port.hpp"

using bw::BwsParser;
using bw::DataSample;
using bw::SerialPort;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("bw_ros_driver");
  auto logger = node->get_logger();
  
  //parameters with defaults
  std::string port = "/dev/ttyUSB0";
  std::string frame_id = "imu_link";
  std::string imu_topic = "/imu/data";
  std::string mag_unit = "gauss";
  std::string protocol = "auto";
  int baud = 115200;
  bool debug = true;
  double ori_cov = -1.0, gyr_cov = -1.0, acc_cov = -1.0;

  port = node->declare_parameter<std::string>("port", port);
  baud = node->declare_parameter<int>("baud", baud);
  frame_id = node->declare_parameter<std::string>("frame_id", frame_id);
  imu_topic = node->declare_parameter<std::string>(
      "topic", imu_topic);  
  mag_unit = node->declare_parameter<std::string>("mag_unit", mag_unit);
  debug = node->declare_parameter<bool>("debug", debug);
  ori_cov = node->declare_parameter<double>("cov_orientation", ori_cov);
  gyr_cov = node->declare_parameter<double>("cov_angular_velocity", gyr_cov);
  acc_cov = node->declare_parameter<double>("cov_linear_acceleration", acc_cov);
  protocol = node->declare_parameter<std::string>(
      "protocol", protocol);  

  //serial port
  SerialPort serial_port(port, baud);
  if (!serial_port.openSerial()) {
    RCLCPP_ERROR(logger, "Open %s failed", port.c_str());
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(logger, "Opened %s @ %d, publishing %s", port.c_str(), baud,
              imu_topic.c_str());

  // ---------- Publisher：IMU & Mag ----------
  auto pub = node->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 200);
  auto mag_pub =
      node->create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", 200);

  const double DEG2RAD = M_PI / 180.0;
  const double G2MS2 = 9.80665;

  BwsParser parser;

  rclcpp::Time t0 = node->now();
  uint64_t last_ok = 0;

  DataSample last_dbg{};
  bool last_dbg_valid = false;

  rclcpp::Rate idle(50);

  while (rclcpp::ok()) {
    uint8_t tmp[1024];
    ssize_t n = serial_port.readSome(tmp, sizeof(tmp));
    if (n < 0 && errno != EAGAIN) {
      if (debug) {
        RCLCPP_WARN_THROTTLE(logger, *node->get_clock(), 1000,
                             "read() error: %d", errno);
      }
    }

    if (n > 0) {
      std::vector<DataSample> out;
      out.reserve(8);
      size_t m = parser.feed(tmp, static_cast<size_t>(n), out);

      for (size_t i = 0; i < m; ++i) {
        const DataSample& s = out[i];

        // ---------- IMU ----------
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = node->now();
        imu_msg.header.frame_id = frame_id;

        for (int k = 0; k < 9; ++k) {
          imu_msg.orientation_covariance[k] = 0.0;
          imu_msg.angular_velocity_covariance[k] = 0.0;
          imu_msg.linear_acceleration_covariance[k] = 0.0;
        }
        imu_msg.orientation_covariance[0] = ori_cov;
        imu_msg.angular_velocity_covariance[0] = gyr_cov;
        imu_msg.linear_acceleration_covariance[0] = acc_cov;

        if (s.has_quat) {
          imu_msg.orientation.w = s.q0;
          imu_msg.orientation.x = s.q1;
          imu_msg.orientation.y = s.q2;
          imu_msg.orientation.z = s.q3;
        } else if (s.has_euler) {
          double roll_rad = static_cast<double>(s.R) * DEG2RAD;
          double pitch_rad = static_cast<double>(s.P) * DEG2RAD;
          double yaw_rad = static_cast<double>(s.Y) * DEG2RAD;

          double s_fun[3] = {std::sin(pitch_rad / 2.0),
                             std::sin(roll_rad / 2.0), std::sin(yaw_rad / 2.0)};
          double c_fun[3] = {std::cos(pitch_rad / 2.0),
                             std::cos(roll_rad / 2.0), std::cos(yaw_rad / 2.0)};
          imu_msg.orientation.w =
              c_fun[0] * c_fun[1] * c_fun[2] - s_fun[0] * s_fun[1] * s_fun[2];
          imu_msg.orientation.x =
              s_fun[0] * c_fun[1] * c_fun[2] - c_fun[0] * s_fun[1] * s_fun[2];
          imu_msg.orientation.y =
              s_fun[0] * c_fun[1] * s_fun[2] + c_fun[0] * s_fun[1] * c_fun[2];
          imu_msg.orientation.z =
              c_fun[0] * c_fun[1] * s_fun[2] + s_fun[0] * s_fun[1] * s_fun[2];
        } else {
          imu_msg.orientation_covariance[0] = -1.0;
        }

        if (s.has_gyro) {
          imu_msg.angular_velocity.x = s.gx_dps * DEG2RAD;
          imu_msg.angular_velocity.y = s.gy_dps * DEG2RAD;
          imu_msg.angular_velocity.z = s.gz_dps * DEG2RAD;
        } else {
          imu_msg.angular_velocity_covariance[0] = -1.0;
        }

        if (s.has_acc) {
          imu_msg.linear_acceleration.x = s.ax_g * G2MS2;
          imu_msg.linear_acceleration.y = s.ay_g * G2MS2;
          imu_msg.linear_acceleration.z = s.az_g * G2MS2;
        } else {
          imu_msg.linear_acceleration_covariance[0] = -1.0;
        }

        pub->publish(imu_msg);

        if (s.has_mag) {
          double scale = 1.0;
          if (mag_unit == "gauss")
            scale = 1e-4;
          else if (mag_unit == "uT" || mag_unit == "ut")
            scale = 1e-6;

          sensor_msgs::msg::MagneticField mmsg;
          mmsg.header = imu_msg.header;
          mmsg.magnetic_field.x = s.mx * scale;
          mmsg.magnetic_field.y = s.my * scale;
          mmsg.magnetic_field.z = s.mz * scale;
          mag_pub->publish(mmsg);
        }

        last_dbg = s;
        last_dbg_valid = true;
      }
    }

    if (debug) {
      static double last_print = 0.0;
      const double now = node->now().seconds();
      const double dt_print = now - last_print;

      if (dt_print >= 1.0) {
        const double dt = (node->now() - t0).seconds();
        const uint64_t ok_now = parser.ok();
        const uint64_t bad_now = parser.bad();
        const double hz =
            dt > 0.0 ? static_cast<double>(ok_now - last_ok) / dt : 0.0;

        if (last_dbg_valid) {
          RCLCPP_INFO(logger,
                      "AUTO BCD52 %.1f Hz  "
                      "P=%+.2f R=%+.2f Y=%+.2f  "
                      "G[deg/s]=[%+.2f %+.2f %+.2f]  "
                      "A[g]=[%+.2f %+.2f %+.2f]  "
                      "M=[%+.5f %+.5f %+.5f]  "
                      "q=[%+.6f %+.6f %+.6f %+.6f]  "
                      "(ok=%lu bad=%lu)",
                      hz, last_dbg.P, last_dbg.R, last_dbg.Y, last_dbg.gx_dps,
                      last_dbg.gy_dps, last_dbg.gz_dps, last_dbg.ax_g,
                      last_dbg.ay_g, last_dbg.az_g, last_dbg.mx, last_dbg.my,
                      last_dbg.mz, last_dbg.q0, last_dbg.q1, last_dbg.q2,
                      last_dbg.q3, static_cast<unsigned long>(ok_now),
                      static_cast<unsigned long>(bad_now));
        } else {
          RCLCPP_INFO(
              logger,
              "AUTO BCD52 %.1f Hz  (waiting frames...)  (ok=%lu bad=%lu)", hz,
              static_cast<unsigned long>(ok_now),
              static_cast<unsigned long>(bad_now));
        }

        last_ok = ok_now;
        t0 = node->now();
        last_print = now;
      }
    }

    idle.sleep();
  }

  serial_port.closeSerial();
  rclcpp::shutdown();
  return 0;
}
