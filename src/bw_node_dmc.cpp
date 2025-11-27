#include <tf2/LinearMath/Quaternion.h>

#include <cerrno>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <string>
#include <vector>

#include "bw_ros2_driver/f3_parser.hpp"
#include "bw_ros2_driver/serial_port.hpp"

using bw::F3Parser;
using bw::F3Sample;
using bw::SerialPort;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("bw_ros_driver_f3");
  auto logger = node->get_logger();

  std::string port = "/dev/ttyUSB0";
  std::string frame_id = "imu_link";
  std::string imu_topic = "/imu/data";
  std::string mag_topic = "/imu/mag";
  std::string mag_unit = "gauss";
  int baud = 921600; 
  bool debug = true;
  double ori_cov = -1.0;
  double gyr_cov = -1.0;
  double acc_cov = -1.0;
  int publish_rate = 0;

  port = node->declare_parameter<std::string>("port", port);
  baud = node->declare_parameter<int>("baud", baud);
  frame_id = node->declare_parameter<std::string>("frame_id", frame_id);
  imu_topic = node->declare_parameter<std::string>("imu_topic", imu_topic);
  mag_topic = node->declare_parameter<std::string>("mag_topic", mag_topic);
  mag_unit = node->declare_parameter<std::string>("mag_unit", mag_unit);
  publish_rate = node->declare_parameter<int>("publish_rate", publish_rate);
  debug = node->declare_parameter<bool>("debug", debug);
  ori_cov = node->declare_parameter<double>("cov_orientation", ori_cov);
  gyr_cov = node->declare_parameter<double>("cov_angular_velocity", gyr_cov);
  acc_cov = node->declare_parameter<double>("cov_linear_acceleration", acc_cov);

  SerialPort serial(port, baud);
  if (!serial.openSerial()) {
    RCLCPP_ERROR(logger, "Failed to open serial port %s @ %d", port.c_str(),
                 baud);
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
      logger,
      "Opened %s @ %d (F3 protocol), publishing IMU to '%s', mag to '%s'",
      port.c_str(), baud, imu_topic.c_str(), mag_topic.c_str());

  auto imu_pub = node->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 200);
  auto mag_pub =
      node->create_publisher<sensor_msgs::msg::MagneticField>(mag_topic, 200);

  F3Parser parser;

  const double DEG2RAD = M_PI / 180.0;
  const double G2MS2 = 9.80665;

  rclcpp::Time t0 = node->now();
  uint64_t last_ok = 0;

  rclcpp::Rate idle(publish_rate > 0 ? publish_rate : 1000);

  while (rclcpp::ok()) {
    uint8_t buf[1024];
    ssize_t n = serial.readSome(buf, sizeof(buf));
    if (n < 0 && errno != EAGAIN) {
      if (debug) {
        RCLCPP_WARN_THROTTLE(logger, *node->get_clock(), 1000,
                             "F3 read() error: %d", errno);
      }
    }

    if (n > 0) {
      std::vector<F3Sample> outs;
      outs.reserve(8);

      size_t m = parser.feed(buf, static_cast<size_t>(n), outs);

      for (size_t i = 0; i < m; ++i) {
        const F3Sample& s = outs[i];
        if (!s.valid) {
          continue;
        }

        // ---------- IMU ----------
        sensor_msgs::msg::Imu msg;
        msg.header.stamp = node->now();
        msg.header.frame_id = frame_id;

        for (int k = 0; k < 9; ++k) {
          msg.orientation_covariance[k] = 0.0;
          msg.angular_velocity_covariance[k] = 0.0;
          msg.linear_acceleration_covariance[k] = 0.0;
        }
        msg.orientation_covariance[0] = ori_cov;
        msg.angular_velocity_covariance[0] = gyr_cov;
        msg.linear_acceleration_covariance[0] = acc_cov;

        // euler angles to quaternion
        double pitch_rad = static_cast<double>(s.pitch_deg) * DEG2RAD;
        double roll_rad = static_cast<double>(s.roll_deg) * DEG2RAD;
        double yaw_rad = static_cast<double>(s.yaw_deg) * DEG2RAD;

        double s_fun[3] = {sin(pitch_rad / 2), sin(roll_rad / 2),
                           sin(yaw_rad / 2)};
        double c_fun[3] = {cos(pitch_rad / 2), cos(roll_rad / 2),
                           cos(yaw_rad / 2)};
        msg.orientation.w =
            c_fun[0] * c_fun[1] * c_fun[2] - s_fun[0] * s_fun[1] * s_fun[2];
        msg.orientation.x =
            s_fun[0] * c_fun[1] * c_fun[2] - c_fun[0] * s_fun[1] * s_fun[2];
        msg.orientation.y =
            s_fun[0] * c_fun[1] * s_fun[2] + c_fun[0] * s_fun[1] * c_fun[2];
        msg.orientation.z =
            c_fun[0] * c_fun[1] * s_fun[2] + s_fun[0] * s_fun[1] * c_fun[2];

        // angular velocity
        msg.angular_velocity.x = static_cast<double>(s.gyro_x_dps) * DEG2RAD;
        msg.angular_velocity.y = static_cast<double>(s.gyro_y_dps) * DEG2RAD;
        msg.angular_velocity.z = static_cast<double>(s.gyro_z_dps) * DEG2RAD;

        // linear acceleration
        msg.linear_acceleration.x = static_cast<double>(s.acc_x_g) * G2MS2;
        msg.linear_acceleration.y = static_cast<double>(s.acc_y_g) * G2MS2;
        msg.linear_acceleration.z = static_cast<double>(s.acc_z_g) * G2MS2;

        imu_pub->publish(msg);

        // ---------- magnetic_field ----------
        double scale = 1.0;
        if (mag_unit == "gauss")
          scale = 1e-4;
        else if (mag_unit == "uT" || mag_unit == "ut")
          scale = 1e-6;

        sensor_msgs::msg::MagneticField mag_msg;
        mag_msg.header = msg.header;
        mag_msg.magnetic_field.x = static_cast<double>(s.mag_x) * scale;
        mag_msg.magnetic_field.y = static_cast<double>(s.mag_y) * scale;
        mag_msg.magnetic_field.z = static_cast<double>(s.mag_z) * scale;
        mag_pub->publish(mag_msg);

        // ---------- Debug  ----------
        if (debug) {
          auto now = node->now();
          double dt = (now - t0).seconds();
          if (dt >= 1.0) {
            uint64_t ok_now = parser.ok();
            double hz = dt > 0.0 ? (ok_now - last_ok) / dt : 0.0;
            last_ok = ok_now;
            uint64_t bad_count = parser.bad();
            uint64_t crc_bad_count = parser.crcBad();
            t0 = now;

            RCLCPP_INFO(logger,
                        "F3 RX ~ %.1f Hz (ok=%lu bad=%lu crc_bad=%lu) "
                        "Euler[deg]=[P=%+.2f R=%+.2f Y=%+.2f] "
                        "G[dps]=[%+.2f %+.2f %+.2f] "
                        "A[g]=[%+.3f %+.3f %+.3f] "
                        "M=[%+.3f %+.3f %+.3f] Temp[C]=%.2f",
                        hz, static_cast<unsigned long>(ok_now),
                        static_cast<unsigned long>(bad_count),
                        static_cast<unsigned long>(crc_bad_count), s.pitch_deg,
                        s.roll_deg, s.yaw_deg, s.gyro_x_dps, s.gyro_y_dps,
                        s.gyro_z_dps, s.acc_x_g, s.acc_y_g, s.acc_z_g, s.mag_x,
                        s.mag_y, s.mag_z, s.temperature_deg_c);
          }
        }
      }
    } else {
      if (debug && parser.ok() == 0) {
        RCLCPP_INFO_THROTTLE(logger, *node->get_clock(), 2000,
                             "F3: waiting for frames on %s ...", port.c_str());
      }
    }

    idle.sleep();
  }

  serial.closeSerial();
  rclcpp::shutdown();
  return 0;
}
