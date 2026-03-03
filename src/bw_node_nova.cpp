#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <tf2/LinearMath/Quaternion.h>

// Standard headers
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

// Project headers
#include "bw_ros_driver/nova_parser.hpp"
#include "bw_ros_driver/serial_port.hpp"

namespace
{
double magScaleToTesla(const std::string& unit)
{
  if (unit == "gauss")
  {
    return 1e-4;
  }
  if (unit == "uT" || unit == "ut")
  {
    return 1e-6;
  }
  return 1.0;
}

void fillCovariance(double value, std::array<double, 9>& refCov)
{
  refCov.fill(0.0);
  refCov[0] = value;
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bw_node_nova");
  auto logger = node->get_logger();

  //-------------------------------------------------------------------//
  // Parameters                                                        //
  //-------------------------------------------------------------------//
  const std::string port = node->declare_parameter<std::string>("port", "/dev/ttyUSB0");
  const int baud = static_cast<int>(node->declare_parameter<int64_t>("baud", 115200));

  const std::string frame_id = node->declare_parameter<std::string>("frame_id", "imu_link");
  const std::string imu_topic = node->declare_parameter<std::string>("imu_topic", "/imu/data");
  const std::string mag_topic = node->declare_parameter<std::string>("mag_topic", "/imu/mag");
  const std::string mag_unit = node->declare_parameter<std::string>("mag_unit", "gauss");

  const double publish_rate = node->declare_parameter<double>("publish_rate", 500.0);
  const bool debug = node->declare_parameter<bool>("debug", true);

  const double cov_orientation = node->declare_parameter<double>("cov_orientation", -1.0);
  const double cov_angular_velocity = node->declare_parameter<double>("cov_angular_velocity", -1.0);
  const double cov_linear_acceleration = node->declare_parameter<double>("cov_linear_acceleration", -1.0);

  //-------------------------------------------------------------------//
  // Open serial port                                                  //
  //-------------------------------------------------------------------//
  bw::SerialPort serial(port, baud);

  if (!serial.openSerial())
  {
    RCLCPP_FATAL(logger, "Failed to open serial port %s", port.c_str());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Opened %s @ %d (Nova/F3), publishing IMU=%s MAG=%s",
              port.c_str(), baud, imu_topic.c_str(), mag_topic.c_str());

  //-------------------------------------------------------------------//
  // Publishers                                                        //
  //-------------------------------------------------------------------//
  auto imu_pub = node->create_publisher<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS());
  auto mag_pub = node->create_publisher<sensor_msgs::msg::MagneticField>(mag_topic, rclcpp::SensorDataQoS());

  //-------------------------------------------------------------------//
  // Parser                                                            //
  //-------------------------------------------------------------------//
  bw::NovaParser parser;

  constexpr double kPi = 3.14159265358979323846;
  constexpr double kDegToRad = kPi / 180.0;
  constexpr double kGToMps2  = 9.80665;

  const double mag_scale = magScaleToTesla(mag_unit);

  //-------------------------------------------------------------------//
  // Debug stats                                                       //
  //-------------------------------------------------------------------//
  const auto clock = node->get_clock();
  rclcpp::Time t_window = clock->now();
  uint64_t last_ok = 0;

  rclcpp::Rate idle(std::max(1.0, publish_rate));

  //-------------------------------------------------------------------//
  // Main loop                                                         //
  //-------------------------------------------------------------------//
  while (rclcpp::ok())
  {
    uint8_t buffer[1024];
    const ssize_t n = serial.readSome(buffer, sizeof(buffer));

    if (n < 0 && errno != EAGAIN)
    {
      if (debug)
      {
        RCLCPP_WARN_THROTTLE(logger, *clock, 1000, "F3 serial read error: errno=%d (%s)", errno, std::strerror(errno));
      }
    }

    if (n > 0)
    {
      std::vector<bw::F3Sample> samples;
      samples.reserve(8);

      const size_t m = parser.feed(buffer, static_cast<size_t>(n), samples);

      for (size_t i = 0; i < m; ++i)
      {
        const bw::F3Sample& s = samples[i];
        if (!s.valid)
        {
          continue;
        }

        //-----------------------------------------------------------------//
        // Build IMU message                                               //
        //-----------------------------------------------------------------//
        sensor_msgs::msg::Imu msg;
        msg.header.stamp = clock->now();
        msg.header.frame_id = frame_id;

        fillCovariance(cov_orientation, msg.orientation_covariance);
        fillCovariance(cov_angular_velocity, msg.angular_velocity_covariance);
        fillCovariance(cov_linear_acceleration, msg.linear_acceleration_covariance);

        const double roll  = static_cast<double>(s.roll_deg) * kDegToRad;
        const double pitch = static_cast<double>(s.pitch_deg) * kDegToRad;
        const double yaw   = static_cast<double>(s.yaw_deg) * kDegToRad;

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q.normalize();

        msg.orientation.w = q.getW();
        msg.orientation.x = q.getX();
        msg.orientation.y = q.getY();
        msg.orientation.z = q.getZ();

        msg.angular_velocity.x = static_cast<double>(s.gyro_x_dps) * kDegToRad;
        msg.angular_velocity.y = static_cast<double>(s.gyro_y_dps) * kDegToRad;
        msg.angular_velocity.z = static_cast<double>(s.gyro_z_dps) * kDegToRad;

        msg.linear_acceleration.x = static_cast<double>(s.acc_x_g) * kGToMps2;
        msg.linear_acceleration.y = static_cast<double>(s.acc_y_g) * kGToMps2;
        msg.linear_acceleration.z = static_cast<double>(s.acc_z_g) * kGToMps2;

        imu_pub->publish(msg);

        //-----------------------------------------------------------------//
        // Magnetic field                                                 //
        //-----------------------------------------------------------------//
        sensor_msgs::msg::MagneticField mmsg;
        mmsg.header = msg.header;

        mmsg.magnetic_field.x = static_cast<double>(s.mag_x) * mag_scale;
        mmsg.magnetic_field.y = static_cast<double>(s.mag_y) * mag_scale;
        mmsg.magnetic_field.z = static_cast<double>(s.mag_z) * mag_scale;

        mag_pub->publish(mmsg);

        //-----------------------------------------------------------------//
        // Debug output                                                   //
        //-----------------------------------------------------------------//
        if (debug)
        {
          const rclcpp::Time now = clock->now();
          if ((now - t_window).seconds() >= 1.0)
          {
            const uint64_t ok_now = parser.ok();
            const double hz = (ok_now >= last_ok) ? static_cast<double>(ok_now - last_ok) : 0.0;

            RCLCPP_INFO(
              logger,
              "F3 RX ~ %.1f Hz (ok=%lu bad=%lu crc_bad=%lu) "
              "Euler[deg]=[P=%+.2f R=%+.2f Y=%+.2f] "
              "G[dps]=[%+.2f %+.2f %+.2f] "
              "A[g]=[%+.3f %+.3f %+.3f] "
              "M=[%+.3f %+.3f %+.3f] "
              "Temp[C]=%.2f time_us=%u",
              hz,
              static_cast<unsigned long>(parser.ok()),
              static_cast<unsigned long>(parser.bad()),
              static_cast<unsigned long>(parser.crcBad()),
              s.pitch_deg, s.roll_deg, s.yaw_deg,
              s.gyro_x_dps, s.gyro_y_dps, s.gyro_z_dps,
              s.acc_x_g, s.acc_y_g, s.acc_z_g,
              s.mag_x, s.mag_y, s.mag_z,
              s.temperature_deg_c,
              static_cast<unsigned>(s.time_us));

            last_ok = ok_now;
            t_window = now;
          }
        }
      }
    }
    else
    {
      if (debug && parser.ok() == 0)
      {
        RCLCPP_INFO_THROTTLE(logger, *clock, 2000, "F3: waiting for frames on %s ...", port.c_str());
      }
    }

    rclcpp::spin_some(node);
    idle.sleep();
  }

  serial.closeSerial();
  rclcpp::shutdown();
  return 0;
}
