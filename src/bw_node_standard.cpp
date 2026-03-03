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
#include "bw_ros2_driver/serial_port.hpp"
#include "bw_ros2_driver/standard_parser.hpp"

namespace
{
double magScaleToTesla(const std::string& unit)
{
  // Default assumes already in tesla.
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
  auto node = std::make_shared<rclcpp::Node>("bw_node_standard");
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

  const double publish_rate = node->declare_parameter<double>("publish_rate", 200.0);
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

  RCLCPP_INFO(logger, "Opened %s @ %d, publishing IMU=%s MAG=%s",
              port.c_str(), baud, imu_topic.c_str(), mag_topic.c_str());

  //-------------------------------------------------------------------//
  // Publishers                                                        //
  //-------------------------------------------------------------------//
  auto imu_pub = node->create_publisher<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS());
  auto mag_pub = node->create_publisher<sensor_msgs::msg::MagneticField>(mag_topic, rclcpp::SensorDataQoS());

  //-------------------------------------------------------------------//
  // Parser                                                            //
  //-------------------------------------------------------------------//
  bw::StandardParser parser;

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

  bw::DataSample last_sample;
  bool last_sample_valid = false;

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
        RCLCPP_WARN_THROTTLE(logger, *clock, 1000, "serial read error: errno=%d (%s)", errno, std::strerror(errno));
      }
    }

    if (n > 0)
    {
      std::vector<bw::DataSample> samples;
      samples.reserve(8);

      const size_t m = parser.feed(buffer, static_cast<size_t>(n), samples);

      for (size_t i = 0; i < m; ++i)
      {
        const bw::DataSample& s = samples[i];

        //-----------------------------------------------------------------//
        // Build IMU message                                               //
        //-----------------------------------------------------------------//
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = clock->now();
        imu_msg.header.frame_id = frame_id;

        fillCovariance(cov_orientation, imu_msg.orientation_covariance);
        fillCovariance(cov_angular_velocity, imu_msg.angular_velocity_covariance);
        fillCovariance(cov_linear_acceleration, imu_msg.linear_acceleration_covariance);

        // Orientation
        if (s.has_quat)
        {
          imu_msg.orientation.w = s.q0;
          imu_msg.orientation.x = s.q1;
          imu_msg.orientation.y = s.q2;
          imu_msg.orientation.z = s.q3;
        }
        else if (s.has_euler)
        {
          const double roll  = s.R * kDegToRad;
          const double pitch = s.P * kDegToRad;
          const double yaw   = s.Y * kDegToRad;

          tf2::Quaternion q;
          q.setRPY(roll, pitch, yaw);
          q.normalize();

          imu_msg.orientation.w = q.getW();
          imu_msg.orientation.x = q.getX();
          imu_msg.orientation.y = q.getY();
          imu_msg.orientation.z = q.getZ();
        }
        else
        {
          imu_msg.orientation_covariance[0] = -1.0;
        }

        // Angular velocity [deg/s] -> [rad/s]
        if (s.has_gyro)
        {
          imu_msg.angular_velocity.x = s.gx_dps * kDegToRad;
          imu_msg.angular_velocity.y = s.gy_dps * kDegToRad;
          imu_msg.angular_velocity.z = s.gz_dps * kDegToRad;
        }
        else
        {
          imu_msg.angular_velocity_covariance[0] = -1.0;
        }

        // Linear acceleration [g] -> [m/s^2]
        if (s.has_acc)
        {
          imu_msg.linear_acceleration.x = s.ax_g * kGToMps2;
          imu_msg.linear_acceleration.y = s.ay_g * kGToMps2;
          imu_msg.linear_acceleration.z = s.az_g * kGToMps2;
        }
        else
        {
          imu_msg.linear_acceleration_covariance[0] = -1.0;
        }

        imu_pub->publish(imu_msg);

        //-----------------------------------------------------------------//
        // Magnetic field (optional)                                      //
        //-----------------------------------------------------------------//
        if (s.has_mag)
        {
          sensor_msgs::msg::MagneticField mag_msg;
          mag_msg.header = imu_msg.header;

          mag_msg.magnetic_field.x = s.mx * mag_scale;
          mag_msg.magnetic_field.y = s.my * mag_scale;
          mag_msg.magnetic_field.z = s.mz * mag_scale;

          mag_pub->publish(mag_msg);
        }

        last_sample = s;
        last_sample_valid = true;
      }
    }

    //-------------------------------------------------------------------//
    // Debug output                                                      //
    //-------------------------------------------------------------------//
    if (debug)
    {
      const rclcpp::Time now = clock->now();
      if ((now - t_window).seconds() >= 1.0)
      {
        const uint64_t ok_now = parser.ok();
        const uint64_t bad_now = parser.bad();
        const double hz = (ok_now >= last_ok) ? static_cast<double>(ok_now - last_ok) : 0.0;

        if (last_sample_valid)
        {
          RCLCPP_INFO(
            logger,
            "STD RX ~ %.1f Hz (ok=%lu bad=%lu) "
            "Euler[deg]=[P=%+.2f R=%+.2f Y=%+.2f] "
            "G[dps]=[%+.2f %+.2f %+.2f] "
            "A[g]=[%+.3f %+.3f %+.3f] "
            "M=[%+.5f %+.5f %+.5f] "
            "q=[%+.6f %+.6f %+.6f %+.6f]",
            hz,
            static_cast<unsigned long>(ok_now),
            static_cast<unsigned long>(bad_now),
            last_sample.P, last_sample.R, last_sample.Y,
            last_sample.gx_dps, last_sample.gy_dps, last_sample.gz_dps,
            last_sample.ax_g, last_sample.ay_g, last_sample.az_g,
            last_sample.mx, last_sample.my, last_sample.mz,
            last_sample.q0, last_sample.q1, last_sample.q2, last_sample.q3);
        }
        else
        {
          RCLCPP_INFO(logger, "STD: waiting for frames on %s ... (ok=%lu bad=%lu)",
                      port.c_str(),
                      static_cast<unsigned long>(ok_now),
                      static_cast<unsigned long>(bad_now));
        }

        last_ok = ok_now;
        t_window = now;
      }
    }

    rclcpp::spin_some(node);
    idle.sleep();
  }

  serial.closeSerial();
  rclcpp::shutdown();
  return 0;
}
