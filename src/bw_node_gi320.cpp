#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <string>
#include <vector>

#include "bw_ros2_driver/inspva_parser.hpp"
#include "bw_ros2_driver/msg/bewis_inspva.hpp"
#include "bw_ros2_driver/serial_port.hpp"

namespace
{
int navSatStatusFromPosStatus(uint8_t pos_status)
{
  switch (pos_status)
  {
    case 0:
      return sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    default:
      return sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  }
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bw_node_gi320");
  auto logger = node->get_logger();

  const std::string port = node->declare_parameter<std::string>("port", "/dev/ttyUSB1");
  const int baud = static_cast<int>(node->declare_parameter<int64_t>("baud", 115200));

  const std::string frame_id = node->declare_parameter<std::string>("frame_id", "gps_link");
  const std::string velocity_frame_id =
    node->declare_parameter<std::string>("velocity_frame_id", "enu");

  const std::string raw_topic =
    node->declare_parameter<std::string>("raw_topic", "/gi320/raw");
  const std::string navsat_topic =
    node->declare_parameter<std::string>("navsat_topic", "/gi320/fix");
  const std::string velocity_topic =
    node->declare_parameter<std::string>("velocity_topic", "/gi320/velocity");

  const bool publish_raw = node->declare_parameter<bool>("publish_raw", true);
  const bool publish_navsat = node->declare_parameter<bool>("publish_navsat", true);
  const bool publish_velocity = node->declare_parameter<bool>("publish_velocity", true);
  const bool debug = node->declare_parameter<bool>("debug", true);
  const double publish_rate = node->declare_parameter<double>("publish_rate", 10);

  bw::SerialPort serial(port, baud);
  if (!serial.openSerial())
  {
    RCLCPP_FATAL(logger, "Failed to open serial port %s", port.c_str());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Opened %s @ %d, publishing INSPVA raw=%s fix=%s vel=%s",
              port.c_str(), baud, raw_topic.c_str(), navsat_topic.c_str(), velocity_topic.c_str());

  rclcpp::Publisher<bw_ros2_driver::msg::BewisInspva>::SharedPtr raw_pub;
  if (publish_raw)
  {
    raw_pub = node->create_publisher<bw_ros2_driver::msg::BewisInspva>(raw_topic, rclcpp::SensorDataQoS());
  }

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_pub;
  if (publish_navsat)
  {
    navsat_pub = node->create_publisher<sensor_msgs::msg::NavSatFix>(navsat_topic, rclcpp::SensorDataQoS());
  }

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub;
  if (publish_velocity)
  {
    vel_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>(velocity_topic, rclcpp::SensorDataQoS());
  }

  bw::InspvaParser parser;

  const auto clock = node->get_clock();
  rclcpp::Time t_window = clock->now();
  uint64_t last_ok = 0;
  bw::InspvaSample last_sample;
  bool have_last_sample = false;

  rclcpp::Rate idle(std::max(1.0, publish_rate));

  while (rclcpp::ok())
  {
    uint8_t buffer[1024];
    const ssize_t n = serial.readSome(buffer, sizeof(buffer));

    if (n < 0 && errno != EAGAIN)
    {
      if (debug)
      {
        RCLCPP_WARN_THROTTLE(logger, *clock, 1000,
                             "INSPVA serial read error: errno=%d (%s)", errno, std::strerror(errno));
      }
    }

    if (n > 0)
    {
      std::vector<bw::InspvaSample> samples;
      samples.reserve(8);
      const size_t m = parser.feed(buffer, static_cast<size_t>(n), samples);

      for (size_t i = 0; i < m; ++i)
      {
        const bw::InspvaSample& s = samples[i];
        const auto stamp = clock->now();

        if (publish_raw)
        {
          bw_ros2_driver::msg::BewisInspva msg;
          msg.header.stamp = stamp;
          msg.header.frame_id = frame_id;
          msg.week = s.week;
          msg.seconds = s.seconds;
          msg.longitude = s.longitude;
          msg.latitude = s.latitude;
          msg.height = s.height;
          msg.north_velocity = s.north_velocity;
          msg.east_velocity = s.east_velocity;
          msg.down_velocity = s.down_velocity;
          msg.up_velocity = s.up_velocity;
          msg.roll = s.roll_deg;
          msg.pitch = s.pitch_deg;
          msg.azimuth = s.azimuth_deg;
          msg.ax = s.ax;
          msg.ay = s.ay;
          msg.az = s.az;
          msg.gx = s.gx;
          msg.gy = s.gy;
          msg.gz = s.gz;
          msg.sats = s.sats;
          msg.pos_status = s.pos_status;
          msg.azi_status = s.azi_status;
          msg.ins_status = s.ins_status;
          msg.temperature = s.temperature_deg_c;
          msg.pdop = s.pdop;
          raw_pub->publish(msg);
        }

        if (publish_navsat)
        {
          sensor_msgs::msg::NavSatFix fix;
          fix.header.stamp = stamp;
          fix.header.frame_id = frame_id;
          fix.status.status = navSatStatusFromPosStatus(s.pos_status);
          fix.status.service = 0;
          fix.latitude = s.latitude;
          fix.longitude = s.longitude;
          fix.altitude = s.height;
          fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
          navsat_pub->publish(fix);
        }

        if (publish_velocity)
        {
          geometry_msgs::msg::TwistStamped vel;
          vel.header.stamp = stamp;
          vel.header.frame_id = velocity_frame_id;
          vel.twist.linear.x = s.east_velocity;
          vel.twist.linear.y = s.north_velocity;
          vel.twist.linear.z = s.up_velocity;
          vel_pub->publish(vel);
        }

        last_sample = s;
        have_last_sample = true;
      }
    }
    else if (debug && parser.ok() == 0)
    {
      RCLCPP_INFO_THROTTLE(logger, *clock, 2000, "INSPVA: waiting for frames on %s ...", port.c_str());
    }

    if (debug)
    {
      const rclcpp::Time now = clock->now();
      if ((now - t_window).seconds() >= 1.0)
      {
        const uint64_t ok_now = parser.ok();
        const double hz = (ok_now >= last_ok) ? static_cast<double>(ok_now - last_ok) : 0.0;

        if (have_last_sample)
        {
          RCLCPP_INFO(
            logger,
            "INSPVAA RX ~ %.1f Hz (ok=%lu bad=%lu crc_bad=%lu) "
            "Week=[%lu] Seconds=[%.3f] "
            "LLH=[%.9f %.9f %.5f] "
            "V_NED=[%+.5f %+.5f %+.5f] "
            "RPA=[%+.5f %+.5f %+.5f] "
            "Axyz=[%+.5f %+.5f %+.5f] "
            "Gxyz=[%+.5f %+.5f %+.5f] "
            "Sats=%u PosStatus=%u AziStatus=%u Temp=%.1f PDOP=%.2f INSStatus=%u",
            hz,
            static_cast<unsigned long>(parser.ok()),
            static_cast<unsigned long>(parser.bad()),
            static_cast<unsigned long>(parser.crcBad()),
            static_cast<unsigned long>(last_sample.week),
            last_sample.seconds,
            last_sample.longitude,
            last_sample.latitude,
            last_sample.height,
            last_sample.north_velocity,
            last_sample.east_velocity,
            last_sample.down_velocity,
            last_sample.roll_deg,
            last_sample.pitch_deg,
            last_sample.azimuth_deg,
            last_sample.ax,
            last_sample.ay,
            last_sample.az,
            last_sample.gx,
            last_sample.gy,
            last_sample.gz,
            last_sample.sats,
            last_sample.pos_status,
            last_sample.azi_status,
            last_sample.temperature_deg_c,
            last_sample.pdop,
            last_sample.ins_status);
        }
        else
        {
          RCLCPP_INFO(
            logger,
            "INSPVA: waiting for frames on %s ... (ok=%lu bad=%lu crc_bad=%lu)",
            port.c_str(),
            static_cast<unsigned long>(parser.ok()),
            static_cast<unsigned long>(parser.bad()),
            static_cast<unsigned long>(parser.crcBad()));
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