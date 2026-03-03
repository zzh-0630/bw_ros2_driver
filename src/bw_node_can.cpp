#include <linux/can.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

// Standard headers
#include <cerrno>
#include <cstring>
#include <string>
#include <vector>

// Project headers
#include "bw_ros2_driver/can_imu_aggregator.hpp"
#include "bw_ros2_driver/can_port.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bw_node_can");
  auto logger = node->get_logger();

  //-------------------------------------------------------------------//
  // Parameters                                                        //
  //-------------------------------------------------------------------//
  const std::string can_iface = node->declare_parameter<std::string>("can_iface", "can0");
  const int64_t can_id_param = node->declare_parameter<int64_t>("can_id", 0x585);
  const int can_id = static_cast<int>(can_id_param);
  const int rx_timeout_ms = static_cast<int>(node->declare_parameter<int64_t>("rx_timeout_ms", 50));

  const std::string imu_topic = node->declare_parameter<std::string>("imu_topic", "/imu/data");
  const std::string frame_id = node->declare_parameter<std::string>("frame_id", "imu_link");
  const bool debug = node->declare_parameter<bool>("debug", false);

  bw::CanImuAggregatorConfig cfg;
  cfg.max_age_ms = static_cast<int>(node->declare_parameter<int64_t>("max_age_ms", 100));
  cfg.prefer_quat = node->declare_parameter<bool>("prefer_quat", true);
  cfg.allow_angle_fallback = node->declare_parameter<bool>("allow_angle_fallback", true);
  cfg.require_acc = node->declare_parameter<bool>("require_acc", true);
  cfg.require_orientation = node->declare_parameter<bool>("require_orientation", true);

  cfg.cov_orientation = node->declare_parameter<double>("cov_orientation", -1.0);
  cfg.cov_angular_velocity = node->declare_parameter<double>("cov_angular_velocity", -1.0);
  cfg.cov_linear_acceleration = node->declare_parameter<double>("cov_linear_acceleration", -1.0);

  cfg.frame_id = frame_id;

  //-------------------------------------------------------------------//
  // Publisher                                                         //
  //-------------------------------------------------------------------//
  auto imu_pub = node->create_publisher<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS());

  //-------------------------------------------------------------------//
  // CAN init                                                          //
  //-------------------------------------------------------------------//
  bw::CanPort can(can_iface);

  const std::vector<uint32_t> filters = { static_cast<uint32_t>(can_id) };

  if (!can.openCan(filters))
  {
    RCLCPP_FATAL(logger, "Failed to open SocketCAN iface=%s: errno=%d (%s)",
                 can_iface.c_str(), errno, std::strerror(errno));
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "bw_node_can started. iface=%s can_id=0x%X imu_topic=%s frame_id=%s",
              can_iface.c_str(), can_id, imu_topic.c_str(), frame_id.c_str());

  //-------------------------------------------------------------------//
  // Aggregator                                                        //
  //-------------------------------------------------------------------//
  bw::CanImuAggregator agg(cfg);

  //-------------------------------------------------------------------//
  // Debug counters                                                    //
  //-------------------------------------------------------------------//
  uint64_t cnt_rx = 0;
  uint64_t cnt_filter = 0;
  uint64_t cnt_pub = 0;
  uint64_t cnt_drop = 0;
  uint64_t cnt_err = 0;

  const auto clock = node->get_clock();
  rclcpp::Time t_stat = clock->now();

  //-------------------------------------------------------------------//
  // Main loop                                                         //
  //-------------------------------------------------------------------//
  while (rclcpp::ok())
  {
    struct can_frame fr;
    const int r = can.readFrame(fr, rx_timeout_ms);

    if (r < 0)
    {
      ++cnt_err;
      RCLCPP_WARN_THROTTLE(logger, *clock, 1000, "CAN read error: errno=%d (%s)", errno, std::strerror(errno));
      rclcpp::spin_some(node);
      continue;
    }

    if (r == 0)
    {
      rclcpp::spin_some(node);
      continue;
    }

    ++cnt_rx;

    // Accept only standard frames with DLC=8.
    if ((fr.can_id & CAN_EFF_FLAG) != 0)
    {
      ++cnt_filter;
      continue;
    }

    const uint32_t id = fr.can_id & CAN_SFF_MASK;
    if (static_cast<int>(id) != can_id || fr.can_dlc != 8)
    {
      ++cnt_filter;
      continue;
    }

    const rclcpp::Time stamp = clock->now();

    sensor_msgs::msg::Imu msg;
    if (agg.ingestAndMaybeBuildImu(fr.data, stamp, msg))
    {
      imu_pub->publish(msg);
      ++cnt_pub;
    }
    else
    {
      // Expected: most frames are intermediate and do not publish.
      ++cnt_drop;
    }

    if (debug)
    {
      const rclcpp::Time now = clock->now();
      if ((now - t_stat).seconds() >= 1.0)
      {
        RCLCPP_INFO(logger, "stats: rx=%lu pub=%lu drop=%lu filter=%lu err=%lu quat_complete=%s",
                    static_cast<unsigned long>(cnt_rx),
                    static_cast<unsigned long>(cnt_pub),
                    static_cast<unsigned long>(cnt_drop),
                    static_cast<unsigned long>(cnt_filter),
                    static_cast<unsigned long>(cnt_err),
                    agg.hasQuatComplete() ? "true" : "false");
        t_stat = now;
      }
    }

    rclcpp::spin_some(node);
  }

  can.closeCan();
  rclcpp::shutdown();
  return 0;
}
