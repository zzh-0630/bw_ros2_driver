/*!
 * \file         can_port.hpp
 * \author       BWSensing
 * \date         2026-01
 * \brief        SocketCAN RAW wrapper (I/O only).
 *
 * Design goals:
 * - Handle CAN I/O (open interface, configure filters, read `can_frame`)
 *
 * Copyright (c) 2026 BWSensing
 * Distributed under the MIT License. See LICENSE for more information.
 */

#ifndef INCLUDE_BW_ROS_DRIVER_CAN_PORT_HPP
#define INCLUDE_BW_ROS_DRIVER_CAN_PORT_HPP

// System headers
#include <linux/can.h>

// Standard headers
#include <cstdint>
#include <string>
#include <vector>

namespace bw
{
class CanPort
{
public:
  explicit CanPort(const std::string& can_interface = "can0");
  ~CanPort();

  /*!
   * \brief Open SocketCAN interface.
   * \param filter_ids If non-empty, only frames with these standard IDs are accepted.
   */
  bool openCan(const std::vector<uint32_t>& filter_ids = {});

  void closeCan();

  bool isOpen() const
  {
    return socket_fd_ >= 0;
  }

  /*!
   * \brief Read one CAN frame with timeout.
   * \return 1 on success, 0 on timeout, -1 on error (errno is set).
   */
  int readFrame(struct can_frame& refOut, int timeout_ms = 1000);

private:
  std::string can_interface_;
  int socket_fd_ = -1;
};

}  // namespace bw

#endif  // INCLUDE_BW_ROS_DRIVER_CAN_PORT_HPP
