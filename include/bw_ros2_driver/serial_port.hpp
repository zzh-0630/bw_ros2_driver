/*!
 * \file         serial_port.hpp
 * \author       BWSensing
 * \date         2025-10-15
 * \brief        Serial port wrapper used by bw_ros_driver.
 *
 * This class is responsible for UART I/O only (open/configure/read/write).
 * Protocol parsing is handled by dedicated parser classes.
 *
 * Copyright (c) 2025 BWSensing
 * Distributed under the MIT License. See LICENSE for more information.
 */

#ifndef INCLUDE_BW_ROS_DRIVER_SERIAL_PORT_HPP
#define INCLUDE_BW_ROS_DRIVER_SERIAL_PORT_HPP

// Standard headers
#include <cstddef>
#include <cstdint>
#include <string>

// System headers
#include <sys/types.h>
#include <termios.h>

namespace bw
{
//---------------------------------------------------------------------//
//- Serial port wrapper                                                -//
//---------------------------------------------------------------------//
class SerialPort
{
public:
  explicit SerialPort(const std::string& port = "/dev/ttyUSB0", int baud = 9600);
  ~SerialPort();

  bool openSerial();
  void closeSerial();
  bool reOpenSerial();

  bool isOpen() const
  {
    return fd_ >= 0;
  }

  int fd() const
  {
    return fd_;
  }

  bool setBaud(int baud);

  int getBaud() const
  {
    return baud_;
  }

  void setPort(const std::string& port)
  {
    port_ = port;
  }

  const std::string& getPort() const
  {
    return port_;
  }

  /*!
   * \brief Write exactly \p len bytes to the serial port.
   * \return Number of bytes written, or -1 on error.
   */
  ssize_t writeAll(const uint8_t* p_data, size_t len);

  /*!
   * \brief Read up to \p max bytes from the serial port.
   * \return Number of bytes read, 0 on timeout, or -1 on error.
   */
  ssize_t readSome(uint8_t* p_buffer, size_t max);

private:
  //-------------------------------------------------------------------//
  //- Private methods                                                  -//
  //-------------------------------------------------------------------//
  bool configureTermios();
  static speed_t mapBaud(int baud);

  //-------------------------------------------------------------------//
  //- Private members                                                  -//
  //-------------------------------------------------------------------//
  std::string port_;
  int baud_;
  int fd_;
};

}  // namespace bw

#endif  // INCLUDE_BW_ROS_DRIVER_SERIAL_PORT_HPP