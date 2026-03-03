/*!
 * \file         standard_parser.hpp
 * \author       BWSensing
 * \date         2025-10-15
 * \brief        Standard (0x77) protocol parser.
 *
 * Frame format (automatic output):
 *   0x77 | LEN | ADDR | CMD | DATA... | CHK
 *
 * Copyright (c) 2025 BWSensing
 * Distributed under the MIT License. See LICENSE for more information.
 */

#ifndef BW_ROS_DRIVER_STANDARD_PARSER_HPP
#define BW_ROS_DRIVER_STANDARD_PARSER_HPP

// Standard headers
#include <cstddef>
#include <cstdint>
#include <vector>

// Project headers
#include "bw_ros_driver/bcd_utils.hpp"

namespace bw
{
//---------------------------------------------------------------------//
//- Parsed sample                                                      -//
//---------------------------------------------------------------------//

/*!
 * \brief One decoded sample extracted from a standard protocol frame.
 *
 * The `has_*` flags indicate which fields are valid for this sample.
 */
struct DataSample
{
  bool has_euler = false;
  bool has_acc   = false;
  bool has_gyro  = false;
  bool has_mag   = false;
  bool has_quat  = false;

  // Euler angles [deg] (P: pitch, R: roll, Y: yaw)
  double P = 0.0;
  double R = 0.0;
  double Y = 0.0;

  // Linear acceleration [g]
  double ax_g = 0.0;
  double ay_g = 0.0;
  double az_g = 0.0;

  // Angular velocity [deg/s]
  double gx_dps = 0.0;
  double gy_dps = 0.0;
  double gz_dps = 0.0;

  // Magnetic field (unit depends on device output)
  double mx = 0.0;
  double my = 0.0;
  double mz = 0.0;

  // Orientation quaternion (w, x, y, z)
  double q0 = 1.0;
  double q1 = 0.0;
  double q2 = 0.0;
  double q3 = 0.0;
};

//---------------------------------------------------------------------//
//- Standard protocol parser                                          -//
//---------------------------------------------------------------------//
class StandardParser
{
public:
  StandardParser();

  void reset();

  /*!
   * \brief Feed raw bytes into the parser and extract complete samples.
   *
   * The method accepts arbitrary chunks (e.g. partial serial reads).
   * Any complete frames found in the input are decoded and appended to \p ref_out.
   *
   * \return Number of samples appended to \p ref_out.
   */
  size_t feed(const uint8_t* p_data, size_t size, std::vector<DataSample>& ref_out);

  uint64_t ok() const
  {
    return ok_cnt_;
  }

  uint64_t bad() const
  {
    return bad_cnt_;
  }

private:
  enum class State
  {
    Wait77,
    ReadLen,
    ReadPayload
  };

  bool onFrame(DataSample& ref_out);

  State st_ = State::Wait77;

  std::vector<uint8_t> buf_;
  uint8_t need_ = 0;

  uint64_t ok_cnt_  = 0;
  uint64_t bad_cnt_ = 0;
};

}  // namespace bw

#endif  // BW_ROS_DRIVER_STANDARD_PARSER_HPP