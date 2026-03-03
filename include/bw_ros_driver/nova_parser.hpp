/*!
 * \file         nova_parser.hpp
 * \author       BWSensing
 * \date         2025-11-22
 * \brief        Nova (0xF3) protocol parser.
 *
 * Frame format (automatic output):
 *   Head(0xF3) | CmdID | DataLen | Count | CRC8 | Data | CheckSum
 *
 * Copyright (c) 2025 BWSensing
 * Distributed under the MIT License. See LICENSE for more information.
 */

#ifndef INCLUDE_BW_ROS_DRIVER_NOVA_PARSER_HPP
#define INCLUDE_BW_ROS_DRIVER_NOVA_PARSER_HPP

// Standard headers
#include <cstddef>
#include <cstdint>
#include <vector>

namespace bw
{
//---------------------------------------------------------------------//
//- Parsed sample                                                      -//
//---------------------------------------------------------------------//
struct F3Sample
{
  bool valid = false;

  uint8_t cmd_id = 0;
  uint8_t count  = 0;

  float pitch_deg = 0.0f;
  float roll_deg  = 0.0f;
  float yaw_deg   = 0.0f;

  float acc_x_g = 0.0f;
  float acc_y_g = 0.0f;
  float acc_z_g = 0.0f;

  float gyro_x_dps = 0.0f;
  float gyro_y_dps = 0.0f;
  float gyro_z_dps = 0.0f;

  float mag_x = 0.0f;
  float mag_y = 0.0f;
  float mag_z = 0.0f;

  float temperature_deg_c = 0.0f;

  uint32_t time_us = 0;
};

//---------------------------------------------------------------------//
//- Nova (F3) protocol parser                                          -//
//---------------------------------------------------------------------//
class NovaParser
{
public:
  NovaParser();

  void reset();

  /*!
   * \brief Feed raw bytes into the parser and extract complete samples.
   * \return Number of samples appended to \p ref_out.
   */
  size_t feed(const uint8_t* p_data, size_t size, std::vector<F3Sample>& ref_out);

  uint64_t ok() const
  {
    return ok_cnt_;
  }

  uint64_t bad() const
  {
    return bad_cnt_;
  }

  uint64_t crcBad() const
  {
    return crc_bad_cnt_;
  }

private:
  enum class State
  {
    WaitHead,
    ReadHeader,
    ReadFrame
  };

  static uint8_t computeCrc8(const uint8_t* p_data, size_t size);

  bool parseFrame(const std::vector<uint8_t>& ref_frame, F3Sample& ref_sample);

  State state_ = State::WaitHead;

  std::vector<uint8_t> buf_;
  size_t expected_len_ = 0;

  uint64_t ok_cnt_      = 0;
  uint64_t bad_cnt_     = 0;
  uint64_t crc_bad_cnt_ = 0;
};

}  // namespace bw

#endif  // INCLUDE_BW_ROS_DRIVER_NOVA_PARSER_HPP
