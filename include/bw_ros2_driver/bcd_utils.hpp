/*!
 * \file         bcd_utils.hpp
 * \author       BWSensing
 * \date         2025-10-15
 * \brief        BCD decoding and checksum helpers (internal).
 *
 * Copyright (c) 2025 BWSensing
 * Distributed under the MIT License. See LICENSE for more information.
 */

#ifndef INCLUDE_BW_ROS_DRIVER_BCD_UTILS_HPP
#define INCLUDE_BW_ROS_DRIVER_BCD_UTILS_HPP

// Standard headers
#include <cstddef>
#include <cstdint>

namespace bw
{
/*!
 * \brief 8-bit additive checksum used by the standard (0x77) protocol.
 */
inline uint8_t sum8(const uint8_t* p_data, size_t size)
{
  unsigned sum = 0;

  for (size_t i = 0; i < size; ++i)
  {
    sum += p_data[i];
  }

  return static_cast<uint8_t>(sum & 0xFFu);
}

//---------------------------------------------------------------------//
//- BCD decode helpers                                                 -//
//---------------------------------------------------------------------//

/*!
 * \brief Decode a 3-byte BCD angle / angular rate value: SXXXYY.
 *
 * - Sign: high nibble of b0 (0=+, 1=-)
 * - Integer: low nibble of b0, then b1 high/low nibbles (XXX)
 * - Fraction: b2 high/low nibbles (YY) => /100
 */
inline double bcd3_angle_or_dps(uint8_t b0, uint8_t b1, uint8_t b2)
{
  const int sign = (b0 >> 4) & 0x1;
  const int d2   = (b0 & 0x0F);
  const int d1   = (b1 >> 4) & 0x0F;
  const int d0   = (b1 & 0x0F);
  const int c1   = (b2 >> 4) & 0x0F;
  const int c2   = (b2 & 0x0F);

  const double value = (d2 * 100 + d1 * 10 + d0) + (c1 * 10 + c2) / 100.0;

  return sign ? -value : value;
}

/*!
 * \brief Decode a 3-byte BCD acceleration value: S X.YYYY.
 */
inline double bcd3_acc_g(uint8_t b0, uint8_t b1, uint8_t b2)
{
  const int sign = (b0 >> 4) & 0x1;
  const int i    = (b0 & 0x0F);
  const int f1   = (b1 >> 4) & 0x0F;
  const int f2   = (b1 & 0x0F);
  const int f3   = (b2 >> 4) & 0x0F;
  const int f4   = (b2 & 0x0F);

  const double value = i + (f1 * 1000 + f2 * 100 + f3 * 10 + f4) / 10000.0;

  return sign ? -value : value;
}

/*!
 * \brief Decode a 3-byte BCD magnetic value: S 0.YYYYY.
 *
 * Unit is defined by the caller (e.g. gauss/uT converted at publish stage).
 */
inline double bcd3_mag_frac5(uint8_t b0, uint8_t b1, uint8_t b2)
{
  const int sign = (b0 >> 4) & 0x1;
  const int f1   = (b0 & 0x0F);
  const int f2   = (b1 >> 4) & 0x0F;
  const int f3   = (b1 & 0x0F);
  const int f4   = (b2 >> 4) & 0x0F;
  const int f5   = (b2 & 0x0F);

  const double value = (f1 * 10000 + f2 * 1000 + f3 * 100 + f4 * 10 + f5) / 100000.0;

  return sign ? -value : value;
}

/*!
 * \brief Decode a 4-byte BCD quaternion component: S X.YYYYYY.
 */
inline double bcd4_q_to_double(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3)
{
  const int sign = (b0 >> 4) & 0x1;
  const int i    = (b0 & 0x0F);

  const int y1 = (b1 >> 4) & 0x0F;
  const int y2 = (b1 & 0x0F);
  const int y3 = (b2 >> 4) & 0x0F;
  const int y4 = (b2 & 0x0F);
  const int y5 = (b3 >> 4) & 0x0F;
  const int y6 = (b3 & 0x0F);

  const double frac = (y1 * 100000 + y2 * 10000 + y3 * 1000 + y4 * 100 + y5 * 10 + y6) / 1e6;
  const double value = i + frac;

  return sign ? -value : value;
}

}  // namespace bw

#endif  // INCLUDE_BW_ROS_DRIVER_BCD_UTILS_HPP