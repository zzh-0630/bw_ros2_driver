/*!
 * \file         can_decode_utils.hpp
 * \author       BWSensing
 * \date         2026-02
 * \brief        Helpers for BW private CAN payload decoding.
 *
 * NOTE:
 * This decoder implements a "single-ID + payload-mux" private format used by some firmwares.
 * If your device uses the DBC-defined multi-ID layout (e.g. 0x585/0x586/0x587), you should
 * replace these helpers with a DBC/bitfield based decoder.
 *
 * Copyright (c) 2026 BWSensing
 * Distributed under the MIT License. See LICENSE for more information.
 */

#ifndef INCLUDE_BW_ROS_DRIVER_CAN_DECODE_UTILS_HPP
#define INCLUDE_BW_ROS_DRIVER_CAN_DECODE_UTILS_HPP

// Standard headers
#include <cmath>
#include <cstdint>

namespace bw
{
//---------------------------------------------------------------------//
//- BCD helpers                                                        -//
//---------------------------------------------------------------------//
inline bool isBcdByte(uint8_t byte)
{
  return ((byte >> 4) <= 9) && ((byte & 0x0F) <= 9);
}

inline bool bcd4ToInt(uint8_t b0, uint8_t b1, int& refOut)
{
  // 2 bytes => 4 digits, e.g. 0x50 0x06 => 5006
  if (!isBcdByte(b0) || !isBcdByte(b1))
  {
    return false;
  }

  refOut = ((b0 >> 4) * 1000) + ((b0 & 0x0F) * 100) + ((b1 >> 4) * 10) + (b1 & 0x0F);
  return true;
}

//---------------------------------------------------------------------//
//- Frame kind                                                         -//
//---------------------------------------------------------------------//
enum class CanFrameKind
{
  Angle,
  Acc,
  Gyro,
  Quat2Comp,  // One CAN frame contains 2 quaternion components (4 bytes each)
  Unknown
};

//---------------------------------------------------------------------//
//- Angle decode                                                       -//
//---------------------------------------------------------------------//
// Pitch/Roll: 3 bytes represent SXXXYY
//  - S: sign digit (0 = positive, 1 = negative) [stored in high nibble of byte0]
//  - XXX: integer part (3 digits): [low nibble of byte0][high nibble of byte1][low nibble of byte1]
//  - YY: decimals (2 digits): [high nibble of byte2][low nibble of byte2]
inline bool decodeSxxxYy3Bytes(const uint8_t p[3], double& refOutDeg)
{
  if (!isBcdByte(p[0]) || !isBcdByte(p[1]) || !isBcdByte(p[2]))
  {
    return false;
  }

  const int sign = (p[0] >> 4);
  if (sign != 0 && sign != 1)
  {
    return false;
  }

  const int hundreds = (p[0] & 0x0F);
  const int tens     = (p[1] >> 4);
  const int ones     = (p[1] & 0x0F);
  const int dec1     = (p[2] >> 4);
  const int dec2     = (p[2] & 0x0F);

  const int int_part = hundreds * 100 + tens * 10 + ones;
  const int frac     = dec1 * 10 + dec2;

  double value = static_cast<double>(int_part) + static_cast<double>(frac) / 100.0;
  if (sign == 1)
  {
    value = -value;
  }

  refOutDeg = value;
  return true;
}

// Yaw: 2 bytes, no decimal digits in raw. Firmware scale is 0.1 deg.
inline bool decodeYaw2BytesBcd(const uint8_t p[2], int& refOutRaw)
{
  if (!isBcdByte(p[0]) || !isBcdByte(p[1]))
  {
    return false;
  }

  refOutRaw = ((p[0] >> 4) * 1000) + ((p[0] & 0x0F) * 100) + ((p[1] >> 4) * 10) + (p[1] & 0x0F);
  return true;
}

struct AngleData
{
  double pitch_deg = 0.0;
  double roll_deg  = 0.0;
  double yaw_deg   = 0.0;  // yaw_raw * 0.1
  int yaw_raw      = 0;    // 4 digits integer
};

inline bool decodeAngleFrame(const uint8_t d[8], AngleData& refOut)
{
  double pitch = 0.0;
  double roll  = 0.0;
  int yaw_raw  = 0;

  if (!decodeSxxxYy3Bytes(&d[0], pitch))
  {
    return false;
  }

  if (!decodeSxxxYy3Bytes(&d[3], roll))
  {
    return false;
  }

  if (!decodeYaw2BytesBcd(&d[6], yaw_raw))
  {
    return false;
  }

  refOut.pitch_deg = pitch;
  refOut.roll_deg  = roll;
  refOut.yaw_raw   = yaw_raw;
  refOut.yaw_deg   = static_cast<double>(yaw_raw) * 0.1;

  return true;
}

//---------------------------------------------------------------------//
//- Acc decode                                                         -//
//---------------------------------------------------------------------//
// Format: [0]=0x54, [1]=0x00, [2..3]=X, [4..5]=Y, [6..7]=Z (each is 4-digit BCD integer v)
// Acc[g] = (v - 5000) / 2500
struct AccData
{
  double ax_g = 0.0;
  double ay_g = 0.0;
  double az_g = 0.0;

  int raw_x = 0;
  int raw_y = 0;
  int raw_z = 0;
};

inline bool decodeAccFrame(const uint8_t d[8], AccData& refOut)
{
  if (d[0] != 0x54 || d[1] != 0x00)
  {
    return false;
  }

  int vx = 0;
  int vy = 0;
  int vz = 0;

  if (!bcd4ToInt(d[2], d[3], vx))
  {
    return false;
  }

  if (!bcd4ToInt(d[4], d[5], vy))
  {
    return false;
  }

  if (!bcd4ToInt(d[6], d[7], vz))
  {
    return false;
  }

  refOut.raw_x = vx;
  refOut.raw_y = vy;
  refOut.raw_z = vz;

  refOut.ax_g = (static_cast<double>(vx) - 5000.0) / 2500.0;
  refOut.ay_g = (static_cast<double>(vy) - 5000.0) / 2500.0;
  refOut.az_g = (static_cast<double>(vz) - 5000.0) / 2500.0;

  return true;
}

//---------------------------------------------------------------------//
//- Gyro decode                                                        -//
//---------------------------------------------------------------------//
// Format: [0]=0x50, [1]=0x00, [2..3]=X, [4..5]=Y, [6..7]=Z (each is 4-digit BCD integer v)
// Gyro[dps] = (v - 5000) / 10
struct GyroData
{
  double gx_dps = 0.0;
  double gy_dps = 0.0;
  double gz_dps = 0.0;

  int raw_x = 0;
  int raw_y = 0;
  int raw_z = 0;
};

inline bool decodeGyroFrame(const uint8_t d[8], GyroData& refOut)
{
  if (d[0] != 0x50 || d[1] != 0x00)
  {
    return false;
  }

  int vx = 0;
  int vy = 0;
  int vz = 0;

  if (!bcd4ToInt(d[2], d[3], vx))
  {
    return false;
  }

  if (!bcd4ToInt(d[4], d[5], vy))
  {
    return false;
  }

  if (!bcd4ToInt(d[6], d[7], vz))
  {
    return false;
  }

  refOut.raw_x = vx;
  refOut.raw_y = vy;
  refOut.raw_z = vz;

  refOut.gx_dps = (static_cast<double>(vx) - 5000.0) / 10.0;
  refOut.gy_dps = (static_cast<double>(vy) - 5000.0) / 10.0;
  refOut.gz_dps = (static_cast<double>(vz) - 5000.0) / 10.0;

  return true;
}

//---------------------------------------------------------------------//
//- Quaternion decode (2 components per frame)                          -//
//---------------------------------------------------------------------//
struct QuatComp
{
  int order   = -1;   // 0..3 for q0..q3
  double value = 0.0; // decoded component value
};

// 4 bytes: p[0] high nibble = order (0..3), low nibble = sign (0=+,1=-)
// p[1] high nibble = integer digit X (0/1), remaining 5 digits are decimals
inline bool decodeQuatComp4Bytes(const uint8_t p[4], QuatComp& refOut)
{
  if (!isBcdByte(p[0]) || !isBcdByte(p[1]) || !isBcdByte(p[2]) || !isBcdByte(p[3]))
  {
    return false;
  }

  const int order = (p[0] >> 4);
  const int sign  = (p[0] & 0x0F);

  if (order < 0 || order > 3)
  {
    return false;
  }
  if (sign != 0 && sign != 1)
  {
    return false;
  }

  const int X  = (p[1] >> 4);
  const int d1 = (p[1] & 0x0F);
  const int d2 = (p[2] >> 4);
  const int d3 = (p[2] & 0x0F);
  const int d4 = (p[3] >> 4);
  const int d5 = (p[3] & 0x0F);

  if (X < 0 || X > 1)
  {
    return false;
  }

  const int dec = d1 * 10000 + d2 * 1000 + d3 * 100 + d4 * 10 + d5;
  double value  = static_cast<double>(X) + static_cast<double>(dec) / 100000.0;

  if (sign == 1)
  {
    value = -value;
  }

  if (std::fabs(value) > 1.05)
  {
    return false;
  }

  refOut.order = order;
  refOut.value = value;
  return true;
}

inline bool decodeQuatFrame2Comp(const uint8_t d[8], QuatComp& refA, QuatComp& refB)
{
  if (!decodeQuatComp4Bytes(&d[0], refA))
  {
    return false;
  }

  if (!decodeQuatComp4Bytes(&d[4], refB))
  {
    return false;
  }

  // Expect pair {0,1} or {2,3} (order may be swapped inside the frame)
  const int mn = (refA.order < refB.order) ? refA.order : refB.order;
  const int mx = (refA.order < refB.order) ? refB.order : refA.order;

  if (!((mn == 0 && mx == 1) || (mn == 2 && mx == 3)))
  {
    return false;
  }

  return true;
}

//---------------------------------------------------------------------//
//- Kind detection (payload mux)                                       -//
//---------------------------------------------------------------------//
inline CanFrameKind detectKindMux0x585(const uint8_t d[8])
{
  if (d[0] == 0x54 && d[1] == 0x00)
  {
    return CanFrameKind::Acc;
  }

  if (d[0] == 0x50 && d[1] == 0x00)
  {
    return CanFrameKind::Gyro;
  }

  QuatComp qa;
  QuatComp qb;
  if (decodeQuatFrame2Comp(d, qa, qb))
  {
    return CanFrameKind::Quat2Comp;
  }

  AngleData angle;
  if (decodeAngleFrame(d, angle))
  {
    return CanFrameKind::Angle;
  }

  return CanFrameKind::Unknown;
}

}  // namespace bw

#endif  // INCLUDE_BW_ROS_DRIVER_CAN_DECODE_UTILS_HPP