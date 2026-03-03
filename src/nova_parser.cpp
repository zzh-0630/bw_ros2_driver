#include "bw_ros_driver/nova_parser.hpp"

// Standard headers
#include <cstring>

namespace bw
{
namespace
{
float readFloatLe(const uint8_t* pData)
{
  const uint32_t u = static_cast<uint32_t>(pData[0]) |
                     (static_cast<uint32_t>(pData[1]) << 8) |
                     (static_cast<uint32_t>(pData[2]) << 16) |
                     (static_cast<uint32_t>(pData[3]) << 24);

  float f = 0.0f;
  std::memcpy(&f, &u, sizeof(float));
  return f;
}
}  // namespace

uint8_t NovaParser::computeCrc8(const uint8_t* pData, size_t size)
{
  uint8_t crc = 0x00;
  constexpr uint8_t kPoly = 0x07;

  for (size_t i = 0; i < size; ++i)
  {
    crc ^= pData[i];

    for (uint8_t j = 0; j < 8; ++j)
    {
      if (crc & 0x80u)
      {
        crc = static_cast<uint8_t>((crc << 1u) ^ kPoly);
      }
      else
      {
        crc <<= 1u;
      }
    }
  }

  return crc;
}

NovaParser::NovaParser()
{
  reset();
}

void NovaParser::reset()
{
  state_ = State::WaitHead;
  buf_.clear();
  expected_len_ = 0;
}

size_t NovaParser::feed(const uint8_t* pData, size_t size, std::vector<F3Sample>& refOut)
{
  size_t produced = 0;

  for (size_t i = 0; i < size; ++i)
  {
    const uint8_t b = pData[i];

    switch (state_)
    {
      case State::WaitHead:
      {
        if (b == 0xF3)
        {
          buf_.clear();
          buf_.push_back(b);
          state_ = State::ReadHeader;
        }
        break;
      }

      case State::ReadHeader:
      {
        buf_.push_back(b);

        // [0]=Head, [1]=CmdID, [2]=DataLen
        if (buf_.size() == 3)
        {
          const uint8_t data_len = buf_[2];

          // NOTE: This keeps the original driver's length convention.
          expected_len_ = static_cast<size_t>(2u + data_len);

          // Minimal sanity guard (avoid tiny frames).
          if (expected_len_ < 10)
          {
            ++bad_cnt_;
            state_ = State::WaitHead;
            buf_.clear();
            expected_len_ = 0;
          }
          else
          {
            state_ = State::ReadFrame;
          }
        }
        break;
      }

      case State::ReadFrame:
      {
        buf_.push_back(b);

        if (buf_.size() == expected_len_)
        {
          const std::vector<uint8_t>& frame = buf_;

          // Frame layout:
          // Head + CmdID + DataLen + Count + CRC8 + Data(56 bytes) + CheckSum
          constexpr size_t kExpectedMinSize = 62;

          if (frame.size() < kExpectedMinSize)
          {
            ++bad_cnt_;
          }
          else
          {
            //-----------------------------------------------------------------//
            // 1) CRC8: from Head to Count (4 bytes)                            //
            //-----------------------------------------------------------------//
            const uint8_t crc_calc = computeCrc8(&frame[0], 4);
            const uint8_t crc_recv = frame[4];

            if (crc_calc == crc_recv)
            {
              //----------------------------------------------------------------//
              // 2) CheckSum: CmdID to last Data (exclude Head)                  //
              //----------------------------------------------------------------//
              uint8_t sum = 0;
              for (size_t k = 1; k < frame.size() - 1; ++k)
              {
                sum = static_cast<uint8_t>(sum + frame[k]);
              }

              const uint8_t checksum_recv = frame.back();
              if (sum == checksum_recv)
              {
                F3Sample sample;
                if (parseFrame(frame, sample))
                {
                  sample.valid = true;
                  refOut.push_back(sample);
                  ++produced;
                  ++ok_cnt_;
                }
                else
                {
                  ++bad_cnt_;
                }
              }
              else
              {
                ++bad_cnt_;
              }
            }
            else
            {
              ++crc_bad_cnt_;
            }
          }

          // Reset for next frame
          state_ = State::WaitHead;
          buf_.clear();
          expected_len_ = 0;
        }
        break;
      }

      default:
      {
        state_ = State::WaitHead;
        buf_.clear();
        expected_len_ = 0;
        break;
      }
    }
  }

  return produced;
}

bool NovaParser::parseFrame(const std::vector<uint8_t>& ref_frame, F3Sample& ref_sample)
{
  // Expected payload is 56 bytes (angles + acc + gyro + mag + temp + time)
  if (ref_frame.size() < 62)
  {
    return false;
  }

  ref_sample.cmd_id = ref_frame[1];
  ref_sample.count  = ref_frame[3];

  // [5..60] Data (56 bytes)
  const uint8_t* d = &ref_frame[5];

  ref_sample.pitch_deg = readFloatLe(d + 0);
  ref_sample.roll_deg  = readFloatLe(d + 4);
  ref_sample.yaw_deg   = readFloatLe(d + 8);

  ref_sample.acc_x_g = readFloatLe(d + 12);
  ref_sample.acc_y_g = readFloatLe(d + 16);
  ref_sample.acc_z_g = readFloatLe(d + 20);

  ref_sample.gyro_x_dps = readFloatLe(d + 24);
  ref_sample.gyro_y_dps = readFloatLe(d + 28);
  ref_sample.gyro_z_dps = readFloatLe(d + 32);

  ref_sample.mag_x = readFloatLe(d + 36);
  ref_sample.mag_y = readFloatLe(d + 40);
  ref_sample.mag_z = readFloatLe(d + 44);

  ref_sample.temperature_deg_c = readFloatLe(d + 48);

  const uint32_t tu = static_cast<uint32_t>(d[52]) |
                      (static_cast<uint32_t>(d[53]) << 8) |
                      (static_cast<uint32_t>(d[54]) << 16) |
                      (static_cast<uint32_t>(d[55]) << 24);
  ref_sample.time_us = tu;

  return true;
}

}  // namespace bw
