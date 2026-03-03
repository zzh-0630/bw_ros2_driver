#include "bw_ros_driver/standard_parser.hpp"

namespace bw
{
namespace
{
constexpr uint8_t kMinLen = 4;       // LEN..CHK inclusive, minimal: LEN+ADDR+CMD+CHK
constexpr uint8_t kMaxLen = 200;     // sanity cap to avoid unbounded growth on desync
}

StandardParser::StandardParser()
{
  reset();
}

void StandardParser::reset()
{
  st_ = State::Wait77;
  buf_.clear();
  need_ = 0;
}

size_t StandardParser::feed(const uint8_t* p_data, size_t size, std::vector<DataSample>& ref_data)
{
  size_t produced = 0;

  for (size_t i = 0; i < size; ++i)
  {
    const uint8_t byte = p_data[i];

    switch (st_)
    {
      case State::Wait77:
      {
        if (byte == 0x77)
        {
          buf_.clear();
          buf_.push_back(byte);
          st_ = State::ReadLen;
        }
        break;
      }

      case State::ReadLen:
      {
        buf_.push_back(byte);
        need_ = byte;

        // Guard against desync / garbage LEN (prevents unbounded buffer growth).
        if (need_ < kMinLen || need_ > kMaxLen)
        {
          ++bad_cnt_;
          st_ = State::Wait77;
          buf_.clear();
          need_ = 0;
          break;
        }

        st_ = State::ReadPayload;
        break;
      }

      case State::ReadPayload:
      {
        buf_.push_back(byte);

        // Collected all bytes for this frame: 1 byte header + LEN bytes total.
        if (buf_.size() == static_cast<size_t>(1 + need_))
        {
          // need_ >= 4 is guaranteed by the guard above.
          const uint8_t cs = bw::sum8(&buf_[1], static_cast<size_t>(need_ - 1));
          const uint8_t chk = buf_.back();

          if (cs == chk)
          {
            DataSample sample;
            if (onFrame(sample))
            {
              ref_data.push_back(sample);
              ++produced;
            }
            ++ok_cnt_;
          }
          else
          {
            ++bad_cnt_;
          }

          st_ = State::Wait77;
        }
        break;
      }

      default:
      {
        st_ = State::Wait77;
        buf_.clear();
        need_ = 0;
        break;
      }
    }
  }

  return produced;
}

bool StandardParser::onFrame(DataSample& ref_data)
{
  if (buf_.size() < 5)
  {
    return false;
  }

  const uint8_t* p = buf_.data();

  if (p[0] != 0x77)
  {
    return false;
  }

  const uint8_t len = p[1];
  const uint8_t cmd = p[3];
  const uint8_t* d = &p[4];

  // Bytes in the DATA field
  const size_t data_len = static_cast<size_t>(len - 4);

  //-------------------------------------------------------------------//
  // Decode based on CMD                                                //
  //-------------------------------------------------------------------//
  if (cmd == 0x59 || cmd == 0x60)
  {
    if (data_len == 52)
    {
      ref_data.P = bw::bcd3_angle_or_dps(d[0], d[1], d[2]);
      ref_data.R = bw::bcd3_angle_or_dps(d[3], d[4], d[5]);
      ref_data.Y = bw::bcd3_angle_or_dps(d[6], d[7], d[8]);
      ref_data.has_euler = true;

      ref_data.ax_g = bw::bcd3_acc_g(d[9], d[10], d[11]);
      ref_data.ay_g = bw::bcd3_acc_g(d[12], d[13], d[14]);
      ref_data.az_g = bw::bcd3_acc_g(d[15], d[16], d[17]);
      ref_data.has_acc = true;

      ref_data.gx_dps = bw::bcd3_angle_or_dps(d[18], d[19], d[20]);
      ref_data.gy_dps = bw::bcd3_angle_or_dps(d[21], d[22], d[23]);
      ref_data.gz_dps = bw::bcd3_angle_or_dps(d[24], d[25], d[26]);
      ref_data.has_gyro = true;

      ref_data.mx = bw::bcd3_mag_frac5(d[27], d[28], d[29]);
      ref_data.my = bw::bcd3_mag_frac5(d[30], d[31], d[32]);
      ref_data.mz = bw::bcd3_mag_frac5(d[33], d[34], d[35]);
      ref_data.has_mag = true;

      ref_data.q0 = bw::bcd4_q_to_double(d[36], d[37], d[38], d[39]);
      ref_data.q1 = bw::bcd4_q_to_double(d[40], d[41], d[42], d[43]);
      ref_data.q2 = bw::bcd4_q_to_double(d[44], d[45], d[46], d[47]);
      ref_data.q3 = bw::bcd4_q_to_double(d[48], d[49], d[50], d[51]);
      ref_data.has_quat = true;

      return true;
    }

    if (data_len == 43)
    {
      ref_data.P = bw::bcd3_angle_or_dps(d[0], d[1], d[2]);
      ref_data.R = bw::bcd3_angle_or_dps(d[3], d[4], d[5]);
      ref_data.Y = bw::bcd3_angle_or_dps(d[6], d[7], d[8]);
      ref_data.has_euler = true;

      ref_data.ax_g = bw::bcd3_acc_g(d[9], d[10], d[11]);
      ref_data.ay_g = bw::bcd3_acc_g(d[12], d[13], d[14]);
      ref_data.az_g = bw::bcd3_acc_g(d[15], d[16], d[17]);
      ref_data.has_acc = true;

      ref_data.gx_dps = bw::bcd3_angle_or_dps(d[18], d[19], d[20]);
      ref_data.gy_dps = bw::bcd3_angle_or_dps(d[21], d[22], d[23]);
      ref_data.gz_dps = bw::bcd3_angle_or_dps(d[24], d[25], d[26]);
      ref_data.has_gyro = true;

      ref_data.q0 = bw::bcd4_q_to_double(d[27], d[28], d[29], d[30]);
      ref_data.q1 = bw::bcd4_q_to_double(d[31], d[32], d[33], d[34]);
      ref_data.q2 = bw::bcd4_q_to_double(d[35], d[36], d[37], d[38]);
      ref_data.q3 = bw::bcd4_q_to_double(d[39], d[40], d[41], d[42]);
      ref_data.has_quat = true;

      return true;
    }
  }

  return false;
}

}  // namespace bw
