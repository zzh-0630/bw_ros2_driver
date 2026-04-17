#include "bw_ros2_driver/inspva_parser.hpp"

#include <algorithm>
#include <cstring>

namespace bw
{
namespace
{
constexpr uint8_t kSync = 0x57;
constexpr uint8_t kCmd = 0x03;
constexpr size_t kCrcBytes = 4;

constexpr size_t kWeekOffset = 5;
constexpr size_t kSecondsOffset = 9;
constexpr size_t kLongitudeOffset = 17;
constexpr size_t kLatitudeOffset = 25;
constexpr size_t kHeightOffset = 33;
constexpr size_t kNorthVelOffset = 41;
constexpr size_t kEastVelOffset = 49;
constexpr size_t kDownVelOffset = 57;
constexpr size_t kRollOffset = 65;
constexpr size_t kPitchOffset = 73;
constexpr size_t kAzimuthOffset = 81;
constexpr size_t kAxOffset = 89;
constexpr size_t kAyOffset = 97;
constexpr size_t kAzOffset = 105;
constexpr size_t kGxOffset = 113;
constexpr size_t kGyOffset = 121;
constexpr size_t kGzOffset = 129;
constexpr size_t kSatsOffset = 137;
constexpr size_t kPosStatusOffset = 138;
constexpr size_t kAziStatusOffset = 139;
constexpr size_t kTemperatureOffset = 140;
constexpr size_t kPdopOffset = 148;
constexpr size_t kStatusOffset = 156;
constexpr size_t kMinFrameLen = 161;
}  // namespace

InspvaParser::InspvaParser()
{
  reset();
}

void InspvaParser::reset()
{
  buf_.clear();
}

bool InspvaParser::frameInfoParityOk(uint8_t frame_info)
{
  uint8_t v = static_cast<uint8_t>(frame_info & 0x7F);
  int ones = 0;
  for (int i = 0; i < 7; ++i)
  {
    ones += (v >> i) & 0x1;
  }
  const uint8_t expected = static_cast<uint8_t>(ones & 0x1);
  return ((frame_info >> 7) & 0x1U) == expected;
}

size_t InspvaParser::segmentWidthFromBits(uint8_t bits)
{
  switch (bits & 0x3U)
  {
    case 0:
      return 1;
    case 1:
      return 2;
    case 2:
      return 4;
    default:
      return 0;
  }
}

uint32_t InspvaParser::readUnsignedBe(const uint8_t* p_data, size_t width)
{
  uint32_t value = 0;
  for (size_t i = 0; i < width; ++i)
  {
    value = (value << 8) | static_cast<uint32_t>(p_data[i]);
  }
  return value;
}

uint32_t InspvaParser::readU32Be(const uint8_t* p_data)
{
  return readUnsignedBe(p_data, 4);
}

uint32_t InspvaParser::readU32Le(const uint8_t* p_data)
{
  return static_cast<uint32_t>(p_data[0]) |
         (static_cast<uint32_t>(p_data[1]) << 8) |
         (static_cast<uint32_t>(p_data[2]) << 16) |
         (static_cast<uint32_t>(p_data[3]) << 24);
}

double InspvaParser::readDoubleLe(const uint8_t* p_data)
{
  uint64_t raw = 0;
  for (size_t i = 0; i < sizeof(double); ++i)
  {
    raw |= (static_cast<uint64_t>(p_data[i]) << (8 * i));
  }

  double value = 0.0;
  std::memcpy(&value, &raw, sizeof(double));
  return value;
}

uint32_t InspvaParser::computeCrc32Msb(const uint8_t* p_data, size_t size)
{
  uint32_t crc = 0xFFFFFFFFu;
  constexpr uint32_t kPoly = 0x04C11DB7u;

  for (size_t i = 0; i < size; ++i)
  {
    crc ^= (static_cast<uint32_t>(p_data[i]) << 24);
    for (int bit = 0; bit < 8; ++bit)
    {
      if (crc & 0x80000000u)
      {
        crc = (crc << 1) ^ kPoly;
      }
      else
      {
        crc <<= 1;
      }
    }
  }

  return crc;
}

size_t InspvaParser::feed(const uint8_t* p_data, size_t size, std::vector<InspvaSample>& ref_out)
{
  buf_.insert(buf_.end(), p_data, p_data + size);

  const size_t before = ref_out.size();
  while (tryParseOneFrame(ref_out))
  {
  }
  return ref_out.size() - before;
}

bool InspvaParser::tryParseOneFrame(std::vector<InspvaSample>& ref_out)
{
  auto it = std::find(buf_.begin(), buf_.end(), kSync);
  if (it != buf_.begin())
  {
    buf_.erase(buf_.begin(), it);
  }

  if (buf_.size() < 3)
  {
    return false;
  }

  if (buf_[0] != kSync)
  {
    buf_.erase(buf_.begin());
    return !buf_.empty();
  }

  const uint8_t frame_info = buf_[1];
  if (!frameInfoParityOk(frame_info))
  {
    ++bad_cnt_;
    buf_.erase(buf_.begin());
    return !buf_.empty();
  }

  const size_t len_width = segmentWidthFromBits((frame_info >> 4) & 0x3U);
  const size_t addr_width = segmentWidthFromBits((frame_info >> 2) & 0x3U);
  const size_t cmd_width = segmentWidthFromBits(frame_info & 0x3U);
  if (len_width == 0 || addr_width == 0 || cmd_width == 0)
  {
    ++bad_cnt_;
    buf_.erase(buf_.begin());
    return !buf_.empty();
  }

  const size_t header_len = 2 + len_width;
  if (buf_.size() < header_len)
  {
    return false;
  }

  const uint32_t length_field = readUnsignedBe(buf_.data() + 2, len_width);
  const size_t expected_len = header_len + static_cast<size_t>(length_field);
  if (expected_len < kMinFrameLen)
  {
    ++bad_cnt_;
    buf_.erase(buf_.begin());
    return !buf_.empty();
  }

  if (buf_.size() < expected_len)
  {
    return false;
  }

  std::vector<uint8_t> frame(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(expected_len));
  buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(expected_len));

  const size_t cmd_offset = 2 + len_width + addr_width;
  const uint32_t cmd = readUnsignedBe(frame.data() + cmd_offset, cmd_width);
  if (cmd != kCmd)
  {
    ++bad_cnt_;
    return !buf_.empty();
  }

  const uint32_t crc_calc = computeCrc32Msb(frame.data() + 1, frame.size() - 1 - kCrcBytes);
  const uint32_t crc_recv = readU32Be(&frame[frame.size() - kCrcBytes]);
  if (crc_calc != crc_recv)
  {
    ++crc_bad_cnt_;
    return !buf_.empty();
  }

  InspvaSample sample;
  if (parseFrame(frame, sample))
  {
    sample.valid = true;
    ref_out.push_back(sample);
    ++ok_cnt_;
  }
  else
  {
    ++bad_cnt_;
  }

  return !buf_.empty();
}

bool InspvaParser::parseFrame(const std::vector<uint8_t>& ref_frame, InspvaSample& ref_sample)
{
  if (ref_frame.size() < kMinFrameLen)
  {
    return false;
  }

  ref_sample.week = readU32Le(&ref_frame[kWeekOffset]);
  ref_sample.seconds = readDoubleLe(&ref_frame[kSecondsOffset]);
  ref_sample.longitude = readDoubleLe(&ref_frame[kLongitudeOffset]);
  ref_sample.latitude = readDoubleLe(&ref_frame[kLatitudeOffset]);
  ref_sample.height = readDoubleLe(&ref_frame[kHeightOffset]);
  ref_sample.north_velocity = readDoubleLe(&ref_frame[kNorthVelOffset]);
  ref_sample.east_velocity = readDoubleLe(&ref_frame[kEastVelOffset]);
  ref_sample.down_velocity = readDoubleLe(&ref_frame[kDownVelOffset]);
  ref_sample.up_velocity = -ref_sample.down_velocity;
  ref_sample.roll_deg = readDoubleLe(&ref_frame[kRollOffset]);
  ref_sample.pitch_deg = readDoubleLe(&ref_frame[kPitchOffset]);
  ref_sample.azimuth_deg = readDoubleLe(&ref_frame[kAzimuthOffset]);
  ref_sample.ax = readDoubleLe(&ref_frame[kAxOffset]);
  ref_sample.ay = readDoubleLe(&ref_frame[kAyOffset]);
  ref_sample.az = readDoubleLe(&ref_frame[kAzOffset]);
  ref_sample.gx = readDoubleLe(&ref_frame[kGxOffset]);
  ref_sample.gy = readDoubleLe(&ref_frame[kGyOffset]);
  ref_sample.gz = readDoubleLe(&ref_frame[kGzOffset]);
  ref_sample.sats = ref_frame[kSatsOffset];
  ref_sample.pos_status = ref_frame[kPosStatusOffset];
  ref_sample.azi_status = ref_frame[kAziStatusOffset];
  ref_sample.temperature_deg_c = readDoubleLe(&ref_frame[kTemperatureOffset]);
  ref_sample.pdop = readDoubleLe(&ref_frame[kPdopOffset]);
  ref_sample.ins_status = ref_frame[kStatusOffset];
  return true;
}

}  // namespace bw
