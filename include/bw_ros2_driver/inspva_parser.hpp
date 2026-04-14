#ifndef INCLUDE_BW_ROS_DRIVER_INSPVA_PARSER_HPP
#define INCLUDE_BW_ROS_DRIVER_INSPVA_PARSER_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

namespace bw
{
struct InspvaSample
{
  bool valid = false;

  uint32_t week = 0;
  double seconds = 0.0;

  double longitude = 0.0;
  double latitude = 0.0;
  double height = 0.0;

  double north_velocity = 0.0;
  double east_velocity = 0.0;
  double down_velocity = 0.0;
  double up_velocity = 0.0;

  double roll_deg = 0.0;
  double pitch_deg = 0.0;
  double azimuth_deg = 0.0;

  double ax = 0.0;
  double ay = 0.0;
  double az = 0.0;

  double gx = 0.0;
  double gy = 0.0;
  double gz = 0.0;

  uint8_t sats = 0;
  uint8_t pos_status = 0;
  uint8_t azi_status = 0;
  uint8_t ins_status = 0;

  double temperature_deg_c = 0.0;
  double pdop = 0.0;
};

class InspvaParser
{
public:
  InspvaParser();

  void reset();

  size_t feed(const uint8_t* p_data, size_t size, std::vector<InspvaSample>& ref_out);

  uint64_t ok() const { return ok_cnt_; }
  uint64_t bad() const { return bad_cnt_; }
  uint64_t crcBad() const { return crc_bad_cnt_; }

private:
  static bool frameInfoParityOk(uint8_t frame_info);
  static size_t segmentWidthFromBits(uint8_t bits);
  static uint32_t readUnsignedBe(const uint8_t* p_data, size_t width);
  static uint32_t readU32Be(const uint8_t* p_data);
  static uint32_t readU32Le(const uint8_t* p_data);
  static double readDoubleLe(const uint8_t* p_data);
  static uint32_t computeCrc32Msb(const uint8_t* p_data, size_t size);

  bool tryParseOneFrame(std::vector<InspvaSample>& ref_out);
  bool parseFrame(const std::vector<uint8_t>& ref_frame, InspvaSample& ref_sample);

  std::vector<uint8_t> buf_;

  uint64_t ok_cnt_ = 0;
  uint64_t bad_cnt_ = 0;
  uint64_t crc_bad_cnt_ = 0;
};

}  // namespace bw

#endif  // INCLUDE_BW_ROS_DRIVER_INSPVA_PARSER_HPP