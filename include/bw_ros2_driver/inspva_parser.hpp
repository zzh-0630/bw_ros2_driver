/*!
 * \file         inspva_parser.hpp
 * \author       BWSensing
 * \date         2026-04-15
 * \brief        INSPVA frame parser.
 *
 * Copyright (c) 2026- BWSensing
 * Distributed under the MIT License. See LICENSE for more information.
 */
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

  /*!
   * \brief Feed raw bytes into the parser and extract complete INSPVA samples.
   *
   * The method accepts arbitrary byte chunks, such as partial serial reads.
   * Incoming data is appended to the internal buffer, and any complete frames
   * are decoded and appended to \p ref_out.
   *
   * \param p_data Pointer to the input byte buffer.
   * \param size Number of bytes available in \p p_data.
   * \param ref_out Output vector receiving decoded samples.
   *
   * \return Number of samples appended to \p ref_out.
   */
  size_t feed(const uint8_t* p_data, size_t size, std::vector<InspvaSample>& ref_out);

  /*!
   * \brief Get the number of successfully decoded frames.
   *
   * \return Count of valid frames parsed since the last reset or construction.
   */
  uint64_t ok() const { return ok_cnt_; }

  /*!
   * \brief Get the number of frames rejected as invalid before CRC validation.
   *
   * This counter is typically incremented for malformed headers, unsupported
   * field widths, invalid command values, or unexpected frame lengths.
   *
   * \return Count of invalid frames.
   */
  uint64_t bad() const { return bad_cnt_; }

  /*!
   * \brief Get the number of frames rejected due to CRC mismatch.
   *
   * \return Count of frames whose CRC check failed.
   */
  uint64_t crcBad() const { return crc_bad_cnt_; }

private:
  /*!
   * \brief Validate the parity rule encoded in the frame_info byte.
   *
   * The protocol uses bit 7 as a parity bit for bits 6..0.
   *
   * \param frame_info Raw frame_info byte from the input stream.
   *
   * \return True if the parity bit matches the expected value.
   */
  static bool frameInfoParityOk(uint8_t frame_info);

  /*!
   * \brief Decode a segment width from the 2-bit width encoding in frame_info.
   *
   * Protocol mapping:
   * - 00 -> 1 byte
   * - 01 -> 2 bytes
   * - 10 -> 4 bytes
   * - 11 -> invalid/reserved
   *
   * \param bits Two-bit encoded width value.
   *
   * \return Width in bytes, or 0 if the encoding is invalid.
   */
  static size_t segmentWidthFromBits(uint8_t bits);

  /*!
   * \brief Read an unsigned big-endian integer of variable width.
   *
   * This helper is used for fields whose width is defined by frame_info,
   * such as the length, address, and command fields.
   *
   * \param p_data Pointer to the first byte of the field.
   * \param width Field width in bytes. Supported widths are typically 1, 2, or 4.
   *
   * \return Parsed unsigned integer value.
   */
  static uint32_t readUnsignedBe(const uint8_t* p_data, size_t width);

  /*!
   * \brief Read a 32-bit big-endian unsigned integer.
   *
   * \param p_data Pointer to the first byte of the 4-byte field.
   *
   * \return Parsed uint32_t value.
   */
  static uint32_t readU32Be(const uint8_t* p_data);

  /*!
   * \brief Read a 32-bit little-endian unsigned integer.
   *
   * \param p_data Pointer to the first byte of the 4-byte field.
   *
   * \return Parsed uint32_t value.
   */
  static uint32_t readU32Le(const uint8_t* p_data);

  /*!
   * \brief Read an IEEE-754 double stored in little-endian byte order.
   *
   * \param p_data Pointer to the first byte of the 8-byte field.
   *
   * \return Parsed double value.
   */
  static double readDoubleLe(const uint8_t* p_data);

  /*!
   * \brief Compute protocol CRC-32 using the MSB-first configuration.
   *
   * The CRC parameters follow the GI320 protocol definition:
   * polynomial 0x04C11DB7, initial value 0xFFFFFFFF, no input reflection,
   * no output reflection, and no final xor.
   *
   * \param p_data Pointer to the start of the CRC input range.
   * \param size Number of bytes to include in the CRC calculation.
   *
   * \return Computed CRC-32 value.
   */
  static uint32_t computeCrc32Msb(const uint8_t* p_data, size_t size);

  /*!
   * \brief Try to consume and decode one complete frame from the internal buffer.
   *
   * This method performs frame synchronization, header validation, length
   * decoding, command filtering, and CRC verification. If a valid frame is
   * found, the decoded sample is appended to \p ref_out.
   *
   * \param ref_out Output vector receiving decoded samples.
   *
   * \return True if parser state advanced and another parse attempt may succeed;
   *         false if more input data is required or no further progress can be made.
   */
  bool tryParseOneFrame(std::vector<InspvaSample>& ref_out);

  /*!
   * \brief Decode payload fields from a validated INSPVA frame.
   *
   * The input frame is assumed to have already passed header and CRC checks.
   * This method extracts structured navigation fields and stores them in
   * \p ref_sample.
   *
   * \param ref_frame Complete validated frame buffer.
   * \param ref_sample Output sample populated with decoded field values.
   *
   * \return True if decoding succeeded; false otherwise.
   */
  bool parseFrame(const std::vector<uint8_t>& ref_frame, InspvaSample& ref_sample);

  std::vector<uint8_t> buf_;

  uint64_t ok_cnt_ = 0;
  uint64_t bad_cnt_ = 0;
  uint64_t crc_bad_cnt_ = 0;
};

}  // namespace bw

#endif  // INCLUDE_BW_ROS_DRIVER_INSPVA_PARSER_HPP