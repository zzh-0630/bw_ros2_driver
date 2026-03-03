/*!
 * \file         can_imu_aggregator.hpp
 * \author       BWSensing
 * \date         2026-02
 * \brief        SocketCAN IMU aggregation helper (ROS 2).
 *
 * The firmware may transmit one "logical IMU sample" as multiple CAN frames.
 * This helper aggregates a sequence of frames into one `sensor_msgs::msg::Imu`.
 *
 * Current aggregation policy (sequence state machine):
 *   ANGLE -> ACC -> GYRO -> QUAT(0/1) -> QUAT(2/3) -> PUBLISH
 *
 * Publication is triggered on the last quaternion frame (Q2/Q3).
 *
 * Copyright (c) 2026 BWSensing
 * Distributed under the MIT License. See LICENSE for more information.
 */

#ifndef INCLUDE_BW_ROS_DRIVER_CAN_IMU_AGGREGATOR_HPP
#define INCLUDE_BW_ROS_DRIVER_CAN_IMU_AGGREGATOR_HPP

#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <cstdint>
#include <string>

#include <rclcpp/time.hpp>

#include "bw_ros_driver/can_decode_utils.hpp"

namespace bw
{

struct CanImuAggregatorConfig
{
  int max_age_ms = 100;              //!< Max allowed time window for one group.
  bool prefer_quat = true;           //!< Prefer quaternion if available.
  bool allow_angle_fallback = true;  //!< Use Euler angles if quaternion is missing.

  bool require_acc = true;           //!< Require acceleration for publish.
  bool require_orientation = true;   //!< Require orientation (quat or angle) for publish.

  std::string frame_id = "imu_link";

  double cov_orientation = -1.0;
  double cov_angular_velocity = -1.0;
  double cov_linear_acceleration = -1.0;
};

class CanImuAggregator
{
public:
  explicit CanImuAggregator(const CanImuAggregatorConfig& cfg) : cfg_(cfg)
  {
  }

  bool ingestAndMaybeBuildImu(const uint8_t d[8], const rclcpp::Time& stamp, sensor_msgs::msg::Imu& refOutMsg)
  {
    const int64_t now_ns = stamp.nanoseconds();

    // Time-out policy: abort a partially collected group when it exceeds max window.
    if (state_ != SeqState::ExpectAngle && t_cycle_start_ns_ != 0)
    {
      if (nsToSec(now_ns - t_cycle_start_ns_) > maxAgeSec())
      {
        resetToExpectAngle();
      }
    }

    const CanFrameKind kind = detectKindMux0x585(d);

    //-----------------------------------------------------------------//
    // Group synchronization: only an ANGLE frame can start a new group. //
    //-----------------------------------------------------------------//
    if (kind == CanFrameKind::Angle)
    {
      AngleData angle;
      if (!decodeAngleFrame(d, angle))
      {
        resetToExpectAngle();
        return false;
      }

      startNewCycle(now_ns);
      ang_ = angle;
      has_angle_ = true;
      t_angle_ns_ = now_ns;

      state_ = SeqState::ExpectAcc;
      return false;
    }

    // Backward-compat: allow ACC as a group starter (some firmwares omit angle frames).
    if (state_ == SeqState::ExpectAngle)
    {
      if (kind == CanFrameKind::Acc)
      {
        AccData acc;
        if (!decodeAccFrame(d, acc))
        {
          resetToExpectAngle();
          return false;
        }

        startNewCycle(now_ns);
        acc_ = acc;
        has_acc_ = true;
        t_acc_ns_ = now_ns;

        state_ = SeqState::ExpectGyro;
        return false;
      }

      return false;
    }

    //-----------------------------------------------------------------//
    // In-group state machine                                           //
    //-----------------------------------------------------------------//
    switch (state_)
    {
      case SeqState::ExpectAcc:
      {
        if (kind != CanFrameKind::Acc)
        {
          resetToExpectAngle();
          return false;
        }

        AccData acc;
        if (!decodeAccFrame(d, acc))
        {
          resetToExpectAngle();
          return false;
        }

        acc_ = acc;
        has_acc_ = true;
        t_acc_ns_ = now_ns;
        state_ = SeqState::ExpectGyro;
        return false;
      }

      case SeqState::ExpectGyro:
      {
        if (kind != CanFrameKind::Gyro)
        {
          resetToExpectAngle();
          return false;
        }

        GyroData gyro;
        if (!decodeGyroFrame(d, gyro))
        {
          resetToExpectAngle();
          return false;
        }

        gyro_ = gyro;
        has_gyro_ = true;
        t_gyro_ns_ = now_ns;
        state_ = SeqState::ExpectQ01;
        return false;
      }

      case SeqState::ExpectQ01:
      {
        if (kind != CanFrameKind::Quat2Comp)
        {
          resetToExpectAngle();
          return false;
        }

        QuatComp a;
        QuatComp b;
        if (!decodeQuatFrame2Comp(d, a, b))
        {
          resetToExpectAngle();
          return false;
        }

        if (classifyQuatPair(a, b) != QuatPair::Pair01)
        {
          resetToExpectAngle();
          return false;
        }

        setQuatComp(a, now_ns);
        setQuatComp(b, now_ns);

        state_ = SeqState::ExpectQ23;
        return false;
      }

      case SeqState::ExpectQ23:
      {
        if (kind != CanFrameKind::Quat2Comp)
        {
          resetToExpectAngle();
          return false;
        }

        QuatComp a;
        QuatComp b;
        if (!decodeQuatFrame2Comp(d, a, b))
        {
          resetToExpectAngle();
          return false;
        }

        if (classifyQuatPair(a, b) != QuatPair::Pair23)
        {
          resetToExpectAngle();
          return false;
        }

        setQuatComp(a, now_ns);
        setQuatComp(b, now_ns);

        // Group end trigger: attempt to build IMU.
        const bool ok = buildImuOnGroupEnd(now_ns, refOutMsg);

        // Always reset after the group ends to avoid cross-group mixing.
        resetToExpectAngle();
        return ok;
      }

      default:
      {
        resetToExpectAngle();
        return false;
      }
    }
  }

  bool hasQuatComplete() const
  {
    return has_q_[0] && has_q_[1] && has_q_[2] && has_q_[3];
  }

private:
  enum class SeqState
  {
    ExpectAngle,
    ExpectAcc,
    ExpectGyro,
    ExpectQ01,
    ExpectQ23
  };

  enum class QuatPair
  {
    Invalid,
    Pair01,
    Pair23
  };

  double maxAgeSec() const
  {
    return static_cast<double>(cfg_.max_age_ms) / 1000.0;
  }

  static double nsToSec(int64_t ns)
  {
    return static_cast<double>(ns) * 1e-9;
  }

  static bool isFresh(int64_t now_ns, int64_t t_ns, double max_age_sec)
  {
    if (t_ns == 0)
    {
      return false;
    }

    return nsToSec(now_ns - t_ns) <= max_age_sec;
  }

  void resetToExpectAngle()
  {
    state_ = SeqState::ExpectAngle;
    t_cycle_start_ns_ = 0;

    has_angle_ = false;
    has_acc_ = false;
    has_gyro_ = false;

    t_angle_ns_ = 0;
    t_acc_ns_ = 0;
    t_gyro_ns_ = 0;

    for (int i = 0; i < 4; ++i)
    {
      has_q_[i] = false;
      t_q_ns_[i] = 0;
    }

    q_[0] = 1.0;
    q_[1] = 0.0;
    q_[2] = 0.0;
    q_[3] = 0.0;
  }

  void startNewCycle(int64_t now_ns)
  {
    resetToExpectAngle();
    t_cycle_start_ns_ = now_ns;
  }

  void setQuatComp(const QuatComp& comp, int64_t now_ns)
  {
    if (comp.order < 0 || comp.order > 3)
    {
      return;
    }

    q_[comp.order] = comp.value;
    has_q_[comp.order] = true;
    t_q_ns_[comp.order] = now_ns;
  }

  static QuatPair classifyQuatPair(const QuatComp& a, const QuatComp& b)
  {
    const int mn = (a.order < b.order) ? a.order : b.order;
    const int mx = (a.order < b.order) ? b.order : a.order;

    if (mn == 0 && mx == 1)
    {
      return QuatPair::Pair01;
    }

    if (mn == 2 && mx == 3)
    {
      return QuatPair::Pair23;
    }

    return QuatPair::Invalid;
  }

  bool quatFreshAndComplete(int64_t now_ns) const
  {
    if (!(has_q_[0] && has_q_[1] && has_q_[2] && has_q_[3]))
    {
      return false;
    }

    const double age = maxAgeSec();
    return isFresh(now_ns, t_q_ns_[0], age) && isFresh(now_ns, t_q_ns_[1], age) && isFresh(now_ns, t_q_ns_[2], age) &&
           isFresh(now_ns, t_q_ns_[3], age);
  }

  bool angleFresh(int64_t now_ns) const
  {
    return has_angle_ && isFresh(now_ns, t_angle_ns_, maxAgeSec());
  }

  bool accFresh(int64_t now_ns) const
  {
    return has_acc_ && isFresh(now_ns, t_acc_ns_, maxAgeSec());
  }

  bool gyroFresh(int64_t now_ns) const
  {
    return has_gyro_ && isFresh(now_ns, t_gyro_ns_, maxAgeSec());
  }

  bool buildImuOnGroupEnd(int64_t now_ns, sensor_msgs::msg::Imu& refOut) const
  {
    // Semantic minimum: gyro must be present.
    if (!gyroFresh(now_ns))
    {
      return false;
    }

    const bool acc_ok = accFresh(now_ns);
    if (cfg_.require_acc && !acc_ok)
    {
      return false;
    }

    // Orientation strategy
    bool use_quat = false;

    if (cfg_.prefer_quat && quatFreshAndComplete(now_ns))
    {
      use_quat = true;
    }
    else if (cfg_.allow_angle_fallback && angleFresh(now_ns))
    {
      use_quat = false;
    }
    else
    {
      if (cfg_.require_orientation)
      {
        return false;
      }
    }

    refOut = sensor_msgs::msg::Imu();
    refOut.header.stamp = rclcpp::Time(now_ns);
    refOut.header.frame_id = cfg_.frame_id;

    refOut.orientation_covariance.fill(0.0);
    refOut.angular_velocity_covariance.fill(0.0);
    refOut.linear_acceleration_covariance.fill(0.0);

    refOut.orientation_covariance[0] = cfg_.cov_orientation;
    refOut.angular_velocity_covariance[0] = cfg_.cov_angular_velocity;
    refOut.linear_acceleration_covariance[0] = cfg_.cov_linear_acceleration;

    //-----------------------------------------------------------------//
    // Orientation                                                      //
    //-----------------------------------------------------------------//
    if (use_quat)
    {
      double w = q_[0];
      double x = q_[1];
      double y = q_[2];
      double z = q_[3];

      const double n = std::sqrt(w * w + x * x + y * y + z * z);
      if (std::isfinite(n) && n > 1e-9)
      {
        w /= n;
        x /= n;
        y /= n;
        z /= n;
      }
      else
      {
        w = 1.0;
        x = 0.0;
        y = 0.0;
        z = 0.0;
      }

      refOut.orientation.w = w;
      refOut.orientation.x = x;
      refOut.orientation.y = y;
      refOut.orientation.z = z;
    }
    else
    {
      constexpr double kPi = 3.14159265358979323846;
      const double deg2rad = kPi / 180.0;

      const double roll = ang_.roll_deg * deg2rad;
      const double pitch = ang_.pitch_deg * deg2rad;
      const double yaw = ang_.yaw_deg * deg2rad;

      tf2::Quaternion q;
      q.setRPY(roll, pitch, yaw);
      q.normalize();

      refOut.orientation.w = q.getW();
      refOut.orientation.x = q.getX();
      refOut.orientation.y = q.getY();
      refOut.orientation.z = q.getZ();
    }

    //-----------------------------------------------------------------//
    // Angular velocity [deg/s] -> [rad/s]                              //
    //-----------------------------------------------------------------//
    constexpr double kPi = 3.14159265358979323846;
    const double deg2rad = kPi / 180.0;

    refOut.angular_velocity.x = gyro_.gx_dps * deg2rad;
    refOut.angular_velocity.y = gyro_.gy_dps * deg2rad;
    refOut.angular_velocity.z = gyro_.gz_dps * deg2rad;

    //-----------------------------------------------------------------//
    // Linear acceleration [g] -> [m/s^2]                               //
    //-----------------------------------------------------------------//
    constexpr double g2ms2 = 9.80665;

    refOut.linear_acceleration.x = acc_.ax_g * g2ms2;
    refOut.linear_acceleration.y = acc_.ay_g * g2ms2;
    refOut.linear_acceleration.z = acc_.az_g * g2ms2;

    return true;
  }

  CanImuAggregatorConfig cfg_;

  SeqState state_ = SeqState::ExpectAngle;
  int64_t t_cycle_start_ns_ = 0;

  AngleData ang_;
  AccData acc_;
  GyroData gyro_;

  double q_[4] = {1.0, 0.0, 0.0, 0.0};
  bool has_q_[4] = {false, false, false, false};
  int64_t t_q_ns_[4] = {0, 0, 0, 0};

  bool has_angle_ = false;
  bool has_acc_ = false;
  bool has_gyro_ = false;

  int64_t t_angle_ns_ = 0;
  int64_t t_acc_ns_ = 0;
  int64_t t_gyro_ns_ = 0;
};

}  // namespace bw

#endif  // INCLUDE_BW_ROS_DRIVER_CAN_IMU_AGGREGATOR_HPP
