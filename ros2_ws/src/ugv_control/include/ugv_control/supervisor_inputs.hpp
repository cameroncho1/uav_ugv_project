#ifndef UGV_CONTROL__SUPERVISOR_INPUTS_HPP_
#define UGV_CONTROL__SUPERVISOR_INPUTS_HPP_

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>

#include <ugv_control/supervisor.hpp>

namespace ugv_control
{
// Source-neutral detector contract; the ROS callback only copies message fields.
struct FaultDecision
{
  unsigned schema_version{1};
  std::string source;
  uint64_t sequence{0};
  int64_t stamp_ns{0};
  bool window_valid{false};
  bool gps_outage{false};
  bool odometry_scale{false};
  bool imu_drift{false};
  bool uav_loss{false};
};

class FaultInput
{
public:
  void reset()
  {
    latest_.reset();
    clear_started_.reset();
    invalid_ = true;
  }

  FaultEvent receive(const FaultDecision & message, int64_t ros_now, int64_t steady_now,
    bool pose_valid)
  {
    if (message.schema_version != 1 || message.source.empty() || !message.window_valid ||
      message.stamp_ns <= 0 || ros_now <= 0 || steady_now < 0 ||
      message.stamp_ns - ros_now > 50000000 ||
      ros_now - message.stamp_ns > kMaximumDecisionAgeNs)
    {
      return invalidate();
    }
    bool discontinuity = false;
    if (latest_) {
      if (message.source != latest_->source) {
        discontinuity = true;
      } else {
        const bool same_window = message.sequence == latest_->sequence &&
          message.stamp_ns == latest_->stamp_ns;
        if (same_window && same_flags(message, *latest_)) {
          // A duplicate cannot renew the receipt lease or count as a clear window.
          return poll(ros_now, steady_now, pose_valid);
        }
        if (message.sequence <= latest_->sequence || message.stamp_ns <= latest_->stamp_ns) {
          return invalidate();
        }
        discontinuity = message.sequence - latest_->sequence != 1 ||
          message.stamp_ns - latest_->stamp_ns > kMaximumDecisionGapNs ||
          steady_now - receipt_ns_ > kMaximumDecisionGapNs;
      }
    }
    latest_ = message;
    receipt_ns_ = steady_now;
    invalid_ = false;
    if (holding_alarm(pose_valid)) {
      clear_started_.reset();
      return FaultEvent::Alarm;
    }
    if (discontinuity) {return invalidate();}
    if (!clear_started_) {clear_started_ = message.stamp_ns;}
    return FaultEvent::ClearWindow;
  }

  FaultEvent poll(int64_t ros_now, int64_t steady_now, bool pose_valid)
  {
    if (invalid_ || !latest_ || ros_now <= 0 ||
      latest_->stamp_ns - ros_now > 50000000 ||
      ros_now - latest_->stamp_ns > kMaximumDecisionAgeNs ||
      steady_now < receipt_ns_ || steady_now - receipt_ns_ > kMaximumDecisionAgeNs)
    {
      return invalidate();
    }
    if (holding_alarm(pose_valid)) {
      clear_started_.reset();
      return FaultEvent::Alarm;
    }
    return FaultEvent::None;
  }

  bool recovery_mature(int64_t ros_now) const
  {
    return !invalid_ && clear_started_ && ros_now >= *clear_started_ &&
           ros_now - *clear_started_ >= kMinimumClearNs;
  }

  int64_t latest_stamp() const {return latest_ ? latest_->stamp_ns : 0;}
  uint64_t latest_sequence() const {return latest_ ? latest_->sequence : 0;}
  std::string latest_source() const {return latest_ ? latest_->source : "";}
  bool decision_valid() const {return !invalid_ && latest_.has_value();}

private:
  FaultEvent invalidate()
  {
    invalid_ = true;
    clear_started_.reset();
    return FaultEvent::Unavailable;
  }

  bool holding_alarm(bool pose_valid) const
  {
    return latest_->odometry_scale || latest_->imu_drift ||
           ((latest_->gps_outage || latest_->uav_loss) && !pose_valid);
  }

  static bool same_flags(const FaultDecision & a, const FaultDecision & b)
  {
    return a.window_valid == b.window_valid && a.gps_outage == b.gps_outage &&
           a.odometry_scale == b.odometry_scale && a.imu_drift == b.imu_drift &&
           a.uav_loss == b.uav_loss;
  }

  std::optional<FaultDecision> latest_;
  std::optional<int64_t> clear_started_;
  int64_t receipt_ns_{0};
  bool invalid_{true};
};

// Goal identity survives invalid messages and is scoped to a ROS clock epoch.
class GoalHistory
{
public:
  void reset_epoch()
  {
    ++epoch_;
    available_ = false;
    new_goal_ = false;
  }

  bool receive(bool valid, int64_t stamp_ns, double x, double y, int64_t ros_now)
  {
    if (!valid || stamp_ns <= 0 || ros_now <= 0 || stamp_ns - ros_now > 50000000 ||
      !std::isfinite(x) || !std::isfinite(y))
    {
      available_ = false;
      return false;
    }
    if (!accepted_ || accepted_epoch_ != epoch_ || stamp_ns > stamp_ns_) {
      accepted_ = true;
      accepted_epoch_ = epoch_;
      stamp_ns_ = stamp_ns;
      x_ = x;
      y_ = y;
      available_ = true;
      new_goal_ = true;
      return true;
    }
    available_ = stamp_ns == stamp_ns_ && x == x_ && y == y_;
    return available_;
  }

  bool available() const {return available_;}
  bool new_goal_pending() const {return new_goal_;}
  bool take_new_goal()
  {
    const auto value = new_goal_;
    new_goal_ = false;
    return value;
  }

private:
  uint64_t epoch_{0}, accepted_epoch_{0};
  int64_t stamp_ns_{0};
  double x_{0}, y_{0};
  bool accepted_{false}, available_{false}, new_goal_{false};
};
}  // namespace ugv_control
#endif
