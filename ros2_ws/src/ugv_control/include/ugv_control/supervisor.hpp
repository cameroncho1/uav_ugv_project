#ifndef UGV_CONTROL__SUPERVISOR_HPP_
#define UGV_CONTROL__SUPERVISOR_HPP_

#include <algorithm>
#include <cstdint>

namespace ugv_control
{
enum class Mode {Init, Navigate, HoldGuard, HoldFault, Goal};
enum class Condition {NoGoal, FaultLatched, InvalidPose, GoalCompleted, InvalidRoute, Always};
enum class FaultEvent {None, ClearWindow, Alarm, Unavailable};
enum class RecoveryAge {None, Young, Mature};
struct Rule {Condition condition; Mode mode;};
}  // namespace ugv_control

#include <ugv_control/supervisor_policy.hpp>

namespace ugv_control
{
// Finite controller memory. Exact clocks and message validation belong to the input adapter.
struct SupervisorState
{
  bool detector_enabled{false};
  Mode mode{Mode::Init};
  bool fault_latched{false};
  unsigned clear_count{0};
  RecoveryAge recovery_age{RecoveryAge::None};
  bool goal_completed{false};
};

struct SupervisorInput
{
  bool have_goal{false};
  bool pose_valid{false};
  bool route_valid{false};
  bool goal_reached{false};
  bool new_goal{false};
  FaultEvent fault{FaultEvent::None};
  bool recovery_mature{false};
  bool clock_reset{false};
};

inline SupervisorState initial_supervisor(bool detector_enabled)
{
  SupervisorState state;
  state.detector_enabled = detector_enabled;
  state.fault_latched = detector_enabled;
  return state;
}

inline bool permits_motion(const SupervisorState & state)
{
  return state.mode == Mode::Navigate;
}

inline const char * mode_name(Mode mode)
{
  switch (mode) {
    case Mode::Init: return "INIT";
    case Mode::Navigate: return "NAVIGATE";
    case Mode::HoldGuard: return "HOLD_GUARD";
    case Mode::HoldFault: return "HOLD_FAULT";
    case Mode::Goal: return "GOAL";
  }
  return "INVALID";
}

inline bool matches(Condition condition, const SupervisorState & state, const SupervisorInput & in)
{
  switch (condition) {
    case Condition::NoGoal: return !in.have_goal;
    case Condition::FaultLatched: return state.fault_latched;
    case Condition::InvalidPose: return !in.pose_valid;
    case Condition::GoalCompleted: return state.goal_completed;
    case Condition::InvalidRoute: return !in.route_valid;
    case Condition::Always: return true;
  }
  return false;
}

// One decision transition; called again immediately before each command publication.
// New goals never clear a fault latch. Rewinds retain completion until a new valid goal.
inline SupervisorState supervise(SupervisorState state, const SupervisorInput & in)
{
  if (in.clock_reset) {
    state.mode = Mode::Init;
    state.fault_latched = state.detector_enabled;
    state.clear_count = 0;
    state.recovery_age = RecoveryAge::None;
    return state;
  }
  if (in.new_goal) {state.goal_completed = false;}
  if (state.detector_enabled) {
    if (in.fault == FaultEvent::Alarm || in.fault == FaultEvent::Unavailable) {
      state.fault_latched = true;
      state.clear_count = 0;
      state.recovery_age = RecoveryAge::None;
    } else if (state.fault_latched) {
      if (in.fault == FaultEvent::ClearWindow) {
        if (state.clear_count == 0) {state.recovery_age = RecoveryAge::Young;}
        state.clear_count = std::min(kClearWindows, state.clear_count + 1);
      }
      if (state.clear_count > 0 && in.recovery_mature) {
        state.recovery_age = RecoveryAge::Mature;
      }
      if (state.clear_count == kClearWindows && state.recovery_age == RecoveryAge::Mature) {
        state.fault_latched = false;
        state.clear_count = 0;
        state.recovery_age = RecoveryAge::None;
      }
    }
  }
  if (in.have_goal && in.pose_valid && in.goal_reached && !state.fault_latched) {
    state.goal_completed = true;
  }
  for (const auto & rule : kSupervisorRules) {
    if (matches(rule.condition, state, in)) {
      state.mode = rule.mode;
      break;
    }
  }
  return state;
}
}  // namespace ugv_control
#endif
