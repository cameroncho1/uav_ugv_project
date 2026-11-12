#include <gtest/gtest.h>
#include <limits>
#include <ugv_control/supervisor_inputs.hpp>

namespace
{
using ugv_control::FaultDecision;
using ugv_control::FaultEvent;
using ugv_control::FaultInput;
using ugv_control::Mode;
using ugv_control::RecoveryAge;
using ugv_control::SupervisorInput;
constexpr int64_t second = 1000000000LL;

FaultDecision clear(uint64_t sequence, int64_t stamp)
{
  FaultDecision message;
  message.source = "controlled-detector-session";
  message.sequence = sequence;
  message.stamp_ns = stamp;
  message.window_valid = true;
  return message;
}

SupervisorInput ready()
{
  SupervisorInput input;
  input.have_goal = input.pose_valid = input.route_valid = true;
  return input;
}

TEST(Supervisor, GoalCompletionSurvivesDriftGuardFaultAndRewind)
{
  auto state = ugv_control::initial_supervisor(false);
  auto input = ready();
  input.goal_reached = true;
  state = ugv_control::supervise(state, input);
  ASSERT_EQ(state.mode, Mode::Goal);
  input.goal_reached = false;
  input.pose_valid = false;
  state = ugv_control::supervise(state, input);
  EXPECT_EQ(state.mode, Mode::HoldGuard);
  input.pose_valid = true;
  state = ugv_control::supervise(state, input);
  EXPECT_EQ(state.mode, Mode::Goal);
  input.clock_reset = true;
  state = ugv_control::supervise(state, input);
  EXPECT_EQ(state.mode, Mode::Init);
  EXPECT_TRUE(state.goal_completed);
  input.clock_reset = false;
  input.new_goal = true;
  state = ugv_control::supervise(state, input);
  EXPECT_EQ(state.mode, Mode::Navigate);
}

TEST(Supervisor, EnabledUnknownInputHoldsAndNewGoalsCannotBypassRecovery)
{
  auto state = ugv_control::initial_supervisor(true);
  auto input = ready();
  input.new_goal = true;
  state = ugv_control::supervise(state, input);
  EXPECT_EQ(state.mode, Mode::HoldFault);
  EXPECT_TRUE(state.fault_latched);
  input.fault = FaultEvent::ClearWindow;
  for (int i = 0; i < 10; ++i) {state = ugv_control::supervise(state, input);}
  EXPECT_EQ(state.clear_count, 10u);
  EXPECT_EQ(state.mode, Mode::HoldFault);
  input.fault = FaultEvent::None;
  input.recovery_mature = true;
  state = ugv_control::supervise(state, input);
  EXPECT_EQ(state.mode, Mode::Navigate);
}

class RecoveryBoundary : public ::testing::TestWithParam<int64_t> {};

TEST_P(RecoveryBoundary, TenEndpointsNeedFiveElapsedSeconds)
{
  FaultInput gate;
  auto state = ugv_control::initial_supervisor(true);
  auto input = ready();
  for (uint64_t i = 0; i < 10; ++i) {
    const auto stamp = 10 * second + static_cast<int64_t>(i) * second / 2;
    input.fault = gate.receive(clear(i, stamp), stamp, 100 * second + stamp, true);
    input.recovery_mature = gate.recovery_mature(stamp);
    state = ugv_control::supervise(state, input);
  }
  ASSERT_EQ(state.clear_count, 10u);
  ASSERT_EQ(state.recovery_age, RecoveryAge::Young);
  const auto now = 15 * second + GetParam();
  input.fault = gate.poll(now, 100 * second + now, true);
  input.recovery_mature = gate.recovery_mature(now);
  state = ugv_control::supervise(state, input);
  EXPECT_EQ(ugv_control::permits_motion(state), GetParam() >= 0);
}
INSTANTIATE_TEST_SUITE_P(NanosecondLimits, RecoveryBoundary,
  ::testing::Values(-1LL, 0LL, 1LL));

TEST(FaultInput, DuplicateDecisionsNeitherCountNorRenewReceiptLease)
{
  FaultInput gate;
  auto state = ugv_control::initial_supervisor(true);
  auto input = ready();
  const auto message = clear(5, 10 * second);
  input.fault = gate.receive(message, 10 * second, 100 * second, true);
  state = ugv_control::supervise(state, input);
  for (int i = 1; i <= 10; ++i) {
    input.fault = gate.receive(message, 10 * second, 100 * second + i * second / 20, true);
    state = ugv_control::supervise(state, input);
  }
  EXPECT_EQ(state.clear_count, 1u);
  EXPECT_FALSE(gate.recovery_mature(10 * second));
  EXPECT_EQ(gate.poll(10 * second, 101 * second + 1, true), FaultEvent::Unavailable);
}

TEST(FaultInput, AcquisitionAndReceiptAgesHaveIndependentClosedLimits)
{
  FaultInput acquisition, receipt;
  acquisition.receive(clear(0, 10 * second), 10 * second, 100 * second, true);
  receipt.receive(clear(0, 10 * second), 10 * second, 100 * second, true);
  EXPECT_EQ(acquisition.poll(11 * second, 100 * second, true), FaultEvent::None);
  EXPECT_EQ(acquisition.poll(11 * second + 1, 100 * second, true), FaultEvent::Unavailable);
  EXPECT_EQ(receipt.poll(10 * second, 101 * second, true), FaultEvent::None);
  EXPECT_EQ(receipt.poll(10 * second, 101 * second + 1, true), FaultEvent::Unavailable);
}

TEST(FaultInput, SequenceGapsSourceRestartsAndOldDecisionsBreakContinuity)
{
  FaultInput gate;
  EXPECT_EQ(gate.receive(clear(1, 10 * second), 10 * second, 100 * second, true),
    FaultEvent::ClearWindow);
  EXPECT_EQ(gate.receive(clear(3, 11 * second), 11 * second, 101 * second, true),
    FaultEvent::Unavailable);
  EXPECT_EQ(gate.receive(clear(4, 11 * second + 1), 11 * second + 1, 101 * second + 1, true),
    FaultEvent::ClearWindow);
  EXPECT_FALSE(gate.recovery_mature(11 * second + 1));
  EXPECT_EQ(gate.receive(clear(2, 10 * second), 11 * second, 101 * second, true),
    FaultEvent::Unavailable);
  auto restarted = clear(0, 12 * second);
  restarted.source = "new-session";
  EXPECT_EQ(gate.receive(restarted, 12 * second, 102 * second, true), FaultEvent::Unavailable);
  EXPECT_EQ(gate.poll(12 * second, 102 * second, true), FaultEvent::Unavailable);
}

TEST(FaultInput, InvalidWindowsAndConflictingDuplicatesCannotClearFaults)
{
  FaultInput gate;
  auto message = clear(0, 10 * second);
  message.window_valid = false;
  EXPECT_EQ(gate.receive(message, 10 * second, 100 * second, true), FaultEvent::Unavailable);
  message.window_valid = true;
  EXPECT_EQ(gate.receive(message, 10 * second, 100 * second, true), FaultEvent::ClearWindow);
  message.imu_drift = true;
  EXPECT_EQ(gate.receive(message, 10 * second, 100 * second, true), FaultEvent::Unavailable);
  message.sequence = 1;
  message.stamp_ns += 1;
  EXPECT_EQ(gate.receive(message, message.stamp_ns, 100 * second + 1, true), FaultEvent::Alarm);
  gate.reset();
  EXPECT_EQ(gate.poll(message.stamp_ns, 100 * second + 2, true), FaultEvent::Unavailable);
}

TEST(FaultInput, LossFlagsDegradeGracefullyButCriticalFaultsAlwaysHold)
{
  FaultInput gate;
  auto message = clear(0, 10 * second);
  message.gps_outage = message.uav_loss = true;
  EXPECT_EQ(gate.receive(message, 10 * second, 100 * second, true), FaultEvent::ClearWindow);
  EXPECT_EQ(gate.poll(10 * second, 100 * second, false), FaultEvent::Alarm);
  message.sequence = 1;
  message.stamp_ns += 1;
  message.odometry_scale = true;
  EXPECT_EQ(gate.receive(message, message.stamp_ns, 100 * second + 1, true), FaultEvent::Alarm);
}

TEST(FaultInput, ExcessiveAcquisitionOrDeliveryGapCannotSupplyRecoveryEvidence)
{
  FaultInput acquisition, delivery;
  acquisition.receive(clear(0, 10 * second), 10 * second, 100 * second, true);
  delivery.receive(clear(0, 10 * second), 10 * second, 100 * second, true);
  EXPECT_EQ(acquisition.receive(clear(1, 12 * second), 12 * second, 100 * second + 1, true),
    FaultEvent::Unavailable);
  EXPECT_EQ(delivery.receive(clear(1, 10 * second + 1), 10 * second + 1, 102 * second, true),
    FaultEvent::Unavailable);
}

TEST(FaultInput, ReceiveValidatesSchemaSourceAndFutureTimestampBoundary)
{
  FaultInput gate;
  auto message = clear(0, 10 * second);
  message.schema_version = 2;
  EXPECT_EQ(gate.receive(message, 10 * second, 100 * second, true), FaultEvent::Unavailable);
  message.schema_version = 1;
  message.source.clear();
  EXPECT_EQ(gate.receive(message, 10 * second, 100 * second, true), FaultEvent::Unavailable);
  message.source = "session";
  message.stamp_ns += 50000000;
  EXPECT_EQ(gate.receive(message, 10 * second, 100 * second, true), FaultEvent::ClearWindow);
  ++message.sequence;
  ++message.stamp_ns;
  EXPECT_EQ(gate.receive(message, 10 * second, 100 * second, true), FaultEvent::Unavailable);
}

TEST(FaultInput, ExactOneSecondGapsRetainContinuity)
{
  FaultInput gate;
  EXPECT_EQ(gate.receive(clear(0, 10 * second), 10 * second, 100 * second, true),
    FaultEvent::ClearWindow);
  EXPECT_EQ(gate.receive(clear(1, 11 * second), 11 * second, 101 * second, true),
    FaultEvent::ClearWindow);
  EXPECT_EQ(gate.receive(clear(2, 12 * second), 12 * second, 102 * second, true),
    FaultEvent::ClearWindow);
}

TEST(GoalHistory, InvalidOrOldMessagesCannotCreateNewRevisions)
{
  ugv_control::GoalHistory goals;
  EXPECT_TRUE(goals.receive(true, 10 * second, 3, 0, 10 * second));
  EXPECT_TRUE(goals.take_new_goal());
  EXPECT_TRUE(goals.receive(true, 10 * second, 3, 0, 11 * second));
  EXPECT_FALSE(goals.take_new_goal());
  EXPECT_FALSE(goals.receive(true, 10 * second, 4, 0, 11 * second));
  EXPECT_FALSE(goals.receive(true, 9 * second, 3, 0, 11 * second));
  EXPECT_FALSE(goals.receive(false, 12 * second, 3, 0, 12 * second));
  EXPECT_TRUE(goals.receive(true, 10 * second, 3, 0, 12 * second));
  EXPECT_FALSE(goals.take_new_goal());
  goals.reset_epoch();
  EXPECT_FALSE(goals.available());
  EXPECT_FALSE(goals.receive(true, 10 * second, 3, 0, 2 * second));
  EXPECT_TRUE(goals.receive(true, 2 * second, 3, 0, 2 * second));
  EXPECT_TRUE(goals.take_new_goal());
}
}  // namespace

TEST(Supervisor, GoalRevisionIsConsumedByFirstDecisionBeforeCompletionDrift)
{
  ugv_control::GoalHistory goals;
  auto state = ugv_control::initial_supervisor(true);
  state.fault_latched = false;
  ASSERT_TRUE(goals.receive(true, second, 3.0, 0.0, second));
  // A detector callback can be the first decision for the accepted revision.
  auto callback = ready();
  callback.new_goal = goals.available() && goals.take_new_goal();
  callback.goal_reached = true;
  state = ugv_control::supervise(state, callback);
  ASSERT_TRUE(state.goal_completed);
  ASSERT_FALSE(goals.new_goal_pending());
  // Drift before the command timer cannot replay the revision and clear completion.
  auto timer = ready();
  timer.new_goal = goals.available() && goals.take_new_goal();
  timer.goal_reached = false;
  state = ugv_control::supervise(state, timer);
  EXPECT_EQ(state.mode, Mode::Goal);
  EXPECT_TRUE(state.goal_completed);
  EXPECT_FALSE(ugv_control::permits_motion(state));
}
