// Batch conformance interface to the same pure transition used by goal_controller.
#include <iostream>
#include <stdexcept>
#include <string>
#include <ugv_control/supervisor.hpp>

int main(int argc, char ** argv)
{
  if (argc == 2 && std::string(argv[1]) == "--identity") {
    std::cout << ugv_control::kSupervisorPolicySha256 << ' ' <<
      ugv_control::kSupervisorSourcesSha256 << '\n';
    return 0;
  }
  int enabled, mode, fault, count, age, completed;
  while (std::cin >> enabled >> mode >> fault >> count >> age >> completed) {
    int goal, pose, route, reached, revised, event, mature, reset;
    if (!(std::cin >> goal >> pose >> route >> reached >> revised >> event >> mature >> reset)) {
      std::cerr << "Incomplete conformance input\n";
      return 2;
    }
    if (mode < 0 || mode > 4 || age < 0 || age > 2 || count < 0 ||
      count > static_cast<int>(ugv_control::kClearWindows) || event < 0 || event > 3)
    {
      std::cerr << "Invalid conformance state/input\n";
      return 2;
    }
    const ugv_control::SupervisorState before{
      enabled != 0, static_cast<ugv_control::Mode>(mode), fault != 0,
      static_cast<unsigned>(count), static_cast<ugv_control::RecoveryAge>(age), completed != 0};
    const ugv_control::SupervisorInput input{
      goal != 0, pose != 0, route != 0, reached != 0, revised != 0,
      static_cast<ugv_control::FaultEvent>(event), mature != 0, reset != 0};
    const auto after = ugv_control::supervise(before, input);
    std::cout << after.detector_enabled << ' ' << static_cast<int>(after.mode) << ' ' <<
      after.fault_latched << ' ' << after.clear_count << ' ' <<
      static_cast<int>(after.recovery_age) << ' ' << after.goal_completed << ' ' <<
      ugv_control::permits_motion(after) << '\n';
  }
  return std::cin.eof() ? 0 : 2;
}
