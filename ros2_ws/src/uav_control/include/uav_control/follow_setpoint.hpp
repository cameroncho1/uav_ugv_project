#ifndef UAV_CONTROL__FOLLOW_SETPOINT_HPP_
#define UAV_CONTROL__FOLLOW_SETPOINT_HPP_

#include <algorithm>
#include <cmath>

namespace uav_control
{
struct NorthEast
{
  double north{0.0}, east{0.0};
};

// Map ENU (x east, y north) to PX4 local NED, given the map position of PX4's local
// origin. This inverts uav_pose.py: map = [east, north, -down] + origin.
inline NorthEast map_to_ned(double x, double y, double origin_x, double origin_y)
{
  return {y - origin_y, x - origin_x};
}

// Move the commanded point toward the target by at most max_step. Offsets within
// deadband are ignored so estimate jitter does not shake the camera.
inline NorthEast step_toward(
  const NorthEast & current, const NorthEast & target, double max_step, double deadband)
{
  const auto dn = target.north - current.north, de = target.east - current.east;
  const auto distance = std::hypot(dn, de);
  if (!std::isfinite(distance) || distance <= deadband) {return current;}
  const auto scale = std::min(1.0, max_step / distance);
  return {current.north + dn * scale, current.east + de * scale};
}
}  // namespace uav_control
#endif  // UAV_CONTROL__FOLLOW_SETPOINT_HPP_
