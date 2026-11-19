"""Requirement checks on normalized records; no simulator-dependent logic."""

from .detector import detection, false_alarms
from .navigation import arrival, clearance_check, localization
from .responses import guard_response, hazard_response, watchdog_response

MONITORS = {
    "R1": (clearance_check, ("truth_pose", "obstacles")),
    "R2": (arrival, ("truth_pose", "estimated_pose", "requested_command", "goal")),
    "R3": (guard_response, ("guard_state", "requested_command")),
    "R4": (watchdog_response, ("events", "actuator_input", "actuator_output", "measured_motion")),
    "R5": (hazard_response, ("events", "route", "obstacles", "requested_command")),
    "R6": (localization, ("estimated_pose", "truth_pose", "guard_state", "uav_observation")),
    "R7": (detection, ("events", "detector_decision")),
    "R8": (false_alarms, ("events", "detector_decision")),
}
