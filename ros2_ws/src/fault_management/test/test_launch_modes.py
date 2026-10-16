"""Source mode defaults must select a producer compatible with its anchor."""

import importlib.util
from pathlib import Path
import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "launch/evaluation_launch.py"
SPEC = importlib.util.spec_from_file_location("evaluation_launch", SCRIPT)
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


def test_camera_defaults_are_complete():
    assert module.resolve_mode("camera") == ("simple_goal_demo", "cooperative_camera_uav_0",
                                            "false", "/fmu/out/vehicle_odometry")


def test_simulation_defaults_keep_fixed_observer():
    assert module.resolve_mode("simulated") == ("uav_ugv", "", "true", "/simulation/uav_odometry")


@pytest.mark.parametrize("kwargs", [{"publish": "true"}, {"topic": "/simulation/uav_odometry"}])
def test_camera_rejects_missing_synthetic_anchor_producer(kwargs):
    with pytest.raises(ValueError):
        module.resolve_mode("camera", **kwargs)
