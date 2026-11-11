#!/usr/bin/env python3
"""Exercise actual Gazebo creation/removal in a private headless world."""

import argparse
import json
import os
import subprocess
import time
from pathlib import Path

from simulation_support.obstacle_scenarios import Box, Operation
from simulation_support.obstacle_world import GazeboWorld
from verification_experiments import process_groups, stop


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    world = args.output / "world.sdf"
    world.write_text("""<sdf version="1.9"><world name="obstacle_lifecycle">
      <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
      <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
      <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    </world></sdf>""")
    os.environ["GZ_PARTITION"] = f"obstacle_lifecycle_{os.getpid()}"
    results = []
    with (args.output / "world.log").open("w") as log:
        process = subprocess.Popen(
            ["gz", "sim", "-s", "-r", str(world)],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        groups = set()
        try:
            adapter = GazeboWorld("obstacle_lifecycle")
            deadline = time.monotonic() + 15
            while True:
                groups |= process_groups(process.pid)
                try:
                    adapter.scene()
                    break
                except RuntimeError:
                    if process.poll() is not None or time.monotonic() > deadline:
                        raise
                    time.sleep(0.1)
            boxes = [
                Box("first", (0.6, 0.6, 0.4), (1.0, 2.0, 0.2, 0.7)),
                Box("second", (1.0, 0.4, 0.6), (-2.0, -1.0, 0.3, -0.4)),
            ]
            for index, box in enumerate(boxes):
                results.append(adapter.apply(Operation(f"spawn_{index}", 0, "spawn", box.id), box))
            try:
                adapter.apply(Operation("duplicate", 0, "spawn", boxes[0].id), boxes[0])
            except RuntimeError as exc:
                assert "existing model" in str(exc)
                results.append(dict(case="duplicate_rejected", passed=True))
            else:
                raise AssertionError("duplicate model name was accepted")
            results.append(adapter.apply(Operation("remove", 0, "remove", boxes[0].id), boxes[0]))
            assert boxes[0].model_name not in adapter.scene()
            assert boxes[1].model_name in adapter.scene()
            results.append(adapter.apply(Operation("respawn", 0, "spawn", boxes[0].id), boxes[0]))
            (args.output / "result.json").write_text(
                json.dumps(dict(passed=True, cases=results), indent=2) + "\n"
            )
        finally:
            stop(process, groups)
    print(json.dumps(dict(passed=True, cases=len(results))))


if __name__ == "__main__":
    main()
