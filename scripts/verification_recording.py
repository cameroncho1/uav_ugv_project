"""Numerical rosbag recording and provenance shared by evaluation fixtures."""

import hashlib
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


class Recording:
    def __init__(self, directory, setup="controlled_navigation", world=None):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.setup = ROOT / "verification/setups" / (setup + ".yaml")
        config = yaml.safe_load(self.setup.read_text())
        topics = ["/clock", *config["topics"]]
        qos = {
            topic: {
                "history": "keep_last",
                "depth": 1000,
                "reliability": "best_effort",
                "durability": "transient_local"
                if topic
                in (
                    "/global_goal_pose",
                    "/local_grid",
                    "/ugv/navigation/route",
                    "/evaluation/faults",
                )
                else "volatile",
            }
            for topic in topics
        }
        qos_path = self.directory / "recorder_qos.yaml"
        qos_path.write_text(yaml.safe_dump(qos))
        self.log = (self.directory / "recorder.log").open("w")
        started_steady_ns = time.monotonic_ns()
        self.process = subprocess.Popen(
            [
                "ros2",
                "bag",
                "record",
                "--storage",
                "sqlite3",
                "-o",
                str(self.directory / "bag"),
                "--qos-profile-overrides-path",
                str(qos_path),
                *topics,
            ],
            stdout=self.log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self.events = (self.directory / "events.jsonl").open("w")
        self.event_sequence = 0
        self.world = world
        source_hashes = {}
        for tree in (
            "ros2_ws/src",
            "scripts",
            "verification/src",
            "verification/setups",
            "simulation",
        ):
            for path in sorted((ROOT / tree).rglob("*")):
                if (
                    path.is_file()
                    and "px4_msgs" not in path.parts
                    and path.suffix
                    in (".py", ".cpp", ".hpp", ".msg", ".yaml", ".sdf", ".sh")
                ):
                    source_hashes[str(path.relative_to(ROOT))] = hashlib.sha256(
                        path.read_bytes()
                    ).hexdigest()
        self.provenance = {
            "schema_version": 1,
            "status": "aborted",
            "started_steady_ns": started_steady_ns,
            "boot_id": Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
            "git_commit": subprocess.check_output(
                ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
            ).strip()
            if (ROOT / ".git").exists()
            else "runtime-mirror",
            "world": str(world) if world else None,
            "python": sys.version,
            "declared_simulation_dependencies": json.loads(
                (ROOT / "simulation/dependencies.json").read_text()
            ),
            "source_sha256": source_hashes,
            "parameters": {
                k: v
                for k, v in os.environ.items()
                if k
                in (
                    "WORLD_FILE",
                    "GOAL_X",
                    "GOAL_Y",
                    "UAV_ASSISTANCE",
                    "SEED",
                    "INITIAL_GPS_ENABLED",
                    "ROS_DOMAIN_ID",
                    "GZ_PARTITION",
                    "GPS_OUTAGE_AFTER_MOTION",
                    "TRIAL_TIMEOUT",
                    "WALL_TIMEOUT",
                    "UAV_ALTITUDE",
                    "UAV_HEADING",
                )
            },
        }

    def command_loss(self, label):
        stamp = time.monotonic_ns()
        self.events.write(
            json.dumps(
                {
                    "time_ns": stamp,
                    "clock": "steady:" + self.provenance["boot_id"],
                    "epoch": 0,
                    "source": "trial_driver",
                    "sequence": self.event_sequence,
                    "data": {"kind": "command_loss", "event_id": label},
                }
            )
            + "\n"
        )
        self.events.flush()
        self.event_sequence += 1
        return stamp

    def close(self, status="aborted"):
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGTERM)
                try:
                    self.process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(self.process.pid, signal.SIGKILL)
                    self.process.wait(timeout=5)
        self.log.close()
        self.events.close()
        if self.process.returncode != 0:
            status = "failed"
        self.provenance.update(
            status=status,
            recorder_exit_code=self.process.returncode,
            ended_steady_ns=time.monotonic_ns(),
        )
        (self.directory / "recording.json").write_text(
            json.dumps(self.provenance, indent=2) + "\n"
        )

    def normalize(self):
        from uav_ugv_verification.adapters.rosbag import import_bag

        return import_bag(
            self.directory / "bag",
            self.setup,
            self.directory / "normalized",
            status=self.provenance["status"],
            world=self.world,
            events=self.directory / "events.jsonl",
        )


def stop_recording(_signum, _frame):
    raise KeyboardInterrupt


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--world", type=Path, required=True)
    parser.add_argument("--result", type=Path)
    args = parser.parse_args()
    recording = Recording(args.directory, "camera_navigation", args.world)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop_recording)
    try:
        while recording.process.poll() is None:
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        status = "aborted"
        if args.result and args.result.is_file():
            status = (
                "completed"
                if json.loads(args.result.read_text())["passed"]
                else "failed"
            )
        recording.close(status)
        recording.normalize()
