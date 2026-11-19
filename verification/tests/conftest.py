"""Small declared traces, independent of simulator implementations."""

import json
from itertools import count

import pytest
import yaml

NS = 1_000_000_000


@pytest.fixture
def run_factory(tmp_path):
    serial = count()

    def create(
        streams,
        *,
        end=3 * NS,
        start=0,
        clock="sim",
        status="completed",
        source_id=None,
        intervals=None,
        run_id=None,
    ):
        index = next(serial)
        root = tmp_path / f"run_{index}"
        root.mkdir()
        manifest = {
            "schema_version": 1,
            "run_id": run_id or f"run_{index}",
            "source_run_id": source_id or f"source_{index}",
            "adapter": "normalized",
            "status": status,
            "intervals": intervals
            or [{"clock": clock, "epoch": 0, "start_ns": start, "end_ns": end}],
            "streams": {},
            "provenance": {"kind": "synthetic", "code_commit": "fixture"},
        }
        for name, rows in streams.items():
            manifest["streams"][name] = f"{name}.jsonl"
            normalized = []
            for sequence, item in enumerate(rows):
                item = dict(item)
                data = item.pop("data")
                record = {
                    "time_ns": item.pop("time_ns"),
                    "clock": clock,
                    "epoch": 0,
                    "sequence": sequence,
                    "source": name,
                    "data": data,
                    **item,
                }
                if name in ("truth_pose", "estimated_pose", "goal", "route"):
                    record.setdefault("frame", "map")
                normalized.append(record)
            (root / f"{name}.jsonl").write_text(
                "".join(json.dumps(r) + "\n" for r in normalized), encoding="utf-8"
            )
        (root / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
        return root

    return create


@pytest.fixture
def profile_factory(tmp_path):
    serial = count()

    def create(requirement="R3", *, parameters=None, gap=10 * NS, clock="sim", rules=None):
        path = tmp_path / f"profile_{next(serial)}.yaml"
        raw = {
            "schema_version": 1,
            "id": "test",
            "version": "1",
            "calibration": "fixture",
            "max_gap_ns": gap,
            "requirements": rules
            if rules is not None
            else {
                requirement: {
                    "applicable": True,
                    "mandatory": True,
                    "clock": clock,
                    "parameters": parameters or {},
                }
            },
        }
        path.write_text(yaml.safe_dump(raw), encoding="utf-8")
        return path

    return create


def sample(time_ns, **data):
    return {"time_ns": time_ns, "data": data}


def command(time_ns, linear=0.0, angular=0.0, **extra):
    return sample(time_ns, linear=linear, angular=angular, **extra)


def guard(time_ns, valid):
    return sample(time_ns, guard_valid=valid, reason="Ready" if valid else "stale")


def event(time_ns, kind, event_id="fault", **data):
    return sample(time_ns, kind=kind, event_id=event_id, **data)


def detector(time_ns, alarm=False, **data):
    return sample(time_ns, alarm=alarm, scope="binary", **data)


def fault(time_ns, kind="fault_start", event_id="fault"):
    return event(time_ns, kind, event_id, sensor="imu", channel="wz", fault_class="noise")


def result(evaluation, requirement):
    return next(r for r in evaluation.results if r.requirement == requirement)
