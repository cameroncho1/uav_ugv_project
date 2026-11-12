"""Frozen experiment plans, append-only attempt accounting, and scalar objectives.

This module has no ROS, simulator, or process-launch dependencies.
"""

from __future__ import annotations

import hashlib
import json
import os
import sqlite3
import time
from pathlib import Path

import yaml

from .validation import UniqueLoader, digest_file, require


def identity(value: dict) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def source_snapshot(repository: Path) -> dict:
    files = {}
    for tree in (
        "scripts",
        "ros2_ws/src",
        "verification/src",
        "verification/setups",
        "verification/profiles",
        "verification/experiments",
        "simulation",
    ):
        for path in sorted((repository / tree).rglob("*")):
            if (
                path.is_file()
                and "px4_msgs" not in path.parts
                and (
                    tree == "simulation"
                    or path.name == "CMakeLists.txt"
                    or path.suffix
                    in (
                        ".py",
                        ".cpp",
                        ".hpp",
                        ".yaml",
                        ".json",
                        ".sdf",
                        ".sh",
                        ".msg",
                        ".xml",
                        ".md",
                        ".tex",
                    )
                )
            ):
                files[str(path.relative_to(repository))] = digest_file(path)
    for name in ("verification/pyproject.toml", "verification/requirements-tested.txt"):
        files[name] = digest_file(repository / name)
    return dict(sha256=identity(files), files=files)


def load_plan(path: Path, repository: Path) -> dict:
    raw = yaml.load(path.read_text(encoding="utf-8"), Loader=UniqueLoader)
    require(raw.get("schema_version") == 1, "unsupported experiment schema")
    require(
        raw.get("seeds") and len(set(raw["seeds"])) == len(raw["seeds"]), "unique seeds required"
    )
    require(all(type(s) is int and s >= 0 for s in raw["seeds"]), "invalid seeds")
    cases = []
    for row in raw["rows"]:
        require(row["kind"] in ("navigation", "guard", "watchdog"), "unsupported experiment kind")
        for seed in raw["seeds"]:
            case = {**row, "seed": seed}
            if row["kind"] == "navigation":
                world = (repository / row["world"]).resolve()
                require(
                    world.is_relative_to(repository.resolve()) and world.is_file(),
                    "invalid world path",
                )
                case["world_sha256"] = digest_file(world)
                require(len(row["goal"]) == 2, "planar goal required")
            case["case_id"] = row["id"] + f"-s{seed}"
            cases.append(case)
    require(len({c["case_id"] for c in cases}) == len(cases), "duplicate case IDs")
    profiles = {
        str(p.relative_to(repository)): digest_file(p)
        for p in sorted((repository / "verification/profiles").glob("m2_*.yaml"))
    }
    plan = dict(
        schema_version=1,
        id=raw["id"],
        plan_sha256=digest_file(path),
        parameters=raw.get("parameters", {}),
        exclusions=raw.get("exclusions", {}),
        profiles=profiles,
        cases=cases,
    )
    plan["expanded_sha256"] = identity(plan)
    return plan


class Ledger:
    """Durable attempts survive interruption; terminal rows are never overwritten."""

    def __init__(self, root: Path, plan: dict, source_sha256: str):
        root.mkdir(parents=True, exist_ok=True)
        self.root = root
        self.source_sha256 = source_sha256
        self.connection = sqlite3.connect(root / "ledger.sqlite3", timeout=30)
        self.connection.execute("PRAGMA journal_mode=WAL")
        self.connection.execute("PRAGMA synchronous=FULL")
        self.connection.executescript(
            "CREATE TABLE IF NOT EXISTS metadata(key TEXT PRIMARY KEY, value TEXT);"
            "CREATE TABLE IF NOT EXISTS attempts("
            "id INTEGER PRIMARY KEY, case_id TEXT, category TEXT, configuration TEXT,"
            "started_ns INTEGER, ended_ns INTEGER, status TEXT, detail TEXT);"
        )
        expected = {"plan": json.dumps(plan, sort_keys=True), "source_sha256": source_sha256}
        previous = dict(self.connection.execute("SELECT key,value FROM metadata"))
        if any(key in previous and previous[key] != value for key, value in expected.items()):
            self.connection.close()
            require(False, "campaign frozen inputs changed; use a new cohort directory")
        with self.connection:
            for key, value in expected.items():
                before = self.connection.execute(
                    "SELECT value FROM metadata WHERE key=?", (key,)
                ).fetchone()
                require(
                    before is None or before[0] == value,
                    "campaign frozen inputs changed; use a new cohort directory",
                )
                self.connection.execute("INSERT OR IGNORE INTO metadata VALUES(?,?)", (key, value))
        self.export()

    def recover(self) -> None:
        # The caller must hold the campaign's exclusive runtime lock first.
        with self.connection:
            self.connection.execute(
                "UPDATE attempts SET status='interrupted',ended_ns=?,detail=? "
                "WHERE status='running'",
                (
                    time.time_ns(),
                    json.dumps({"reason": "previous runner exited without terminal status"}),
                ),
            )
        self.export()

    def start(self, case: dict, category="matrix") -> dict:
        with self.connection:
            cursor = self.connection.execute(
                "INSERT INTO attempts(case_id,category,configuration,started_ns,status,detail) "
                "VALUES(?,?,?,?,?,?)",
                (
                    case["case_id"],
                    category,
                    json.dumps(case, sort_keys=True),
                    time.time_ns(),
                    "running",
                    "{}",
                ),
            )
        attempt = {
            "id": cursor.lastrowid,
            "directory": str(self.root / "attempts" / f"{cursor.lastrowid:04d}-{case['case_id']}"),
        }
        Path(attempt["directory"]).mkdir(parents=True)
        self.export()
        return attempt

    def finish(self, attempt_id: int, status: str, detail: dict) -> None:
        require(
            status
            in (
                "passed",
                "failed",
                "inconclusive",
                "startup_failed",
                "timed_out",
                "interrupted",
                "invalid",
            ),
            "invalid terminal status",
        )
        with self.connection:
            cursor = self.connection.execute(
                "UPDATE attempts SET ended_ns=?,status=?,detail=? WHERE id=? AND status='running'",
                (
                    time.time_ns(),
                    status,
                    json.dumps(detail, sort_keys=True, allow_nan=False),
                    attempt_id,
                ),
            )
            require(cursor.rowcount == 1, "attempt is absent or already terminal")
        self.export()

    def rows(self) -> list[dict]:
        return [
            dict(
                id=row[0],
                case_id=row[1],
                category=row[2],
                configuration=json.loads(row[3]),
                started_ns=row[4],
                ended_ns=row[5],
                status=row[6],
                detail=json.loads(row[7]),
            )
            for row in self.connection.execute("SELECT * FROM attempts ORDER BY id")
        ]

    def attempted(self, case_id: str, category="matrix") -> bool:
        return (
            self.connection.execute(
                "SELECT 1 FROM attempts WHERE case_id=? AND category=? "
                "AND status NOT IN ('running','interrupted')",
                (case_id, category),
            ).fetchone()
            is not None
        )

    def export(self) -> None:
        metadata = dict(self.connection.execute("SELECT key,value FROM metadata"))
        value = dict(
            schema_version=1,
            plan=json.loads(metadata["plan"]),
            source_sha256=metadata["source_sha256"],
            attempts=self.rows(),
        )
        target = self.root / "ledger.json"
        temporary = target.with_suffix(".tmp")
        with temporary.open("w", encoding="utf-8", newline="\n") as handle:
            json.dump(value, handle, indent=2, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        temporary.replace(target)

    def close(self):
        self.connection.close()


def objective(result: dict, requirement="R1") -> float | None:
    selected = next(r for r in result["results"] if r["requirement"] == requirement)
    if selected["verdict"] not in ("PASS", "FAIL") or selected["missing_data_count"]:
        return None
    key = {
        "R1": "minimum_clearance_m",
        "R3": "minimum_response_slack_ns",
        "R5": "minimum_response_slack_ns",
    }[requirement]
    values = [row[key] for row in selected["metrics"].get("by_interval", []) if key in row]
    return min(values) if values else None
