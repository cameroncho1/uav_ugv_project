"""Bind a reusable timing profile to a recording's verified Linux boot clock."""

from pathlib import Path

import yaml

from ..validation import UniqueLoader, load_profile, require
from .files import FileReader


def bind_profile(run: Path, template: Path, output: Path) -> dict:
    reader = FileReader(run)
    reader.inspection()
    clocks = {i.clock for i in reader.manifest.intervals if i.clock.startswith("steady:")}
    raw = yaml.load(template.read_text(encoding="utf-8-sig"), Loader=UniqueLoader)
    for rule in raw["requirements"].values():
        if rule.get("clock") == "steady:recording":
            require(len(clocks) == 1, "profile binding requires one verified steady clock")
            rule["clock"] = next(iter(clocks))
    require(not output.exists(), "bound profile already exists")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8", newline="\n")
    load_profile(output)
    return {"profile": str(output), "steady_clocks": sorted(clocks)}
