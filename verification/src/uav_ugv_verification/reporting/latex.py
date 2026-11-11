"""Compile static report artifacts without shell escape or installer dialogs."""

import re
import subprocess
from pathlib import Path

from ..validation import require


def compile_pdf(output: Path, stem="paper", *, maximum_pages=10) -> int:
    command = ["pdflatex", "-no-shell-escape", "-interaction=nonstopmode", "-halt-on-error"]
    if "MiKTeX" in subprocess.check_output(["pdflatex", "--version"], text=True):
        command.append("--disable-installer")
    with (output / "latex.log").open("w") as log:
        subprocess.run(
            [*command, stem + ".tex"],
            cwd=output,
            stdout=log,
            stderr=subprocess.STDOUT,
            check=True,
            timeout=120,
        )
    text = (output / (stem + ".log")).read_text(errors="replace")
    match = re.search(r"Output written on .*?\.pdf \((\d+) pages?", text)
    require(match is not None, "LaTeX output page count is unavailable")
    pages = int(match.group(1))
    require(pages <= maximum_pages, f"Generated report exceeds {maximum_pages} pages")
    return pages
