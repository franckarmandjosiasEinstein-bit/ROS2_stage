"""report -- one acceptance report per commissioning stage.

Why this exists. A commissioning stage is not "we ran it and it looked fine".
It is a numbered test with a written criterion, a measured value, and a verdict
that a third party can check afterwards. Every stage node in this package ends
by calling `Report.finish()`, which:

  * prints a human-readable verdict to the console (what you read on site),
  * writes results/stage<N>_<slug>_<timestamp>.json (what you keep),
  * writes results/stage<N>_<slug>_<timestamp>.md (what goes in the report).

The JSON matters more than it looks. Stage 2 measures the real odometry drift;
stage 5 has to compare against it; the Phase B report quotes both. Retyping
numbers off a terminal is how measurements get corrupted, so nothing here is
only printed.

A stage with no PASS/FAIL criterion is not a stage. If a check cannot be
expressed as "measured value <op> threshold", it belongs in the field notes,
not here.
"""

from __future__ import annotations

import json
import os
import platform
import socket
from datetime import datetime, timezone


# Where reports land. Overridable so a field laptop can write to a USB stick:
#   export YOUBOT_COMMISSIONING_RESULTS=/media/usb/serre_2026_04
DEFAULT_RESULTS = os.path.expanduser("~/youbot_commissioning_results")


def results_dir() -> str:
    path = os.environ.get("YOUBOT_COMMISSIONING_RESULTS", DEFAULT_RESULTS)
    os.makedirs(path, exist_ok=True)
    return path


class Check:
    """One pass/fail criterion: a measured value against a threshold."""

    OPS = {
        "<":  lambda a, b: a < b,
        "<=": lambda a, b: a <= b,
        ">":  lambda a, b: a > b,
        ">=": lambda a, b: a >= b,
        "==": lambda a, b: a == b,
    }

    def __init__(self, name: str, measured, op: str, threshold,
                 unit: str = "", note: str = ""):
        if op not in self.OPS:
            raise ValueError(f"unknown comparison {op!r}")
        self.name = name
        self.measured = measured
        self.op = op
        self.threshold = threshold
        self.unit = unit
        self.note = note

    @property
    def passed(self) -> bool:
        if self.measured is None:
            return False          # not measured is not the same as passed
        return bool(self.OPS[self.op](self.measured, self.threshold))

    def _fmt(self, v) -> str:
        if v is None:
            return "not measured"
        if isinstance(v, float):
            return f"{v:.4g}{(' ' + self.unit) if self.unit else ''}"
        return f"{v}{(' ' + self.unit) if self.unit else ''}"

    def line(self) -> str:
        mark = "PASS" if self.passed else "FAIL"
        return (f"  [{mark}] {self.name}: {self._fmt(self.measured)} "
                f"(required {self.op} {self._fmt(self.threshold)})")

    def as_dict(self) -> dict:
        return {
            "name": self.name,
            "measured": self.measured,
            "op": self.op,
            "threshold": self.threshold,
            "unit": self.unit,
            "note": self.note,
            "passed": self.passed,
        }


class Report:
    """Accumulates checks and observations for one stage, then writes them out.

    `logger` is the node's rclpy logger, so the verdict lands in the same
    console stream as everything else the stage printed.
    """

    def __init__(self, stage: int, slug: str, title: str, logger=None,
                 platform_name: str = "unknown"):
        self.stage = stage
        self.slug = slug
        self.title = title
        self.logger = logger
        self.platform_name = platform_name     # "sim" or "hardware"
        self.started = datetime.now(timezone.utc)
        self.checks: list[Check] = []
        self.data: dict = {}                   # free-form measured quantities
        self.notes: list[str] = []

    # --- collecting ---------------------------------------------------
    def check(self, name, measured, op, threshold, unit="", note="") -> Check:
        c = Check(name, measured, op, threshold, unit, note)
        self.checks.append(c)
        return c

    def record(self, key: str, value) -> None:
        """A measured quantity with no pass/fail of its own, but that a later
        stage or the written report will need (e.g. the UMBmark coefficients)."""
        self.data[key] = value

    def note(self, text: str) -> None:
        self.notes.append(text)
        self._say(text)

    def _say(self, text: str) -> None:
        if self.logger is not None:
            self.logger.info(text)
        else:
            print(text)

    # --- verdict ------------------------------------------------------
    @property
    def passed(self) -> bool:
        return bool(self.checks) and all(c.passed for c in self.checks)

    def finish(self) -> bool:
        """Print, write JSON + Markdown, return the overall verdict."""
        verdict = "PASS" if self.passed else "FAIL"
        if not self.checks:
            verdict = "INCONCLUSIVE"

        self._say("")
        self._say(f"===== STAGE {self.stage}: {self.title} -- {verdict} =====")
        for c in self.checks:
            self._say(c.line())
            if c.note:
                self._say(f"         {c.note}")
        if not self.checks:
            self._say("  no criterion was evaluated -- the stage did not "
                      "collect enough data to conclude.")
        self._say("")

        stamp = self.started.strftime("%Y%m%d_%H%M%S")
        base = os.path.join(results_dir(),
                            f"stage{self.stage}_{self.slug}_{stamp}")
        payload = {
            "stage": self.stage,
            "slug": self.slug,
            "title": self.title,
            "platform": self.platform_name,
            "verdict": verdict,
            "started_utc": self.started.isoformat(),
            "finished_utc": datetime.now(timezone.utc).isoformat(),
            "host": socket.gethostname(),
            "os": platform.platform(),
            "checks": [c.as_dict() for c in self.checks],
            "data": self.data,
            "notes": self.notes,
        }
        try:
            with open(base + ".json", "w") as fh:
                json.dump(payload, fh, indent=2, default=str)
            with open(base + ".md", "w") as fh:
                fh.write(self._markdown(verdict))
            self._say(f"report written to {base}.json / .md")
        except OSError as exc:
            self._say(f"WARNING: could not write the report: {exc}")

        return self.passed

    def _markdown(self, verdict: str) -> str:
        out = [f"# Stage {self.stage} --- {self.title}", ""]
        out.append(f"**Verdict: {verdict}**")
        out.append("")
        out.append(f"- Platform: `{self.platform_name}`")
        out.append(f"- Started: {self.started.isoformat()}")
        out.append(f"- Host: `{socket.gethostname()}`")
        out.append("")
        out.append("## Criteria")
        out.append("")
        out.append("| Criterion | Measured | Required | Verdict |")
        out.append("|---|---|---|---|")
        for c in self.checks:
            unit = f" {c.unit}" if c.unit else ""
            meas = "not measured" if c.measured is None else (
                f"{c.measured:.4g}{unit}" if isinstance(c.measured, float)
                else f"{c.measured}{unit}")
            thr = (f"{c.threshold:.4g}{unit}"
                   if isinstance(c.threshold, float) else f"{c.threshold}{unit}")
            out.append(f"| {c.name} | {meas} | `{c.op} {thr}` | "
                       f"{'PASS' if c.passed else 'FAIL'} |")
        if self.data:
            out.append("")
            out.append("## Measured quantities")
            out.append("")
            out.append("```json")
            out.append(json.dumps(self.data, indent=2, default=str))
            out.append("```")
        if self.notes:
            out.append("")
            out.append("## Field notes")
            out.append("")
            for n in self.notes:
                out.append(f"- {n}")
        out.append("")
        return "\n".join(out)
