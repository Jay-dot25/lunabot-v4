#!/usr/bin/env python3
"""Run the complete LunaBot static validation suite and produce one report."""

from __future__ import annotations

import argparse
import ast
import json
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "config/validation.yaml"


def load_config(path: Path) -> dict:
    """Load the JSON-compatible YAML config using only the standard library."""
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid validation configuration {path}: {exc}") from exc
    required = {"python_roots", "shell_roots", "phase_validators"}
    missing = sorted(required - value.keys())
    if missing:
        raise ValueError(f"validation configuration missing: {', '.join(missing)}")
    return value


def result(name: str, category: str, status: str, detail: str = "",
           duration: float = 0.0) -> dict:
    return {
        "name": name,
        "category": category,
        "status": status,
        "detail": detail,
        "duration_seconds": round(duration, 3),
    }


def timed_check(name: str, category: str, function: Callable[[], str]) -> dict:
    started = time.monotonic()
    try:
        detail = function()
        return result(name, category, "pass", detail, time.monotonic() - started)
    except Exception as exc:  # each failure belongs in the aggregate report
        return result(name, category, "fail", str(exc), time.monotonic() - started)


def discover_files(roots: list[str], pattern: str) -> list[Path]:
    files: set[Path] = set()
    excluded = {"build", "install", "log", "__pycache__", ".venv", ".git"}
    for relative in roots:
        path = ROOT / relative
        if path.is_file() and path.match(pattern):
            files.add(path)
        elif path.is_dir():
            files.update(
                candidate for candidate in path.rglob(pattern)
                if candidate.is_file()
                and not excluded.intersection(candidate.relative_to(ROOT).parts)
            )
    return sorted(files)


def python_check(path: Path) -> str:
    ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    compile(path.read_text(encoding="utf-8"), str(path), "exec")
    return "syntax and bytecode compilation succeeded"


def shell_check(path: Path) -> str:
    proc = subprocess.run(
        ["bash", "-n", str(path)], cwd=ROOT, capture_output=True, text=True,
        check=False,
    )
    if proc.returncode:
        raise RuntimeError(proc.stderr.strip() or f"bash -n exited {proc.returncode}")
    return "bash -n succeeded"


def xml_check(path: Path) -> str:
    ET.parse(path)
    return "XML is well formed"


def snapshot_reports() -> dict[Path, bytes | None]:
    paths = set(ROOT.glob("evidence/**/static_validation.txt"))
    return {path: path.read_bytes() if path.exists() else None for path in paths}


def restore_reports(saved: dict[Path, bytes | None]) -> None:
    current = set(ROOT.glob("evidence/**/static_validation.txt"))
    for path in current | set(saved):
        original = saved.get(path)
        if original is None:
            if path.exists() and path not in saved:
                path.unlink()
        else:
            path.write_bytes(original)


def phase_check(path: Path, preserve: bool) -> tuple[str, str, float]:
    saved = snapshot_reports() if preserve else {}
    started = time.monotonic()
    try:
        proc = subprocess.run(
            [sys.executable, str(path)], cwd=ROOT, capture_output=True, text=True,
            check=False, timeout=180,
        )
    except subprocess.TimeoutExpired as exc:
        return "fail", f"timed out after {exc.timeout} seconds", time.monotonic() - started
    finally:
        if preserve:
            restore_reports(saved)
    output = "\n".join((proc.stdout + "\n" + proc.stderr).strip().splitlines()[-8:])
    status = "pass" if proc.returncode == 0 else "fail"
    detail = f"exit={proc.returncode}"
    if output:
        detail += "; tail: " + output.replace("\n", " | ")
    return status, detail, time.monotonic() - started


def git_metadata() -> dict:
    def command(*args: str) -> str:
        proc = subprocess.run(
            ["git", *args], cwd=ROOT, capture_output=True, text=True, check=False,
        )
        return proc.stdout.strip() if proc.returncode == 0 else "unavailable"
    return {
        "commit": command("rev-parse", "HEAD"),
        "branch": command("branch", "--show-current"),
    }


def write_reports(report: dict, directory: Path) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "validation_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8")
    lines = [
        "# LunaBot Baseline Validation Report",
        "",
        f"- Generated: `{report['generated_at_utc']}`",
        f"- Commit: `{report['git']['commit']}`",
        f"- Branch: `{report['git']['branch']}`",
        f"- Result: **{report['summary']['overall'].upper()}**",
        f"- Checks: {report['summary']['passed']} passed, "
        f"{report['summary']['failed']} failed, {report['summary']['skipped']} skipped",
        "",
        "## Checks",
        "",
        "| Status | Category | Check | Detail |",
        "|---|---|---|---|",
    ]
    for item in report["checks"]:
        detail = item["detail"].replace("|", "\\|").replace("\n", " ")
        lines.append(
            f"| {item['status'].upper()} | {item['category']} | "
            f"`{item['name']}` | {detail} |")
    lines.extend([
        "",
        "## Meaning",
        "",
        "This report proves repository-level static integrity only. It does not "
        "claim that ROS 2, Gazebo, SLAM, navigation, or the final mission ran on "
        "this machine. Runtime acceptance must be performed on a workstation "
        "that passes `python3 tools/check_environment.py`.",
        "",
    ])
    (directory / "validation_report.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--no-phase-validators", action="store_true",
                        help="run syntax/XML checks but skip legacy validators")
    parser.add_argument("--no-write", action="store_true",
                        help="do not update evidence/baseline reports")
    args = parser.parse_args()
    try:
        config = load_config(args.config)
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    checks: list[dict] = []
    python_files = discover_files(config["python_roots"], "*.py")
    for path in python_files:
        rel = path.relative_to(ROOT).as_posix()
        checks.append(timed_check(rel, "python", lambda p=path: python_check(p)))

    shell_files = discover_files(config["shell_roots"], "*.sh")
    if config.get("shell_root_launchers", True):
        shell_files.extend(sorted(path for path in ROOT.glob("launch-*") if path.is_file()))
    for path in sorted(set(shell_files)):
        rel = path.relative_to(ROOT).as_posix()
        checks.append(timed_check(rel, "shell", lambda p=path: shell_check(p)))

    extensions = set(config.get("xml_extensions", [".sdf"]))
    xml_files = sorted(path for path in (ROOT / "src").rglob("*")
                       if path.is_file() and path.suffix in extensions)
    for path in xml_files:
        rel = path.relative_to(ROOT).as_posix()
        checks.append(timed_check(rel, "xml", lambda p=path: xml_check(p)))

    for relative in config.get("config_validators", []):
        path = ROOT / relative
        if not path.is_file():
            checks.append(result(relative, "config-validator", "fail", "file missing"))
            continue
        print(f"[RUN ] config-validator: {relative}", flush=True)
        status, detail, duration = phase_check(path, preserve=False)
        checks.append(result(relative, "config-validator", status, detail, duration))
        print(f"[{status.upper():4}] config-validator: {relative} "
              f"({duration:.1f}s)", flush=True)

    for relative in config["phase_validators"]:
        path = ROOT / relative
        if args.no_phase_validators:
            checks.append(result(relative, "legacy-validator", "skipped",
                                 "disabled by --no-phase-validators"))
        elif not path.is_file():
            checks.append(result(relative, "legacy-validator", "fail", "file missing"))
        else:
            # Several historical validators intentionally rerun earlier phase
            # gates, so this complete audit can take several minutes. Print
            # progress before starting each captured subprocess to make it
            # clear that the command is active rather than hung.
            print(f"[RUN ] legacy-validator: {relative}", flush=True)
            status, detail, duration = phase_check(
                path, bool(config.get("preserve_validator_outputs", True)))
            checks.append(result(relative, "legacy-validator", status, detail, duration))
            print(f"[{status.upper():4}] legacy-validator: {relative} "
                  f"({duration:.1f}s)", flush=True)

    counts = {state: sum(item["status"] == state for item in checks)
              for state in ("pass", "fail", "skipped")}
    report = {
        "schema_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "git": git_metadata(),
        "summary": {
            "overall": "pass" if counts["fail"] == 0 else "fail",
            "total": len(checks),
            "passed": counts["pass"],
            "failed": counts["fail"],
            "skipped": counts["skipped"],
        },
        "checks": checks,
    }
    if not args.no_write:
        output = ROOT / config.get("generated_report_directory", "evidence/baseline")
        write_reports(report, output)

    for item in checks:
        symbol = {"pass": "PASS", "fail": "FAIL", "skipped": "SKIP"}[item["status"]]
        print(f"[{symbol}] {item['category']}: {item['name']}")
        if item["status"] == "fail":
            print(f"       {item['detail']}")
    print("=" * 72)
    print(f"RESULT: {counts['pass']} passed, {counts['fail']} failed, "
          f"{counts['skipped']} skipped")
    return 0 if counts["fail"] == 0 else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nValidation cancelled by user; no PASS report was written.",
              file=sys.stderr)
        raise SystemExit(130)
