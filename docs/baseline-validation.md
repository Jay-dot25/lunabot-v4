# Phase 0 — Baseline validation

## Purpose

Phase 0 establishes a repeatable, honest gate before goal-level development.
It checks source integrity and runs all existing Phase A–L static validators,
without claiming that a simulator mission has run.

## Commands

```bash
python3 tools/check_environment.py
python3 -m unittest discover -s tests -p 'test_*.py'
python3 tools/validate_all.py
```

For a quick source-only check that intentionally skips legacy phase validators:

```bash
python3 tools/validate_all.py --no-phase-validators --no-write
```

## Checks

- Every Python source under configured roots parses and compiles.
- Every shell script and root launcher passes `bash -n`.
- Every SDF file is well-formed XML.
- Every existing Phase A–L static validator exits successfully.
- Legacy validator reports are restored after execution to avoid audit side
  effects.
- JSON and Markdown aggregate reports are generated.

Configuration is in `config/validation.yaml`. It is deliberately
JSON-compatible YAML so the gate has no third-party Python dependency.

## Exit codes

- `0`: no failed checks;
- `1`: one or more checks failed;
- `2`: invalid validation configuration.

Skipped checks are reported but do not fail the command. The normal complete
command does not skip any configured static validator.

## Runtime boundary

This sandbox may not contain ROS 2, Gazebo, RViz, or colcon. Their absence is
an environment limitation, not a static pass and not a project runtime failure.
A final runtime gate must run on a machine where the capability report says
`ROS/Gazebo runtime ready: yes`.

The runtime command remains:

```bash
HEADLESS=1 DEMO=1 AUTO_GOAL=true REQUIRE_MANUAL_GOAL=false \
  EVIDENCE=1 ./launch-l
```

That command must not be described as passed until it has actually completed
on a compatible workstation.
