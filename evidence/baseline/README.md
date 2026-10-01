# Baseline validation evidence

Run the complete repository-level static gate from the repository root:

```bash
python3 tools/validate_all.py
```

Generated outputs:

- `validation_report.json` — machine-readable result;
- `validation_report.md` — human-readable summary;
- `environment.json` — optional capability report generated with:

  ```bash
  python3 tools/check_environment.py --output evidence/baseline/environment.json
  ```

The unified validator preserves the phase validators' existing tracked reports,
so a baseline audit does not rewrite Phase A–L evidence as a side effect.

Static success does **not** prove runtime success. ROS 2/Gazebo runtime checks
must be executed on a compatible workstation. Use
`python3 tools/check_environment.py` to distinguish installed capabilities from
missing dependencies.
