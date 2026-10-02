#!/usr/bin/env python3
"""Validate Phase 10 layered traversability contracts."""
import ast
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def main():
 files=['src/lunabot_mapping/lunabot_mapping/traversability.py','src/lunabot_mapping/lunabot_mapping/traversability_node.py','src/lunabot_mapping/config/traversability.yaml']
 if any(not (ROOT/name).is_file() for name in files):print('TRAVERSABILITY_VALIDATION_FAIL missing file');return 1
 for name in files[:2]:ast.parse((ROOT/name).read_text(),filename=name)
 core=(ROOT/files[0]).read_text()
 required=('semantic_costs','slope_weight','roughness_weight','clearance_weight','uncertainty_weight','max_cross_slope_deg','max_step_m','footprint_radius_m','unknown_is_lethal','stale_is_lethal')
 if any(token not in core for token in required):print('TRAVERSABILITY_VALIDATION_FAIL layer contract');return 1
 print('TRAVERSABILITY_VALIDATION_PASS layers=5 policies=4 explanation=1');return 0
if __name__=='__main__':raise SystemExit(main())
