#!/usr/bin/env python3
"""Honest static gate for Phase E trained five-class RGB-D perception.

This verifies source/artifact contracts only. It never claims ROS/Gazebo runtime,
semantic quality, GUI inspection, relaunch, or clean-process acceptance.
"""
import ast
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
checks = []

def check(name, ok, detail=""):
    checks.append((name, bool(ok), detail))

def text(rel):
    return (ROOT / rel).read_text(encoding="utf-8")

def run(*cmd):
    return subprocess.run(cmd, cwd=ROOT, text=True, stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE)

required = [
    "launch-a", "launch-b", "launch-c", "launch-d", "launch-e",
    "scripts/launch-e.sh", "scripts/terrain_segmentation.py",
    "tools/capture_phase_e_dataset.py", "tools/annotate_phase_e_dataset.py",
    "tools/train_terrain_mlp_gazebo.py", "models/terrain_mlp_v2.json",
    "rviz/phase_e.rviz", "docs/phase-5-launch-e.md",
    "evidence/phase-e-launch-e/README.md",
    "evidence/phase-e-launch-e/verification_checklist.md",
]
for rel in required:
    check(f"file exists: {rel}", (ROOT / rel).is_file())
for rel in ("launch-e", "scripts/launch-e.sh", "scripts/terrain_segmentation.py",
            "tools/train_terrain_mlp_gazebo.py", "tools/validate_phase_e.py"):
    p = ROOT / rel
    check(f"executable: {rel}", p.is_file() and bool(p.stat().st_mode & 0o111))
for rel in ("scripts/terrain_segmentation.py", "tools/train_terrain_mlp_gazebo.py",
            "tools/validate_phase_e.py"):
    try:
        ast.parse(text(rel)); check(f"Python syntax: {rel}", True)
    except Exception as exc:
        check(f"Python syntax: {rel}", False, str(exc))
for rel in ("launch-e", "scripts/launch-e.sh"):
    result = run("bash", "-n", rel)
    check(f"bash syntax: {rel}", result.returncode == 0, result.stderr.strip())

# Closed baseline validators must still pass; do not couple to obsolete totals.
for phase in "abcd":
    rel = f"tools/validate_phase_{phase}.py"
    result = run(sys.executable, rel)
    check(f"Phase {phase.upper()} static baseline remains green",
          result.returncode == 0 and "ALL PASS" in result.stdout,
          result.stdout[-160:].strip())

launch, node, trainer = map(text, ("scripts/launch-e.sh",
                                  "scripts/terrain_segmentation.py",
                                  "tools/train_terrain_mlp_gazebo.py"))
rviz, docs = text("rviz/phase_e.rviz"), text("docs/phase-5-launch-e.md")
wrapper = text("launch-e")
labels = ["BEDROCK", "REGOLITH", "ROCK", "CRATER", "SHADOW"]
features = ["red", "green", "blue", "depth_norm", "row_norm",
            "depth_gradient", "texture"]
try:
    model = json.loads(text("models/terrain_mlp_v2.json"))
    check("Gazebo-trained model artifact parses as JSON", True)
except Exception as exc:
    model = {}; check("Gazebo-trained model artifact parses as JSON", False, str(exc))
check("model format is versioned", model.get("format") == "lunabot_mlp_v2")
check("model exact class order", model.get("labels") == labels)
check("model exact seven features", model.get("features") == features)
check("model architecture is 7-12-8-5", model.get("architecture") == [7, 12, 8, 5])
check("model uses ReLU", model.get("activation") == "relu")
training = model.get("training", {})
check("artifact records supervised training", "annotated Gazebo RGB-D captures" in training.get("method", ""))
check("artifact uses whole-frame split", training.get("split") == "whole-frame 80/20")
check("held-out accuracy is recorded",
      0 < float(training.get("held_out_accuracy", 0)) <= 1)
check("artifact has five-row confusion matrix",
      len(training.get("confusion_matrix", [])) == 5 and
      all(len(row) == 5 for row in training.get("confusion_matrix", [])))
for key, shape in (("w1", (7, 12)), ("b1", (12,)), ("w2", (12, 8)),
                   ("b2", (8,)), ("w3", (8, 5)), ("b3", (5,))):
    value = model.get(key, [])
    actual = (len(value), len(value[0])) if value and isinstance(value[0], list) else (len(value),)
    check(f"model tensor {key} shape {shape}", actual == shape)

capture = text("tools/capture_phase_e_dataset.py")
annotator = text("tools/annotate_phase_e_dataset.py")
check("capture consumes live RGB-D", "sensor_msgs.msg import Image" in capture and
      "rgb_topic" in capture and "depth_topic" in capture)
check("capture enforces timestamp synchronization", "abs(rs - ds)" in capture)
check("capture saves sensor arrays and provenance", "np.savez_compressed" in capture and
      "rgb_stamp_ns" in capture and "depth_encoding" in capture)
check("annotation uses explicit IGNORE 255", "IGNORE" in annotator and
      "bytearray([255])" in annotator)
check("annotation exposes exact five classes", all(label in annotator for label in labels))

# Trainer must actually optimize weights from annotated Gazebo captures.
for token, name in (("default_rng", "deterministic RNG"), ("dw3", "backpropagation"),
                    ("epochs", "training epochs"), ("held_out_accuracy", "held-out metric"),
                    ("whole-frame 80/20", "whole-frame split"),
                    ("per_class_recall", "per-class recall")):
    check(f"trainer contains {name}", token in trainer)
check("Gazebo trainer uses NumPy", "import numpy as np" in trainer)
check("trainer exact labels", all(f"'{label}'" in trainer for label in labels))

# Runtime inference contract.
model_text = json.dumps(model)
for token, name in (("import numpy as np", "NumPy inference"),
                    ("features @ self.w1", "first trained layer"),
                    ("h1 @ self.w2", "second trained layer"),
                    ("h2 @ self.w3", "output layer"),
                    ("np.argmax", "class selection"),
                    ("np.gradient", "depth geometry"),
                    ("texture", "local texture"),
                    ("rows = np.linspace", "row feature")):
    check(f"node uses {name}", token in node or token in model_text)
check("node exact label constants", all(f"{label} = {i}" in node for i, label in enumerate(labels)))
check("node rejects mismatched model labels", "label order does not match" in node)
check("node supports common RGB encodings", all(x in node for x in ("rgb8", "bgr8", "rgba8", "bgra8")))
check("node supports metric depth encodings", all(x in node for x in ("16uc1", "32fc1", "64fc1")))
check("node publishes mono8 mask and rgb8 overlay", '"mono8"' in node and '"rgb8"' in node)
check("node reports all class counts", all(f'name.lower()' in node for _ in [0]) and "minlength=5" in node)
check("node publishes trained-model status", "trained=true" in node and "SEGMENTATION_PASS" in node)
check("node status is transient local", "TRANSIENT_LOCAL" in node)
check("node never publishes velocity", "/cmd_vel" not in node)
check("obsolete three-class contract absent", all(x not in node for x in ("UNKNOWN =", "TERRAIN =", "OBSTACLE =")))

# Independent launcher, dependencies, runtime gates and inherited safety.
check("wrapper resolves path and execs launcher", "readlink -f" in wrapper and "scripts/launch-e.sh" in wrapper)
noncomment = "\n".join(x for x in launch.splitlines() if not x.lstrip().startswith("#"))
check("launcher does not invoke prior launcher", all(f"launch-{p}" not in noncomment for p in "abcd"))
check("launcher requires model artifact", 'TERRAIN_MODEL="$REPO_DIR/models/terrain_mlp_v2.json"' in launch and
      '"$TERRAIN_MODEL"' in launch)
check("launcher checks NumPy dependency", "import numpy" in launch and "python3-numpy" in launch)
check("launcher passes model path", '-p model_path:="$TERRAIN_MODEL"' in launch)
check("manual goal is default", 'AUTO_GOAL="${AUTO_GOAL:-false}"' in launch)
check("manual mode defers goal-dependent message checks",
      "DEFERRED (manual goal not selected)" in launch)
check("DEMO explicitly enables auto goal", 'if [ "$DEMO" = "1" ]' in launch and "AUTO_GOAL=true" in launch)
check("launcher validates semantic messages", all(t in launch for t in
      ("/lunabot/terrain/segmentation", "/lunabot/terrain/overlay",
       "/lunabot/terrain/segmentation/status")))
check("launcher requires processed-frame status", "SEGMENTATION_PASS" in launch)
check("launcher preserves Phase D A*", 'NAV_PATH="$REPO_DIR/scripts/astar_navigation.py"' in launch)
check("launcher preserves Phase B boundary", '--input-topic /cmd_vel_in --output-topic /cmd_vel' in launch and
      '-p cmd_topic:=/cmd_vel_in' in launch)
check("launcher uses process groups and bounded cleanup", "setsid" in launch and
      'kill -KILL -"$pid"' in launch and "terrain_segmentation.py" in launch)
check("launcher saves fresh map", 'rm -f "$EVIDENCE_DIR/phase_e_map.yaml"' in launch and
      "save_map_timeout:=30.0" in launch)

check("RViz keeps map and A* displays", "Fixed Frame: map" in rviz and "Name: A* Path" in rviz)
check("RViz shows semantic overlay", "/lunabot/terrain/overlay" in rviz)
check("RViz provides semantic mask", "/lunabot/terrain/segmentation" in rviz)
check("RViz provides manual goal tool and selected-goal display",
      "rviz_default_plugins/SetGoal" in rviz and "Selected Goal" in rviz and
      rviz.count("/goal_pose") >= 2)
check("RViz mask scale includes label 4", "Max: 4" in rviz)
check("RViz includes exact five-class legend", all(label in rviz for label in labels))
for phrase in ("whole-frame", "captured", "annotated", "IGNORE",
               "Phase E remains incomplete", "stale processes",
               "BEDROCK", "REGOLITH", "ROCK", "CRATER", "SHADOW"):
    check(f"docs disclose: {phrase}", phrase in docs)
check("docs do not claim approval", "Phase E is approved" not in docs and
      "runtime gates are explicitly approved" not in docs)

passed = sum(ok for _, ok, _ in checks)
for name, ok, detail in checks:
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f" - {detail}" if detail else ""))
print(f"\nRESULT: {passed}/{len(checks)} checks passed - " +
      ("ALL PASS" if passed == len(checks) else "FAIL"))
report = ROOT / "evidence/phase-e-launch-e/static_validation.txt"
report.write_text("\n".join(f"[{'PASS' if ok else 'FAIL'}] {name}" for name, ok, _ in checks) +
                  f"\n\nRESULT: {passed}/{len(checks)} checks passed\n", encoding="utf-8")
sys.exit(0 if passed == len(checks) else 1)
