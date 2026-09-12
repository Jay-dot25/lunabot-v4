#!/usr/bin/env python3
"""
LunaBot V4 - Phase A - static validation
=========================================
Validates every Phase A artifact that can be checked WITHOUT running Gazebo:

  1. world + model SDF: XML well-formedness + semantic checks
     (plugins, joints, links, sensors, mesh files on disk, gravity)
  2. topic contract: every sensor topic in model.sdf is bridged in launch-a.sh
  3. spawn height: launch-a.sh SPAWN_Z matches terrain_stats.txt (deterministic)
  4. OBJ meshes: vertex/face counts, index bounds, bounding box
  5. scripts: bash -n syntax, python py_compile
  6. rviz config: required display topic types present
  7. documentation + entry points present

Writes evidence/phase-a-launch-a/static_validation.txt
Exit code: 0 = all PASS, 1 = at least one FAIL.
"""
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD_DIR = os.path.join(REPO, "src", "lunabot_gazebo", "worlds")
EVIDENCE = os.path.join(REPO, "evidence", "phase-a-launch-a")

WORLD = os.path.join(WORLD_DIR, "lunar_world.sdf")
MODEL = os.path.join(REPO, "src", "lunabot_gazebo", "models", "lunabot_v4", "model.sdf")
LAUNCH = os.path.join(REPO, "scripts", "launch-a.sh")
TELEOP = os.path.join(REPO, "scripts", "wasd_teleop.py")
RVIZ = os.path.join(REPO, "rviz", "phase_a.rviz")
ENTRYP = os.path.join(REPO, "launch-a")
DOCS = os.path.join(REPO, "docs", "phase-a-launch-a.md")
STATS = os.path.join(EVIDENCE, "terrain_stats.txt")
VIS_OBJ = os.path.join(WORLD_DIR, "meshes", "lunar_terrain.obj")
COL_OBJ = os.path.join(WORLD_DIR, "meshes", "lunar_terrain_collision.obj")

results = []


def check(name, ok, detail=""):
    results.append((name, bool(ok), detail))
    mark = "PASS" if ok else "FAIL"
    print(f"[{mark}] {name}" + (f" - {detail}" if detail and not ok else ""))
    return bool(ok)


def load_sdf(path):
    """Return the <world> element (world files)."""
    return ET.parse(path).getroot().find("world")


def load_model_sdf(path):
    """Return the <model> element (model files)."""
    return ET.parse(path).getroot().find("model")


# ---------------------------------------------------------------- 1. SDFs
def validate_sdfs():
    root = ET.parse(WORLD).getroot()
    world = root.find("world")
    check("world SDF XML well-formed", root.tag == "sdf" and world is not None)
    check("world name == lunar_world",
          world.get("name") == "lunar_world", f"got {world.get('name')}")

    g = world.find("gravity").text.split()
    check("lunar gravity (0 0 -1.62)",
          g == ["0", "0", "-1.62"], f"got {g}")

    plugins = {p.get("name") for p in world.findall("plugin")}
    for req in ("ignition::gazebo::systems::Physics",
                "ignition::gazebo::systems::SceneBroadcaster",
                "ignition::gazebo::systems::UserCommands",
                "ignition::gazebo::systems::Sensors",
                "ignition::gazebo::systems::Contact"):
        check(f"world plugin {req.split('::')[-1]}", req in plugins)

    # terrain model + meshes on disk
    terrain = world.find("./model[@name='lunar_terrain']")
    check("terrain model present", terrain is not None)
    if terrain is not None:
        check("terrain static", terrain.find("static").text == "true")
        for kind in ("collision", "visual"):
            uri = terrain.find(f"link/{kind}/geometry/mesh/uri").text
            local = uri.replace("file://", "")
            path = os.path.normpath(os.path.join(WORLD_DIR, local))
            check(f"terrain {kind} mesh exists ({local})", os.path.isfile(path), path)

    horizon = world.find("./model[@name='lunar_horizon']")
    check("horizon catch-plane present", horizon is not None)
    check("GUI camera configured", world.find("gui/camera") is not None)

    # model
    mroot = ET.parse(MODEL).getroot()
    m = mroot.find("model")
    check("model SDF XML well-formed", mroot.tag == "sdf" and m is not None)
    check("model name == lunabot_v4", m.get("name") == "lunabot_v4")
    links = {l.get("name") for l in m.findall("link")}
    joints = {j.get("name"): j for j in m.findall("joint")}
    check("6 wheel joints",
          len([n for n in joints if "wheel_joint" in n]) == 6,
          str(sorted(n for n in joints if "wheel_joint" in n)))

    bad_parents = [n for n, j in joints.items()
                   if j.find("parent").text not in links or j.find("child").text not in links]
    check("all joint parent/child resolve to links", not bad_parents, str(bad_parents))

    expected_links = {"chassis", "left_rocker", "right_rocker", "left_bogie",
                      "right_bogie", "left_front_wheel", "right_front_wheel",
                      "left_middle_wheel", "right_middle_wheel",
                      "left_rear_wheel", "right_rear_wheel", "sensor_head",
                      "imu_link", "sensor_mast"}
    check("key links present", expected_links <= links,
          str(expected_links - links))

    sensors = {s.get("name"): s for s in m.iter("sensor")}
    for sname, stype, topic in (("rgb_camera", "camera", "/lunabot/camera/image_raw"),
                                ("depth_camera", "depth_camera", "/lunabot/depth/image_raw"),
                                ("lidar", "gpu_lidar", "/lunabot/lidar/scan"),
                                ("imu", "imu", "/lunabot/imu")):
        check(f"sensor {sname} (type {stype})",
              sname in sensors and sensors[sname].get("type") == stype)
        if sname in sensors:
            check(f"sensor {sname} topic {topic}",
                  sensors[sname].find("topic").text == topic,
                  sensors[sname].find("topic").text)

    # DiffDrive plugin
    dd = next(p for p in m.iter("plugin")
              if p.get("filename") == "ignition-gazebo-diff-drive-system")
    dd_joints = [e.text for e in dd.findall("left_joint")] + [e.text for e in dd.findall("right_joint")]
    check("DiffDrive joints all exist", all(j in joints for j in dd_joints),
          str([j for j in dd_joints if j not in joints]))
    check("DiffDrive odom topic /lunabot/odom",
          dd.find("odom_topic").text == "/lunabot/odom")
    check("DiffDrive tf frames odom/chassis",
          dd.find("frame_id").text == "odom" and dd.find("child_frame_id").text == "chassis")
    check("DiffDrive cmd topic /cmd_vel", dd.find("topic").text == "/cmd_vel")

    # joint controllers reference real joints
    js_pubs = [p for p in m.iter("plugin")
               if p.get("filename") == "ignition-gazebo-joint-position-controller-system"]
    bad = [e.text for p in js_pubs for e in p.findall("joint_name") if e.text not in joints]
    check("steer/mast controllers reference real joints", not bad, str(bad))
    jsp = [p for p in m.iter("plugin")
           if p.get("filename") == "ignition-gazebo-joint-state-publisher-system"]
    bad = [e.text for p in jsp for e in p.findall("joint_name") if e.text not in joints]
    check("joint-state publisher references real joints", not bad, str(bad))


# ---------------------------------------------------------------- 2. topic contract
def validate_topic_contract():
    m = load_model_sdf(MODEL)
    topics = {s.find("topic").text for s in m.iter("sensor")
              if s.find("topic") is not None}
    launch = open(LAUNCH).read()
    for t in sorted(topics):
        check(f"bridge maps {t} (gazebo->ros)", f"{t}@sensor_msgs" in launch
              or f"{t}@nav_msgs" in launch)
    check("bridge maps /cmd_vel (ros->gazebo)",
          re.search(r"/cmd_vel@geometry_msgs/msg/Twist\]\S+\.Twist", launch) is not None)
    check("bridge maps /tf Pose_V -> TFMessage",
          re.search(r"/tf@tf2_msgs/msg/TFMessage\[\S+\.Pose_V", launch) is not None)


# ---------------------------------------------------------------- 3. spawn height
def validate_spawn_height():
    launch = open(LAUNCH).read()
    m = re.search(r'SPAWN_Z="(-?[\d.]+)"', launch)
    stats = open(STATS).read()
    h = re.search(r"spawn_height_m\s*:\s*(-?[\d.]+)", stats)
    if m and h:
        ok = abs(float(m.group(1)) - (float(h.group(1)) + 0.48)) < 0.02
        check("SPAWN_Z matches terrain stats (h0 + 0.48)", ok,
              f"launch={m.group(1)} h0={h.group(1)}")
    else:
        check("SPAWN_Z / terrain stats parseable", False, "regex mismatch")


# ---------------------------------------------------------------- 4. OBJ meshes
def validate_obj(path, expect_color):
    n_v = n_vn = n_c = 0
    max_idx = 0
    bounds = [[1e9] * 3, [-1e9] * 3]
    with open(path) as fh:
        for line in fh:
            p = line.split()
            if not p:
                continue
            if p[0] == "v":
                n_v += 1
                for i, v in enumerate(p[1:4]):
                    v = float(v)
                    bounds[0][i] = min(bounds[0][i], v)
                    bounds[1][i] = max(bounds[1][i], v)
            elif p[0] == "vn":
                n_vn += 1
            elif p[0] == "c":
                n_c += 1
            elif p[0] == "f":
                for ref in p[1:]:
                    max_idx = max(max_idx, int(ref.split("/")[0]))
    name = os.path.basename(path)
    check(f"{name}: has vertices", n_v > 0, str(n_v))
    check(f"{name}: vertex count square grid", int(round(n_v ** 0.5)) ** 2 == n_v, str(n_v))
    check(f"{name}: normals for all vertices", n_vn == n_v, f"vn={n_vn} v={n_v}")
    check(f"{name}: color vertices expected={expect_color}",
          (n_c == n_v) if expect_color else (n_c == 0), f"c={n_c}")
    check(f"{name}: face indices in bounds", 0 < max_idx <= n_v, str(max_idx))
    ext = [bounds[1][i] - bounds[0][i] for i in range(3)]
    check(f"{name}: extent ~400x400 m", 395 < ext[0] < 405 and 395 < ext[1] < 405,
          str([round(e, 1) for e in ext]))
    check(f"{name}: relief within [-12, +10] m",
          bounds[1][2] < 10 and bounds[0][2] > -12,
          f"z=[{bounds[0][2]:.2f}, {bounds[1][2]:.2f}]")
    return n_v


# ---------------------------------------------------------------- 5. scripts
def validate_scripts():
    for sh in (LAUNCH, ENTRYP, os.path.join(EVIDENCE, "collect_evidence.sh")):
        if not os.path.isfile(sh):
            check(f"bash syntax: {os.path.relpath(sh, REPO)}", False, "missing")
            continue
        r = subprocess.run(["bash", "-n", sh], capture_output=True)
        check(f"bash syntax: {os.path.relpath(sh, REPO)}", r.returncode == 0,
              r.stderr.decode()[:200])
    for py in (TELEOP,
               os.path.join(REPO, "tools", "generate_lunar_terrain.py"),
               os.path.join(REPO, "tools", "plot_lidar_scan.py")):
        r = subprocess.run([sys.executable, "-m", "py_compile", py], capture_output=True)
        check(f"python compile: {os.path.relpath(py, REPO)}", r.returncode == 0,
              r.stderr.decode()[:200])
    for f in (LAUNCH, ENTRYP):
        check(f"executable: {os.path.relpath(f, REPO)}", os.access(f, os.X_OK))


# ---------------------------------------------------------------- 6. rviz
def validate_rviz():
    if not os.path.isfile(RVIZ):
        check("rviz config exists", False)
        return
    txt = open(RVIZ).read()
    check("rviz config exists", True)
    check("rviz fixed frame odom", "Fixed Frame: odom" in txt)
    check("rviz displays LaserScan topic", "type: sensor_msgs/msg/LaserScan" in txt)
    check("rviz displays Image topic", "type: sensor_msgs/msg/Image" in txt)
    check("rviz TF display", "rviz_default_plugins/TF" in txt)


# ---------------------------------------------------------------- 7. docs/entry
def validate_docs():
    check("docs/phase-a-launch-a.md exists", os.path.isfile(DOCS))
    if os.path.isfile(DOCS):
        txt = open(DOCS).read()
        for sec in ("## 1. Objective", "## 11. Launch Command", "## 16. Success Criteria",
                    "## 22. Evidence", "## 23. Known Limitations"):
            check(f"docs section {sec}", sec in txt)
    check("terrain stats file", os.path.isfile(STATS))
    check("evidence README", os.path.isfile(os.path.join(EVIDENCE, "README.md")))
    check("verification checklist",
          os.path.isfile(os.path.join(EVIDENCE, "verification_checklist.md")))


def main():
    print("=" * 60)
    print("LUNABOT V4 - PHASE A - STATIC VALIDATION")
    print("=" * 60)
    print("-- SDF files --")
    validate_sdfs()
    print("-- topic contract --")
    validate_topic_contract()
    print("-- spawn height --")
    validate_spawn_height()
    print("-- meshes --")
    validate_obj(VIS_OBJ, expect_color=True)
    validate_obj(COL_OBJ, expect_color=False)
    print("-- scripts --")
    validate_scripts()
    print("-- rviz --")
    validate_rviz()
    print("-- docs --")
    validate_docs()

    fails = [r for r in results if not r[1]]
    print("=" * 60)
    print(f"RESULT: {len(results) - len(fails)}/{len(results)} checks passed"
          + (f", {len(fails)} FAILED" if fails else " - ALL PASS"))
    print("=" * 60)

    os.makedirs(EVIDENCE, exist_ok=True)
    out = os.path.join(EVIDENCE, "static_validation.txt")
    with open(out, "w") as fh:
        fh.write("LunaBot V4 - Phase A - static validation report\n")
        fh.write(f"date: {os.popen('date -u +%FT%TZ').read().strip()}\n")
        fh.write("=" * 60 + "\n")
        for name, ok, detail in results:
            fh.write(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f" - {detail}" if detail else "") + "\n")
        fh.write("=" * 60 + "\n")
        fh.write(f"RESULT: {len(results) - len(fails)}/{len(results)} checks passed\n")
    print(f"report: {out}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
