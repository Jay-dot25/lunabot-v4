#!/usr/bin/env python3
"""Generate Phase 2-only Gazebo Fortress capture scenes and their mesh assets.

These are visual/simulation scenes for collecting RGB examples. They do not
produce semantic ground-truth masks; all labels still require human annotation
and review. The Phase 1 world and launch files are intentionally untouched.
"""
from __future__ import annotations

import math
import random
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "lunabot_phase1/src/lunabot_phase2_perception"
MODEL_DIR = PACKAGE / "models/eyes_of_luna_terrain"
MESH_DIR = MODEL_DIR / "meshes"
WORLD_DIR = PACKAGE / "worlds"

SCENES = {
    "scene01": {
        "sun": "-0.48 0.17 -0.86",
        "craters": [
            # Put a pronounced bowl in the central, mid-range view. The previous
            # crater was technically inside horizontal FOV but tiny at the horizon.
            (-14.55, 1.72, 1.9, 0.75, 0.30, 0.50),
            (2.0, 6.0, 3.0, 0.82, 0.18, 0.90),
            (13.0, -5.0, 2.4, 0.62, 0.14, 0.72),
        ],
        # Spread bedrock to the opposite side of the crater and increase only
        # scene01 outcrop relief so it is not hidden behind the foreground boulder.
        "bedrock_pose": (-8.62, 0.72, 0.0, -0.22),
        "bedrock_height_scale": 2.0,
        "bedrock_width_scale": 0.80,
        "boulders": [
            (-14.05, 5.48, 0.78, 0.66, 0.65, 0.0),
            (-3.5, -8.0, 0.52, 0.46, 0.50, 0.55),
            (7.5, 10.0, 0.92, 0.72, 0.80, 1.10),
            (17.0, 1.5, 0.60, 0.82, 0.56, 0.25),
        ],
        # The camera remains level/unchanged. Place the crater in the central
        # foreground view, with boulder left and bedrock right, all clear of spawn.
        "spawn": (-21.0, -1.0, 0.40),
        "bedrock_seed": 117,
    },
    "scene02": {
        "sun": "-0.28 -0.42 -0.86",
        "craters": [
            (-8.0, -5.0, 2.7, 0.78, 0.17, 0.82),
            (3.5, 4.5, 2.1, 0.52, 0.12, 0.70),
            (14.0, -2.5, 3.3, 0.94, 0.20, 1.00),
        ],
        "bedrock_pose": (-11.0, 5.0, 0.0, -0.22),
        "boulders": [
            (-17.0, -0.5, 0.65, 0.78, 0.56, 0.40),
            (-2.0, 10.0, 0.88, 0.61, 0.85, 0.90),
            (8.5, -10.0, 0.54, 0.74, 0.52, 0.15),
            (18.0, 5.5, 0.72, 0.90, 0.64, 1.35),
        ],
        "spawn": (-21.0, 1.0, 0.03),
        "bedrock_seed": 229,
    },
    "scene03": {
        "sun": "-0.62 0.06 -0.78",
        "craters": [
            (-9.0, 4.0, 3.0, 0.88, 0.19, 0.94),
            (2.5, -6.0, 2.3, 0.58, 0.13, 0.76),
            (13.0, 5.5, 2.8, 0.76, 0.17, 0.86),
        ],
        "bedrock_pose": (-12.0, -5.0, 0.0, 0.32),
        "boulders": [
            (-16.0, -2.5, 0.70, 0.58, 0.73, 0.10),
            (-1.0, 9.0, 0.56, 0.86, 0.50, 0.70),
            (7.0, 9.0, 0.82, 0.67, 0.76, 1.25),
            (18.0, -6.0, 0.62, 0.78, 0.60, 0.45),
        ],
        "spawn": (-21.0, -1.5, -0.04),
        "bedrock_seed": 331,
    },
}


def _write_obj(path: Path, vertices: list[tuple[float, float, float]],
               faces: list[tuple[int, ...]], comment: str) -> None:
    """Write indexed OBJ geometry with one computed normal per vertex.

    Fortress' DART collision importer requires per-vertex normals for OBJ mesh
    shapes; vertices/faces without ``vn`` records can crash physics initialization.
    Face indices accepted here are 1-based OBJ vertex indices.
    """
    accumulators = [[0.0, 0.0, 0.0] for _ in vertices]
    for face in faces:
        if len(face) < 3:
            continue
        first = face[0] - 1
        for offset in range(1, len(face) - 1):
            indices = (first, face[offset] - 1, face[offset + 1] - 1)
            if any(index < 0 or index >= len(vertices) for index in indices):
                raise ValueError(f"face index outside vertex array while writing {path}")
            a, b, c = (vertices[index] for index in indices)
            ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
            ac = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
            normal = (
                ab[1] * ac[2] - ab[2] * ac[1],
                ab[2] * ac[0] - ab[0] * ac[2],
                ab[0] * ac[1] - ab[1] * ac[0],
            )
            for index in set(indices):
                for axis in range(3):
                    accumulators[index][axis] += normal[axis]

    normals = []
    for normal in accumulators:
        length = math.sqrt(sum(component * component for component in normal))
        if length < 1e-12:
            raise ValueError(f"vertex without a usable normal while writing {path}")
        normals.append(tuple(component / length for component in normal))

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii", newline="\n") as stream:
        stream.write(f"# {comment}\n")
        for x, y, z in vertices:
            stream.write(f"v {x:.5f} {y:.5f} {z:.5f}\n")
        for x, y, z in normals:
            stream.write(f"vn {x:.7f} {y:.7f} {z:.7f}\n")
        for face in faces:
            stream.write("f " + " ".join(f"{index}//{index}" for index in face) + "\n")


def _terrain_height(x: float, y: float,
                    craters: list[tuple[float, float, float, float, float, float]]) -> float:
    height = 0.0
    for cx, cy, radius, depth, rim_height, rim_width in craters:
        radius_from_center = math.hypot(x - cx, y - cy)
        if radius_from_center <= radius:
            bowl = -depth * max(0.0, 1.0 - (radius_from_center / radius) ** 2) ** 1.35
            height = min(height, bowl)
        elif radius_from_center < radius + rim_width:
            phase = (radius_from_center - radius) / rim_width
            rim = rim_height * math.sin(math.pi * phase)
            height = max(height, rim)
    return height


def _generate_regolith_mesh(scene: str, craters: list[tuple[float, float, float, float, float, float]]) -> None:
    x_min, x_max, y_min, y_max, step = -24.0, 24.0, -18.0, 18.0, 0.5
    x_count = round((x_max - x_min) / step) + 1
    y_count = round((y_max - y_min) / step) + 1
    vertices: list[tuple[float, float, float]] = []
    for j in range(y_count):
        y = y_min + j * step
        for i in range(x_count):
            x = x_min + i * step
            vertices.append((x, y, _terrain_height(x, y, craters)))

    def index(i: int, j: int) -> int:
        return j * x_count + i + 1

    faces: list[tuple[int, int, int]] = []
    for j in range(y_count - 1):
        for i in range(x_count - 1):
            lower_left = index(i, j)
            lower_right = index(i + 1, j)
            upper_right = index(i + 1, j + 1)
            upper_left = index(i, j + 1)
            # Counter-clockwise from above (positive Z normals).
            faces.append((lower_left, lower_right, upper_right))
            faces.append((lower_left, upper_right, upper_left))
    _write_obj(MESH_DIR / f"regolith_{scene}.obj", vertices, faces,
               f"Phase 2 {scene} regolith terrain with explicit bowl-and-rim craters")


def _generate_bedrock_mesh(scene: str, seed: int, height_scale: float = 1.0,
                           width_scale: float = 1.0) -> None:
    if not math.isfinite(height_scale) or height_scale <= 0.0:
        raise ValueError("bedrock height scale must be finite and positive")
    if not math.isfinite(width_scale) or width_scale <= 0.0:
        raise ValueError("bedrock width scale must be finite and positive")
    rng = random.Random(seed)
    count = 12
    base_z = 0.025
    base: list[tuple[float, float, float]] = []
    top: list[tuple[float, float, float]] = []
    for i in range(count):
        angle = 2.0 * math.pi * i / count
        factor = 0.87 + rng.random() * 0.25
        x = width_scale * 3.1 * factor * math.cos(angle)
        y = width_scale * 2.15 * factor * math.sin(angle)
        base.append((x, y, base_z))
        top_height = 0.18 + rng.random() * 0.11
        top.append((x, y, base_z + height_scale * (top_height - base_z)))

    peak_height = 0.43 + rng.random() * 0.06
    vertices = base + top + [
        (0.15 * width_scale, -0.10 * width_scale,
         base_z + height_scale * (peak_height - base_z)),
        (0.0, 0.0, base_z),
    ]
    top_center = 2 * count + 1
    bottom_center = 2 * count + 2
    faces: list[tuple[int, ...]] = []
    for i in range(count):
        nxt = (i + 1) % count
        # Broad, faceted top; outward side faces; closed bottom.
        faces.append((top_center, count + i + 1, count + nxt + 1))
        faces.append((i + 1, nxt + 1, count + nxt + 1))
        faces.append((i + 1, count + nxt + 1, count + i + 1))
        faces.append((bottom_center, nxt + 1, i + 1))
    _write_obj(MESH_DIR / f"bedrock_{scene}.obj", vertices, faces,
               f"Phase 2 {scene} faceted bedrock outcrop; hand labels still required")


def _normalize(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    length = math.sqrt(sum(component * component for component in vector))
    return tuple(component / length for component in vector)


def _generate_boulder_mesh() -> None:
    vertices = [
        (1.0, 0.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 1.0, 0.0),
        (0.0, -1.0, 0.0), (0.0, 0.0, 1.0), (0.0, 0.0, -1.0),
    ]
    faces = [
        (4, 0, 2), (4, 2, 1), (4, 1, 3), (4, 3, 0),
        (5, 2, 0), (5, 1, 2), (5, 3, 1), (5, 0, 3),
    ]
    for _ in range(2):
        midpoint_cache: dict[tuple[int, int], int] = {}

        def midpoint(a: int, b: int) -> int:
            key = tuple(sorted((a, b)))
            if key not in midpoint_cache:
                point = _normalize(tuple((vertices[a][axis] + vertices[b][axis]) / 2.0
                                         for axis in range(3)))
                midpoint_cache[key] = len(vertices)
                vertices.append(point)
            return midpoint_cache[key]

        refined = []
        for a, b, c in faces:
            ab, bc, ca = midpoint(a, b), midpoint(b, c), midpoint(c, a)
            refined.extend(((a, ab, ca), (ab, b, bc), (ca, bc, c), (ab, bc, ca)))
        faces = refined

    rng = random.Random(7253)
    rough_vertices = []
    for x, y, z in vertices:
        radius = 0.86 + rng.random() * 0.28
        rough_vertices.append((x * radius, y * radius, z * radius))
    _write_obj(MESH_DIR / "boulder_irregular.obj", rough_vertices,
               [tuple(index + 1 for index in face) for face in faces],
               "Low-poly irregular boulder used as simulated geometry, not an automatic label")


def _boulder_model(number: int, spec: tuple[float, float, float, float, float, float]) -> str:
    x, y, sx, sy, sz, yaw = spec
    z = sz + 0.02
    return f"""    <model name=\"boulder_{number:02d}\">
      <static>true</static><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.3f}</pose>
      <link name=\"boulder_link\">
        <collision name=\"boulder_collision\"><geometry><mesh>
          <uri>model://eyes_of_luna_terrain/meshes/boulder_irregular.obj</uri><scale>{sx:.3f} {sy:.3f} {sz:.3f}</scale>
        </mesh></geometry></collision>
        <visual name=\"boulder_visual\"><geometry><mesh>
          <uri>model://eyes_of_luna_terrain/meshes/boulder_irregular.obj</uri><scale>{sx:.3f} {sy:.3f} {sz:.3f}</scale>
        </mesh></geometry><material>
          <ambient>0.24 0.24 0.23 1</ambient><diffuse>0.33 0.33 0.31 1</diffuse>
          <specular>0.04 0.04 0.04 1</specular>
        </material></visual>
      </link>
    </model>"""


def _world_sdf(scene: str, spec: dict) -> str:
    spawn_x, spawn_y, spawn_yaw = spec["spawn"]
    bedrock_x, bedrock_y, _, bedrock_yaw = spec["bedrock_pose"]
    boulders = "\n".join(_boulder_model(index, boulder)
                           for index, boulder in enumerate(spec["boulders"], 1))
    return f"""<?xml version=\"1.0\"?>
<sdf version=\"1.8\">
  <world name=\"eyes_of_luna_capture_{scene}\">
    <physics name=\"lunar_capture_physics\" type=\"ignored\">
      <max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor>
    </physics>
    <gravity>0 0 -1.62</gravity>
    <scene><ambient>0.24 0.24 0.25 1</ambient><background>0.008 0.010 0.018 1</background><shadows>true</shadows></scene>
    <plugin filename=\"ignition-gazebo-physics-system\" name=\"gz::sim::systems::Physics\"/>
    <plugin filename=\"ignition-gazebo-user-commands-system\" name=\"gz::sim::systems::UserCommands\"/>
    <plugin filename=\"ignition-gazebo-scene-broadcaster-system\" name=\"gz::sim::systems::SceneBroadcaster\"/>
    <plugin filename=\"ignition-gazebo-sensors-system\" name=\"gz::sim::systems::Sensors\"><render_engine>ogre2</render_engine></plugin>
    <light type=\"directional\" name=\"capture_sun\">
      <pose>0 0 30 0 0 0</pose><cast_shadows>true</cast_shadows>
      <diffuse>0.88 0.86 0.80 1</diffuse><specular>0.10 0.10 0.10 1</specular>
      <direction>{spec['sun']}</direction>
    </light>

    <!-- This mesh is a visual/collision surface with real concave crater bowls and raised rims.
         It is simulated terrain, not a semantic camera or verified pixel ground truth. -->
    <model name=\"regolith_surface\"><static>true</static>
      <link name=\"regolith_link\">
        <collision name=\"regolith_collision\"><geometry><mesh>
          <uri>model://eyes_of_luna_terrain/meshes/regolith_{scene}.obj</uri>
        </mesh></geometry><surface><friction><ode><mu>1.2</mu><mu2>1.2</mu2></ode></friction></surface></collision>
        <visual name=\"regolith_visual\"><geometry><mesh>
          <uri>model://eyes_of_luna_terrain/meshes/regolith_{scene}.obj</uri>
        </mesh></geometry><material>
          <ambient>0.38 0.36 0.32 1</ambient><diffuse>0.48 0.46 0.41 1</diffuse>
          <specular>0.02 0.02 0.02 1</specular>
        </material></visual>
      </link>
    </model>

    <!-- Faceted bedrock outcrop; height/width vary by capture scene for visibility. -->
    <model name=\"bedrock_outcrop\"><static>true</static>
      <pose>{bedrock_x:.3f} {bedrock_y:.3f} 0 0 0 {bedrock_yaw:.3f}</pose>
      <link name=\"bedrock_link\">
        <collision name=\"bedrock_collision\"><geometry><mesh>
          <uri>model://eyes_of_luna_terrain/meshes/bedrock_{scene}.obj</uri>
        </mesh></geometry></collision>
        <visual name=\"bedrock_visual\"><geometry><mesh>
          <uri>model://eyes_of_luna_terrain/meshes/bedrock_{scene}.obj</uri>
        </mesh></geometry><material>
          <ambient>0.35 0.37 0.39 1</ambient><diffuse>0.43 0.45 0.47 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
        </material></visual>
      </link>
    </model>

{boulders}

    <include><uri>model://lunabot_rover</uri><name>lunabot</name>
      <pose>{spawn_x:.3f} {spawn_y:.3f} 0 0 0 {spawn_yaw:.3f}</pose>
    </include>
  </world>
</sdf>
"""


def generate() -> list[Path]:
    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    WORLD_DIR.mkdir(parents=True, exist_ok=True)
    (MODEL_DIR / "model.config").write_text(
        """<?xml version=\"1.0\"?>\n<model>\n  <name>eyes_of_luna_terrain</name>\n  <version>1.0</version>\n  <sdf version=\"1.8\">model.sdf</sdf>\n  <description>Phase 2 capture-only terrain mesh assets; no semantic masks included.</description>\n</model>\n""", encoding="utf-8")
    (MODEL_DIR / "model.sdf").write_text(
        """<?xml version=\"1.0\"?>\n<sdf version=\"1.8\"><model name=\"eyes_of_luna_terrain\"><static>true</static><link name=\"terrain_link\"><visual name=\"terrain_visual\"><geometry><mesh><uri>model://eyes_of_luna_terrain/meshes/regolith_scene01.obj</uri></mesh></geometry></visual></link></model></sdf>\n""",
        encoding="utf-8")
    _generate_boulder_mesh()
    generated = [MODEL_DIR / "model.config", MODEL_DIR / "model.sdf",
                 MESH_DIR / "boulder_irregular.obj"]
    for scene, spec in SCENES.items():
        _generate_regolith_mesh(scene, spec["craters"])
        _generate_bedrock_mesh(
            scene, spec["bedrock_seed"],
            height_scale=spec.get("bedrock_height_scale", 1.0),
            width_scale=spec.get("bedrock_width_scale", 1.0),
        )
        world_path = WORLD_DIR / f"eyes_of_luna_capture_{scene}.sdf"
        world_path.write_text(_world_sdf(scene, spec), encoding="utf-8")
        generated.extend((MESH_DIR / f"regolith_{scene}.obj",
                          MESH_DIR / f"bedrock_{scene}.obj", world_path))
    return generated


if __name__ == "__main__":
    outputs = generate()
    for output in outputs:
        print(output.relative_to(ROOT))
