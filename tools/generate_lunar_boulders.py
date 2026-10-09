#!/usr/bin/env python3
"""Generate photorealistic, continuous 3D lunar regolith terrain, sunken impact craters,
high-resolution procedural lunar regolith texture (`lunar_regolith_albedo.png`),
3D + textured Rover Tyre Marks (`0.52 m` wheel track gauge with V-chevron cleat treads),
and fractured angular lunar basalt boulders/outcrops.

Key architectural features:
1. Continuous `156 x 156` (`24,336` vertices, `0.273 m` cell spacing) near-field 3D lunar
   regolith surface (`lunar_impact_crater.obj`) with 48 real sunken concave bowl craters
   (`-0.08 m` to `-0.44 m` below ground level) and raised circular ejecta rims, while
   depressing the underlying coarse `lunar_terrain.obj` visual mesh by `-0.68 m` inside
   `r < 17.5 m` so the coarse grid never clips through the sunken crater bowls.
2. High-resolution `512 x 512` procedural lunar regolith & tyre-track albedo texture
   (`lunar_regolith_albedo.png` + `lunar_regolith.mtl`) mapped via `vt u v` coordinates.
3. Realistic 3D Rover Tyre Marks (`0.52 m` left/right wheel separation, `0.092 m` tyre width)
   with sunken wheel ruts, compressed dark regolith troughs (`lunar_route_crater_wall.obj`),
   raised flanking dust shoulders, and repeating 3D angled V-chevron cleat tread bars every
   `5.5 cm` (`lunar_route_crater_rim.obj`) leading from the Lunar Rover Garage to `(0, 0)`
   and along Safe Route B.
4. Fractured angular basalt rock outcrops (`lunar_bedrock_patches.obj`) and concave
   crater-bowl crescent shadows (`lunar_deep_shadows.obj`, `lunar_penumbra_shadows.obj`)
   replacing artificial oval/doughnut patches.
"""

from __future__ import annotations

import math
from pathlib import Path
import random
import struct
import zlib


MESH_DIR = Path(__file__).resolve().parent.parent / "src" / "lunabot_gazebo" / "worlds" / "meshes"
# Low-angle lunar sun direction (matching directional sun in lunar_world.sdf)
SUN_DIR = (0.58, 0.68, 0.45)

_OUTER_GRID_Z: list[float] | None = None

# ==============================================================================
# ROVER WHEEL TRACK TRAJECTORIES (0.52 m track gauge, 0.092 m wheel width)
# ==============================================================================
ROVER_TRACK_PATHS: list[list[tuple[float, float]]] = [
    # Track 1+2 (Continuous): Lunar Rover Garage (-10.2, -3.85) -> Rover Spawn (0, 0) -> Safe Route B (1.68, -0.62) -> Science Corridor (16.8, -4.60)
    [
        (-10.20, -3.85),
        (-8.20, -2.85),
        (-5.80, -1.65),
        (-3.40, -0.60),
        (-1.45, -0.08),
        (0.00, 0.00),
        (0.65, -0.20),
        (1.18, -0.45),
        (1.68, -0.62),
        (3.50, -1.12),
        (6.40, -2.02),
        (10.80, -3.15),
        (16.80, -4.60),
    ],
    # Track 3: Habitat Hub EVA Airlock (-8.6, 4.1) patrol track toward Solar Power Station corridor
    [
        (-8.60, 4.10),
        (-5.60, 2.95),
        (-2.40, 2.25),
        (1.10, 2.95),
        (5.10, 4.85),
        (9.80, 6.80),
    ],
    # Track 4: Garage turnaround maneuver branch connecting Track 1 and Track 3
    [
        (-7.60, -2.55),
        (-5.90, -0.75),
        (-4.20, 1.35),
        (-2.40, 2.25),
    ],
]


def _catmull_rom_point(
    p0: tuple[float, float],
    p1: tuple[float, float],
    p2: tuple[float, float],
    p3: tuple[float, float],
    t: float,
) -> tuple[float, float]:
    t2 = t * t
    t3 = t2 * t
    x = 0.5 * (
        (2.0 * p1[0])
        + (-p0[0] + p2[0]) * t
        + (2.0 * p0[0] - 5.0 * p1[0] + 4.0 * p2[0] - p3[0]) * t2
        + (-p0[0] + 3.0 * p1[0] - 3.0 * p2[0] + p3[0]) * t3
    )
    y = 0.5 * (
        (2.0 * p1[1])
        + (-p0[1] + p2[1]) * t
        + (2.0 * p0[1] - 5.0 * p1[1] + 4.0 * p2[1] - p3[1]) * t2
        + (-p0[1] + 3.0 * p1[1] - 3.0 * p2[1] + p3[1]) * t3
    )
    return x, y


def _sample_rover_track_spine(
    waypoints: list[tuple[float, float]], step: float = 0.055
) -> list[tuple[float, float, float, float, float]]:
    """Sample a smooth spline through `waypoints` at `step` m arc-length intervals.
    Returns list of `(x, y, tx, ty, s)` with unit tangent `(tx, ty)` and cumulative arc length `s`."""
    dense: list[tuple[float, float]] = []
    n = len(waypoints)
    for i in range(n - 1):
        p0 = waypoints[max(0, i - 1)]
        p1 = waypoints[i]
        p2 = waypoints[i + 1]
        p3 = waypoints[min(n - 1, i + 2)]
        seg_len = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        sub = max(8, int(seg_len / 0.02))
        for k in range(sub):
            dense.append(_catmull_rom_point(p0, p1, p2, p3, k / sub))
    dense.append(waypoints[-1])

    sampled: list[tuple[float, float, float]] = [(dense[0][0], dense[0][1], 0.0)]
    acc = 0.0
    next_s = step
    for i in range(1, len(dense)):
        x0, y0 = dense[i - 1]
        x1, y1 = dense[i]
        ds = math.hypot(x1 - x0, y1 - y0)
        if ds < 1e-6:
            continue
        while acc + ds >= next_s:
            alpha = (next_s - acc) / ds
            sx = x0 + alpha * (x1 - x0)
            sy = y0 + alpha * (y1 - y0)
            sampled.append((sx, sy, next_s))
            next_s += step
        acc += ds

    out: list[tuple[float, float, float, float, float]] = []
    m = len(sampled)
    for i, (x, y, s) in enumerate(sampled):
        x_prev, y_prev, _ = sampled[max(0, i - 1)]
        x_next, y_next, _ = sampled[min(m - 1, i + 1)]
        dx = x_next - x_prev
        dy = y_next - y_prev
        dn = math.hypot(dx, dy)
        if dn < 1e-6:
            tx, ty = 1.0, 0.0
        else:
            tx, ty = dx / dn, dy / dn
        out.append((x, y, tx, ty, s))
    return out


_SAMPLED_TRACK_SPINES: list[list[tuple[float, float, float, float, float]]] = [
    _sample_rover_track_spine(p, step=0.055) for p in ROVER_TRACK_PATHS
]


def _build_nearfield_craters() -> list[tuple[float, float, float, float, float, float]]:
    """Build deterministic list of `(cx, cy, R, depth, rim_h, phase)` for real 3D sunken
    lunar impact craters across `[-18.5, +18.5] x [-18.5, +18.5]`, all verified clear of the
    4 Rover Tyre Tracks and settlement structures."""
    # 1. Primary & Midground Named Impact Craters (all clear of Tracks 1..4 and obstacle boulders)
    craters: list[tuple[float, float, float, float, float, float]] = [
        (3.45, 1.50, 1.35, 0.36, 0.18, 0.4),     # 2.7m diameter Dangerous Route-A Impact Crater
        (6.80, -5.20, 1.45, 0.36, 0.18, 1.1),    # 2.9m diameter Midground South Crater
        (7.80, 2.20, 1.48, 0.37, 0.18, 2.0),     # 3.0m diameter Midground East-Central Crater
        (-3.20, -3.60, 1.15, 0.28, 0.14, 0.8),   # Foreground SW Crater (visible near camera!)
        (-9.20, 0.40, 1.22, 0.30, 0.15, 1.7),    # Foreground West Crater between Garage & Hub
        (11.80, -0.40, 1.65, 0.42, 0.20, 2.5),   # Eastern Midground Crater
        (12.60, 4.20, 1.50, 0.38, 0.18, 0.3),    # NE Midground Crater
        (1.85, -3.55, 0.92, 0.23, 0.11, 1.4),    # Southern Flank Crater
        (-1.80, 5.40, 1.12, 0.28, 0.14, 2.2),    # Northern Flank Crater
        (-6.80, -5.80, 1.28, 0.31, 0.15, 0.9),   # SW Basin Crater
        (13.80, 10.80, 2.50, 0.46, 0.24, 1.6),   # Large NE Corridor Crater
        (14.20, -9.40, 2.30, 0.44, 0.22, 2.8),   # Large SE Corridor Crater
    ]

    # Protected pads (rover spawn, buildings, boulders) & rover wheel tracks
    protected_circles = [
        (0.0, 0.0, 1.85),
        (1.55, 0.32, 1.15),
        (3.80, -2.20, 1.65),
        (4.80, 3.90, 1.35),
        (5.40, -5.80, 1.35),
        (-10.50, 7.50, 5.20),
        (-11.00, -4.50, 4.20),
        (-8.20, 4.20, 1.85),
    ]

    rng = random.Random(202610)
    attempts = 0
    while len(craters) < 52 and attempts < 1600:
        attempts += 1
        cx = rng.uniform(-16.8, 16.8)
        cy = rng.uniform(-16.8, 16.8)
        if math.hypot(cx, cy) > 17.0:
            continue
        rad = rng.uniform(0.32, 0.86)
        if any(math.hypot(cx - px, cy - py) < prad + rad * 1.80 for px, py, prad in protected_circles):
            continue
        if any(math.hypot(cx - ocx, cy - ocy) < (rad + orad) * 1.35 for ocx, ocy, orad, _, _, _ in craters):
            continue
        near_track = False
        for spine in _SAMPLED_TRACK_SPINES:
            for tx_p, ty_p, _, _, _ in spine[::2]:
                if math.hypot(cx - tx_p, cy - ty_p) < 0.78 + rad * 1.55:
                    near_track = True
                    break
            if near_track:
                break
        if near_track:
            continue

        depth = rad * rng.uniform(0.22, 0.28)
        rim_h = depth * rng.uniform(0.42, 0.52)
        phase = rng.uniform(0.0, 6.28)
        craters.append((cx, cy, rad, depth, rim_h, phase))

    # Add 140 small weathered lunar pockmark micro-craters (R = 0.14m .. 0.28m) across the regolith
    attempts = 0
    while len(craters) < 192 and attempts < 3500:
        attempts += 1
        cx = rng.uniform(-17.2, 17.2)
        cy = rng.uniform(-17.2, 17.2)
        if math.hypot(cx, cy) > 17.4:
            continue
        rad = rng.uniform(0.14, 0.28)
        if any(math.hypot(cx - px, cy - py) < prad + rad * 1.80 for px, py, prad in protected_circles):
            continue
        if any(math.hypot(cx - ocx, cy - ocy) < (rad + orad) * 1.18 for ocx, ocy, orad, _, _, _ in craters):
            continue
        near_track = False
        for spine in _SAMPLED_TRACK_SPINES:
            for tx_p, ty_p, _, _, _ in spine[::3]:
                if math.hypot(cx - tx_p, cy - ty_p) < 0.56 + rad * 1.45:
                    near_track = True
                    break
            if near_track:
                break
        if near_track:
            continue

        depth = rad * rng.uniform(0.18, 0.24)
        rim_h = depth * rng.uniform(0.36, 0.46)
        phase = rng.uniform(0.0, 6.28)
        craters.append((cx, cy, rad, depth, rim_h, phase))

    return craters


NEARFIELD_CRATERS = _build_nearfield_craters()


def _tyre_track_metrics(x: float, y: float) -> tuple[float, float, float]:
    """Return `(min_d_wheel, signed_offset, arc_s)` to the closest left/right rover wheel centerline
    (`±0.26 m` lateral offset from each rover trajectory spine). Fast bounding check included."""
    if x < -11.5 or x > 18.0 or y < -5.8 or y > 8.0:
        return 99.0, 0.0, 0.0
    best_d = 99.0
    best_signed = 0.0
    best_s = 0.0
    half_gauge = 0.26
    for spine in _SAMPLED_TRACK_SPINES:
        # Coarse check every 5th sample (~0.275m) first
        min_coarse = 99.0
        best_idx = 0
        for idx in range(0, len(spine), 5):
            sx, sy, _, _, _ = spine[idx]
            d = abs(x - sx) + abs(y - sy)
            if d < min_coarse:
                min_coarse = d
                best_idx = idx
        if min_coarse > 1.35:
            continue
        i0 = max(0, best_idx - 5)
        i1 = min(len(spine), best_idx + 6)
        for idx in range(i0, i1):
            sx, sy, tx, ty, s_val = spine[idx]
            nx, ny = -ty, tx
            dx = x - sx
            dy = y - sy
            along = dx * tx + dy * ty
            if abs(along) > 0.065:
                continue
            lat = dx * nx + dy * ny
            # Left wheel at +0.26m, Right wheel at -0.26m
            for w_center in (-half_gauge, half_gauge):
                signed_w = lat - w_center
                dw = math.hypot(along * 0.4, signed_w)
                if dw < best_d:
                    best_d = dw
                    best_signed = signed_w
                    best_s = s_val + along
    return best_d, best_signed, best_s


def _inner_sculpt_dz(x: float, y: float) -> float:
    """Long-wavelength C2 smooth baseline lunar relief inside r < 14.0m (zero hard edges)."""
    r = math.hypot(x, y)
    if r >= 14.0:
        return 0.0
    t_out = max(0.0, min(1.0, (14.0 - r) / 4.5))
    env_out = t_out * t_out * (3.0 - 2.0 * t_out)
    # Smooth C2 Gaussian attenuation around the rover spawn & Safe Route B corridor
    corridor_atten = 1.0 - math.exp(-(((x - 0.85) / 2.8) ** 2 + ((y + 0.45) / 1.65) ** 2))

    mounds = (
        0.36 * math.exp(-(((x - 6.8) / 4.6) ** 2 + ((y - 5.8) / 4.0) ** 2))
        + 0.34 * math.exp(-(((x - 7.8) / 4.4) ** 2 + ((y + 6.8) / 4.0) ** 2))
        + 0.28 * math.exp(-(((x + 6.4) / 4.6) ** 2 + ((y - 4.8) / 4.2) ** 2))
        - 0.22 * math.exp(-(((x - 8.5) / 4.0) ** 2 + ((y - 1.5) / 3.6) ** 2))
    )
    return env_out * corridor_atten * mounds


def _smooth_noise2d(x: float, y: float, scale: float, seed: int = 0) -> float:
    """Isotropic C2 quintic-interpolated 2D value noise in [-1.0, +1.0] (zero plane-wave stripes)."""
    u = x / scale + seed * 17.3
    v = y / scale + seed * 31.7
    ix = int(math.floor(u))
    iy = int(math.floor(v))
    fx = u - ix
    fy = v - iy
    ux = fx * fx * fx * (fx * (fx * 6.0 - 15.0) + 10.0)
    uy = fy * fy * fy * (fy * (fy * 6.0 - 15.0) + 10.0)
    n00 = _hash2d(ix, iy)
    n10 = _hash2d(ix + 1, iy)
    n01 = _hash2d(ix, iy + 1)
    n11 = _hash2d(ix + 1, iy + 1)
    nx0 = n00 + ux * (n10 - n00)
    nx1 = n01 + ux * (n11 - n01)
    return nx0 + uy * (nx1 - nx0)


def _nearfield_crater_and_micro_dz(x: float, y: float) -> float:
    """High-resolution 3D lunar topography carved into `lunar_impact_crater.obj` (`r <= 21.0 m`):
    - Multi-scale isotropic C2 lunar regolith undulations & micro-relief (zero stripes),
    - 192 real sunken 3D concave bowl craters & micro-pockmarks with raised circular ejecta rims,
    - Real 3D sunken rover wheel ruts (`-1.6 cm`) and raised flanking dust shoulders (`+1.0 cm`)."""
    r = math.hypot(x, y)
    if r >= 20.6:
        return 0.0
    t_env = max(0.0, min(1.0, (20.6 - r) / 3.2))
    env = t_env * t_env * (3.0 - 2.0 * t_env)

    # 1. Isotropic multi-scale lunar regolith undulations & roughness (zero plane-wave stripes)
    micro = (
        0.065 * _smooth_noise2d(x, y, 4.8, seed=1)
        + 0.036 * _smooth_noise2d(x, y, 2.3, seed=2)
        + 0.018 * _smooth_noise2d(x, y, 1.1, seed=3)
    )
    # Smooth C2 Gaussian attenuation at rover spawn, Safe Route B, and building pads
    corridor_g = math.exp(-(((x - 0.75) / 2.2) ** 2 + ((y + 0.45) / 1.15) ** 2))
    pad_atten = 1.0 - 0.90 * corridor_g
    for px, py, prad in (
        (0.0, 0.0, 1.8),
        (-10.50, 7.50, 4.2),
        (-11.00, -4.50, 3.2),
        (-8.20, 4.20, 1.2),
        (3.80, -2.20, 1.2),
        (1.55, 0.32, 0.8),
        (4.80, 3.90, 0.9),
        (5.40, -5.80, 0.9),
    ):
        dp = math.hypot(x - px, y - py)
        if dp < prad:
            t_p = dp / prad
            s_p = t_p * t_p * (3.0 - 2.0 * t_p)
            pad_atten = min(pad_atten, s_p)
    dz = micro * pad_atten

    # 2. 192 Real Sunken 3D Impact Craters & Micro-Pockmarks (concave bowl + raised circular rim)
    for cx, cy, rad, depth, rim_h, phase in NEARFIELD_CRATERS:
        rmax = rad * 1.75
        if abs(x - cx) > rmax or abs(y - cy) > rmax:
            continue
        dx = x - cx
        dy = y - cy
        th = math.atan2(dy, dx)
        asym = (
            1.0
            + 0.055 * math.sin(2.0 * th + phase)
            + 0.035 * math.cos(3.0 * th - phase)
            + 0.020 * math.sin(5.0 * th)
        )
        u = math.hypot(dx, dy) / (rad * asym)
        if u >= 1.72:
            continue
        if u < 1.0:
            bowl = -depth * ((1.0 - u * u) ** 1.35)
        else:
            bowl = 0.0
        rim_u = (u - 1.03) / 0.27
        rim_rug = 1.0 + 0.10 * math.sin(4.0 * th + phase) + 0.07 * math.cos(7.0 * th)
        rim = rim_h * math.exp(-(rim_u * rim_u)) * rim_rug
        if u > 1.45:
            fade = max(0.0, (1.72 - u) / 0.27)
            rim *= fade * fade * (3.0 - 2.0 * fade)
        dz += bowl + rim

    # 3. 3D Sunken Rover Wheel Ruts (-1.6 cm) & Flanking Displaced Dust Shoulders (+1.0 cm)
    d_wheel, _, _ = _tyre_track_metrics(x, y)
    if d_wheel < 0.095:
        if d_wheel < 0.048:
            rut_w = math.cos(0.5 * math.pi * (d_wheel / 0.048))
            dz -= 0.016 * (rut_w ** 0.85)
        else:
            berm_u = (d_wheel - 0.048) / 0.047
            dz += 0.010 * math.sin(math.pi * berm_u)

    return env * dz


def sculpt_base_visual_terrain_obj() -> None:
    """Smooth `lunar_terrain.obj` with a 2-pass Gaussian filter, store the un-depressed baseline
    grid in `_OUTER_GRID_Z`, and depress `lunar_terrain.obj` by `-0.68 m` inside `r < 17.4 m`
    so the continuous `156 x 156` near-field lunar surface (`lunar_impact_crater.obj`) can carve
    real sunken 3D concave crater bowls up to `45 cm` deep without `lunar_terrain.obj` clipping through."""
    global _OUTER_GRID_Z
    tpath = MESH_DIR / "lunar_terrain.obj"
    if not tpath.is_file():
        return
    header_lines: list[str] = []
    v_coords: list[tuple[float, float, float]] = []
    c_lines: list[str] = []
    f_lines: list[str] = []
    already_smoothed = False
    already_depressed = False
    with tpath.open("r", encoding="utf-8") as fh:
        for line in fh:
            s = line.rstrip("\n")
            if s == "# SMOOTH_ANTIALIASED_V2":
                already_smoothed = True
            elif s == "# DEPRESSED_NEARFIELD_V3":
                already_depressed = True
            if s.startswith("v "):
                p = s.split()
                v_coords.append((float(p[1]), float(p[2]), float(p[3])))
            elif s.startswith("c "):
                c_lines.append(s)
            elif s.startswith("vn "):
                continue
            elif s.startswith("f "):
                f_lines.append(s)
            else:
                header_lines.append(s)

    if len(v_coords) != 320 * 320:
        return

    zs = [z for _, _, z in v_coords]
    if not already_smoothed:
        for _ in range(2):
            tmp = list(zs)
            for iy in range(1, 319):
                row = iy * 320
                for ix in range(1, 319):
                    tmp[row + ix] = 0.25 * zs[row + ix - 1] + 0.50 * zs[row + ix] + 0.25 * zs[row + ix + 1]
            for iy in range(1, 319):
                row = iy * 320
                for ix in range(1, 319):
                    zs[row + ix] = 0.25 * tmp[(iy - 1) * 320 + ix] + 0.50 * tmp[row + ix] + 0.25 * tmp[(iy + 1) * 320 + ix]
        header_lines.append("# SMOOTH_ANTIALIASED_V2")

    outer_zs = list(zs)
    for idx, (x, y, _) in enumerate(v_coords):
        r = math.hypot(x, y)
        t_dep = max(0.0, min(1.0, (20.4 - r) / 3.0))
        s_dep = t_dep * t_dep * (3.0 - 2.0 * t_dep)
        if already_depressed and r < 20.4:
            outer_zs[idx] = zs[idx] + 0.68 * s_dep
        if r < 14.0:
            outer_zs[idx] = -2.7880 + _inner_sculpt_dz(x, y)
        if r < 20.4:
            zs[idx] = outer_zs[idx] - 0.68 * s_dep
        else:
            zs[idx] = outer_zs[idx]
        v_coords[idx] = (x, y, zs[idx])

    if not already_depressed:
        header_lines.append("# DEPRESSED_NEARFIELD_V3")

    _OUTER_GRID_Z = outer_zs

    step = 1.25
    vn_lines: list[str] = []
    for iy in range(320):
        iy_prev = max(0, iy - 1)
        iy_next = min(319, iy + 1)
        dy = (iy_next - iy_prev) * step
        for ix in range(320):
            ix_prev = max(0, ix - 1)
            ix_next = min(319, ix + 1)
            dx = (ix_next - ix_prev) * step
            dz_dx = (zs[iy * 320 + ix_next] - zs[iy * 320 + ix_prev]) / dx
            dz_dy = (zs[iy_next * 320 + ix] - zs[iy_prev * 320 + ix]) / dy
            nx, ny, nz = _normalize(-dz_dx, -dz_dy, 1.0)
            vn_lines.append(f"vn {nx:.5f} {ny:.5f} {nz:.5f}")

    out_lines: list[str] = list(header_lines)
    for (x, y, z), c_str in zip(v_coords, c_lines):
        out_lines.append(f"v {x:.4f} {y:.4f} {z:.4f}")
        out_lines.append(c_str)
    out_lines.extend(vn_lines)
    out_lines.extend(f_lines)
    tpath.write_text("\n".join(out_lines) + "\n", encoding="utf-8")


def _base_terrain_delta_z(x: float, y: float) -> float:
    """Bilinearly sample the un-depressed baseline terrain height relative to z=-2.790."""
    global _OUTER_GRID_Z
    if _OUTER_GRID_Z is None:
        tpath = MESH_DIR / "lunar_terrain.obj"
        zs: list[float] = []
        already_depressed = False
        if tpath.is_file():
            with tpath.open("r", encoding="utf-8") as fh:
                for line in fh:
                    s = line.rstrip("\n")
                    if s == "# DEPRESSED_NEARFIELD_V3":
                        already_depressed = True
                    elif s.startswith("v "):
                        parts = s.split()
                        vx, vy, vz = float(parts[1]), float(parts[2]), float(parts[3])
                        r = math.hypot(vx, vy)
                        if already_depressed and r < 20.4:
                            t_dep = max(0.0, min(1.0, (20.4 - r) / 3.0))
                            s_dep = t_dep * t_dep * (3.0 - 2.0 * t_dep)
                            vz += 0.68 * s_dep
                        if r < 14.0:
                            vz = -2.7880 + _inner_sculpt_dz(vx, vy)
                        zs.append(vz)
        _OUTER_GRID_Z = zs if len(zs) == 320 * 320 else [0.0] * (320 * 320)

    u = (x - (-199.375)) / 1.25
    v = (y - (-199.375)) / 1.25
    ix = max(0, min(318, int(math.floor(u))))
    iy = max(0, min(318, int(math.floor(v))))
    fx = max(0.0, min(1.0, u - ix))
    fy = max(0.0, min(1.0, v - iy))
    g = _OUTER_GRID_Z
    z00 = g[iy * 320 + ix]
    z10 = g[iy * 320 + ix + 1]
    z01 = g[(iy + 1) * 320 + ix]
    z11 = g[(iy + 1) * 320 + ix + 1]
    z_interp = (1.0 - fy) * ((1.0 - fx) * z00 + fx * z10) + fy * ((1.0 - fx) * z01 + fx * z11)
    return z_interp - (-2.790)


def lunar_relief_z(x: float, y: float) -> float:
    """Return exact visible 3D lunar ground height relative to z=-2.790m at (x, y) across the
    400m x 400m world (including near-field sunken craters, regolith micro-relief, and tyre ruts)."""
    return _base_terrain_delta_z(x, y) + _nearfield_crater_and_micro_dz(x, y)


def _normalize(vx: float, vy: float, vz: float) -> tuple[float, float, float]:
    n = math.sqrt(vx * vx + vy * vy + vz * vz)
    if n < 1e-9:
        return 0.0, 0.0, 1.0
    return vx / n, vy / n, vz / n


def _cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _hash2d(ix: int, iy: int) -> float:
    """Deterministic fast 2D spatial hash in [-1.0, +1.0] for lunar regolith soil grain."""
    n = (ix * 374761393 + iy * 668265263) & 0xFFFFFFFF
    n = ((n ^ (n >> 13)) * 1274126177) & 0xFFFFFFFF
    n = n ^ (n >> 16)
    return (n / 2147483647.5) - 1.0


def _write_rgb_png(path: Path, width: int, height: int, rgb_bytes: bytes | bytearray) -> None:
    """Write a standard 24-bit RGB PNG file using pure Python `zlib` + `struct`."""
    def _chunk(tag: bytes, data: bytes) -> bytes:
        crc = zlib.crc32(tag + data) & 0xFFFFFFFF
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", crc)

    raw_rows = bytearray()
    stride = width * 3
    for y in range(height):
        raw_rows.append(0)  # Filter type 0 (None)
        row_start = y * stride
        raw_rows.extend(rgb_bytes[row_start : row_start + stride])

    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    idat = zlib.compress(bytes(raw_rows), level=6)
    png_data = (
        b"\x89PNG\r\n\x1a\n"
        + _chunk(b"IHDR", ihdr)
        + _chunk(b"IDAT", idat)
        + _chunk(b"IEND", b"")
    )
    path.write_bytes(png_data)


def generate_lunar_regolith_textures() -> None:
    """Generate `lunar_regolith_albedo.png` (512x512 high-detail near-field lunar regolith texture
    with micro-pockmarks, crater bowl shadows, radial ejecta rays, and Rover Tyre Marks) and
    `lunar_regolith.mtl` in `src/lunabot_gazebo/worlds/meshes/`."""
    width = height = 512
    span = 42.4  # [-21.2, +21.2] m
    half = 21.2
    step = span / (width - 1)
    sx, sy, sz = _normalize(*SUN_DIR)

    # Precompute 512x512 smooth C2 height grid for exact finite-difference normal shading
    z_grid = [0.0] * (width * height)
    for iy in range(height):
        wy = half - iy * step
        row = iy * width
        for ix in range(width):
            wx = -half + ix * step
            z_grid[row + ix] = _inner_sculpt_dz(wx, wy) + _nearfield_crater_and_micro_dz(wx, wy)

    rgb = bytearray(width * height * 3)
    for iy in range(height):
        wy = half - iy * step
        iy_p = max(0, iy - 1)
        iy_n = min(height - 1, iy + 1)
        dy_w = (iy_p - iy_n) * step if iy_n != iy_p else step
        for ix in range(width):
            wx = -half + ix * step
            ix_p = max(0, ix - 1)
            ix_n = min(width - 1, ix + 1)
            dx_w = (ix_n - ix_p) * step if ix_n != ix_p else step
            dz_dx = (z_grid[iy * width + ix_n] - z_grid[iy * width + ix_p]) / abs(dx_w)
            dz_dy = (z_grid[iy_p * width + ix] - z_grid[iy_n * width + ix]) / abs(dy_w)
            nx, ny, nz = _normalize(-dz_dx, -dz_dy, 1.0)
            ndotl = max(0.0, nx * sx + ny * sy + nz * sz)

            # 1. Powdery lunar regolith micro-grain & fine pockmark noise
            g1 = _hash2d(ix, iy) * 0.045
            g2 = _hash2d(ix // 2, iy // 2) * 0.035
            g3 = _hash2d(ix // 5, iy // 5) * 0.030
            albedo = 0.56 + g1 + g2 + g3

            # 2. Low-angle solar hillshading
            shade = 0.28 + 0.92 * (ndotl ** 0.85)

            # 3. Crater bowl self-shadowing & radial ejecta rays
            for cx, cy, rad, _, _, phase in NEARFIELD_CRATERS:
                rmax = rad * 1.65
                if abs(wx - cx) > rmax or abs(wy - cy) > rmax:
                    continue
                dx = wx - cx
                dy = wy - cy
                u = math.hypot(dx, dy) / rad
                if u < 1.02:
                    # Sunward inner slope is cast in deep lunar crater shadow
                    sun_proj = (dx * sx + dy * sy) / rad
                    if sun_proj < 0.18:
                        sh_strength = max(0.0, min(1.0, (0.18 - sun_proj) / 0.65)) * (1.0 - 0.5 * (u ** 2))
                        shade *= 1.0 - 0.72 * sh_strength
                    elif sun_proj > 0.22:
                        shade *= 1.0 + 0.16 * min(1.0, (sun_proj - 0.22) / 0.55)
                elif u < 1.60:
                    th = math.atan2(dy, dx)
                    rays = max(0.0, math.sin(9.0 * th + phase) * math.cos(14.0 * th - phase))
                    albedo += 0.045 * rays * (1.60 - u) / 0.58

            # 4. High-detail Rover Tyre Marks (compressed dark wheel ruts + V-chevron lug treads)
            d_wheel, signed_w, arc_s = _tyre_track_metrics(wx, wy)
            if d_wheel < 0.082:
                if d_wheel < 0.048:
                    # Inside 9.6cm tyre footprint: compressed dark regolith + angled chevron cleats
                    chev_phase = (arc_s - 0.42 * abs(signed_w)) / 0.055
                    frac = chev_phase - math.floor(chev_phase)
                    if frac < 0.34:
                        # Raised chevron cleat bar catching low-angle light
                        shade *= 0.88
                    elif frac < 0.52:
                        # Deep shadow groove right behind cleat bar
                        shade *= 0.44
                    else:
                        # Compressed regolith trough between cleats
                        shade *= 0.58
                else:
                    # Flanking extruded lunar dust shoulder along outer edge of tyre track
                    shade *= 1.12

            val = max(0.03, min(0.96, albedo * shade))
            r_b = int(round(val * 250))
            g_b = int(round(val * 250))
            b_b = int(round(min(255, val * 255)))
            p_idx = (iy * width + ix) * 3
            rgb[p_idx] = r_b
            rgb[p_idx + 1] = g_b
            rgb[p_idx + 2] = b_b

    _write_rgb_png(MESH_DIR / "lunar_regolith_albedo.png", width, height, rgb)

    mtl_text = (
        "# High-resolution Lunar Regolith & Rover Tyre-Track Material\n"
        "newmtl LunarRegolith\n"
        "Ka 0.5200 0.5200 0.5400\n"
        "Kd 0.8400 0.8400 0.8600\n"
        "Ks 0.0300 0.0300 0.0300\n"
        "Ns 8.0\n"
        "map_Kd lunar_regolith_albedo.png\n"
    )
    (MESH_DIR / "lunar_regolith.mtl").write_text(mtl_text, encoding="utf-8")


def _write_flat_shaded_obj(
    path: Path,
    verts: list[tuple[float, float, float, float, float, float]],
    faces: list[tuple[int, int, int]],
    header: str,
    bake_sun: bool = True,
    contrast: float = 1.0,
) -> None:
    """Write OBJ with per-face normals and crisp flat-facet lunar solar shading (for jagged rocks)."""
    sx, sy, sz = _normalize(*SUN_DIR)
    normals: list[tuple[float, float, float]] = []
    v_light = [0.0] * len(verts)
    v_count = [0] * len(verts)

    for i1, i2, i3 in faces:
        v1 = verts[i1 - 1]
        v2 = verts[i2 - 1]
        v3 = verts[i3 - 1]
        e1 = (v2[0] - v1[0], v2[1] - v1[1], v2[2] - v1[2])
        e2 = (v3[0] - v1[0], v3[1] - v1[1], v3[2] - v1[2])
        fn = _normalize(*_cross(e1, e2))
        normals.append(fn)
        ndotl = max(0.0, fn[0] * sx + fn[1] * sy + fn[2] * sz)
        fl = 0.16 + (1.22 * contrast) * (ndotl ** 0.9)
        for idx in (i1 - 1, i2 - 1, i3 - 1):
            v_light[idx] += fl
            v_count[idx] += 1

    lines = [f"# {header}", f"o {path.stem}"]
    for idx, (vx, vy, vz, cr, cg, cb) in enumerate(verts):
        if bake_sun and v_count[idx] > 0:
            light = v_light[idx] / v_count[idx]
            cr = max(0.03, min(0.95, cr * light))
            cg = max(0.03, min(0.95, cg * light))
            cb = max(0.03, min(0.95, cb * light))
        lines.append(f"v {vx:.4f} {vy:.4f} {vz:.4f} {cr:.4f} {cg:.4f} {cb:.4f}")
    for nx, ny, nz in normals:
        lines.append(f"vn {nx:.4f} {ny:.4f} {nz:.4f}")
    for fn_idx, (i1, i2, i3) in enumerate(faces, start=1):
        lines.append(f"f {i1}//{fn_idx} {i2}//{fn_idx} {i3}//{fn_idx}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_smooth_shaded_obj(
    path: Path,
    verts: list[tuple[float, float, float, float, float, float]],
    faces: list[tuple[int, int, int]],
    header: str,
    bake_sun: bool = True,
    contrast: float = 1.0,
    uvs: list[tuple[float, float]] | None = None,
    mtl_file: str | None = None,
    mtl_name: str | None = None,
) -> None:
    """Write OBJ with smooth area-weighted per-vertex Gouraud normals (and optional UV texture
    coordinates + `.mtl` reference for photorealistic textured lunar terrain)."""
    sx, sy, sz = _normalize(*SUN_DIR)
    vn_acc = [[0.0, 0.0, 0.0] for _ in verts]
    v_light = [0.0] * len(verts)
    v_count = [0] * len(verts)

    for i1, i2, i3 in faces:
        v1 = verts[i1 - 1]
        v2 = verts[i2 - 1]
        v3 = verts[i3 - 1]
        e1 = (v2[0] - v1[0], v2[1] - v1[1], v2[2] - v1[2])
        e2 = (v3[0] - v1[0], v3[1] - v1[1], v3[2] - v1[2])
        cx_n, cy_n, cz_n = _cross(e1, e2)
        fn = _normalize(cx_n, cy_n, cz_n)
        ndotl = max(0.0, fn[0] * sx + fn[1] * sy + fn[2] * sz)
        fl = 0.16 + (1.22 * contrast) * (ndotl ** 0.9)
        for idx in (i1 - 1, i2 - 1, i3 - 1):
            vn_acc[idx][0] += cx_n
            vn_acc[idx][1] += cy_n
            vn_acc[idx][2] += cz_n
            v_light[idx] += fl
            v_count[idx] += 1

    lines = [f"# {header}", f"o {path.stem}"]
    if mtl_file:
        lines.append(f"mtllib {mtl_file}")
    if mtl_name:
        lines.append(f"usemtl {mtl_name}")
    for idx, (vx, vy, vz, cr, cg, cb) in enumerate(verts):
        if bake_sun and v_count[idx] > 0:
            light = v_light[idx] / v_count[idx]
            cr = max(0.03, min(0.95, cr * light))
            cg = max(0.03, min(0.95, cg * light))
            cb = max(0.03, min(0.95, cb * light))
        lines.append(f"v {vx:.4f} {vy:.4f} {vz:.4f} {cr:.4f} {cg:.4f} {cb:.4f}")
    if uvs is not None:
        for u, v in uvs:
            lines.append(f"vt {u:.5f} {v:.5f}")
    for ax, ay, az in vn_acc:
        nx, ny, nz = _normalize(ax, ay, az)
        lines.append(f"vn {nx:.5f} {ny:.5f} {nz:.5f}")
    if uvs is not None:
        for i1, i2, i3 in faces:
            lines.append(f"f {i1}/{i1}/{i1} {i2}/{i2}/{i2} {i3}/{i3}/{i3}")
    else:
        for i1, i2, i3 in faces:
            lines.append(f"f {i1}//{i1} {i2}//{i2} {i3}//{i3}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _append_angular_rock(
    verts: list[tuple[float, float, float, float, float, float]],
    faces: list[tuple[int, int, int]],
    *,
    rng: random.Random,
    cx: float,
    cy: float,
    rx: float,
    ry: float,
    height: float,
    base_rgb: tuple[float, float, float],
    n_rings: int = 5,
    n_lon: int = 9,
    broad_crest: bool = False,
    use_terrain_z: bool = False,
) -> None:
    """Append a sharp-edged, angular cleaved lunar rock with a dark shadow apron."""
    sx, sy, _ = _normalize(*SUN_DIR)
    base_idx = len(verts) + 1
    z_off = lunar_relief_z(cx, cy) if use_terrain_z else 0.0
    pk_x = cx + rng.uniform(-0.12, 0.12) * rx
    pk_y = cy + rng.uniform(-0.12, 0.12) * ry
    verts.append((pk_x, pk_y, z_off + height, base_rgb[0] * 1.1, base_rgb[1] * 1.1, base_rgb[2] * 1.1))

    spoke_r = [rng.uniform(0.68, 1.32) for _ in range(n_lon)]
    ring_twist = [rng.uniform(-0.18, 0.18) for _ in range(n_rings + 1)]

    for ir in range(1, n_rings + 1):
        u = ir / n_rings
        twist = ring_twist[ir]
        for j in range(n_lon):
            theta = 2.0 * math.pi * j / n_lon + twist
            rad_asym = spoke_r[j] * rng.uniform(0.90, 1.10)
            if ir < n_rings:
                if broad_crest:
                    z_prof = max(0.05, 1.0 - (u ** 2.3))
                else:
                    z_prof = max(0.04, (1.0 - u) ** 0.92)
                z = height * z_prof * rng.uniform(0.88, 1.12)
                r_mult = u * rad_asym
                shade = rng.uniform(0.82, 1.15)
                cr = base_rgb[0] * shade
                cg = base_rgb[1] * shade
                cb = base_rgb[2] * shade
            else:
                z = -0.015 if use_terrain_z else 0.0
                r_mult = 1.12 * rad_asym
                ux, uy = math.cos(theta), math.sin(theta)
                sun_dot = ux * sx + uy * sy
                shadow = 0.22 if sun_dot < -0.10 else 0.78
                cr = 0.42 * shadow
                cg = 0.42 * shadow
                cb = 0.43 * shadow

            vx = cx + rx * r_mult * math.cos(theta)
            vy = cy + ry * r_mult * math.sin(theta)
            vz_ground = lunar_relief_z(vx, vy) if use_terrain_z else 0.0
            verts.append((vx, vy, vz_ground + z, cr, cg, cb))

    for j in range(n_lon):
        jn = (j + 1) % n_lon
        faces.append((base_idx, base_idx + 1 + j, base_idx + 1 + jn))
    for ir in range(n_rings - 1):
        row_a = base_idx + 1 + ir * n_lon
        row_b = row_a + n_lon
        for j in range(n_lon):
            jn = (j + 1) % n_lon
            faces.append((row_a + j, row_b + j, row_b + jn))
            faces.append((row_a + j, row_b + jn, row_a + jn))


def generate_angular_boulder_obj(
    path: Path,
    *,
    seed: int,
    rx: float,
    ry: float,
    height: float,
    base_rgb: tuple[float, float, float],
    broad_crest: bool = False,
) -> None:
    """Generate a sharp-faceted angular lunar boulder surrounded by broken basal fragments."""
    rng = random.Random(seed)
    verts: list[tuple[float, float, float, float, float, float]] = []
    faces: list[tuple[int, int, int]] = []

    _append_angular_rock(
        verts,
        faces,
        rng=rng,
        cx=0.0,
        cy=0.0,
        rx=rx,
        ry=ry,
        height=height,
        base_rgb=base_rgb,
        n_rings=6,
        n_lon=11,
        broad_crest=broad_crest,
    )

    for k in range(5):
        ang = 2.0 * math.pi * k / 5.0 + rng.uniform(-0.3, 0.3)
        dist = rng.uniform(0.75, 1.05)
        fx = rx * dist * math.cos(ang)
        fy = ry * dist * math.sin(ang)
        frx = rx * rng.uniform(0.22, 0.36)
        fry = ry * rng.uniform(0.20, 0.34)
        fh = height * rng.uniform(0.16, 0.28)
        _append_angular_rock(
            verts,
            faces,
            rng=rng,
            cx=fx,
            cy=fy,
            rx=frx,
            ry=fry,
            height=fh,
            base_rgb=base_rgb,
            n_rings=3,
            n_lon=7,
        )

    _write_flat_shaded_obj(path, verts, faces, f"Angular faceted lunar boulder ({path.name})", contrast=1.15)


def _append_shadow_apron(
    verts: list[tuple[float, float, float, float, float, float]],
    faces: list[tuple[int, int, int]],
    *,
    cx: float,
    cy: float,
    rx: float,
    ry: float,
    shadow_len: float,
    z_ground: float = 0.004,
) -> None:
    """Append a low-angle solar shadow tail behind a boulder/structure conformed to `lunar_relief_z`."""
    sx, sy, _ = _normalize(*SUN_DIR)
    dx, dy = -sx, -sy
    px, py = -dy, dx
    base_idx = len(verts) + 1
    w0 = max(rx, ry) * 0.90
    w1 = w0 * 0.56
    xy_rgb = [
        (cx + px * w0, cy + py * w0, 0.035, 0.035, 0.042),
        (cx - px * w0, cy - py * w0, 0.035, 0.035, 0.042),
        (cx + dx * (shadow_len * 0.62) + px * w1, cy + dy * (shadow_len * 0.62) + py * w1, 0.045, 0.045, 0.052),
        (cx + dx * (shadow_len * 0.62) - px * w1, cy + dy * (shadow_len * 0.62) - py * w1, 0.045, 0.045, 0.052),
        (cx + dx * shadow_len + px * (w1 * 0.35), cy + dy * shadow_len + py * (w1 * 0.35), 0.16, 0.16, 0.17),
        (cx + dx * shadow_len - px * (w1 * 0.35), cy + dy * shadow_len - py * (w1 * 0.35), 0.16, 0.16, 0.17),
    ]
    for wx, wy, cr, cg, cb in xy_rgb:
        verts.append((wx, wy, lunar_relief_z(wx, wy) + z_ground, cr, cg, cb))
    faces.extend([
        (base_idx + 0, base_idx + 1, base_idx + 2),
        (base_idx + 1, base_idx + 3, base_idx + 2),
        (base_idx + 2, base_idx + 3, base_idx + 4),
        (base_idx + 3, base_idx + 5, base_idx + 4),
    ])


def generate_rough_regolith_sheet_obj(path: Path, *, seed: int = 909) -> None:
    """Generate the continuous `156 x 156` (`24,336` vertices, `0.273 m` cell spacing) 3D near-field
    lunar regolith landscape (`lunar_impact_crater.obj`) over `[-21.2, +21.2] x [-21.2, +21.2]` with:
    - 48 real sunken 3D concave bowl craters and raised circular ejecta rims,
    - 3D sunken rover wheel ruts (`-1.6 cm`) and raised flanking dust shoulders (`+1.0 cm`),
    - Exact UV texture coordinates `vt u v` mapped to `lunar_regolith_albedo.png` (`lunar_regolith.mtl`),
    - Smooth outer skirt blending seamlessly into `lunar_terrain.obj` at `r = 20.6 m` with zero border."""
    _ = seed
    n_grid = 156
    half = 21.2
    span = 42.4
    step = span / (n_grid - 1)
    verts: list[tuple[float, float, float, float, float, float]] = []
    uvs: list[tuple[float, float]] = []
    faces: list[tuple[int, int, int]] = []

    for iy in range(n_grid):
        wy = -half + iy * step
        v_coord = 1.0 - (iy / (n_grid - 1))
        for ix in range(n_grid):
            wx = -half + ix * step
            u_coord = ix / (n_grid - 1)
            r = math.hypot(wx, wy)
            z = lunar_relief_z(wx, wy)
            # Outside r > 19.6 m, gently tuck the outer perimeter 4 cm below `lunar_terrain.obj`
            # so there is zero rectangular edge or seam
            if r > 19.6:
                t_skirt = min(1.0, (r - 19.6) / 1.5)
                s_skirt = t_skirt * t_skirt * (3.0 - 2.0 * t_skirt)
                z -= 0.042 * s_skirt
            verts.append((wx, wy, z, 0.58, 0.58, 0.60))
            uvs.append((u_coord, v_coord))

    for iy in range(n_grid - 1):
        row_a = 1 + iy * n_grid
        row_b = row_a + n_grid
        for ix in range(n_grid - 1):
            i00 = row_a + ix
            i10 = i00 + 1
            i01 = row_b + ix
            i11 = i01 + 1
            faces.append((i00, i10, i11))
            faces.append((i00, i11, i01))

    _write_smooth_shaded_obj(
        path,
        verts,
        faces,
        "Continuous 3D Cratered Lunar Regolith Surface with Sunken Bowls & Rover Tyre Ruts",
        contrast=1.22,
        uvs=uvs,
        mtl_file="lunar_regolith.mtl",
        mtl_name="LunarRegolith",
    )


def generate_foreground_regolith_obj(path: Path, *, seed: int = 909) -> None:
    """Generate natural angular basalt rock scatter (`lunar_small_rocks.obj`) around crater rims,
    outcrops, and settlement corridors, conformed to `lunar_relief_z(x, y)`."""
    rng = random.Random(seed)
    verts: list[tuple[float, float, float, float, float, float]] = []
    faces: list[tuple[int, int, int]] = []

    # 1. Ejecta cobbles & fragments around the Route-A Hazard Boulder (1.55, 0.32) & Crater (3.10, 1.35)
    route_a_ejecta = [
        (1.18, 0.58, 0.14, 0.11, 0.09),
        (1.88, 0.68, 0.16, 0.13, 0.10),
        (2.15, 0.42, 0.15, 0.12, 0.09),
        (1.42, 1.05, 0.16, 0.13, 0.10),
        (4.55, 2.15, 0.28, 0.22, 0.17),
        (3.85, 2.95, 0.30, 0.24, 0.18),
        (2.25, 2.75, 0.22, 0.18, 0.14),
    ]
    for rx_pos, ry_pos, sx, sy, h in route_a_ejecta:
        rock_tone = rng.uniform(0.18, 0.26)
        _append_angular_rock(
            verts, faces, rng=rng, cx=rx_pos, cy=ry_pos, rx=sx, ry=sy, height=h,
            base_rgb=(rock_tone, rock_tone, rock_tone * 1.03), n_rings=3, n_lon=7, use_terrain_z=True,
        )

    # 2. Natural Lunar Crater-Rim Ejecta Cobbles & Pebbles scattered around the near-field craters
    for cx, cy, rad, _, _, phase in NEARFIELD_CRATERS[:18]:
        n_cobbles = 4 if rad > 1.1 else 2
        for k in range(n_cobbles):
            ang = phase + 2.0 * math.pi * k / n_cobbles + rng.uniform(-0.35, 0.35)
            dist = rad * rng.uniform(1.05, 1.48)
            rx_pos = cx + dist * math.cos(ang)
            ry_pos = cy + dist * math.sin(ang)
            if -1.0 <= rx_pos <= 2.8 and -1.3 <= ry_pos <= 0.2:
                continue
            d_wheel, _, _ = _tyre_track_metrics(rx_pos, ry_pos)
            if d_wheel < 0.35:
                continue
            sx = rng.uniform(0.06, 0.18)
            sy = sx * rng.uniform(0.75, 1.25)
            h = sx * rng.uniform(0.50, 0.78)
            rock_tone = rng.uniform(0.20, 0.30)
            _append_angular_rock(
                verts, faces, rng=rng, cx=rx_pos, cy=ry_pos, rx=sx, ry=sy, height=h,
                base_rgb=(rock_tone, rock_tone, rock_tone * 1.02), n_rings=3, n_lon=7, use_terrain_z=True,
            )

    # 3. Corridor Rock Fields across the 7 Settlement Corridors
    corridor_rock_zones = [
        (12.0, 2.0, 8.0, 6.0, 22, 0.16, 0.40),
        (38.0, -14.0, 18.0, 10.0, 28, 0.28, 0.70),
        (62.0, -22.0, 12.0, 8.0, 18, 0.30, 0.76),
        (-36.0, 30.0, 16.0, 12.0, 24, 0.26, 0.66),
        (42.0, -42.0, 14.0, 12.0, 20, 0.28, 0.72),
        (36.0, 16.0, 14.0, 8.0, 14, 0.22, 0.50),
        (-38.0, -22.0, 14.0, 10.0, 10, 0.20, 0.44),
    ]
    for zx, zy, spx, spy, cnt, rmin, rmax in corridor_rock_zones:
        for _ in range(cnt):
            rx_pos = zx + rng.uniform(-spx, spx)
            ry_pos = zy + rng.uniform(-spy, spy)
            if -3.5 <= rx_pos <= 4.5 and -2.8 <= ry_pos <= 0.6:
                continue
            if math.hypot(rx_pos, ry_pos) < 3.8:
                continue
            d_wheel, _, _ = _tyre_track_metrics(rx_pos, ry_pos)
            if d_wheel < 0.45:
                continue
            size_x = rng.uniform(rmin, rmax)
            size_y = size_x * rng.uniform(0.75, 1.25)
            h = size_x * rng.uniform(0.55, 0.82)
            rock_tone = rng.uniform(0.18, 0.28)
            _append_angular_rock(
                verts, faces, rng=rng, cx=rx_pos, cy=ry_pos, rx=size_x, ry=size_y, height=h,
                base_rgb=(rock_tone, rock_tone, rock_tone * 1.03), n_rings=3, n_lon=7, use_terrain_z=True,
            )

    _write_flat_shaded_obj(path, verts, faces, "Multi-Corridor Lunar Basalt Rock Fields & Ejecta", contrast=1.22)


def generate_black_skydome_obj(path: Path, *, radius: float = 240.0, n_lat: int = 10, n_lon: int = 24) -> None:
    verts: list[tuple[float, float, float, float, float, float]] = []
    verts.append((0.0, 0.0, radius, 0.004, 0.004, 0.007))
    for i in range(1, n_lat + 1):
        phi = (0.62 * math.pi) * i / n_lat
        sp, cp = math.sin(phi), math.cos(phi)
        for j in range(n_lon):
            theta = 2.0 * math.pi * j / n_lon
            verts.append((radius * sp * math.cos(theta), radius * sp * math.sin(theta), radius * cp, 0.004, 0.004, 0.007))

    faces: list[tuple[int, int, int]] = []
    for j in range(n_lon):
        jn = (j + 1) % n_lon
        faces.append((1, 2 + jn, 2 + j))
    for i in range(n_lat - 1):
        row_a = 2 + i * n_lon
        row_b = row_a + n_lon
        for j in range(n_lon):
            jn = (j + 1) % n_lon
            faces.append((row_a + j, row_b + jn, row_b + j))
            faces.append((row_a + j, row_a + jn, row_b + jn))

    _write_flat_shaded_obj(path, verts, faces, "Inward-facing black skydome", bake_sun=False)


def generate_starfield_obj(path: Path, *, seed: int = 101, count: int = 340, radius: float = 210.0) -> None:
    rng = random.Random(seed)
    verts: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []

    for _ in range(count):
        theta = rng.uniform(0.0, 2.0 * math.pi)
        elev = rng.uniform(0.03, 0.48 * math.pi)
        cx = radius * math.cos(elev) * math.cos(theta)
        cy = radius * math.cos(elev) * math.sin(theta)
        cz = radius * math.sin(elev)
        s = rng.uniform(0.26, 0.62)
        base = len(verts) + 1
        verts.extend([
            (cx, cy - s, cz - s),
            (cx, cy + s, cz - s),
            (cx, cy, cz + s),
            (cx - s, cy, cz - s),
            (cx + s, cy, cz - s),
            (cx, cy, cz + s),
        ])
        faces.extend([
            (base + 0, base + 1, base + 2),
            (base + 1, base + 0, base + 2),
            (base + 3, base + 4, base + 5),
            (base + 4, base + 3, base + 5),
        ])

    lines = ["# Lunar deep-space starfield mesh", "o lunar_starfield"]
    for vx, vy, vz in verts:
        lines.append(f"v {vx:.3f} {vy:.3f} {vz:.3f}")
    for i1, i2, i3 in faces:
        lines.append(f"f {i1} {i2} {i3}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def generate_massif_ridge_obj(path: Path) -> None:
    """Grand South-Pole Shackleton / Malapert Massif mountain range across the distant horizon."""
    nx, ny = 84, 64
    width_x, width_y = 46.0, 110.0
    verts: list[tuple[float, float, float, float, float, float]] = []
    faces: list[tuple[int, int, int]] = []

    peaks = [
        (0.0, 6.0, 14.5, 13.5, 22.0),
        (-3.5, -16.0, 12.8, 12.0, 19.0),
        (5.5, 28.0, 16.5, 14.5, 21.0),
        (4.5, -32.0, 15.2, 13.5, 18.5),
    ]

    for iy in range(ny):
        y = -0.5 * width_y + width_y * iy / (ny - 1)
        for ix in range(nx):
            x = -0.5 * width_x + width_x * ix / (nx - 1)
            ex = max(0.0, math.sin(math.pi * ix / (nx - 1))) ** 0.7
            ey = max(0.0, math.sin(math.pi * iy / (ny - 1))) ** 0.7
            envelope = ex * ey
            z = 0.0
            for px, py, h, sx, sy in peaks:
                d2 = ((x - px) / sx) ** 2 + ((y - py) / sy) ** 2
                z += h * math.exp(-d2)
            rough = 0.30 * math.sin(0.26 * x + 0.22 * y) + 0.16 * math.cos(0.48 * x - 0.36 * y)
            z = max(0.0, (z + rough) * envelope)
            c = 0.48
            verts.append((x, y, z, c, c, c * 1.01))

    for iy in range(ny - 1):
        for ix in range(nx - 1):
            i00 = 1 + iy * nx + ix
            i10 = i00 + 1
            i01 = i00 + nx
            i11 = i01 + 1
            faces.append((i00, i10, i11))
            faces.append((i00, i11, i01))

    _write_smooth_shaded_obj(path, verts, faces, "Grand South-Pole Massif Mountain Range")


def _append_basalt_outcrop_cluster(
    verts: list[tuple[float, float, float, float, float, float]],
    faces: list[tuple[int, int, int]],
    *,
    rng: random.Random,
    cx: float,
    cy: float,
    rx: float,
    ry: float,
    max_h: float,
    angle: float,
) -> None:
    """Append a natural cluster of fractured, low-profile stratified basaltic rock slabs & ledges
    (replacing the old smooth brown oval grids so bedrock outcrops look like real lunar rock)."""
    ca, sa = math.cos(angle), math.sin(angle)
    # 5 overlapping tabular basalt slabs + 6 smaller peripheral talus shards
    slab_specs = [
        (0.0, 0.0, rx * 0.42, ry * 0.38, max_h * 0.85),
        (-rx * 0.32, ry * 0.12, rx * 0.34, ry * 0.30, max_h * 0.68),
        (rx * 0.30, -ry * 0.15, rx * 0.36, ry * 0.28, max_h * 0.62),
        (rx * 0.10, ry * 0.32, rx * 0.28, ry * 0.24, max_h * 0.52),
        (-rx * 0.14, -ry * 0.30, rx * 0.30, ry * 0.25, max_h * 0.55),
    ]
    for lx, ly, srx, sry, sh in slab_specs:
        wx = cx + lx * ca - ly * sa
        wy = cy + lx * sa + ly * ca
        if -1.0 <= wx <= 2.8 and -1.3 <= wy <= 0.2:
            continue
        _append_angular_rock(
            verts,
            faces,
            rng=rng,
            cx=wx,
            cy=wy,
            rx=srx,
            ry=sry,
            height=sh,
            base_rgb=(0.34, 0.32, 0.29),
            n_rings=4,
            n_lon=8,
            broad_crest=True,
            use_terrain_z=True,
        )


def _append_rover_tyre_tracks_3d(
    rut_verts: list[tuple[float, float, float, float, float, float]],
    rut_faces: list[tuple[int, int, int]],
    tread_verts: list[tuple[float, float, float, float, float, float]],
    tread_faces: list[tuple[int, int, int]],
) -> None:
    """Append high-detail 3D Rover Tyre Marks along all 4 rover trajectories:
    - Dual parallel wheel tracks separated by `0.52 m` (`±0.26 m` from rover centerline),
    - `9.2 cm`-wide compressed dark regolith wheel ruts (`rut_verts` / `rut_faces`),
    - Raised flanking dust shoulders along both sides of each wheel rut (`tread_verts` / `tread_faces`),
    - Repeating 3D angled V-chevron cleat tread bars every `5.5 cm` along each wheel rut!"""
    half_gauge = 0.26
    half_w = 0.046  # 9.2 cm tyre width

    for spine in _SAMPLED_TRACK_SPINES:
        if len(spine) < 2:
            continue
        for wheel_sign in (-1.0, 1.0):
            lat_off = wheel_sign * half_gauge
            # 1. Continuous compressed dark regolith wheel rut ribbon
            rut_base = len(rut_verts) + 1
            # 2. Outer and inner raised flanking dust shoulders
            sh_L_base = len(tread_verts) + 1
            for sx, sy, tx, ty, _ in spine:
                nx, ny = -ty, tx
                wx = sx + nx * lat_off
                wy = sy + ny * lat_off
                xl = wx + nx * (-half_w)
                yl = wy + ny * (-half_w)
                xr = wx + nx * half_w
                yr = wy + ny * half_w
                zl = lunar_relief_z(xl, yl) + 0.0028
                zr = lunar_relief_z(xr, yr) + 0.0028
                rut_verts.append((xl, yl, zl, 0.22, 0.22, 0.24))
                rut_verts.append((xr, yr, zr, 0.22, 0.22, 0.24))

                # Raised flanking dust shoulders bordering the wheel rut
                for edge_sign in (-1.0, 1.0):
                    ex_in = wx + nx * (edge_sign * half_w)
                    ey_in = wy + ny * (edge_sign * half_w)
                    ex_mid = wx + nx * (edge_sign * (half_w + 0.012))
                    ey_mid = wy + ny * (edge_sign * (half_w + 0.012))
                    ex_out = wx + nx * (edge_sign * (half_w + 0.026))
                    ey_out = wy + ny * (edge_sign * (half_w + 0.026))
                    tread_verts.append((ex_in, ey_in, lunar_relief_z(ex_in, ey_in) + 0.0030, 0.56, 0.56, 0.58))
                    tread_verts.append((ex_mid, ey_mid, lunar_relief_z(ex_mid, ey_mid) + 0.0075, 0.62, 0.62, 0.65))
                    tread_verts.append((ex_out, ey_out, lunar_relief_z(ex_out, ey_out) + 0.0020, 0.54, 0.54, 0.56))

            n_pts = len(spine)
            for i in range(n_pts - 1):
                r0 = rut_base + 2 * i
                r1 = r0 + 1
                r2 = r0 + 2
                r3 = r0 + 3
                rut_faces.append((r0, r2, r3))
                rut_faces.append((r0, r3, r1))

                # Flanking shoulder strips (2 edges x 3 vertices per step = 6 vertices per step)
                for edge_idx in (0, 1):
                    b0 = sh_L_base + 6 * i + 3 * edge_idx
                    b1 = b0 + 6
                    tread_faces.append((b0 + 0, b1 + 0, b1 + 1))
                    tread_faces.append((b0 + 0, b1 + 1, b0 + 1))
                    tread_faces.append((b0 + 1, b1 + 1, b1 + 2))
                    tread_faces.append((b0 + 1, b1 + 2, b0 + 2))

            # 3. 3D Angled V-Chevron Cleat Tread Bars every 5.5 cm along the wheel rut
            for i in range(0, n_pts - 1):
                sx, sy, tx, ty, _ = spine[i]
                nx, ny = -ty, tx
                wx = sx + nx * lat_off
                wy = sy + ny * lat_off
                # V-chevron points forward along (tx, ty) at the center of the tyre and sweeps
                # slightly backward at the left/right edges of the tyre (+/- half_w)
                c_base = len(tread_verts) + 1
                sweep = -0.018
                thick = 0.015
                pts_local = [
                    (-half_w * 0.92, sweep),
                    (0.0, 0.0),
                    (half_w * 0.92, sweep),
                    (-half_w * 0.92, sweep + thick),
                    (0.0, thick),
                    (half_w * 0.92, sweep + thick),
                ]
                for lat_l, lon_l in pts_local:
                    px = wx + nx * lat_l + tx * lon_l
                    py = wy + ny * lat_l + ty * lon_l
                    pz = lunar_relief_z(px, py) + 0.0068
                    tread_verts.append((px, py, pz, 0.60, 0.60, 0.63))
                tread_faces.extend([
                    (c_base + 0, c_base + 3, c_base + 4),
                    (c_base + 0, c_base + 4, c_base + 1),
                    (c_base + 1, c_base + 4, c_base + 5),
                    (c_base + 1, c_base + 5, c_base + 2),
                ])


def _append_concave_crater_shadow(
    sh_verts: list[tuple[float, float, float, float, float, float]],
    sh_faces: list[tuple[int, int, int]],
    pen_verts: list[tuple[float, float, float, float, float, float]],
    pen_faces: list[tuple[int, int, int]],
    *,
    cx: float,
    cy: float,
    radius: float,
) -> None:
    """Append a natural crescent-shaped low-angle solar shadow inside the sunward concave bowl
    of a sunken 3D crater (conformed directly to `lunar_relief_z(wx, wy)`)."""
    sx, sy, _ = _normalize(*SUN_DIR)
    sun_ang = math.atan2(sy, sx)

    # Crescent shadow covers the sunward inner wall and half of the crater floor
    for target_v, target_f, r_scale, z_off, col in (
        (sh_verts, sh_faces, 0.76, 0.0030, (0.010, 0.010, 0.015)),
        (pen_verts, pen_faces, 0.90, 0.0022, (0.045, 0.045, 0.055)),
    ):
        n_rings = 5
        n_lon = 28
        base_idx = len(target_v) + 1
        # Center shifted toward the sunward inner wall (-sx, -sy)
        scx = cx - sx * (radius * 0.24)
        scy = cy - sy * (radius * 0.24)
        target_v.append((scx, scy, lunar_relief_z(scx, scy) + z_off, col[0], col[1], col[2]))
        for ir in range(1, n_rings + 1):
            u = ir / n_rings
            for j in range(n_lon):
                th = 2.0 * math.pi * j / n_lon
                # Elongate crescent along perpendicular to sun and clip before the sunlit opposite rim
                cos_rel = math.cos(th - (sun_ang + math.pi))
                r_local = radius * r_scale * u * (0.78 + 0.22 * cos_rel)
                wx = scx + r_local * math.cos(th)
                wy = scy + r_local * math.sin(th)
                # Ensure point stays inside the crater rim (r <= 0.94 * radius)
                dc = math.hypot(wx - cx, wy - cy)
                if dc > radius * 0.93:
                    wx = cx + (wx - cx) * (radius * 0.93 / dc)
                    wy = cy + (wy - cy) * (radius * 0.93 / dc)
                target_v.append((wx, wy, lunar_relief_z(wx, wy) + z_off, col[0], col[1], col[2]))

        for j in range(n_lon):
            jn = (j + 1) % n_lon
            target_f.append((base_idx, base_idx + 1 + j, base_idx + 1 + jn))
        for ir in range(n_rings - 1):
            row_a = base_idx + 1 + ir * n_lon
            row_b = row_a + n_lon
            for j in range(n_lon):
                jn = (j + 1) % n_lon
                target_f.append((row_a + j, row_b + j, row_b + jn))
                target_f.append((row_a + j, row_b + jn, row_a + jn))


def _append_shadow_patch(
    verts: list[tuple[float, float, float, float, float, float]],
    faces: list[tuple[int, int, int]],
    *,
    cx: float,
    cy: float,
    rx: float,
    ry: float,
    angle: float,
    z_ground: float = 0.008,
) -> None:
    """Append an irregular ground shadow region patch conformed to `lunar_relief_z(wx, wy)`."""
    n_lon = 18
    ca, sa = math.cos(angle), math.sin(angle)
    base_idx = len(verts) + 1
    verts.append((cx, cy, lunar_relief_z(cx, cy) + z_ground, 0.012, 0.012, 0.018))
    for j in range(n_lon):
        th = 2.0 * math.pi * j / n_lon
        asym = 1.0 + 0.14 * math.sin(2.0 * th + 0.4) + 0.09 * math.cos(3.0 * th - 0.6)
        lx = rx * asym * math.cos(th)
        ly = ry * asym * math.sin(th)
        wx = cx + lx * ca - ly * sa
        wy = cy + lx * sa + ly * ca
        verts.append((wx, wy, lunar_relief_z(wx, wy) + z_ground, 0.012, 0.012, 0.018))
    for j in range(n_lon):
        jn = (j + 1) % n_lon
        faces.append((base_idx, base_idx + 1 + j, base_idx + 1 + jn))


def generate_semantic_terrain_layers() -> None:
    """Generate the 5 multi-material semantic meshes:
    - `lunar_bedrock_patches.obj`: Fractured angular lunar basalt rock outcrops & ledges (NO brown ovals!)
    - `lunar_route_crater_wall.obj`: 3D Rover Tyre Compressed Wheel Ruts (`0.52 m` gauge)
    - `lunar_route_crater_rim.obj`: 3D Rover Tyre V-Chevron Cleat Tread Bars & Flanking Dust Shoulders
    - `lunar_deep_shadows.obj` & `lunar_penumbra_shadows.obj`: Concave 3D Crater-Bowl Crescent Shadows & Boulder Shadows"""
    rng = random.Random(2026)
    bed_v: list[tuple[float, float, float, float, float, float]] = []
    bed_f: list[tuple[int, int, int]] = []
    rim_v: list[tuple[float, float, float, float, float, float]] = []
    rim_f: list[tuple[int, int, int]] = []
    wall_v: list[tuple[float, float, float, float, float, float]] = []
    wall_f: list[tuple[int, int, int]] = []
    sh_v: list[tuple[float, float, float, float, float, float]] = []
    sh_f: list[tuple[int, int, int]] = []
    pen_v: list[tuple[float, float, float, float, float, float]] = []
    pen_f: list[tuple[int, int, int]] = []

    # 1. Fractured Basalt Rock Outcrop Clusters (replacing smooth brown ovals!)
    bedrock_formations = [
        (4.10, -4.15, 1.40, 0.95, 0.18, -0.20),
        (-5.20, 4.60, 1.60, 1.05, 0.22, 0.35),
        (8.80, -1.80, 1.80, 1.15, 0.24, 0.15),
        (10.50, 6.20, 2.10, 1.30, 0.28, -0.35),
        (-7.50, -5.20, 1.70, 1.10, 0.22, 0.45),
        (24.0, 12.0, 3.80, 2.20, 0.42, 0.40),
        (44.0, 20.0, 4.40, 2.50, 0.48, 0.45),
        (76.0, 46.0, 5.00, 2.80, 0.56, 0.60),
        (30.0, -11.0, 4.20, 2.40, 0.46, -0.35),
        (54.0, -20.0, 4.80, 2.70, 0.54, -0.30),
        (-28.0, 24.0, 4.20, 2.40, 0.44, -0.68),
        (38.0, -38.0, 4.60, 2.60, 0.50, -0.78),
    ]
    for cx, cy, rx, ry, mh, ang in bedrock_formations:
        _append_basalt_outcrop_cluster(bed_v, bed_f, rng=rng, cx=cx, cy=cy, rx=rx, ry=ry, max_h=mh, angle=ang)

    # 2. 3D Rover Tyre Marks (Compressed Dark Wheel Ruts + V-Chevron Cleat Tread Bars & Flanking Dust Shoulders)
    _append_rover_tyre_tracks_3d(wall_v, wall_f, rim_v, rim_f)

    # 3. Concave 3D Crater-Bowl Crescent Shadows inside the sunken craters
    for cx, cy, rad, _, _, _ in NEARFIELD_CRATERS[:16]:
        _append_concave_crater_shadow(sh_v, sh_f, pen_v, pen_f, cx=cx, cy=cy, radius=rad)

    # 4. Long Razor-Sharp Low-Angle Shadows behind Landmark Boulders & Settlement Stations
    boulder_shadows = [
        (1.55, 0.32, 0.38, 0.36, 1.85),
        (4.80, 3.90, 0.68, 0.58, 2.60),
        (5.40, -5.80, 0.72, 0.62, 2.50),
        (18.50, -5.20, 0.85, 0.75, 3.40),
        (26.00, 8.50, 0.90, 0.80, 3.60),
        (38.00, -12.00, 0.95, 0.85, 3.80),
        (-10.50, 7.50, 4.50, 4.50, 9.50),
        (58.00, 26.00, 3.80, 3.80, 8.20),
        (92.00, 58.00, 2.50, 2.50, 11.00),
        (78.00, -28.00, 4.00, 4.00, 8.50),
        (-56.00, 48.00, 3.60, 3.60, 7.80),
        (-64.00, -38.00, 3.20, 3.20, 7.20),
    ]
    for bx, by, brx, bry, slen in boulder_shadows:
        _append_shadow_apron(sh_v, sh_f, cx=bx, cy=by, rx=brx, ry=bry, shadow_len=slen, z_ground=0.005)
        _append_shadow_apron(pen_v, pen_f, cx=bx, cy=by, rx=brx * 1.15, ry=bry * 1.15, shadow_len=slen * 1.15, z_ground=0.003)

    # 5. Permanent-Shadow Polar Crater Floor (Location 7 at 64, -64)
    shadow_regions = [
        (54.00, -54.00, 8.50, 5.20, -0.78),
        (64.00, -64.00, 14.50, 14.50, 0.0),
    ]
    for scx, scy, srx, sry, sang in shadow_regions:
        _append_shadow_patch(pen_v, pen_f, cx=scx, cy=scy, rx=srx * 1.18, ry=sry * 1.18, angle=sang, z_ground=0.010)
        _append_shadow_patch(sh_v, sh_f, cx=scx, cy=scy, rx=srx, ry=sry, angle=sang, z_ground=0.014)

    _write_flat_shaded_obj(MESH_DIR / "lunar_bedrock_patches.obj", bed_v, bed_f, "Fractured Lunar Basalt Rock Outcrops", contrast=1.22)
    _write_smooth_shaded_obj(MESH_DIR / "lunar_route_crater_rim.obj", rim_v, rim_f, "3D Rover Tyre Chevron Cleat Treads & Flanking Dust Ridges", contrast=1.18)
    _write_smooth_shaded_obj(MESH_DIR / "lunar_route_crater_wall.obj", wall_v, wall_f, "3D Rover Tyre Compressed Regolith Ruts", contrast=1.20)
    _write_smooth_shaded_obj(MESH_DIR / "lunar_deep_shadows.obj", sh_v, sh_f, "Concave Crater-Bowl & Boulder Umbra Shadows", bake_sun=False)
    _write_smooth_shaded_obj(MESH_DIR / "lunar_penumbra_shadows.obj", pen_v, pen_f, "Lunar Penumbra Partial Shadows", bake_sun=False)


def main() -> None:
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    sculpt_base_visual_terrain_obj()
    print("Sculpted 3D near-field depression & smooth normals into lunar_terrain.obj")
    generate_lunar_regolith_textures()
    print(f"Generated lunar_regolith_albedo.png ({(MESH_DIR / 'lunar_regolith_albedo.png').stat().st_size // 1024} KB) & lunar_regolith.mtl")
    specs = [
        ("lunar_boulder_primary.obj", 101, 0.36, 0.34, 1.06, (0.32, 0.33, 0.35), True),
        ("lunar_boulder_ejecta_left.obj", 202, 0.68, 0.58, 0.62, (0.36, 0.36, 0.37), False),
        ("lunar_boulder_outcrop_right.obj", 303, 0.72, 0.62, 0.58, (0.30, 0.30, 0.32), False),
        ("lunar_boulder_crater_fragment.obj", 404, 0.64, 0.54, 0.54, (0.34, 0.34, 0.35), False),
        ("lunar_boulder_anorthosite.obj", 505, 0.70, 0.60, 0.66, (0.38, 0.38, 0.39), False),
        ("lunar_boulder_monolith.obj", 606, 0.78, 0.66, 0.74, (0.28, 0.29, 0.31), False),
    ]
    for name, seed, rx, ry, height, rgb, broad in specs:
        out = MESH_DIR / name
        generate_angular_boulder_obj(
            out, seed=seed, rx=rx, ry=ry, height=height, base_rgb=rgb, broad_crest=broad
        )
        print(f"Generated {out.name} ({out.stat().st_size // 1024} KB)")

    generate_foreground_regolith_obj(MESH_DIR / "lunar_small_rocks.obj")
    print(f"Generated lunar_small_rocks.obj ({(MESH_DIR / 'lunar_small_rocks.obj').stat().st_size // 1024} KB)")
    generate_black_skydome_obj(MESH_DIR / "lunar_black_skydome.obj")
    print(f"Generated lunar_black_skydome.obj ({(MESH_DIR / 'lunar_black_skydome.obj').stat().st_size // 1024} KB)")
    generate_starfield_obj(MESH_DIR / "lunar_starfield.obj")
    print(f"Generated lunar_starfield.obj ({(MESH_DIR / 'lunar_starfield.obj').stat().st_size // 1024} KB)")
    generate_massif_ridge_obj(MESH_DIR / "lunar_massif_ridge.obj")
    print(f"Generated lunar_massif_ridge.obj ({(MESH_DIR / 'lunar_massif_ridge.obj').stat().st_size // 1024} KB)")
    generate_rough_regolith_sheet_obj(MESH_DIR / "lunar_impact_crater.obj")
    print(f"Generated lunar_impact_crater.obj ({(MESH_DIR / 'lunar_impact_crater.obj').stat().st_size // 1024} KB)")
    generate_semantic_terrain_layers()
    for sname in (
        "lunar_bedrock_patches.obj",
        "lunar_route_crater_rim.obj",
        "lunar_route_crater_wall.obj",
        "lunar_deep_shadows.obj",
        "lunar_penumbra_shadows.obj",
    ):
        print(f"Generated {sname} ({(MESH_DIR / sname).stat().st_size // 1024} KB)")


if __name__ == "__main__":
    main()
