#!/usr/bin/env python3
"""Generate photorealistic, high-contrast lunar terrain & jagged angular rock meshes.

Matches the reference lunar surface imagery:
1. High-contrast rough lunar regolith sheet (`lunar_foreground_regolith.obj`) with
   baked low-angle solar shadows, dual wheel tracks, and 95 sharp, angular,
   flat-faceted basalt rock shards scattered across the foreground and midground.
2. Sharp, polygonal, flat-faceted lunar boulders (`lunar_boulder_*.obj`) with
   crisp sunlit vs. deep-charcoal shadow faces and basal rock fragments.
3. Rolling lunar hills (`lunar_massif_ridge.obj`) across the horizon.
4. Pitch-black space skydome (`lunar_black_skydome.obj`) & starfield (`lunar_starfield.obj`).
"""

from __future__ import annotations

import math
from pathlib import Path
import random


MESH_DIR = Path(__file__).resolve().parent.parent / "src" / "lunabot_gazebo" / "worlds" / "meshes"
# Low-angle lunar sun direction (from left-front-above, matching reference image)
SUN_DIR = (0.58, 0.68, 0.45)

_BASE_GRID_Z: list[float] | None = None


def _inner_sculpt_dz(x: float, y: float) -> float:
    """Idempotent, anti-aliased 3D lunar regolith relief inside r < 14.0m (long wavelengths only,
    zero high-frequency Nyquist grid pixelation), tapering smoothly to 0.0 at r = 14.0m."""
    r = math.hypot(x, y)
    if r >= 14.0:
        return 0.0
    t_out = max(0.0, min(1.0, (14.0 - r) / 4.5))
    env_out = t_out * t_out * (3.0 - 2.0 * t_out)
    t_in = max(0.0, min(1.0, (r - 2.2) / 3.2))
    env_in = t_in * t_in * (3.0 - 2.0 * t_in)

    if -1.0 <= x <= 4.2 and -2.2 <= y <= 0.15:
        return 0.0

    # Long-wavelength smooth lunar swells & Gaussian hills (wavelength >= 14m -> silky smooth on 1.25m grid)
    swells = (
        0.28 * math.sin(0.32 * x + 0.26 * y)
        + 0.22 * math.cos(0.38 * x - 0.30 * y)
    )
    mounds = (
        0.48 * math.exp(-(((x - 6.8) / 4.4) ** 2 + ((y - 5.8) / 3.8) ** 2))
        + 0.44 * math.exp(-(((x - 7.8) / 4.2) ** 2 + ((y + 6.8) / 3.8) ** 2))
        + 0.38 * math.exp(-(((x + 6.4) / 4.4) ** 2 + ((y - 4.8) / 4.0) ** 2))
        - 0.32 * math.exp(-(((x - 8.5) / 3.8) ** 2 + ((y - 1.5) / 3.4) ** 2))
    )
    dz = env_out * env_in * (swells + mounds)
    if r < 3.55:
        dz = max(-0.06, min(0.09, dz))
    return dz


def sculpt_base_visual_terrain_obj() -> None:
    """Apply a 2-pass Gaussian binomial [1,2,1]/4 anti-aliasing filter across `lunar_terrain.obj`
    to eliminate 1.25m grid pixelation spikes, sculpt smooth near-field relief (r < 14m),
    and recompute smooth Gouraud vertex normals `vn`."""
    global _BASE_GRID_Z
    tpath = MESH_DIR / "lunar_terrain.obj"
    if not tpath.is_file():
        return
    header_lines: list[str] = []
    v_coords: list[tuple[float, float, float]] = []
    c_lines: list[str] = []
    f_lines: list[str] = []
    already_smoothed = False
    with tpath.open("r", encoding="utf-8") as fh:
        for line in fh:
            s = line.rstrip("\n")
            if s == "# SMOOTH_ANTIALIASED_V2":
                already_smoothed = True
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
        # 2-pass separable [1, 2, 1]/4 Gaussian binomial anti-aliasing filter to remove 1-cell pyramid pixelation
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

    for idx, (x, y, _) in enumerate(v_coords):
        if math.hypot(x, y) < 14.0:
            zs[idx] = -2.7880 + _inner_sculpt_dz(x, y)
        v_coords[idx] = (x, y, zs[idx])

    # Recompute smooth central-difference Gouraud vertex normals across the 320x320 grid (spacing = 1.25m)
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
    _BASE_GRID_Z = list(zs)


def _base_terrain_delta_z(x: float, y: float) -> float:
    """Bilinearly sample `lunar_terrain.obj` (320x320 over [-199.375, 199.375]) relative to z=-2.790 (exact signed)."""
    global _BASE_GRID_Z
    if _BASE_GRID_Z is None:
        tpath = MESH_DIR / "lunar_terrain.obj"
        zs: list[float] = []
        if tpath.is_file():
            with tpath.open("r", encoding="utf-8") as fh:
                for line in fh:
                    if line.startswith("v "):
                        parts = line.split()
                        zs.append(float(parts[3]))
        _BASE_GRID_Z = zs if len(zs) == 320 * 320 else [0.0] * (320 * 320)

    u = (x - (-199.375)) / 1.25
    v = (y - (-199.375)) / 1.25
    ix = max(0, min(318, int(math.floor(u))))
    iy = max(0, min(318, int(math.floor(v))))
    fx = max(0.0, min(1.0, u - ix))
    fy = max(0.0, min(1.0, v - iy))
    g = _BASE_GRID_Z
    z00 = g[iy * 320 + ix]
    z10 = g[iy * 320 + ix + 1]
    z01 = g[(iy + 1) * 320 + ix]
    z11 = g[(iy + 1) * 320 + ix + 1]
    z_interp = (1.0 - fy) * ((1.0 - fx) * z00 + fx * z10) + fy * ((1.0 - fx) * z01 + fx * z11)
    return z_interp - (-2.790)


def lunar_relief_z(x: float, y: float) -> float:
    """Return exact 3D lunar ground height relative to z=-2.790m at (x, y) across the 400m x 400m world."""
    return _base_terrain_delta_z(x, y)


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


def _write_flat_shaded_obj(
    path: Path,
    verts: list[tuple[float, float, float, float, float, float]],
    faces: list[tuple[int, int, int]],
    header: str,
    bake_sun: bool = True,
    contrast: float = 1.0,
) -> None:
    """Write OBJ with per-face normals and crisp flat-facet lunar solar shading."""
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
) -> None:
    """Write OBJ with smooth area-weighted per-vertex Gouraud normals (`f i1//i1 i2//i2 i3//i3`)
    so continuous lunar terrain, crater bowls, regolith berms, and mountain ridges render
    smoothly in Gazebo Ogre without faceted/pixelated triangle edges."""
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
    for idx, (vx, vy, vz, cr, cg, cb) in enumerate(verts):
        if bake_sun and v_count[idx] > 0:
            light = v_light[idx] / v_count[idx]
            cr = max(0.03, min(0.95, cr * light))
            cg = max(0.03, min(0.95, cg * light))
            cb = max(0.03, min(0.95, cb * light))
        lines.append(f"v {vx:.4f} {vy:.4f} {vz:.4f} {cr:.4f} {cg:.4f} {cb:.4f}")
    for ax, ay, az in vn_acc:
        nx, ny, nz = _normalize(ax, ay, az)
        lines.append(f"vn {nx:.5f} {ny:.5f} {nz:.5f}")
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
    # Slight random peak offset for asymmetric angular fracture look
    pk_x = cx + rng.uniform(-0.12, 0.12) * rx
    pk_y = cy + rng.uniform(-0.12, 0.12) * ry
    verts.append((pk_x, pk_y, z_off + height, base_rgb[0] * 1.1, base_rgb[1] * 1.1, base_rgb[2] * 1.1))

    # Random angular facet radii around the rock perimeter
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
                    # Sharp angular pyramidal/blocky rock profile
                    z_prof = max(0.04, (1.0 - u) ** 0.92)
                z = height * z_prof * rng.uniform(0.88, 1.12)
                r_mult = u * rad_asym
                shade = rng.uniform(0.82, 1.15)
                cr = base_rgb[0] * shade
                cg = base_rgb[1] * shade
                cb = base_rgb[2] * shade
            else:
                # Basal contact ring flush on ground with dark cast shadow on anti-sun side
                z = 0.0
                r_mult = 1.12 * rad_asym
                ux, uy = math.cos(theta), math.sin(theta)
                sun_dot = ux * sx + uy * sy
                # Anti-sun side has a crisp dark lunar shadow apron on the ground
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

    # Main jagged angular boulder
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

    # Add 5 smaller angular talus fragments around its base so it merges naturally with the ground
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
    z_ground: float = 0.006,
) -> None:
    """Append a long South-Pole low-angle solar shadow tail (umbra + penumbra) behind a rock/mound,
    conformed to the 3D lunar surface `lunar_relief_z(x, y)`."""
    sx, sy, _ = _normalize(*SUN_DIR)
    dx, dy = -sx, -sy
    px, py = -dy, dx
    base_idx = len(verts) + 1
    w0 = max(rx, ry) * 0.92
    w1 = w0 * 0.58
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


def _append_regolith_berm(
    verts: list[tuple[float, float, float, float, float, float]],
    faces: list[tuple[int, int, int]],
    *,
    cx: float,
    cy: float,
    rx: float,
    ry: float,
    max_h: float,
    angle: float,
) -> None:
    """Append a discrete, high-resolution, smooth-shaded 3D regolith drift ridge / corridor berm
    whose outer edge dips 2.5cm beneath `lunar_terrain.obj` so it merges organically with zero seams."""
    nx, ny = 28, 22
    ca, sa = math.cos(angle), math.sin(angle)
    base_idx = len(verts) + 1
    for iy in range(ny):
        v = -1.0 + 2.0 * iy / (ny - 1)
        for ix in range(nx):
            u = -1.0 + 2.0 * ix / (nx - 1)
            r = min(1.0, math.hypot(u, v))
            # Smooth C2 cosine bell profile (zero sharp facet corners)
            edge_w = 0.5 * (1.0 + math.cos(math.pi * r))
            ripples = (
                0.78
                + 0.14 * math.sin(2.4 * u + 1.4 * v)
                + 0.08 * math.cos(3.2 * v - 1.6 * u)
            )
            z_local = max_h * (edge_w ** 1.15) * ripples - 0.025 * (1.0 - edge_w)
            if ix in (0, nx - 1) or iy in (0, ny - 1) or edge_w <= 0.01:
                z_local = -0.028
            lx = u * rx * (1.0 + 0.08 * math.sin(2.0 * v))
            ly = v * ry * (1.0 + 0.08 * math.cos(2.0 * u))
            wx = cx + lx * ca - ly * sa
            wy = cy + lx * sa + ly * ca
            z = lunar_relief_z(wx, wy) + z_local
            if math.hypot(wx, wy) < 3.55:
                z = min(z, 0.140)
            if -0.8 <= wx <= 2.6 and -1.25 <= wy <= 0.10:
                z = min(z, 0.025)
            verts.append((wx, wy, z, 0.56, 0.56, 0.58))

    for iy in range(ny - 1):
        for ix in range(nx - 1):
            i00 = base_idx + iy * nx + ix
            i10 = i00 + 1
            i01 = i00 + nx
            i11 = i01 + 1
            faces.append((i00, i10, i11))
            faces.append((i00, i11, i01))


def generate_rough_regolith_sheet_obj(path: Path, *, seed: int = 909) -> None:
    """Generate discrete 3D-sculpted lunar regolith ridges, hummocks, and corridor berms across the
    Settlement Corridors (emerging seamlessly from `lunar_terrain.obj` with smooth Gouraud normals)."""
    _ = seed
    verts: list[tuple[float, float, float, float, float, float]] = []
    faces: list[tuple[int, int, int]] = []

    berm_specs = [
        # Near-field departure & route-decision hummocks (flanking Route A & Route B without blocking Safe Route B)
        (5.60, 4.20, 3.80, 2.20, 0.42, 0.25),
        (6.40, -6.20, 4.20, 2.40, 0.46, -0.30),
        (-5.20, -5.80, 4.00, 2.30, 0.38, 0.40),
        (-4.80, 6.40, 4.20, 2.50, 0.44, -0.25),
        # Corridor 1: Habitat -> Solar Power Station (58, 26)
        (16.0, 7.5, 6.5, 3.4, 0.65, 0.42),
        (32.0, 14.5, 7.5, 3.8, 0.78, 0.38),
        (46.0, 21.0, 7.0, 3.6, 0.72, 0.45),
        # Corridor 2: Solar Station -> Comms Relay Tower (92, 58)
        (72.0, 40.0, 8.5, 4.2, 0.92, 0.62),
        # Corridor 3: Habitat -> Science Station (78, -28) Rocky Ridge & Crater Field
        (18.0, -8.5, 6.8, 3.5, 0.68, -0.35),
        (36.0, -14.0, 8.2, 4.0, 0.88, -0.32),
        (58.0, -22.0, 9.0, 4.4, 1.05, -0.28),
        # Corridor 4: Habitat -> Resource / Mining Site (-56, 48)
        (-20.0, 17.0, 7.0, 3.6, 0.72, -0.68),
        (-38.0, 32.0, 8.0, 4.2, 0.86, -0.72),
        # Corridor 5: Habitat -> Landing Zone (-64, -38)
        (-24.0, -14.0, 6.8, 3.4, 0.58, 0.52),
        (-46.0, -27.0, 7.5, 3.8, 0.64, 0.48),
        # Corridor 6: Habitat -> Permanently Shadowed Crater (48, -48 / 64, -64)
        (20.0, -22.0, 7.2, 3.8, 0.74, -0.78),
        (36.0, -36.0, 8.4, 4.2, 0.95, -0.78),
    ]
    for cx, cy, rx, ry, mh, ang in berm_specs:
        _append_regolith_berm(verts, faces, cx=cx, cy=cy, rx=rx, ry=ry, max_h=mh, angle=ang)

    _write_smooth_shaded_obj(path, verts, faces, "3D-Sculpted Lunar Settlement Corridor Regolith Berms & Ridges", contrast=1.18)


def generate_foreground_regolith_obj(path: Path, *, seed: int = 909) -> None:
    """Generate dark angular basalt rock fields along the 7 Settlement Corridors & Hazard Zones
    conformed to the 3D lunar surface `lunar_relief_z(x, y)`."""
    rng = random.Random(seed)
    verts: list[tuple[float, float, float, float, float, float]] = []
    faces: list[tuple[int, int, int]] = []

    # 1. Focused ejecta fragments around the Route-A Hazard Zone (boulder at 1.55, 0.32 & crater at 3.10, 1.35)
    route_a_ejecta = [
        (1.18, 0.58, 0.16, 0.13, 0.10),
        (1.88, 0.68, 0.18, 0.15, 0.11),
        (2.15, 0.42, 0.17, 0.14, 0.10),
        (1.42, 1.05, 0.18, 0.14, 0.11),
        (4.55, 2.15, 0.32, 0.25, 0.20),
        (3.85, 2.95, 0.34, 0.27, 0.22),
        (2.25, 2.75, 0.26, 0.21, 0.16),
    ]
    for rx_pos, ry_pos, sx, sy, h in route_a_ejecta:
        rock_tone = rng.uniform(0.18, 0.26)
        _append_angular_rock(
            verts, faces, rng=rng, cx=rx_pos, cy=ry_pos, rx=sx, ry=sy, height=h,
            base_rgb=(rock_tone, rock_tone, rock_tone * 1.03), n_rings=3, n_lon=7, use_terrain_z=True,
        )

    # 2. Corridor Rock Fields (Science Rocky Ridge, Mining Site Boulder Belt, Shadow Crater Rim, Solar Slope)
    corridor_rock_zones = [
        # (center_x, center_y, spread_x, spread_y, count, min_r, max_r)
        (12.0, 2.0, 8.0, 6.0, 22, 0.18, 0.44),     # Mid-range departure basin rocks
        (38.0, -14.0, 18.0, 10.0, 28, 0.28, 0.72), # Science Station Rocky Ridge & Crater Field
        (62.0, -22.0, 12.0, 8.0, 18, 0.30, 0.78),  # Science Station approach boulder belt
        (-36.0, 30.0, 16.0, 12.0, 24, 0.26, 0.68), # Mining Site rough regolith & boulder corridor
        (42.0, -42.0, 14.0, 12.0, 20, 0.28, 0.74), # Permanently Shadowed Crater rim ejecta field
        (36.0, 16.0, 14.0, 8.0, 14, 0.22, 0.52),   # Solar Power Station ridge slope rocks
        (-38.0, -22.0, 14.0, 10.0, 10, 0.20, 0.46),# Sparse Landing Zone plain rocks
    ]
    for zx, zy, spx, spy, cnt, rmin, rmax in corridor_rock_zones:
        for _ in range(cnt):
            rx_pos = zx + rng.uniform(-spx, spx)
            ry_pos = zy + rng.uniform(-spy, spy)
            if -3.5 <= rx_pos <= 4.5 and -2.8 <= ry_pos <= 0.6:
                continue
            if math.hypot(rx_pos, ry_pos) < 3.8:
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


def _append_bedrock_formation(
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
    """Append a high-resolution, smooth-shaded exposed BEDROCK terrain shelf (not a decorative rock)."""
    _ = rng
    nx, ny = 24, 18
    ca, sa = math.cos(angle), math.sin(angle)
    base_idx = len(verts) + 1
    for iy in range(ny):
        v = -1.0 + 2.0 * iy / (ny - 1)
        for ix in range(nx):
            u = -1.0 + 2.0 * ix / (nx - 1)
            r = min(1.0, math.hypot(u, v))
            edge_w = 0.5 * (1.0 + math.cos(math.pi * r))
            # Smooth geological strata terraces without jagged quantizer steps
            strata = 0.62 + 0.22 * math.sin(2.8 * u + 1.2 * v) + 0.14 * math.cos(4.2 * v - 1.8 * u)
            z_local = max(0.006, max_h * (edge_w ** 0.85) * strata)
            if ix in (0, nx - 1) or iy in (0, ny - 1) or edge_w <= 0.02:
                z_local = 0.002
            lx = u * rx * (1.0 + 0.09 * math.sin(2.5 * v))
            ly = v * ry * (1.0 + 0.09 * math.cos(2.2 * u))
            wx = cx + lx * ca - ly * sa
            wy = cy + lx * sa + ly * ca
            z = lunar_relief_z(wx, wy) + z_local
            if math.hypot(wx, wy) < 3.55:
                z = min(z, 0.155)
            verts.append((wx, wy, z, 0.28, 0.25, 0.22))

    for iy in range(ny - 1):
        for ix in range(nx - 1):
            i00 = base_idx + iy * nx + ix
            i10 = i00 + 1
            i01 = i00 + nx
            i11 = i01 + 1
            faces.append((i00, i10, i11))
            faces.append((i00, i11, i01))


def _append_crater_components(
    rim_verts: list[tuple[float, float, float, float, float, float]],
    rim_faces: list[tuple[int, int, int]],
    wall_verts: list[tuple[float, float, float, float, float, float]],
    wall_faces: list[tuple[int, int, int]],
    shadow_verts: list[tuple[float, float, float, float, float, float]],
    shadow_faces: list[tuple[int, int, int]],
    penumbra_verts: list[tuple[float, float, float, float, float, float]],
    penumbra_faces: list[tuple[int, int, int]],
    *,
    cx: float,
    cy: float,
    radius: float,
    rim_h: float,
    wall_z_bottom: float,
    n_lon: int = 56,
    central_peak: bool = False,
) -> None:
    """Append high-resolution, smooth-shaded multi-material 3D crater layers conformed to `lunar_relief_z(wx, wy)`."""
    sx, sy, _ = _normalize(*SUN_DIR)

    # 1. Raised ejecta rim collar (r from 0.84*R to 1.50*R, 9 smooth radial rings)
    r_rings = [0.84, 0.90, 0.96, 1.02, 1.09, 1.18, 1.28, 1.39, 1.50]
    z_prof = [0.36, 0.68, 0.92, 1.00, 0.84, 0.56, 0.30, 0.10, 0.01]
    rim_base = len(rim_verts) + 1
    for ir, r_mult in enumerate(r_rings):
        for j in range(n_lon):
            theta = 2.0 * math.pi * j / n_lon
            asym = 1.0 + 0.05 * math.sin(3.0 * theta + 0.5) + 0.03 * math.cos(5.0 * theta - 0.4)
            r = radius * r_mult * asym
            rug = 1.0 + 0.08 * math.sin(4.0 * theta) + 0.05 * math.cos(7.0 * theta)
            z_local = max(0.008, rim_h * z_prof[ir] * rug)
            if ir == len(r_rings) - 1:
                z_local = 0.003
            wx = cx + r * math.cos(theta)
            wy = cy + r * math.sin(theta)
            z = lunar_relief_z(wx, wy) + z_local
            if math.hypot(wx, wy) < 3.55:
                z = min(z, 0.16)
            if -0.8 <= wx <= 2.5 and -1.25 <= wy <= 0.05:
                z = min(z, 0.032)
            rim_verts.append((wx, wy, z, 0.80, 0.80, 0.82))

    for ir in range(len(r_rings) - 1):
        row_a = rim_base + ir * n_lon
        row_b = row_a + n_lon
        for j in range(n_lon):
            jn = (j + 1) % n_lon
            rim_faces.append((row_a + j, row_b + j, row_b + jn))
            rim_faces.append((row_a + j, row_b + jn, row_a + jn))

    # 2. Steep inner crater wall (r from 0.40*R to 0.94*R, 6 smooth radial rings)
    w_rings = [0.40, 0.52, 0.64, 0.75, 0.85, 0.94]
    w_z = [
        wall_z_bottom,
        wall_z_bottom + 0.14 * rim_h,
        wall_z_bottom + 0.34 * rim_h,
        wall_z_bottom + 0.56 * rim_h,
        wall_z_bottom + 0.74 * rim_h,
        0.86 * rim_h,
    ]
    wall_base = len(wall_verts) + 1
    for ir, r_mult in enumerate(w_rings):
        for j in range(n_lon):
            theta = 2.0 * math.pi * j / n_lon
            asym = 1.0 + 0.05 * math.sin(3.0 * theta + 0.5)
            r = radius * r_mult * asym
            z_local = max(0.008, w_z[ir])
            wx = cx + r * math.cos(theta)
            wy = cy + r * math.sin(theta)
            z = lunar_relief_z(wx, wy) + z_local
            if math.hypot(wx, wy) < 3.55:
                z = min(z, 0.15)
            if -0.8 <= wx <= 2.5 and -1.25 <= wy <= 0.05:
                z = min(z, 0.030)
            wall_verts.append((wx, wy, z, 0.18, 0.18, 0.20))

    for ir in range(len(w_rings) - 1):
        row_a = wall_base + ir * n_lon
        row_b = row_a + n_lon
        for j in range(n_lon):
            jn = (j + 1) % n_lon
            wall_faces.append((row_a + j, row_b + j, row_b + jn))
            wall_faces.append((row_a + j, row_b + jn, row_a + jn))

    # Optional smooth central rebound peak inside large craters
    if central_peak:
        pk_base = len(wall_verts) + 1
        cz0 = lunar_relief_z(cx, cy)
        wall_verts.append((cx, cy, cz0 + rim_h * 0.58, 0.22, 0.22, 0.24))
        for j in range(24):
            th = 2.0 * math.pi * j / 24
            px = cx + radius * 0.24 * math.cos(th)
            py = cy + radius * 0.24 * math.sin(th)
            wall_verts.append((px, py, lunar_relief_z(px, py) + 0.015, 0.18, 0.18, 0.20))
        for j in range(24):
            jn = (j + 1) % 24
            wall_faces.append((pk_base, pk_base + 1 + j, pk_base + 1 + jn))

    # 3. Pitch-black permanently shadowed crater bowl floor (offset slightly toward sunward wall)
    sh_base = len(shadow_verts) + 1
    sh_cx = cx + sx * (radius * 0.08)
    sh_cy = cy + sy * (radius * 0.08)
    shadow_verts.append((sh_cx, sh_cy, lunar_relief_z(sh_cx, sh_cy) + 0.014, 0.01, 0.01, 0.015))
    for j in range(n_lon):
        theta = 2.0 * math.pi * j / n_lon
        r = radius * 0.68 * (1.0 + 0.05 * math.sin(3.0 * theta))
        wx = sh_cx + r * math.cos(theta)
        wy = sh_cy + r * math.sin(theta)
        shadow_verts.append((wx, wy, lunar_relief_z(wx, wy) + 0.015, 0.01, 0.01, 0.015))
    for j in range(n_lon):
        jn = (j + 1) % n_lon
        shadow_faces.append((sh_base, sh_base + 1 + j, sh_base + 1 + jn))

    # 4. Anti-sunward exterior crater rim shadow apron (penumbra + umbra)
    _append_shadow_apron(
        penumbra_verts,
        penumbra_faces,
        cx=cx - sx * (radius * 0.95),
        cy=cy - sy * (radius * 0.95),
        rx=radius * 0.85,
        ry=radius * 0.85,
        shadow_len=radius * 0.95,
        z_ground=0.010,
    )


def _append_shadow_patch(
    verts: list[tuple[float, float, float, float, float, float]],
    faces: list[tuple[int, int, int]],
    *,
    cx: float,
    cy: float,
    rx: float,
    ry: float,
    angle: float,
    z_ground: float = 0.012,
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
    """Generate full-scale multi-material .obj meshes across the 7 Settlement Corridors & Hazard Zones."""
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

    # 1. Exposed BEDROCK Terraces distributed along the Settlement Corridors & Ridges
    bedrock_formations = [
        (3.80, -3.85, 2.60, 1.50, 0.18, -0.20),    # Right flanking bedrock shelf bordering Safe Route B
        (-4.80, 4.20, 2.80, 1.60, 0.22, 0.35),     # Habitat Hub departure bedrock terrace
        (8.80, -1.80, 3.20, 1.80, 0.26, 0.15),     # Midground eastern bedrock plateau
        (10.50, 6.20, 3.80, 2.10, 0.35, -0.35),    # Solar corridor lower bedrock shelf
        (-7.50, -4.80, 3.00, 1.70, 0.24, 0.45),    # Western departure bedrock terrace
        (24.0, 12.0, 5.50, 2.80, 0.48, 0.40),      # Corridor 1: Solar Ridge stepped bedrock shelf
        (44.0, 20.0, 6.20, 3.20, 0.56, 0.45),      # Corridor 1: Solar Power Station upper ridge bedrock
        (76.0, 46.0, 7.00, 3.60, 0.68, 0.60),      # Corridor 2: Comms Relay Tower promontory bedrock
        (30.0, -11.0, 6.00, 3.10, 0.54, -0.35),    # Corridor 3: Science Station Rocky Ridge outcrop 1
        (54.0, -20.0, 7.20, 3.60, 0.66, -0.30),    # Corridor 3: Science Station Rocky Ridge outcrop 2
        (-28.0, 24.0, 6.00, 3.20, 0.52, -0.68),    # Corridor 4: Mining Site NW plateau bedrock
        (38.0, -38.0, 6.50, 3.40, 0.60, -0.78),    # Corridor 6: Shadow Crater rim bedrock shelf
    ]
    for cx, cy, rx, ry, mh, ang in bedrock_formations:
        _append_bedrock_formation(bed_v, bed_f, rng=rng, cx=cx, cy=cy, rx=rx, ry=ry, max_h=mh, angle=ang)

    # 2. Planetary-Scale Craters across the Settlement Corridors:
    #    - 3.2m diameter Dangerous Route-A Impact Crater at (3.10, 1.35) blocking direct Route A
    #    - Crater Field along the Science Station corridor ((14.5, 11.5), (24.0, -7.0), (46.0, -16.0))
    #    - Corridor Craters along Solar Ridge, Mining Site, and Landing Zone
    #    - Giant 32m diameter Permanently Shadowed Crater Rim & Steep Wall at (64.0, -64.0)
    crater_specs = [
        (3.10, 1.35, 1.60, 0.26, 0.016, False),    # 3.2m diameter Dangerous Route-A Impact Crater!
        (6.80, -4.80, 1.45, 0.25, 0.018, False),   # 2.9m diameter Midground South Crater
        (7.20, 4.20, 1.50, 0.26, 0.018, False),    # 3.0m diameter Midground North Crater
        (14.50, 11.50, 5.20, 0.82, 0.024, True),   # 10.4m diameter Solar Corridor Flank Crater
        (24.00, -7.00, 4.40, 0.68, 0.022, True),   # 8.8m diameter Science Corridor Crater Field 1
        (46.00, -16.00, 5.80, 0.92, 0.026, True),  # 11.6m diameter Science Corridor Crater Field 2
        (-32.00, 26.00, 4.80, 0.74, 0.024, True),  # 9.6m diameter Mining Corridor Crater
        (-34.00, -20.00, 4.20, 0.62, 0.022, False),# 8.4m diameter Landing Corridor Mare Crater
        (64.00, -64.00, 16.00, 1.85, 0.040, True), # 32.0m diameter Permanently Shadowed Polar Crater!
    ]
    for cx, cy, rad, rh, wz, cpeak in crater_specs:
        _append_crater_components(
            rim_v, rim_f, wall_v, wall_f, sh_v, sh_f, pen_v, pen_f,
            cx=cx, cy=cy, radius=rad, rim_h=rh, wall_z_bottom=wz, central_peak=cpeak,
        )

    # 3. Long Razor-Sharp South-Pole Shadows behind Landmark Boulders & Settlement Stations
    boulder_shadows = [
        (1.55, 0.32, 0.38, 0.36, 1.85),      # Primary Route-A hazard boulder
        (4.80, 3.90, 0.68, 0.58, 2.60),      # Left crater-rim ejecta boulder
        (5.40, -5.80, 0.72, 0.62, 2.50),     # Right bedrock-edge boulder
        (18.50, -5.20, 0.85, 0.75, 3.40),    # Science corridor landmark boulder 1
        (26.00, 8.50, 0.90, 0.80, 3.60),     # Solar corridor landmark boulder
        (38.00, -12.00, 0.95, 0.85, 3.80),   # Science Rocky Ridge monolith boulder
        (-10.50, 7.50, 4.50, 4.50, 9.50),    # Location 1: Lunar Habitat Hub long shadow
        (58.00, 26.00, 3.80, 3.80, 8.20),    # Location 2: Solar Power Station shadow
        (92.00, 58.00, 2.50, 2.50, 11.00),   # Location 3: Comms Relay Tower long shadow
        (78.00, -28.00, 4.00, 4.00, 8.50),   # Location 4: Science Station shadow
        (-56.00, 48.00, 3.60, 3.60, 7.80),   # Location 5: Mining Site shadow
        (-64.00, -38.00, 3.20, 3.20, 7.20),  # Location 6: Landing Zone Lander shadow
    ]
    for bx, by, brx, bry, slen in boulder_shadows:
        _append_shadow_apron(sh_v, sh_f, cx=bx, cy=by, rx=brx, ry=bry, shadow_len=slen, z_ground=0.018)
        _append_shadow_apron(pen_v, pen_f, cx=bx, cy=by, rx=brx * 1.18, ry=bry * 1.18, shadow_len=slen * 1.18, z_ground=0.014)

    # 4. Dedicated Shadow Terrain Regions (SUNLIT REGOLITH -> SHADOW across corridors & Permanent-Shadow Crater)
    shadow_regions = [
        (3.15, -1.95, 1.25, 0.68, -0.25),    # Shadow pocket flanking the Safe Route B target sector
        (2.05, 0.95, 1.10, 0.55, 0.22),      # Deep shadow trough along blocked Route A between boulder & crater
        (12.20, 2.60, 2.40, 1.25, -0.15),    # Mid-corridor shadow zone
        (34.00, -13.00, 4.50, 2.40, -0.32),  # Science Station Rocky Ridge shadow zone
        (54.00, -54.00, 8.50, 5.20, -0.78),  # Approach shadow apron into Permanently Shadowed Crater
        (64.00, -64.00, 14.50, 14.50, 0.0),  # Massive Permanent-Shadow Crater Floor (Location 7)!
    ]
    for scx, scy, srx, sry, sang in shadow_regions:
        _append_shadow_patch(pen_v, pen_f, cx=scx, cy=scy, rx=srx * 1.22, ry=sry * 1.22, angle=sang, z_ground=0.014)
        _append_shadow_patch(sh_v, sh_f, cx=scx, cy=scy, rx=srx, ry=sry, angle=sang, z_ground=0.018)

    _write_smooth_shaded_obj(MESH_DIR / "lunar_bedrock_patches.obj", bed_v, bed_f, "Exposed Lunar Bedrock Formations", contrast=1.25)
    _write_smooth_shaded_obj(MESH_DIR / "lunar_route_crater_rim.obj", rim_v, rim_f, "Raised Lunar Crater Rims", contrast=1.25)
    _write_smooth_shaded_obj(MESH_DIR / "lunar_route_crater_wall.obj", wall_v, wall_f, "Steep Inner Lunar Crater Walls", contrast=1.30)
    _write_smooth_shaded_obj(MESH_DIR / "lunar_deep_shadows.obj", sh_v, sh_f, "Deep Lunar South-Pole Umbra Shadows", bake_sun=False)
    _write_smooth_shaded_obj(MESH_DIR / "lunar_penumbra_shadows.obj", pen_v, pen_f, "Lunar Penumbra Partial Shadows", bake_sun=False)


def main() -> None:
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    sculpt_base_visual_terrain_obj()
    print("Sculpted 3D near-field topography & normals into lunar_terrain.obj")
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
