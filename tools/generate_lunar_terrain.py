#!/usr/bin/env python3
"""
LunaBot V4 - Phase A - Lunar terrain generator (deterministic)
===============================================================

Generates the lunar terrain mesh used by
  src/lunabot_gazebo/worlds/lunar_world.sdf

Outputs (relative to this repo):
  src/lunabot_gazebo/worlds/meshes/lunar_terrain.obj           (visual, high-res, per-vertex color)
  src/lunabot_gazebo/worlds/meshes/lunar_terrain_collision.obj (collision, low-res)
  evidence/phase-a-launch-a/terrain_stats.txt
  evidence/phase-a-launch-a/terrain_preview_topdown.png
  evidence/phase-a-launch-a/terrain_preview_perspective.png

Terrain model (matches the Phase A visual reference spec):
  * 400 m x 400 m area, centered on world origin, 1.25 m grid (visual)
  * large-scale rolling relief (multi-octave value noise)
  * fine rocky roughness (short-wavelength noise)
  * dense crater field:
      - small  : ~220 craters, 3-10 m diameter
      - medium :  ~70 craters, 12-30 m diameter
      - large  :   7 craters, 44-90 m diameter (with central peaks)
  * smooth spawn pad at the origin (rover spawn zone)
  * flat boundary shelf so the terrain blends into the horizon plane
  * grey/monochrome per-vertex albedo with crater shadow/rim modulation

Usage:
  python3 tools/generate_lunar_terrain.py             # all outputs
  python3 tools/generate_lunar_terrain.py --no-previews
  python3 tools/generate_lunar_terrain.py --seed 7    # different field, still deterministic

Requires: numpy (pillow + matplotlib only needed for previews)
"""

import argparse
import json
import math
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD_MESH_DIR = os.path.join(REPO, "src", "lunabot_gazebo", "worlds", "meshes")
EVIDENCE_DIR = os.path.join(REPO, "evidence", "phase-a-launch-a")

# ----------------------------------------------------------------------
# Parameters
# ----------------------------------------------------------------------
EXTENT = 400.0          # m, terrain spans [-200, 200] in x and y
N_FINE = 320            # visual grid resolution (cells)
N_COLL = 100            # collision grid resolution (cells)
SEED = 42

SPAWN_RADIUS = 14.0     # m, flat-ish pad around origin (fully blended inside)
SPAWN_FADE = 24.0       # m, blend completes here
BND_IN = 170.0          # m, boundary shelf starts
BND_OUT = 200.0         # m, boundary shelf is fully flat


def value_noise(rng, n, extent, wavelength):
    """2-D value noise: random lattice resampled with smoothstep bilinear."""
    m = max(2, int(round(extent / wavelength)))
    g = rng.random((m + 1, m + 1))
    u = np.linspace(0.0, m, n)
    U, V = np.meshgrid(u, u, indexing="xy")
    ix = np.clip(np.floor(U).astype(int), 0, m - 1)
    jx = np.clip(np.floor(V).astype(int), 0, m - 1)
    fx = U - ix
    fx = fx * fx * (3.0 - 2.0 * fx)
    fy = V - jx
    fy = fy * fy * (3.0 - 2.0 * fy)
    a = g[jx, ix]
    b = g[jx, ix + 1]
    c = g[jx + 1, ix]
    d = g[jx + 1, ix + 1]
    ab = a + (b - a) * fx
    cd = c + (d - c) * fx
    return (ab + (cd - ab) * fy) * 2.0 - 1.0   # -> [-1, 1]


def base_relief(x, y, rng):
    z = (
        3.2 * value_noise(rng, N_FINE, EXTENT, 260.0)
        + 1.8 * value_noise(rng, N_FINE, EXTENT, 130.0)
        + 0.9 * value_noise(rng, N_FINE, EXTENT, 65.0)
        + 0.45 * value_noise(rng, N_FINE, EXTENT, 32.0)
        + 0.35 * value_noise(rng, N_FINE, EXTENT, 10.0)
        + 0.18 * value_noise(rng, N_FINE, EXTENT, 5.0)
        + 0.002 * (x - y)                       # gentle overall slope
    )
    return z


def apply_crater(z, dep, rim, x, y, cx, cy, R, D):
    """Parabolic bowl + gaussian rim (+ central peak for large craters)."""
    d = np.sqrt((x - cx) ** 2 + (y - cy) ** 2)
    bowl = D * (1.0 - np.clip(d / R, 0.0, 1.0) ** 2)
    bowl = np.where(d <= R, bowl, 0.0)
    rimh = 0.35 * D * np.exp(-(((d - R) / (0.30 * R)) ** 2))
    rimh = np.where(d < 1.6 * R, rimh, 0.0)
    peak = np.zeros_like(d)
    if R >= 22.0:
        pk = 0.30 * D * np.exp(-((d / (0.18 * R)) ** 2))
        peak = np.where(d < 0.4 * R, pk, 0.0)
    z = z - bowl + rimh + peak
    dep = np.maximum(dep, bowl / D)
    rim = np.maximum(rim, rimh / (0.35 * D))
    return z, dep, rim


def make_craters(rng):
    """Returns (cx, cy, R, D, class) list, sorted large -> small."""
    cr = []
    placed = []
    attempts = 0
    while len(placed) < 7 and attempts < 4000:
        attempts += 1
        R = rng.uniform(22.0, 45.0)
        cx, cy = rng.uniform(-180.0, 180.0), rng.uniform(-180.0, 180.0)
        if math.hypot(cx, cy) < 40.0:
            continue
        if any(math.hypot(cx - px, cy - py) < (R + pR) * 1.15 for px, py, pR in placed):
            continue
        placed.append((cx, cy, R))
        cr.append((cx, cy, R, R * rng.uniform(0.14, 0.22), "large"))
    for _ in range(70):
        R = rng.uniform(6.0, 15.0)
        while True:
            cx, cy = rng.uniform(-190.0, 190.0), rng.uniform(-190.0, 190.0)
            if math.hypot(cx, cy) > 20.0:
                break
        cr.append((cx, cy, R, R * rng.uniform(0.12, 0.20), "medium"))
    for _ in range(220):
        R = rng.uniform(1.5, 5.0)
        cx, cy = rng.uniform(-195.0, 195.0), rng.uniform(-195.0, 195.0)
        if math.hypot(cx, cy) < 12.0:
            cx, cy = cx * 2.0, cy * 2.0
        cr.append((cx, cy, R, R * rng.uniform(0.08, 0.16), "small"))
    cr.sort(key=lambda t: -t[2])   # apply largest first
    return cr


def write_obj(path, x, y, z, normals, colors=None, grid_shape=None):
    """Write OBJ with per-vertex normals (and optional per-vertex colors).

    Vertices are stored row-major on a regular grid (grid_shape = (rows, cols)).
    """
    if grid_shape is None:
        rows = cols = int(math.sqrt(x.shape[0]))
        if rows * cols != x.shape[0]:
            raise ValueError("cannot infer grid shape from vertex count")
    else:
        rows, cols = grid_shape
    n = x.shape[0]
    if n != rows * cols:
        raise ValueError(f"vertex count {n} != grid {rows}x{cols}")
    tmp = path + ".tmp"
    path_final = path
    path = tmp
    with open(path, "w") as fh:
        fh.write("# LunaBot V4 - generated lunar terrain (deterministic)\n")
        # vertices in chunks to bound memory
        buf = []
        for i in range(n):
            if colors is not None:
                cr, cg, cb = colors[i]
                buf.append(f"v {x[i]:.4f} {y[i]:.4f} {z[i]:.4f}\n"
                           f"vn {normals[i, 0]:.5f} {normals[i, 1]:.5f} {normals[i, 2]:.5f}\n"
                           f"c {cr:.4f} {cg:.4f} {cb:.4f}\n")
            else:
                buf.append(f"v {x[i]:.4f} {y[i]:.4f} {z[i]:.4f}\n"
                           f"vn {normals[i, 0]:.5f} {normals[i, 1]:.5f} {normals[i, 2]:.5f}\n")
            if len(buf) >= 8192:
                fh.write("".join(buf))
                buf.clear()
        if buf:
            fh.write("".join(buf))
        # faces (row-major grid, +z-facing winding)
        buf = []
        for r in range(rows - 1):
            for c in range(cols - 1):
                a = r * cols + c + 1        # (r, c)
                b = r * cols + c + 2        # (r, c+1)
                d = (r + 1) * cols + c + 1  # (r+1, c)
                e = (r + 1) * cols + c + 2  # (r+1, c+1)
                buf.append(f"f {a}//{a} {b}//{b} {e}//{e}\n"
                           f"f {a}//{a} {e}//{e} {d}//{d}\n")
                if len(buf) >= 8192:
                    fh.write("".join(buf))
                    buf.clear()
        if buf:
            fh.write("".join(buf))
        size = os.path.getsize(path)
        if size > 200e6:
            os.remove(path)
            raise RuntimeError(f"safety valve: {path_final} grew to {size/1e6:.0f} MB, aborting")
    os.replace(tmp, path_final)   # atomic: never leaves a corrupt mesh behind


def bilinear_sample(z, n_fine, n_coarse):
    """Sample fine height field (n_fine x n_fine) onto a coarse grid (n_c, n_c)."""
    idx = np.linspace(0, n_fine - 1, n_coarse)
    i0 = np.floor(idx).astype(int)
    i1 = np.clip(i0 + 1, 0, n_fine - 1)
    f = (idx - i0)[:, None]                 # (n_c, 1)
    zi = z[i0] * (1.0 - f) + z[i1] * f      # (n_c, n_fine)  -> axis 0
    out = zi[:, i0] * (1.0 - f) + zi[:, i1] * f   # (n_c, n_c)  -> axis 1
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=SEED)
    ap.add_argument("--no-previews", action="store_true")
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    os.makedirs(WORLD_MESH_DIR, exist_ok=True)
    os.makedirs(EVIDENCE_DIR, exist_ok=True)

    cell = EXTENT / N_FINE
    xs = -EXTENT / 2 + (np.arange(N_FINE) + 0.5) * cell
    X, Y = np.meshgrid(xs, xs, indexing="xy")     # rows = y, cols = x
    Z = base_relief(X, Y, rng)

    dep = np.zeros((N_FINE, N_FINE))
    rim = np.zeros((N_FINE, N_FINE))
    craters = make_craters(rng)
    for cx, cy, R, D, _cls in craters:
        Z, dep, rim = apply_crater(Z, dep, rim, X, Y, cx, cy, R, D)

    # smooth spawn pad at origin
    d0 = np.sqrt(X ** 2 + Y ** 2)
    h0 = float(Z[d0 < 10.0].mean())
    fade = np.clip((d0 - SPAWN_RADIUS) / (SPAWN_FADE - SPAWN_RADIUS), 0.0, 1.0)
    w = 0.5 - 0.5 * np.cos(np.pi * fade)
    Z = Z * w + h0 * (1.0 - w)

    # flat boundary shelf -> blends into the horizon plane
    zmean = float(Z.mean())
    fb = np.clip((d0 - BND_IN) / (BND_OUT - BND_IN), 0.0, 1.0)
    fb = fb * fb * (3.0 - 2.0 * fb)
    Z = Z * (1.0 - fb) + zmean * fb

    # smooth normals from the gradient
    dzdx, dzdy = np.gradient(Z, cell)
    normals = np.stack([-dzdx, -dzdy, np.ones_like(Z)], axis=-1)
    normals /= (np.linalg.norm(normals, axis=-1, keepdims=True) + 1e-9)

    # grey/monochrome per-vertex albedo with crater modulation
    alb = 0.50 + 0.06 * value_noise(rng, N_FINE, EXTENT, 120.0) + 0.02 * value_noise(rng, N_FINE, EXTENT, 20.0)
    col = np.clip(alb - 0.16 * np.clip(dep, 0, 1.2) + 0.07 * np.clip(rim, 0, 1.0), 0.30, 0.72)
    fb2 = np.clip((d0 - BND_IN) / (BND_OUT - BND_IN), 0.0, 1.0)
    col = col * (1 - fb2) + (0.46 + 0.015 * value_noise(rng, N_FINE, EXTENT, 40.0)) * fb2
    col = np.clip(col, 0.30, 0.72)
    colors = np.repeat(col[..., None], 3, axis=-1)

    # ---- write meshes ----
    vis_path = os.path.join(WORLD_MESH_DIR, "lunar_terrain.obj")
    write_obj(vis_path, X.ravel(), Y.ravel(), Z.ravel(),
              normals.reshape(-1, 3), colors.reshape(-1, 3), grid_shape=(N_FINE, N_FINE))

    Zc = bilinear_sample(Z, N_FINE, N_COLL)
    dzdx_c, dzdy_c = np.gradient(Zc, EXTENT / N_COLL)
    nc = np.stack([-dzdx_c, -dzdy_c, np.ones_like(Zc)], axis=-1)
    nc /= (np.linalg.norm(nc, axis=-1, keepdims=True) + 1e-9)
    xs_c = -EXTENT / 2 + (np.arange(N_COLL) + 0.5) * (EXTENT / N_COLL)
    Xc, Yc = np.meshgrid(xs_c, xs_c, indexing="xy")
    coll_path = os.path.join(WORLD_MESH_DIR, "lunar_terrain_collision.obj")
    write_obj(coll_path, Xc.ravel(), Yc.ravel(), Zc.ravel(),
              nc.reshape(-1, 3), None, grid_shape=(N_COLL, N_COLL))

    # ---- stats ----
    n_large = sum(1 for t in craters if t[4] == "large")
    n_med = sum(1 for t in craters if t[4] == "medium")
    n_small = sum(1 for t in craters if t[4] == "small")
    stats = {
        "seed": args.seed,
        "extent_m": EXTENT,
        "visual_grid": f"{N_FINE}x{N_FINE} ({cell:.2f} m)",
        "collision_grid": f"{N_COLL}x{N_COLL} ({EXTENT / N_COLL:.1f} m)",
        "visual_vertices": N_FINE * N_FINE,
        "visual_triangles": 2 * (N_FINE - 1) * (N_FINE - 1),
        "z_min_m": round(float(Z.min()), 3),
        "z_max_m": round(float(Z.max()), 3),
        "z_mean_m": round(float(Z.mean()), 3),
        "spawn_height_m": round(h0, 3),
        "craters": {"large": n_large, "medium": n_med, "small": n_small},
        "large_craters": [
            {"x": round(c[0], 1), "y": round(c[1], 1), "R_m": round(c[2], 1), "depth_m": round(c[3], 2)}
            for c in craters if c[4] == "large"
        ],
        "visual_obj_mb": round(os.path.getsize(vis_path) / 1e6, 1),
        "collision_obj_mb": round(os.path.getsize(coll_path) / 1e6, 1),
    }
    stats_path = os.path.join(EVIDENCE_DIR, "terrain_stats.txt")
    with open(stats_path, "w") as fh:
        fh.write("LunaBot V4 - Phase A - lunar terrain statistics\n")
        fh.write("=" * 50 + "\n")
        for k, v in stats.items():
            fh.write(f"{k:20s}: {json.dumps(v)}\n")
        fh.write("\nSuggested rover spawn (model root z):\n")
        fh.write(f"  z = {h0 + 0.48:.3f} m   # wheel bottom = model_root_z - 0.43, +5 cm drop margin\n")
    print(json.dumps(stats, indent=2))

    if not args.no_previews:
        try:
            make_previews(X, Y, Z, col, craters)
            print("previews written to", EVIDENCE_DIR)
        except ImportError as e:
            print(f"WARNING: previews skipped ({e})")
    return 0


def make_previews(X, Y, Z, col, craters):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    from matplotlib.tri import Triangulation

    # sun direction (light travel vector from world SDF): (-0.5, 0.5, -0.6)
    L = np.array([0.5, -0.5, 0.6])
    L /= np.linalg.norm(L)
    dzdx, dzdy = np.gradient(Z, EXTENT / N_FINE)
    Nn = np.stack([-dzdx, -dzdy, np.ones_like(Z)], axis=-1)
    Nn /= (np.linalg.norm(Nn, axis=-1, keepdims=True) + 1e-9)
    shade = np.clip(Nn[..., 0] * L[0] + Nn[..., 1] * L[1] + Nn[..., 2] * L[2], 0, 1)
    surf = 0.16 + 0.62 * shade * (col / 0.5)

    # ---------- top-down ----------
    fig, ax = plt.subplots(figsize=(10, 10), dpi=140)
    ax.imshow(surf, extent=[-EXTENT / 2, EXTENT / 2, EXTENT / 2, -EXTENT / 2],
              cmap="gray", vmin=0, vmax=1, origin="lower")
    ax.contour(Z, levels=24, colors="w", alpha=0.15, linewidths=0.4)
    for cx, cy, R, D, cls in craters:
        if cls == "large":
            ax.add_patch(Circle((cx, cy), R, fill=False, ec="orange", lw=1.0, alpha=0.85))
            ax.text(cx, cy + R + 5, f"R={R:.0f}m", color="orange", fontsize=8, ha="center")
    ax.add_patch(Circle((0, 0), 20, fill=False, ec="cyan", lw=1.2, ls="--"))
    ax.text(0, 27, "spawn pad", color="cyan", fontsize=9, ha="center")
    ax.set_xlim(-EXTENT / 2, EXTENT / 2)
    ax.set_ylim(-EXTENT / 2, EXTENT / 2)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("LunaBot V4 - lunar terrain, top-down (400 m x 400 m)")
    ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(os.path.join(EVIDENCE_DIR, "terrain_preview_topdown.png"))
    plt.close(fig)

    # ---------- perspective (matches Gazebo GUI camera at (0, -300, 200)) ----------
    step = 2
    Xs2 = X[::step, ::step]
    Ys2 = Y[::step, ::step]
    Zs2 = Z[::step, ::step]
    Cs2 = 0.16 + 0.62 * shade[::step, ::step] * (col[::step, ::step] / 0.5)
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    tri2d = Triangulation(Xs2.ravel(), Ys2.ravel())
    t = tri2d.triangles
    Xs, Ys, Zs = Xs2.ravel(), Ys2.ravel(), Zs2.ravel()
    per_tri = Cs2.ravel()[t].mean(axis=1)
    verts = np.stack((Xs[t], Ys[t], Zs[t]), axis=-1)          # (M, 3, 3)
    cmap = plt.get_cmap("gray")
    fc = cmap(np.clip(per_tri, 0, 1)[:, None])               # (M, 4) rgba
    fig = plt.figure(figsize=(11, 7.5), dpi=140)
    ax = fig.add_subplot(111, projection="3d")
    ax.add_collection3d(Poly3DCollection(verts, facecolors=fc, edgecolors="none", linewidths=0))
    ax.set_xlim(-EXTENT / 2, EXTENT / 2)
    ax.set_ylim(-EXTENT / 2, EXTENT / 2)
    ax.set_zlim(float(Z.min()) - 2, float(Z.max()) + 2)
    # mark rover spawn
    ax.scatter([0], [0], [Z[N_FINE // 2, N_FINE // 2] + 0.8], c="red", s=30, depthshade=False)
    ax.set_box_aspect((1, 1, 0.16))
    ax.view_init(elev=27, azim=-90)
    ax.dist = 27
    ax.set_title("LunaBot V4 - lunar terrain, perspective (view ~ Gazebo GUI camera)")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    fig.tight_layout()
    fig.savefig(os.path.join(EVIDENCE_DIR, "terrain_preview_perspective.png"))
    plt.close(fig)


if __name__ == "__main__":
    sys.exit(main())
