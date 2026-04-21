"""
Obstacle Detection + Bird's-Eye View — RealSense D435i
=======================================================
Merges the Intel RealSense point-cloud viewer with ground-plane estimation
and obstacle detection.

Left window  — 3-D point cloud coloured by classification:
                  grey   = out-of-range / unclassified
                  green  = ground
                  red    = boulder  (above ground)
                  blue   = crater   (below ground)

Right window — Bird's-eye-view occupancy map with obstacle circles

Keyboard shortcuts (same as the original Intel example):
    [p]     Pause / resume
    [r]     Reset 3-D view
    [d]     Cycle decimation  (1×, 2×, 4×)
    [z]     Toggle point scaling
    [h]     Toggle height-map colouring  (vs original RGB texture)
    [q/ESC] Quit

Usage:
    python obstacle_bev.py
    python obstacle_bev.py --height 0.30 --pitch 25
    python obstacle_bev.py --height 0.30 --pitch 25 --verify
"""

import argparse
import math
import time

import cv2
import numpy as np
import pyrealsense2 as rs
from scipy.ndimage import center_of_mass, find_objects, gaussian_filter, label


# ─────────────────────────────────────────────────────────────────────────────
#  CONFIG
# ─────────────────────────────────────────────────────────────────────────────
class Cfg:
    CAM_HEIGHT       = 0.85    # metres above ground
    CAM_PITCH_DEG    = 45.0    # degrees downward from horizontal (0 = horizontal)

    MIN_DEPTH        = 0.15    # m
    MAX_DEPTH        = 5.0     # m

    BOULDER_H_MIN    = 0.25    # m above plane → obstacle (tightened for 30-50cm obstacles)
    BOULDER_H_MAX    = 0.55    # m above plane → ceiling / person (ignored)
    CRATER_D_MIN     = 0.06    # m below plane → obstacle
    CRATER_D_MAX     = 0.55    # m below plane → noise / invalid (ignored)

    GRID_N           = 250     # BEV grid cells per side
    RES              = 0.03    # m / cell  (250 × 0.03 = 7.5 m)
    DECAY            = 0.75    # temporal decay per frame
    BIN_THRESH       = 0.28
    MIN_CLUSTER      = 25      # min cells for a valid obstacle (increased for fewer false positives)
    MAX_OBSTACLE_DIST= 3.0     # m - only detect obstacles within this distance

    RANSAC_N         = 80
    RANSAC_THRESH    = 0.03    # m inlier distance (tightened for flatter ground)
    RANSAC_MIN_INL   = 300     # min inliers (increased for better ground fit)
    PLANE_EMA        = 0.80    # smoothing: higher = slower adaptation (more stable)
    PLANE_MAX_C_DELTA= 0.08    # max allowed intercept shift per frame (tightened)

    BEV_SCALE        = 2       # BEV pixel scale
    VERIFY_MODE      = False


# ─────────────────────────────────────────────────────────────────────────────
#  GROUND PLANE
# ─────────────────────────────────────────────────────────────────────────────
def make_prior_plane(height_m: float, pitch_deg: float) -> np.ndarray:
    """
    Analytical ground-plane prior for  y_cam = a·x_cam + b·z_cam + c.

    Camera frame: x right, y DOWN, z forward.
    Camera is height_m above the floor, pitched pitch_deg downward.

        y_cam = −tan(θ)·z_cam + h/cos(θ)
        → a = 0,  b = −tan(θ),  c = h/cos(θ)

    Convention:
        pitch_deg = 0   → camera horizontal, ground at y_cam = h everywhere
        pitch_deg = 25  → camera tilted 25° downward (typical robot mount)
        pitch_deg < 0   → camera tilted upward (unusual)
    """
    th = np.radians(pitch_deg)
    return np.array([0.0, -np.tan(th), height_m / np.cos(th)])


def estimate_ground_plane(
    verts:  np.ndarray,
    prior:  np.ndarray,
) -> np.ndarray | None:
    """RANSAC fit of y = a·x + b·z + c, biased toward the prior plane."""
    if len(verts) < 500:
        return None

    x, y, z = verts[:, 0], verts[:, 1], verts[:, 2]
    a0, b0, c0 = prior

    near  = np.abs(y - (a0 * x + b0 * z + c0)) < Cfg.RANSAC_THRESH * 3
    lower = y > np.percentile(y, 25)
    xc, yc, zc = x[near | lower], y[near | lower], z[near | lower]
    if len(xc) < 100:
        return None

    best_inl, best_coeff = 0, None
    rng = np.random.default_rng()

    for _ in range(Cfg.RANSAC_N):
        s = rng.choice(len(xc), 3, replace=False)
        A = np.stack([xc[s], zc[s], np.ones(3)], axis=1)
        try:
            coeff = np.linalg.solve(A, yc[s])
        except np.linalg.LinAlgError:
            continue
        if abs(coeff[2] - prior[2]) > Cfg.PLANE_MAX_C_DELTA * 3:
            continue
        y_pred  = coeff[0] * x + coeff[1] * z + coeff[2]
        inliers = int(np.sum(np.abs(y - y_pred) < Cfg.RANSAC_THRESH))
        if inliers > best_inl:
            best_inl, best_coeff = inliers, coeff

    if best_coeff is None or best_inl < Cfg.RANSAC_MIN_INL:
        return None
    if abs(best_coeff[2] - prior[2]) > Cfg.PLANE_MAX_C_DELTA * 2:
        return None

    y_pred_all = best_coeff[0] * x + best_coeff[1] * z + best_coeff[2]
    mask = np.abs(y - y_pred_all) < Cfg.RANSAC_THRESH
    if mask.sum() < 100:
        return None

    coeff_r, _, _, _ = np.linalg.lstsq(
        np.stack([x[mask], z[mask], np.ones(mask.sum())], axis=1),
        y[mask], rcond=None,
    )
    return coeff_r


def smooth_plane(cur: np.ndarray, new: np.ndarray | None) -> np.ndarray:
    if new is None:
        return cur
    return Cfg.PLANE_EMA * cur + (1.0 - Cfg.PLANE_EMA) * new


def height_above_ground(verts: np.ndarray, plane: np.ndarray) -> np.ndarray:
    """Positive = above ground (boulder), negative = below (crater)."""
    a, b, c = plane
    return (a * verts[:, 0] + b * verts[:, 2] + c) - verts[:, 1]


# ─────────────────────────────────────────────────────────────────────────────
#  POINT CLASSIFICATION  (per-vertex colour for the 3-D view)
# ─────────────────────────────────────────────────────────────────────────────
# BGR colours
_COL_GROUND  = np.array([50,  180,  60], dtype=np.uint8)   # green
_COL_BOULDER = np.array([40,   50, 220], dtype=np.uint8)   # red
_COL_CRATER  = np.array([210,  60,  30], dtype=np.uint8)   # blue
_COL_OTHER   = np.array([80,   80,  80], dtype=np.uint8)   # dark grey


def classify_verts(verts: np.ndarray, plane: np.ndarray) -> np.ndarray:
    """
    Returns (N, 3) uint8 BGR colour array for every vertex based on its
    height relative to the estimated ground plane.
    """
    colours = np.full((len(verts), 3), _COL_OTHER, dtype=np.uint8)
    h = height_above_ground(verts, plane)

    is_ground  = np.abs(h) < Cfg.RANSAC_THRESH
    is_boulder = (h >  Cfg.BOULDER_H_MIN) & (h <  Cfg.BOULDER_H_MAX)
    is_crater  = (h < -Cfg.CRATER_D_MIN)  & (h > -Cfg.CRATER_D_MAX)

    colours[is_ground]  = _COL_GROUND
    colours[is_boulder] = _COL_BOULDER
    colours[is_crater]  = _COL_CRATER
    return colours


# ─────────────────────────────────────────────────────────────────────────────
#  OCCUPANCY GRID + OBSTACLE EXTRACTION
# ─────────────────────────────────────────────────────────────────────────────
def update_occupancy(
    occ_map: np.ndarray,
    verts:   np.ndarray,
    plane:   np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    h_vals = height_above_ground(verts, plane)
    dist_vals = np.sqrt(verts[:, 0]**2 + verts[:, 2]**2)  # Distance from camera
    
    mask = (
        ((h_vals >  Cfg.BOULDER_H_MIN) & (h_vals <  Cfg.BOULDER_H_MAX)) |
        ((h_vals < -Cfg.CRATER_D_MIN)  & (h_vals > -Cfg.CRATER_D_MAX))
    ) & (dist_vals < Cfg.MAX_OBSTACLE_DIST)  # Only obstacles within range
    
    obs     = verts[mask]
    layer   = np.zeros_like(occ_map)

    if len(obs) > 0:
        # robot frame: x_r = z_cam (forward),  y_r = -x_cam (left)
        row = (Cfg.GRID_N // 2 - (obs[:, 2] / Cfg.RES)).astype(int)
        col = (Cfg.GRID_N // 2 + (obs[:, 0] / Cfg.RES)).astype(int)   # note: -(-x) = x

        valid = (row >= 0) & (row < Cfg.GRID_N) & (col >= 0) & (col < Cfg.GRID_N)
        np.add.at(layer, (row[valid], col[valid]), 1.0)
        if layer.max() > 0:
            layer = np.clip(layer / 8.0, 0.0, 1.0)
        layer = gaussian_filter(layer, sigma=1.2)

    occ_map = Cfg.DECAY * occ_map + (1.0 - Cfg.DECAY) * layer
    return occ_map, (occ_map > Cfg.BIN_THRESH)


def extract_obstacles(binary: np.ndarray) -> list[tuple[float, float, float]]:
    labeled, n = label(binary)
    if n == 0:
        return []
    obstacles = []
    for i, sl in enumerate(find_objects(labeled)):
        region = labeled[sl] == (i + 1)
        size   = int(region.sum())
        if size < Cfg.MIN_CLUSTER:
            continue
        cy, cx = center_of_mass(region)
        row    = sl[0].start + cy
        col    = sl[1].start + cx
        x_fwd  = (Cfg.GRID_N // 2 - row) * Cfg.RES
        y_lat  = (col - Cfg.GRID_N // 2) * Cfg.RES
        radius = max(float(np.sqrt(size / np.pi) * Cfg.RES), 0.05)
        if x_fwd > 0.1:
            obstacles.append((float(x_fwd), float(y_lat), radius))
    obstacles.sort(key=lambda o: o[0])
    return obstacles


# ─────────────────────────────────────────────────────────────────────────────
#  3-D POINT CLOUD RENDERER  (adapted from Intel example)
# ─────────────────────────────────────────────────────────────────────────────
class AppState:
    def __init__(self):
        self.pitch = math.radians(-10)
        self.yaw   = math.radians(-15)
        self.translation = np.array([0, 0, -1], dtype=np.float32)
        self.distance    = 2.0
        self.prev_mouse  = (0, 0)
        self.mouse_btns  = [False, False, False]
        self.paused      = False
        self.decimate    = 1
        self.scale       = True
        self.show_height = True   # colour by height classification vs RGB

    def reset(self):
        self.pitch, self.yaw, self.distance = 0, 0, 2.0
        self.translation[:] = [0, 0, -1]

    @property
    def rotation(self):
        Rx, _ = cv2.Rodrigues((self.pitch, 0, 0))
        Ry, _ = cv2.Rodrigues((0, self.yaw, 0))
        return np.dot(Ry, Rx).astype(np.float32)

    @property
    def pivot(self):
        return self.translation + np.array([0, 0, self.distance], np.float32)


def project(v, out_shape, state: AppState):
    h_out, w_out = out_shape[:2]
    view_aspect  = h_out / w_out
    with np.errstate(divide='ignore', invalid='ignore'):
        proj = v[:, :2] / v[:, 2:3] * (w_out * view_aspect, h_out) + (w_out / 2.0, h_out / 2.0)
    proj[v[:, 2] < 0.03] = np.nan
    return proj


def view_transform(v, state: AppState):
    return np.dot(v - state.pivot, state.rotation) + state.pivot - state.translation


def line3d(out, pt1, pt2, state: AppState, color=(128, 128, 128), thickness=1):
    p0 = project(pt1.reshape(-1, 3), out.shape, state)[0]
    p1 = project(pt2.reshape(-1, 3), out.shape, state)[0]
    if np.isnan(p0).any() or np.isnan(p1).any():
        return
    rect   = (0, 0, out.shape[1], out.shape[0])
    inside, p0c, p1c = cv2.clipLine(rect, tuple(p0.astype(int)), tuple(p1.astype(int)))
    if inside:
        cv2.line(out, p0c, p1c, color, thickness, cv2.LINE_AA)


def draw_grid(out, pos, state: AppState, size=1, n=10, color=(80, 80, 80)):
    pos = np.array(pos, np.float32)
    s, s2 = size / n, size / 2
    for i in range(n + 1):
        x = -s2 + i * s
        line3d(out, view_transform(pos + [x, 0, -s2], state),
               view_transform(pos + [x, 0,  s2], state), state, color)
        z = -s2 + i * s
        line3d(out, view_transform(pos + [-s2, 0, z], state),
               view_transform(pos + [ s2, 0, z], state), state, color)


def draw_axes(out, pos, state: AppState, size=0.1, thickness=2):
    line3d(out, pos, pos + view_transform(np.array([size, 0, 0], np.float32), state) * 0,
           state, (0, 0, 255), thickness)
    rot = state.rotation
    line3d(out, pos, pos + np.dot([size, 0, 0], rot), state, (255,   0,   0), thickness)
    line3d(out, pos, pos + np.dot([0, size, 0], rot), state, (  0, 255,   0), thickness)
    line3d(out, pos, pos + np.dot([0, 0, size], rot), state, (  0,   0, 255), thickness)


def render_pointcloud(
    out:       np.ndarray,
    verts:     np.ndarray,
    texcoords: np.ndarray,
    color_src: np.ndarray,
    state:     AppState,
    vert_cols: np.ndarray | None = None,
):
    """
    Render point cloud into `out`.

    If vert_cols is given (N,3 BGR), each point is drawn with its per-vertex
    colour (height-classification mode).  Otherwise, UV-maps from color_src.
    """
    v_tf  = view_transform(verts, state)
    order = v_tf[:, 2].argsort()[::-1]   # painter's sort back→front
    proj  = project(v_tf[order], out.shape, state)

    if state.scale:
        proj *= 0.5 ** state.decimate

    h_out, w_out = out.shape[:2]
    j, i = proj.astype(np.uint32).T
    mask = (i < h_out) & (j < w_out)

    if vert_cols is not None:
        cols = vert_cols[order][mask]               # (M, 3)  BGR
        out[i[mask], j[mask]] = cols
    else:
        cw, ch = color_src.shape[:2][::-1]
        tc = (texcoords[order][mask] * (cw, ch) + 0.5).astype(np.uint32)
        u  = np.clip(tc[:, 1], 0, ch - 1)
        v_ = np.clip(tc[:, 0], 0, cw - 1)
        out[i[mask], j[mask]] = color_src[u, v_]


# ─────────────────────────────────────────────────────────────────────────────
#  BEV RENDERER
# ─────────────────────────────────────────────────────────────────────────────
_P = dict(
    bg      =(18,  18,  18),
    grid    =(35,  35,  35),
    axis    =(55,  55,  55),
    ring    =(48,  48,  48),
    obs_ring=(0,  160, 255),
    obs_dot =(0,  220, 255),
    label   =(255, 220,  80),
    robot   =(0,  255, 100),
    fwd     =(100, 255, 100),
    legend_g=(50,  180,  60),
    legend_r=(40,   50, 220),
    legend_b=(210,  60,  30),
)


def draw_bev(
    occ_map:   np.ndarray,
    obstacles: list[tuple[float, float, float]],
    plane:     np.ndarray,
) -> np.ndarray:
    S  = Cfg.BEV_SCALE
    N  = Cfg.GRID_N
    sz = N * S
    mid = (N // 2) * S

    canvas = np.full((sz, sz, 3), _P['bg'], dtype=np.uint8)

    # ── Occupancy heatmap ───────────────────────────────────────────────────
    heat     = cv2.resize((occ_map * 255).astype(np.uint8), (sz, sz),
                          interpolation=cv2.INTER_NEAREST)
    heat_bgr = cv2.applyColorMap(heat, cv2.COLORMAP_OCEAN)
    canvas[heat > 15] = heat_bgr[heat > 15]

    # ── 1 m grid ────────────────────────────────────────────────────────────
    cells_m = max(1, int(1.0 / Cfg.RES))
    for i in range(0, N, cells_m):
        p = i * S
        cv2.line(canvas, (0, p), (sz, p), _P['grid'], 1)
        cv2.line(canvas, (p, 0), (p, sz), _P['grid'], 1)

    # ── Distance rings ──────────────────────────────────────────────────────
    for d in range(1, int(Cfg.MAX_DEPTH) + 1):
        r_px = int(d / Cfg.RES * S)
        cv2.circle(canvas, (mid, mid), r_px, _P['ring'], 1)
        ly = mid - r_px
        if 4 < ly < sz - 4:
            cv2.putText(canvas, f"{d}m", (mid + 4, ly + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.30, (75, 75, 75), 1, cv2.LINE_AA)

    # ── Axes ────────────────────────────────────────────────────────────────
    cv2.line(canvas, (mid, 0), (mid, sz), _P['axis'], 1)
    cv2.line(canvas, (0, mid), (sz, mid), _P['axis'], 1)

    # ── Obstacle circles ────────────────────────────────────────────────────
    for xf, yl, r in obstacles:
        row  = int((N // 2 - xf / Cfg.RES) * S)
        col  = int((N // 2 + yl / Cfg.RES) * S)   # NOTE: y_r = -x_cam, so col shift is via y_lat directly
        r_px = max(5, int(r / Cfg.RES * S))
        cv2.circle(canvas, (col, row), r_px + 3, _P['obs_ring'], 2)
        cv2.circle(canvas, (col, row), 3,         _P['obs_dot'],  -1)
        cv2.putText(canvas, f"({xf:.1f},{yl:+.1f})",
                    (col + r_px + 4, row + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, _P['label'], 1, cv2.LINE_AA)

    # ── Robot marker ────────────────────────────────────────────────────────
    tri = np.array([[mid, mid - 10], [mid - 7, mid + 7], [mid + 7, mid + 7]])
    cv2.fillPoly(canvas, [tri], _P['robot'])

    # ── Legend ──────────────────────────────────────────────────────────────
    for row_i, (lbl, col) in enumerate([
        ("Ground",  _P['legend_g']),
        ("Boulder", _P['legend_r']),
        ("Crater",  _P['legend_b']),
    ]):
        y0 = 14 + row_i * 16
        cv2.rectangle(canvas, (4, y0 - 9), (13, y0), col, -1)
        cv2.putText(canvas, lbl, (17, y0),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, (200, 200, 200), 1, cv2.LINE_AA)

    # ── Overlays ─────────────────────────────────────────────────────────────
    cv2.putText(canvas, "FWD", (mid - 14, 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.36, _P['fwd'], 1, cv2.LINE_AA)
    cv2.putText(canvas, f"obstacles: {len(obstacles)}", (sz - 95, 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.36, (180, 180, 180), 1, cv2.LINE_AA)
    cv2.putText(canvas,
                f"h={Cfg.CAM_HEIGHT:.2f}m  pitch={Cfg.CAM_PITCH_DEG:.1f}deg  "
                f"plane_c={plane[2]:.3f}",
                (4, sz - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.30, (100, 100, 100), 1, cv2.LINE_AA)

    return canvas


# ─────────────────────────────────────────────────────────────────────────────
#  MOUSE CALLBACK
# ─────────────────────────────────────────────────────────────────────────────
def make_mouse_cb(state: AppState, out_ref: list):
    def mouse_cb(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:  state.mouse_btns[0] = True
        if event == cv2.EVENT_LBUTTONUP:    state.mouse_btns[0] = False
        if event == cv2.EVENT_RBUTTONDOWN:  state.mouse_btns[1] = True
        if event == cv2.EVENT_RBUTTONUP:    state.mouse_btns[1] = False
        if event == cv2.EVENT_MBUTTONDOWN:  state.mouse_btns[2] = True
        if event == cv2.EVENT_MBUTTONUP:    state.mouse_btns[2] = False

        if event == cv2.EVENT_MOUSEMOVE:
            out = out_ref[0]
            if out is None:
                state.prev_mouse = (x, y)
                return
            h_out, w_out = out.shape[:2]
            dx = x - state.prev_mouse[0]
            dy = y - state.prev_mouse[1]
            if state.mouse_btns[0]:
                state.yaw   += float(dx) / w_out * 2
                state.pitch -= float(dy) / h_out * 2
            elif state.mouse_btns[1]:
                dp = np.array([dx / w_out, dy / h_out, 0], np.float32)
                state.translation -= np.dot(state.rotation, dp)
            elif state.mouse_btns[2]:
                dz = math.sqrt(dx**2 + dy**2) * math.copysign(0.01, -dy)
                state.translation[2] += dz
                state.distance       -= dz

        if event == cv2.EVENT_MOUSEWHEEL:
            dz = math.copysign(0.1, flags)
            state.translation[2] += dz
            state.distance       -= dz

        state.prev_mouse = (x, y)
    return mouse_cb


# ─────────────────────────────────────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────────────────────────────────────
def main() -> None:
    parser = argparse.ArgumentParser(
        description="RealSense obstacle BEV",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--height", type=float, default=Cfg.CAM_HEIGHT,
                        help="Camera height above ground (m)")
    parser.add_argument("--pitch",  type=float, default=Cfg.CAM_PITCH_DEG,
                        help="Camera downward pitch (degrees). 0=horizontal, 25=tilted down 25°")
    parser.add_argument("--verify", action="store_true",
                        help="Print per-frame debug to console")
    args = parser.parse_args()
    Cfg.CAM_HEIGHT    = args.height
    Cfg.CAM_PITCH_DEG = args.pitch
    Cfg.VERIFY_MODE   = args.verify

    # ── RealSense setup ──────────────────────────────────────────────────────
    pipeline = rs.pipeline()
    config   = rs.config()

    pw = rs.pipeline_wrapper(pipeline)
    pp = config.resolve(pw)
    device = pp.get_device()

    has_rgb = any(
        s.get_info(rs.camera_info.name) == 'RGB Camera'
        for s in device.sensors
    )
    if not has_rgb:
        print("ERROR: depth camera with colour sensor required")
        return

    config.enable_stream(rs.stream.depth, rs.format.z16,  30)
    config.enable_stream(rs.stream.color, rs.format.bgr8, 30)
    pipeline.start(config)

    profile          = pipeline.get_active_profile()
    depth_profile    = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
    depth_intrinsics = depth_profile.get_intrinsics()
    w_cam, h_cam     = depth_intrinsics.width, depth_intrinsics.height

    pc        = rs.pointcloud()
    decimate  = rs.decimation_filter()
    decimate.set_option(rs.option.filter_magnitude, 2)

    # ── State ────────────────────────────────────────────────────────────────
    app_state  = AppState()
    occ_map    = np.zeros((Cfg.GRID_N, Cfg.GRID_N), dtype=np.float32)
    prior      = make_prior_plane(Cfg.CAM_HEIGHT, Cfg.CAM_PITCH_DEG)
    cur_plane  = prior.copy()
    out_3d     = np.zeros((h_cam, w_cam, 3), dtype=np.uint8)
    out_ref    = [out_3d]

    print(f"\n╔════════════════════════════════════════════════════════════╗")
    print(f"║  RealSense Obstacle Detector + World-Frame Viewer          ║")
    print(f"╚════════════════════════════════════════════════════════════╝")
    print(f"\n[Camera Configuration]")
    print(f"  • Height above ground:  {Cfg.CAM_HEIGHT:.2f} m")
    print(f"  • Pitch down angle:     {Cfg.CAM_PITCH_DEG:.1f}°")
    print(f"\n[Obstacle Detection Parameters]")
    print(f"  • Boulder height range: {Cfg.BOULDER_H_MIN:.2f} - {Cfg.BOULDER_H_MAX:.2f} m")
    print(f"  • Max obstacle distance: {Cfg.MAX_OBSTACLE_DIST:.1f} m")
    print(f"  • Min cluster size:     {Cfg.MIN_CLUSTER} cells")
    print(f"\n[Ground Plane Prior]  y_cam = {prior[0]:.4f}·x_cam + {prior[1]:.4f}·z_cam + {prior[2]:.4f}")
    print(f"\n[Controls]")
    print(f"  • [p]     Pause/Resume camera")
    print(f"  • [r]     Reset view")
    print(f"  • [d]     Cycle decimation (1×, 2×, 4×)")
    print(f"  • [h]     Toggle height classification colours")
    print(f"  • [q/ESC] Quit")
    print(f"\n[View]")
    print(f"  • Ground plane fixed at y=0 (world frame, bottom of view)")
    print(f"  • Cyan marker = camera position at height {Cfg.CAM_HEIGHT:.2f} m")
    print(f"  • Green = ground  |  Red = boulders (above ground)  |  Blue = craters (below ground)\n")

    # ── Windows ──────────────────────────────────────────────────────────────
    WIN_3D  = "Point Cloud (h = toggle height colours)"
    WIN_BEV = "Bird's-Eye View"

    cv2.namedWindow(WIN_3D,  cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow(WIN_BEV, cv2.WINDOW_AUTOSIZE)
    cv2.resizeWindow(WIN_3D, w_cam, h_cam)
    cv2.setMouseCallback(WIN_3D, make_mouse_cb(app_state, out_ref))

    verts     = np.zeros((1, 3), np.float32)
    texcoords = np.zeros((1, 2), np.float32)
    color_img = np.zeros((h_cam, w_cam, 3), np.uint8)

    try:
        while True:
            t0 = time.time()

            # ── Grab frames ─────────────────────────────────────────────────
            if not app_state.paused:
                frames      = pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                if not depth_frame or not color_frame:
                    continue

                depth_frame = decimate.process(depth_frame)
                depth_intrinsics = rs.video_stream_profile(
                    depth_frame.profile).get_intrinsics()
                w_cam, h_cam = depth_intrinsics.width, depth_intrinsics.height

                color_img   = np.asanyarray(color_frame.get_data())

                # Build point cloud
                pc.map_to(color_frame)
                points    = pc.calculate(depth_frame)
                v_raw, t_raw = points.get_vertices(), points.get_texture_coordinates()
                verts     = np.asanyarray(v_raw).view(np.float32).reshape(-1, 3)
                texcoords = np.asanyarray(t_raw).view(np.float32).reshape(-1, 2)

                # ── Filter depth range ──────────────────────────────────────
                z = verts[:, 2]
                valid = (z > Cfg.MIN_DEPTH) & (z < Cfg.MAX_DEPTH)
                verts_f     = verts[valid]
                texcoords_f = texcoords[valid]

                # ── Ground plane ────────────────────────────────────────────
                raw_plane = estimate_ground_plane(verts_f, cur_plane)
                cur_plane = smooth_plane(cur_plane, raw_plane)

                # ── Filter points by height (remove > 50cm above ground) ─────
                h_vals = height_above_ground(verts_f, cur_plane)
                height_filter = h_vals <= 0.50
                verts_f = verts_f[height_filter]
                texcoords_f = texcoords_f[height_filter]

                # ── Occupancy + obstacles ────────────────────────────────────
                occ_map, binary = update_occupancy(occ_map, verts_f, cur_plane)
                obstacles       = extract_obstacles(binary)

                # ── Per-vertex colours for height mode ──────────────────────
                vert_cols = classify_verts(verts_f, cur_plane) if app_state.show_height else None

                if Cfg.VERIFY_MODE:
                    status = "ok" if raw_plane is not None else "EMA fallback"
                    print(f"[RANSAC {status}]  "
                          f"a={cur_plane[0]:.4f} b={cur_plane[1]:.4f} "
                          f"c={cur_plane[2]:.4f}  "
                          f"obstacles={len(obstacles)}")
                    for i, (xf, yl, r) in enumerate(obstacles):
                        print(f"  [{i+1}] x={xf:.3f} m  y={yl:+.3f} m  r={r:.3f} m")

            # ── Render 3-D view ──────────────────────────────────────────────
            if out_3d.shape[:2] != (h_cam, w_cam):
                out_3d  = np.zeros((h_cam, w_cam, 3), dtype=np.uint8)
                out_ref[0] = out_3d

            out_3d.fill(0)
            draw_grid(out_3d, (0, 0.5, 1), app_state)
            draw_axes(out_3d, view_transform(np.array([0, 0, 0], np.float32), app_state),
                      app_state)

            if not app_state.scale or out_3d.shape[:2] == (h_cam, w_cam):
                render_pointcloud(out_3d, verts_f, texcoords_f, color_img,
                                  app_state, vert_cols)
            else:
                tmp = np.zeros((h_cam, w_cam, 3), dtype=np.uint8)
                render_pointcloud(tmp, verts_f, texcoords_f, color_img,
                                  app_state, vert_cols)
                tmp = cv2.resize(tmp, out_3d.shape[:2][::-1],
                                 interpolation=cv2.INTER_NEAREST)
                np.putmask(out_3d, tmp > 0, tmp)

            dt = time.time() - t0
            mode_str = "height" if app_state.show_height else "RGB"
            cv2.setWindowTitle(
                WIN_3D,
                f"Point Cloud [{mode_str}]  {w_cam}x{h_cam}  "
                f"{1/dt:.0f} FPS  {'PAUSED' if app_state.paused else ''}",
            )

            # ── Render BEV ───────────────────────────────────────────────────
            bev = draw_bev(occ_map, obstacles, cur_plane)

            cv2.imshow(WIN_3D,  out_3d)
            cv2.imshow(WIN_BEV, bev)

            # ── Key handling ─────────────────────────────────────────────────
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            elif key == ord('r'):
                app_state.reset()
            elif key == ord('p'):
                app_state.paused ^= True
            elif key == ord('d'):
                app_state.decimate = (app_state.decimate + 1) % 3
                decimate.set_option(rs.option.filter_magnitude,
                                    2 ** app_state.decimate)
            elif key == ord('z'):
                app_state.scale ^= True
            elif key == ord('h'):
                app_state.show_height ^= True
                print(f"[h] height colours: {app_state.show_height}")

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("Stopped.")


if __name__ == "__main__":
    main()