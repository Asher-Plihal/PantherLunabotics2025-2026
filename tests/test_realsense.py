"""
Obstacle Detection for Navigation — RealSense D435i  (v2)
==========================================================
Detects boulders (above ground) and craters (below ground) and outputs
(x_fwd, y_lat, radius) in robot frame for a PD+CBF controller.

Robot frame convention (right-hand):
    x  →  forward  (camera z)
    y  →  left     (camera -x)
    z  →  up       (unused here)

Key improvements over v1:
  • Camera height + pitch prior seeds and constrains RANSAC — prevents the
    ground plane from drifting wildly as the robot pitches over rough terrain.
  • Exponential moving average (EMA) smooths the plane estimate over time so a
    single bumpy frame does not flip the whole occupancy map.
  • Height-map debug window is fully implemented: back-projects 3D points to
    pixel coordinates and colour-codes by classification.
  • Depth colourmap uses a FIXED MIN_DEPTH→MAX_DEPTH scale so colours are
    consistent frame-to-frame (per-frame normalisation made it useless for
    distance estimation).
  • Proper colourbar with actual distance tick marks.
  • Distance rings on BEV for spatial reference.
  • VERIFY_MODE lives in Cfg (not re-declared inside the loop every frame).
  • CLI args for camera height and pitch so you can tune without editing code.

Usage:
    python obstacle_detection.py
    python obstacle_detection.py --height 0.35 --pitch 30
    python obstacle_detection.py --height 0.35 --pitch 30 --verify
"""

import argparse

import cv2
import numpy as np
import pyrealsense2 as rs
from scipy.ndimage import center_of_mass, find_objects, gaussian_filter, label


# ─────────────────────────────────────────────────────────────────────────────
#  CONFIG
# ─────────────────────────────────────────────────────────────────────────────
class Cfg:
    # ── Camera mounting — used to derive the ground-plane prior ──────────────
    # These are the values you measure on your robot. They seed RANSAC so the
    # detector starts from a sensible estimate even on the very first frame.
    CAM_HEIGHT      = 0.30    # metres above ground
    CAM_PITCH_DEG   = 25.0    # degrees downward from horizontal

    # ── Depth range ───────────────────────────────────────────────────────────
    MIN_DEPTH       = 0.25    # m  (closer = noisy / clipped)
    MAX_DEPTH       = 5.0     # m

    # ── Height thresholds relative to the estimated ground plane ─────────────
    # Competition spec: boulders 30-40 cm dia., craters up to ~50 cm deep.
    # Thresholds are intentionally loose to catch partially-visible objects.
    BOULDER_H_MIN   = 0.08    # m above ground  →  treat as obstacle
    BOULDER_H_MAX   = 0.60    # m above ground  →  above this = ceiling / person
    CRATER_D_MIN    = 0.06    # m below ground  →  treat as obstacle
    CRATER_D_MAX    = 0.55    # m below ground  →  deeper = sensor noise / invalid

    # ── Occupancy grid ────────────────────────────────────────────────────────
    GRID_N          = 250     # cells per side
    RES             = 0.03    # m / cell  →  250 × 0.03 = 7.5 m square view
    DECAY           = 0.75    # per-frame temporal decay
    BIN_THRESH      = 0.28    # occupancy threshold for the binary map
    MIN_CLUSTER     = 15      # min grid cells for a valid obstacle cluster

    # ── RANSAC ground-plane fitting ───────────────────────────────────────────
    # Fits:  y_cam = a·x_cam + b·z_cam + c   (camera frame, y positive-down)
    # BP-1 simulant has ~2 cm gravel → inlier threshold must be looser than that.
    RANSAC_N        = 80      # iterations
    RANSAC_THRESH   = 0.05    # m — inlier distance
    RANSAC_MIN_INL  = 200     # minimum inlier count to accept a solution

    # ── Ground-plane EMA smoothing ────────────────────────────────────────────
    # new_plane = EMA * current + (1 - EMA) * ransac_result
    # Higher value → slower adaptation, more stable; lower → faster but noisier.
    PLANE_EMA       = 0.70

    # Reject a RANSAC result whose intercept (c) differs from the current
    # estimate by more than this — stops one bad frame from flipping the plane.
    PLANE_MAX_C_DELTA = 0.12  # m

    # ── Visualisation ─────────────────────────────────────────────────────────
    BEV_SCALE       = 2       # pixel scale factor for the BEV window
    SHOW_HEIGHT_MAP = True    # show height-classification debug window
    VERIFY_MODE     = False   # print per-frame debug to console


# ─────────────────────────────────────────────────────────────────────────────
#  REALSENSE INITIALISATION
# ─────────────────────────────────────────────────────────────────────────────
def init_realsense():
    pipeline = rs.pipeline()
    cfg      = rs.config()
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,  30)
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    profile      = pipeline.start(cfg)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale  = depth_sensor.get_depth_scale()

    depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
    intr          = depth_profile.get_intrinsics()

    # Post-processing chain: spatial smoothing → temporal → hole fill.
    # Skipping decimation so intrinsics stay valid for de-projection.
    spatial  = rs.spatial_filter()
    spatial.set_option(rs.option.filter_magnitude,    2)
    spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
    spatial.set_option(rs.option.filter_smooth_delta, 20)

    temporal = rs.temporal_filter()
    temporal.set_option(rs.option.filter_smooth_alpha, 0.4)
    temporal.set_option(rs.option.filter_smooth_delta, 20)

    hole_fill = rs.hole_filling_filter()
    align     = rs.align(rs.stream.color)

    print(f"[RealSense] depth scale : {depth_scale:.6f} m/unit")
    print(f"[Grid]      {Cfg.GRID_N}×{Cfg.GRID_N} cells "
          f"@ {Cfg.RES} m = {Cfg.GRID_N * Cfg.RES:.1f} m × {Cfg.GRID_N * Cfg.RES:.1f} m")

    return pipeline, depth_scale, intr, spatial, temporal, hole_fill, align


# ─────────────────────────────────────────────────────────────────────────────
#  GROUND-PLANE PRIOR FROM CAMERA MOUNTING
# ─────────────────────────────────────────────────────────────────────────────
def make_prior_plane(height_m: float, pitch_deg: float) -> np.ndarray:
    """
    Compute analytical ground-plane coefficients [a, b, c] for
        y_cam = a·x_cam + b·z_cam + c
    from measured camera height and downward pitch angle.

    Derivation:
        World ground plane: z_world = 0 (flat floor).
        Camera is at height h above the floor, pitched θ downward.
        In camera frame the ground normal is [0, −cosθ, sinθ].
        Plane equation:  −cosθ · y_cam + sinθ · z_cam = h
            → y_cam = (sinθ / cosθ) · z_cam − h / cosθ
            → a = 0,  b = tan(θ),  c = −h / cos(θ)

    With typical values (h=0.30 m, θ=25°):
        b ≈ 0.466,  c ≈ −0.331
    meaning at z=1 m the ground y_cam is about 0.135 m (downward), which
    matches the camera looking down at a near floor point.
    """
    th = np.radians(pitch_deg)
    a  = 0.0
    b  = np.tan(th)
    c  = -height_m / np.cos(th)
    return np.array([a, b, c])


# ─────────────────────────────────────────────────────────────────────────────
#  DEPTH → POINT CLOUD  (camera frame)
# ─────────────────────────────────────────────────────────────────────────────
# Camera frame (RealSense convention):
#   x → right,  y → DOWN,  z → forward
def depth_to_pointcloud(
    depth_img: np.ndarray,
    depth_scale: float,
    intr,
) -> tuple[np.ndarray, np.ndarray]:
    """
    De-project every valid depth pixel into 3-D camera-frame coordinates.

    Returns
    -------
    pts : (N, 3) float32  [x_cam, y_cam, z_cam]
    pix : (N, 2) int32    [u, v]  — pixel coordinates, same index as pts,
                                    used to back-project classifications onto
                                    the image in draw_height_map().
    """
    h, w = depth_img.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))

    z     = depth_img.astype(np.float32) * depth_scale
    valid = (z > Cfg.MIN_DEPTH) & (z < Cfg.MAX_DEPTH)

    z = z[valid];  u = u[valid];  v = v[valid]
    x = (u - intr.ppx) / intr.fx * z
    y = (v - intr.ppy) / intr.fy * z

    pts = np.stack([x, y, z], axis=1).astype(np.float32)   # (N, 3)
    pix = np.stack([u, v],    axis=1).astype(np.int32)      # (N, 2)
    return pts, pix


# ─────────────────────────────────────────────────────────────────────────────
#  GROUND-PLANE ESTIMATION  (RANSAC + EMA smoothing)
# ─────────────────────────────────────────────────────────────────────────────
def estimate_ground_plane(
    points: np.ndarray,
    prior_plane: np.ndarray,
) -> np.ndarray | None:
    """
    RANSAC fit of  y_cam = a·x_cam + b·z_cam + c.

    Two ways the prior helps robustness:
      1. Candidate sampling is biased toward points near the prior plane,
         so each 3-point sample is likely to actually be on the ground.
      2. Solutions whose intercept (c) deviates too far from the prior are
         rejected outright — this prevents a single tilted frame or a large
         boulder filling the view from flipping the estimate.

    Returns refined [a, b, c] or None if no good solution was found.
    The caller should then fall back to the EMA-smoothed current estimate.
    """
    if len(points) < 500:
        return None

    x, y, z = points[:, 0], points[:, 1], points[:, 2]

    # Candidate points: close to the prior plane, or in the lower y percentile.
    a0, b0, c0 = prior_plane
    near  = np.abs(y - (a0 * x + b0 * z + c0)) < Cfg.RANSAC_THRESH * 3
    lower = y > np.percentile(y, 25)

    xc = x[near | lower];  yc = y[near | lower];  zc = z[near | lower]
    if len(xc) < 100:
        return None

    best_inl   = 0
    best_coeff = None
    rng        = np.random.default_rng()   # unseeded for variety each call

    for _ in range(Cfg.RANSAC_N):
        s = rng.choice(len(xc), 3, replace=False)
        A = np.stack([xc[s], zc[s], np.ones(3)], axis=1)

        try:
            coeff = np.linalg.solve(A, yc[s])
        except np.linalg.LinAlgError:
            continue

        # Reject solutions that have drifted far from the prior intercept.
        if abs(coeff[2] - prior_plane[2]) > Cfg.PLANE_MAX_C_DELTA * 3:
            continue

        y_pred  = coeff[0] * x + coeff[1] * z + coeff[2]
        inliers = int(np.sum(np.abs(y - y_pred) < Cfg.RANSAC_THRESH))

        if inliers > best_inl:
            best_inl   = inliers
            best_coeff = coeff

    if best_coeff is None or best_inl < Cfg.RANSAC_MIN_INL:
        return None

    # Secondary sanity: intercept still within allowed range.
    if abs(best_coeff[2] - prior_plane[2]) > Cfg.PLANE_MAX_C_DELTA * 2:
        return None

    # Least-squares refinement on all inliers.
    y_pred_all = best_coeff[0] * x + best_coeff[1] * z + best_coeff[2]
    mask       = np.abs(y - y_pred_all) < Cfg.RANSAC_THRESH
    if mask.sum() < 100:
        return None

    A_ref, _, _, _ = np.linalg.lstsq(
        np.stack([x[mask], z[mask], np.ones(mask.sum())], axis=1),
        y[mask], rcond=None,
    )
    return A_ref


def smooth_plane(current: np.ndarray, new: np.ndarray | None) -> np.ndarray:
    """EMA blend of plane coefficients. Returns current unchanged if new is None."""
    if new is None:
        return current
    return Cfg.PLANE_EMA * current + (1.0 - Cfg.PLANE_EMA) * new


def height_above_ground(points: np.ndarray, plane: np.ndarray) -> np.ndarray:
    """
    Signed height of each point above the ground plane [metres].
        positive → above ground (potential boulder)
        negative → below ground (potential crater)

    In camera frame y is positive-downward, so:
        height = y_ground_at(x,z) − y_point
    A point physically above the ground has a smaller (less positive) y_cam
    than the ground at the same (x,z), so this difference is positive. ✓
    """
    a, b, c  = plane
    y_ground = a * points[:, 0] + b * points[:, 2] + c
    return y_ground - points[:, 1]


# ─────────────────────────────────────────────────────────────────────────────
#  OCCUPANCY GRID
# ─────────────────────────────────────────────────────────────────────────────
# Grid layout:
#   row 0     = farthest forward (z = MAX_DEPTH)
#   row N//2  = camera position  (z = 0)
#   col N//2  = directly ahead   (y_robot = 0)
#   col < N//2 = left,  col > N//2 = right
#
# robot frame:  x_r = z_cam  (forward),  y_r = −x_cam  (left)
def update_occupancy(
    occ_map: np.ndarray,
    points:  np.ndarray,
    plane:   np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    h = height_above_ground(points, plane)

    obstacle_mask = (
        ((h >  Cfg.BOULDER_H_MIN) & (h <  Cfg.BOULDER_H_MAX)) |
        ((h < -Cfg.CRATER_D_MIN)  & (h > -Cfg.CRATER_D_MAX))
    )
    obs = points[obstacle_mask]

    new_layer = np.zeros_like(occ_map)

    if len(obs) > 0:
        x_r = obs[:, 2]           # robot forward = camera z
        y_r = -obs[:, 0]          # robot left    = −camera x

        row = (Cfg.GRID_N // 2 - (x_r / Cfg.RES)).astype(int)
        col = (Cfg.GRID_N // 2 + (y_r / Cfg.RES)).astype(int)

        valid = (row >= 0) & (row < Cfg.GRID_N) & (col >= 0) & (col < Cfg.GRID_N)
        np.add.at(new_layer, (row[valid], col[valid]), 1.0)

        if new_layer.max() > 0:
            new_layer = np.clip(new_layer / 8.0, 0.0, 1.0)
        new_layer = gaussian_filter(new_layer, sigma=1.2)

    occ_map = Cfg.DECAY * occ_map + (1.0 - Cfg.DECAY) * new_layer
    return occ_map, (occ_map > Cfg.BIN_THRESH)


# ─────────────────────────────────────────────────────────────────────────────
#  OBSTACLE EXTRACTION
# ─────────────────────────────────────────────────────────────────────────────
def extract_obstacles(binary_map: np.ndarray) -> list[tuple[float, float, float]]:
    """
    Label connected components in the binary occupancy map and return a list of
        (x_fwd [m],  y_lat [m],  radius [m])
    in robot frame, sorted by distance (closest first), ready for a CBF controller.
    """
    labeled, n = label(binary_map)
    if n == 0:
        return []

    obstacles = []
    for i, sl in enumerate(find_objects(labeled)):
        region = labeled[sl] == (i + 1)
        size   = int(region.sum())
        if size < Cfg.MIN_CLUSTER:
            continue

        cy, cx = center_of_mass(region)
        row = sl[0].start + cy
        col = sl[1].start + cx

        x_fwd = (Cfg.GRID_N // 2 - row) * Cfg.RES
        y_lat = (col - Cfg.GRID_N // 2) * Cfg.RES
        radius = max(float(np.sqrt(size / np.pi) * Cfg.RES), 0.05)

        if x_fwd > 0.1:   # only keep obstacles in front of the robot
            obstacles.append((float(x_fwd), float(y_lat), radius))

    obstacles.sort(key=lambda o: o[0])
    return obstacles


# ─────────────────────────────────────────────────────────────────────────────
#  VISUALISATION
# ─────────────────────────────────────────────────────────────────────────────
_P = {
    'bg':       (18,  18,  18),
    'grid':     (35,  35,  35),
    'axis':     (55,  55,  55),
    'ring':     (48,  48,  48),
    'obs_ring': (0,  160, 255),
    'obs_dot':  (0,  220, 255),
    'label':    (255, 220,  80),
    'robot':    (0,  255, 100),
    'forward':  (100, 255, 100),
}


def draw_bev(
    occ_map:    np.ndarray,
    binary_map: np.ndarray,
    obstacles:  list[tuple[float, float, float]],
) -> np.ndarray:
    """Bird's-eye-view with heatmap, distance rings, and labelled obstacle circles."""
    S  = Cfg.BEV_SCALE
    N  = Cfg.GRID_N
    sz = N * S
    mid = (N // 2) * S

    canvas = np.full((sz, sz, 3), _P['bg'], dtype=np.uint8)

    # ── Soft occupancy heatmap ──────────────────────────────────────────────
    heat = cv2.resize(
        (occ_map * 255).astype(np.uint8), (sz, sz),
        interpolation=cv2.INTER_NEAREST,
    )
    heat_bgr = cv2.applyColorMap(heat, cv2.COLORMAP_OCEAN)
    canvas[heat > 15] = heat_bgr[heat > 15]

    # ── 1 m grid lines ──────────────────────────────────────────────────────
    cells_per_m = max(1, int(1.0 / Cfg.RES))
    for i in range(0, N, cells_per_m):
        p = i * S
        cv2.line(canvas, (0, p), (sz, p), _P['grid'], 1)
        cv2.line(canvas, (p, 0), (p, sz), _P['grid'], 1)

    # ── Distance rings ──────────────────────────────────────────────────────
    for d_m in range(1, int(Cfg.MAX_DEPTH) + 1):
        r_px = int(d_m / Cfg.RES * S)
        cv2.circle(canvas, (mid, mid), r_px, _P['ring'], 1)
        label_y = mid - r_px
        if 4 < label_y < sz - 4:
            cv2.putText(canvas, f"{d_m}m", (mid + 4, label_y + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, (75, 75, 75), 1, cv2.LINE_AA)

    # ── Central axes ────────────────────────────────────────────────────────
    cv2.line(canvas, (mid, 0), (mid, sz), _P['axis'], 1)
    cv2.line(canvas, (0, mid), (sz, mid), _P['axis'], 1)

    # ── Obstacle circles ────────────────────────────────────────────────────
    for xf, yl, r in obstacles:
        row  = int((N // 2 - xf / Cfg.RES) * S)
        col  = int((N // 2 + yl / Cfg.RES) * S)
        r_px = max(5, int(r / Cfg.RES * S))

        cv2.circle(canvas, (col, row), r_px + 3, _P['obs_ring'], 2)
        cv2.circle(canvas, (col, row), 3,         _P['obs_dot'],  -1)
        cv2.putText(canvas, f"({xf:.1f},{yl:+.1f})",
                    (col + r_px + 4, row + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, _P['label'], 1, cv2.LINE_AA)

    # ── Robot marker (triangle pointing forward = up) ────────────────────────
    tri = np.array([[mid, mid - 10], [mid - 7, mid + 7], [mid + 7, mid + 7]])
    cv2.fillPoly(canvas, [tri], _P['robot'])

    # ── Text overlays ────────────────────────────────────────────────────────
    cv2.putText(canvas, "FWD", (mid - 15, 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, _P['forward'], 1, cv2.LINE_AA)
    cv2.putText(canvas,
                f"{N * Cfg.RES:.0f}m x {N * Cfg.RES:.0f}m  |  {Cfg.RES*100:.0f}cm/cell",
                (4, sz - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, (100, 100, 100), 1, cv2.LINE_AA)
    cv2.putText(canvas, f"obstacles: {len(obstacles)}", (4, 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 180), 1, cv2.LINE_AA)

    return canvas


def draw_height_map(
    pts:   np.ndarray,
    pix:   np.ndarray,
    plane: np.ndarray,
    shape: tuple[int, int],
) -> np.ndarray:
    """
    Colour every valid depth pixel by its height classification:
        Green  = ground    (within ±RANSAC_THRESH of plane)
        Red    = boulder   (above BOULDER_H_MIN)
        Blue   = crater    (below CRATER_D_MIN)
        Dark grey = in range but unclassified

    pts  and  pix  share the same index ordering from depth_to_pointcloud(),
    so we can scatter classifications directly to pixel positions.
    """
    h_img, w_img = shape
    vis = np.full((h_img, w_img, 3), 20, dtype=np.uint8)   # very dark background

    if plane is None or len(pts) == 0:
        return vis

    heights = height_above_ground(pts, plane)
    us      = pix[:, 0]
    vs      = pix[:, 1]

    # Validity mask (should always be true, but guard against edge cases)
    valid = (us >= 0) & (us < w_img) & (vs >= 0) & (vs < h_img)
    us = us[valid];  vs = vs[valid];  heights = heights[valid]

    # ── Assign colours per class ─────────────────────────────────────────────
    # Process in order: ground last so it overpaints anything under the threshold.
    is_boulder = (heights >  Cfg.BOULDER_H_MIN) & (heights <  Cfg.BOULDER_H_MAX)
    is_crater  = (heights < -Cfg.CRATER_D_MIN)  & (heights > -Cfg.CRATER_D_MAX)
    is_ground  = np.abs(heights) < Cfg.RANSAC_THRESH

    # Unclassified in-range points: scale grey by height for visual depth cue
    other = ~is_ground & ~is_boulder & ~is_crater
    h_norm = np.clip((heights[other] + 0.5) / 1.0, 0.0, 1.0)
    grey   = (h_norm * 90 + 20).astype(np.uint8)
    vis[vs[other],   us[other]]   = np.stack([grey, grey, grey], axis=1)

    vis[vs[is_crater],  us[is_crater]]  = (200,  60,   0)   # blue-ish
    vis[vs[is_boulder], us[is_boulder]] = (0,    60, 210)   # red-ish  (BGR)
    vis[vs[is_ground],  us[is_ground]]  = (50,  160,  60)   # green

    # ── Legend ───────────────────────────────────────────────────────────────
    legend = [
        ("Ground",  (50,  160,  60)),
        ("Boulder", (0,    60, 210)),
        ("Crater",  (200,  60,   0)),
    ]
    for row_i, (lbl, col) in enumerate(legend):
        y0 = 16 + row_i * 18
        cv2.rectangle(vis, (4, y0 - 11), (15, y0), col, -1)
        cv2.putText(vis, lbl, (20, y0),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (210, 210, 210), 1, cv2.LINE_AA)

    # ── Plane stats overlay ──────────────────────────────────────────────────
    pct_ground  = 100.0 * is_ground.sum()  / max(len(heights), 1)
    pct_boulder = 100.0 * is_boulder.sum() / max(len(heights), 1)
    pct_crater  = 100.0 * is_crater.sum()  / max(len(heights), 1)
    cv2.putText(vis,
                f"gnd {pct_ground:.0f}%  bldr {pct_boulder:.1f}%  crtr {pct_crater:.1f}%",
                (4, h_img - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, (140, 140, 140), 1, cv2.LINE_AA)

    return vis


def draw_depth_map(depth_img: np.ndarray, depth_scale: float) -> np.ndarray:
    """
    Depth image with a FIXED colour scale (MIN_DEPTH → MAX_DEPTH) so colours
    are stable across frames.  Near = warm (red/yellow), far = cool (blue).
    Invalid / out-of-range pixels are shown as black.

    A colour bar with distance tick-marks is appended on the right.
    """
    depth_m = depth_img.astype(np.float32) * depth_scale

    # Normalise to [0, 1] within the configured range.
    depth_norm = np.clip(
        (depth_m - Cfg.MIN_DEPTH) / (Cfg.MAX_DEPTH - Cfg.MIN_DEPTH),
        0.0, 1.0,
    )

    # Invert so near = high value in JET → warm colours (natural intuition).
    depth_u8 = ((1.0 - depth_norm) * 255).astype(np.uint8)

    # Black out pixels that are outside the valid range.
    depth_u8[(depth_img == 0) | (depth_m < Cfg.MIN_DEPTH) | (depth_m > Cfg.MAX_DEPTH)] = 0

    colored = cv2.applyColorMap(depth_u8, cv2.COLORMAP_JET)

    # ── Colour bar ───────────────────────────────────────────────────────────
    bar_h = colored.shape[0]
    bar_w = 46

    # Gradient: top = far (cold/blue), bottom = near (warm/red)  →  0..255
    grad = np.linspace(0, 255, bar_h, dtype=np.uint8).reshape(-1, 1)
    grad = np.repeat(grad, bar_w, axis=1)
    bar  = cv2.applyColorMap(grad, cv2.COLORMAP_JET)

    # Tick marks every 0.5 m
    depth_range = Cfg.MAX_DEPTH - Cfg.MIN_DEPTH
    n_ticks     = int(depth_range / 0.5) + 1
    for i in range(n_ticks):
        d     = Cfg.MIN_DEPTH + i * 0.5
        # Near = bottom of bar (frac = 0), far = top (frac = 1).
        frac  = (d - Cfg.MIN_DEPTH) / depth_range
        tick_y = int((1.0 - frac) * (bar_h - 1))
        cv2.line(bar, (0, tick_y), (8, tick_y), (255, 255, 255), 1)
        label_str = f"{d:.1f}m"
        cv2.putText(bar, label_str, (10, tick_y + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.30, (255, 255, 255), 1, cv2.LINE_AA)

    cv2.putText(bar, "near", (2, bar_h - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.28, (200, 200, 200), 1)
    cv2.putText(bar, "far",  (4, 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.28, (200, 200, 200), 1)

    return np.hstack([colored, bar])


def draw_rgb_overlay(
    rgb:       np.ndarray,
    obstacles: list[tuple[float, float, float]],
    intr,
) -> np.ndarray:
    """Project detected obstacle positions back onto the colour image."""
    out = rgb.copy()
    for xf, yl, r in obstacles:
        z_cam = xf
        x_cam = -yl

        if z_cam < Cfg.MIN_DEPTH:
            continue

        u    = int(x_cam / z_cam * intr.fx + intr.ppx)
        v    = int(0.0   / z_cam * intr.fy + intr.ppy)   # project at ground level
        r_px = max(6, int(r / z_cam * intr.fx))

        if 0 <= u < out.shape[1] and 0 <= v < out.shape[0]:
            cv2.circle(out, (u, v), r_px, (0,  80, 255), 2)
            cv2.circle(out, (u, v), 3,    (0, 200, 255), -1)
            cv2.putText(out, f"{xf:.1f}m", (u + r_px + 2, v),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 200), 1, cv2.LINE_AA)
    return out


# ─────────────────────────────────────────────────────────────────────────────
#  MAIN LOOP
# ─────────────────────────────────────────────────────────────────────────────
def main() -> None:
    parser = argparse.ArgumentParser(
        description="RealSense D435i obstacle detector — BEV output for CBF controller",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--height", type=float, default=Cfg.CAM_HEIGHT,
                        help="Camera height above ground (metres)")
    parser.add_argument("--pitch",  type=float, default=Cfg.CAM_PITCH_DEG,
                        help="Camera downward pitch (degrees)")
    parser.add_argument("--verify", action="store_true",
                        help="Print per-frame debug info to console")
    args = parser.parse_args()

    Cfg.CAM_HEIGHT    = args.height
    Cfg.CAM_PITCH_DEG = args.pitch
    Cfg.VERIFY_MODE   = args.verify

    pipeline, depth_scale, intr, spatial, temporal, hole_fill, align = init_realsense()

    occ_map   = np.zeros((Cfg.GRID_N, Cfg.GRID_N), dtype=np.float32)
    prior     = make_prior_plane(Cfg.CAM_HEIGHT, Cfg.CAM_PITCH_DEG)
    cur_plane = prior.copy()

    print(f"\n[Prior plane]  a={prior[0]:.4f}  b={prior[1]:.4f}  c={prior[2]:.4f}")
    print(f"               height={Cfg.CAM_HEIGHT} m   pitch={Cfg.CAM_PITCH_DEG}°")
    print("\nRunning — press  q  to quit.")
    print("CBF outputs: (x_fwd [m],  y_lat [m],  radius [m])\n")

    try:
        while True:
            frames  = pipeline.wait_for_frames()
            aligned = align.process(frames)

            depth_frame = aligned.get_depth_frame()
            color_frame = aligned.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # ── Post-process depth ──────────────────────────────────────────
            depth_frame = spatial.process(depth_frame)
            depth_frame = temporal.process(depth_frame)
            depth_frame = hole_fill.process(depth_frame)

            depth = np.asanyarray(depth_frame.get_data())
            rgb   = np.asanyarray(color_frame.get_data())

            # ── 1. Point cloud (camera frame) ────────────────────────────────
            pts, pix = depth_to_pointcloud(depth, depth_scale, intr)
            if len(pts) < 500:
                continue

            # ── 2. Ground plane — RANSAC biased by prior + EMA smoothing ─────
            raw_plane = estimate_ground_plane(pts, cur_plane)
            cur_plane = smooth_plane(cur_plane, raw_plane)

            # ── 3. Occupancy grid ────────────────────────────────────────────
            occ_map, binary = update_occupancy(occ_map, pts, cur_plane)

            # ── 4. Obstacle cluster extraction ──────────────────────────────
            obstacles = extract_obstacles(binary)

            # ── 5. Console output ────────────────────────────────────────────
            if Cfg.VERIFY_MODE:
                ransac_status = "RANSAC ok" if raw_plane is not None else "RANSAC failed (using EMA)"
                print(f"\n[{ransac_status}]  "
                      f"a={cur_plane[0]:.4f}  b={cur_plane[1]:.4f}  c={cur_plane[2]:.4f}  "
                      f"(inferred cam height ≈ {-cur_plane[2]:.3f} m)")
                for i, (xf, yl, r) in enumerate(obstacles):
                    print(f"  obs {i+1}: x_fwd={xf:.3f} m  y_lat={yl:+.3f} m  r={r:.3f} m")

            if obstacles:
                lines = [f"  [{i+1}] x={xf:.3f} m  y={yl:+.3f} m  r={r:.3f} m"
                         for i, (xf, yl, r) in enumerate(obstacles)]
                print("Obstacles:\n" + "\n".join(lines))
            else:
                print("Obstacles: none")

            # ── 6. Render windows ────────────────────────────────────────────
            bev       = draw_bev(occ_map, binary, obstacles)
            rgb_out   = draw_rgb_overlay(rgb, obstacles, intr)
            depth_vis = draw_depth_map(depth, depth_scale)

            cv2.imshow("Bird's-Eye View",  bev)
            cv2.imshow("RGB + Obstacles",  rgb_out)
            cv2.imshow("Depth Map",        depth_vis)

            if Cfg.SHOW_HEIGHT_MAP:
                h_map = draw_height_map(pts, pix, cur_plane, depth.shape)
                cv2.imshow("Height Map (debug)", h_map)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("Stopped.")


if __name__ == "__main__":
    main()