"""
Obstacle Detection for Navigation — RealSense D435i
====================================================
Detects boulders (above ground) and craters (below ground) and outputs
(x_fwd, y_lat, radius) in robot frame for use with a PD+CBF controller.

Robot frame convention (right-hand):
    x  →  forward  (camera z)
    y  →  left     (camera -x)
    z  →  up       (-camera y, unused here)

Bird's-eye view:
    Camera/robot sits at bottom-centre.
    Forward = up in the image.
    Lateral = left / right.
"""

import pyrealsense2 as rs
import numpy as np
import cv2
from scipy.ndimage import gaussian_filter, label, find_objects, center_of_mass

# ─────────────────────────────────────────────
#  CONFIG
# ─────────────────────────────────────────────
class Cfg:
    # Depth range
    MIN_DEPTH        = 0.25   # m  (very close = noisy)
    MAX_DEPTH        = 4.0    # m

    # Height thresholds relative to ground plane
    BOULDER_H_MIN    = 0.3   # m above ground  → obstacle
    BOULDER_H_MAX    = 0.5    # m above ground  → above this = ceiling / person
    CRATER_D_MIN     = 0.3   # m below ground  → obstacle
    CRATER_D_MAX     = 0.5   # m below ground  → deeper = invalid

    # Occupancy grid
    GRID_N           = 250    # cells per side
    RES              = 0.03   # m / cell  → 250 × 0.03 = 7.5 m square view
    DECAY            = 0.72   # per-frame temporal decay (lower = more reactive)
    BIN_THRESH       = 0.28   # threshold for binary obstacle map
    MIN_CLUSTER      = 20     # min grid cells to count as a real obstacle

    # RANSAC (ground plane in camera frame: y = a·x + b·z + c)
    RANSAC_N         = 60
    RANSAC_THRESH    = 0.025  # m  inlier distance
    RANSAC_MIN_INL   = 250

    # Viz
    BEV_SCALE        = 2      # pixel scale factor for BEV window
    SHOW_HEIGHT_MAP  = True   # show height-coloured debug window


# ─────────────────────────────────────────────
#  REALSENSE INITIALISATION
# ─────────────────────────────────────────────
def init_realsense():
    pipeline = rs.pipeline()
    config   = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,  30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    profile      = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale  = depth_sensor.get_depth_scale()

    # Camera intrinsics (from the depth stream, used for de-projection)
    depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
    intr          = depth_profile.get_intrinsics()

    # Post-processing filters — improve depth quality without changing resolution
    # (spatial + temporal + hole_filling; skip decimation to keep intrinsics valid)
    spatial  = rs.spatial_filter()
    spatial.set_option(rs.option.filter_magnitude,    2)
    spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
    spatial.set_option(rs.option.filter_smooth_delta, 20)

    temporal = rs.temporal_filter()
    temporal.set_option(rs.option.filter_smooth_alpha, 0.4)
    temporal.set_option(rs.option.filter_smooth_delta, 20)

    hole_fill = rs.hole_filling_filter()

    align = rs.align(rs.stream.color)   # align depth → colour frame

    print(f"[RealSense] depth scale : {depth_scale:.6f} m/unit")
    print(f"[Grid]      {Cfg.GRID_N}×{Cfg.GRID_N} cells "
          f"@ {Cfg.RES} m = {Cfg.GRID_N * Cfg.RES:.1f} m × {Cfg.GRID_N * Cfg.RES:.1f} m")

    return pipeline, depth_scale, intr, spatial, temporal, hole_fill, align


# ─────────────────────────────────────────────
#  DEPTH → POINT CLOUD  (camera frame)
# ─────────────────────────────────────────────
# Camera frame axes (RealSense convention):
#   x → right,  y → DOWN,  z → forward
#
def depth_to_pointcloud(depth_img, depth_scale, intr):
    """Return (N,3) array of [x_cam, y_cam, z_cam] for valid depth pixels."""
    h, w = depth_img.shape
    u, v  = np.meshgrid(np.arange(w), np.arange(h))

    z = depth_img.astype(np.float32) * depth_scale
    valid = (z > Cfg.MIN_DEPTH) & (z < Cfg.MAX_DEPTH)

    z = z[valid];  u = u[valid];  v = v[valid]
    x = (u - intr.ppx) / intr.fx * z   # right  (+x = right)
    y = (v - intr.ppy) / intr.fy * z   # down   (+y = down)

    return np.stack([x, y, z], axis=1)  # shape (N, 3)


# ─────────────────────────────────────────────
#  GROUND PLANE  (RANSAC in camera frame)
# ─────────────────────────────────────────────
# We fit the plane  y_cam = a·x_cam + b·z_cam + c
# (ground has roughly constant y_cam for flat terrain)
# NOTE: y_cam is the vertical axis (positive = down toward ground).
#
def estimate_ground_plane(points):
    """
    RANSAC fit of y = a·x + b·z + c in camera frame.
    Returns (a, b, c) or None on failure.
    """
    if len(points) < 500:
        return None

    x, y, z = points[:, 0], points[:, 1], points[:, 2]

    # Restrict to mid-to-lower rows of scene (likely ground, not sky)
    lower_half = y > np.percentile(y, 30)
    x, y, z = x[lower_half], y[lower_half], z[lower_half]

    if len(x) < 200:
        return None

    best_inl   = 0
    best_coeff = None
    rng        = np.random.default_rng(0)

    for _ in range(Cfg.RANSAC_N):
        s  = rng.choice(len(x), 3, replace=False)
        A  = np.stack([x[s], z[s], np.ones(3)], axis=1)
        b_ = y[s]

        try:
            coeff = np.linalg.solve(A, b_)
        except np.linalg.LinAlgError:
            continue

        y_pred  = coeff[0] * x + coeff[1] * z + coeff[2]
        inliers = np.sum(np.abs(y - y_pred) < Cfg.RANSAC_THRESH)

        if inliers > best_inl:
            best_inl   = inliers
            best_coeff = coeff

    if best_coeff is None or best_inl < Cfg.RANSAC_MIN_INL:
        return None   # plane not found; caller uses fallback

    # Refine with all inliers
    a, b, c    = best_coeff
    y_pred_all = a * x + b * z + c
    mask       = np.abs(y - y_pred_all) < Cfg.RANSAC_THRESH
    if mask.sum() < 100:
        return None

    A_ref   = np.stack([x[mask], z[mask], np.ones(mask.sum())], axis=1)
    coeff_r, _, _, _ = np.linalg.lstsq(A_ref, y[mask], rcond=None)
    return coeff_r   # [a, b, c]


def height_above_ground(points, plane):
    """
    Signed height of each point above the ground plane (positive = above).
    In camera frame y is DOWN, so:
        height = y_ground_at(x,z)  -  y_point
    Positive → point is above ground (boulder).
    Negative → point is below ground (crater).
    """
    a, b, c   = plane
    y_ground  = a * points[:, 0] + b * points[:, 2] + c
    return y_ground - points[:, 1]   # positive = above ground


# ─────────────────────────────────────────────
#  OCCUPANCY MAP  (bird's eye view, camera-centred)
# ─────────────────────────────────────────────
# Grid layout:
#   Row 0   = farthest forward  (z = MAX_DEPTH)
#   Row N/2 = camera position   (z = 0)
#   Row N   = behind camera
#   Col N/2 = directly ahead    (x = 0)
#   Col < N/2 = left,  Col > N/2 = right
#
# robot forward  x_r = z_cam
# robot left     y_r = -x_cam
#
def update_occupancy(occ_map, points, plane):
    """
    Update occupancy map with current obstacle points.
    Returns (updated_map, binary_map).
    """
    h = height_above_ground(points, plane)

    # Select boulders (above ground) and craters (below ground)
    obstacle_mask = (
        ((h >  Cfg.BOULDER_H_MIN) & (h <  Cfg.BOULDER_H_MAX)) |
        ((h < -Cfg.CRATER_D_MIN)  & (h > -Cfg.CRATER_D_MAX))
    )
    obs = points[obstacle_mask]

    new_layer = np.zeros_like(occ_map)

    if len(obs) > 0:
        x_r = obs[:, 2]          # forward  (z_cam)
        y_r = -obs[:, 0]         # left     (-x_cam)

        # Map to grid indices
        row = (Cfg.GRID_N // 2 - (x_r / Cfg.RES)).astype(int)
        col = (Cfg.GRID_N // 2 + (y_r / Cfg.RES)).astype(int)   # left = col < N/2

        valid = (row >= 0) & (row < Cfg.GRID_N) & (col >= 0) & (col < Cfg.GRID_N)
        row, col = row[valid], col[valid]

        # Count hits per cell then normalise
        np.add.at(new_layer, (row, col), 1.0)
        if new_layer.max() > 0:
            new_layer = np.clip(new_layer / 8.0, 0.0, 1.0)

        new_layer = gaussian_filter(new_layer, sigma=1.2)

    # Temporal integration with decay
    occ_map = Cfg.DECAY * occ_map + (1.0 - Cfg.DECAY) * new_layer

    return occ_map, (occ_map > Cfg.BIN_THRESH)


# ─────────────────────────────────────────────
#  OBSTACLE EXTRACTION
# ─────────────────────────────────────────────
def extract_obstacles(binary_map):
    """
    Label connected components and return a list of
        (x_fwd, y_lat, radius)   [metres, robot frame]
    ready to feed into your CBF controller.
    """
    labeled, n = label(binary_map)
    if n == 0:
        return []

    obstacles = []
    slices    = find_objects(labeled)

    for i, sl in enumerate(slices):
        region = labeled[sl] == (i + 1)
        size   = int(region.sum())

        if size < Cfg.MIN_CLUSTER:
            continue

        cy, cx = center_of_mass(region)
        row    = sl[0].start + cy
        col    = sl[1].start + cx

        x_fwd = (Cfg.GRID_N // 2 - row) * Cfg.RES   # forward (m)
        y_lat = (col - Cfg.GRID_N // 2) * Cfg.RES   # left positive (m)

        # Estimated obstacle radius from cluster area
        radius = float(np.sqrt(size / np.pi) * Cfg.RES)
        radius = max(radius, 0.05)

        # Only keep obstacles in front of the camera
        if x_fwd > 0.1:
            obstacles.append((float(x_fwd), float(y_lat), radius))

    # Sort by proximity (closest first)
    obstacles.sort(key=lambda o: o[0])
    return obstacles


# ─────────────────────────────────────────────
#  VISUALISATION
# ─────────────────────────────────────────────
_PALETTE = {
    'bg':       (18,  18,  18),
    'grid':     (35,  35,  35),
    'axis':     (55,  55,  55),
    'occupied': (45,  60,  80),
    'obs_ring': (0,  160, 255),
    'obs_dot':  (0,  220, 255),
    'label':    (255, 220,  80),
    'robot':    (0,  255, 100),
    'forward':  (100, 255, 100),
}


def draw_bev(occ_map, binary_map, obstacles):
    """Render a clean bird's-eye-view window."""
    S  = Cfg.BEV_SCALE
    N  = Cfg.GRID_N
    sz = N * S

    canvas = np.full((sz, sz, 3), _PALETTE['bg'], dtype=np.uint8)

    # ── Soft occupancy heatmap ──
    heat = (occ_map * 255).astype(np.uint8)
    heat = cv2.resize(heat, (sz, sz), interpolation=cv2.INTER_NEAREST)
    heat_bgr = cv2.applyColorMap(heat, cv2.COLORMAP_OCEAN)
    mask2d   = heat > 15
    canvas[mask2d] = heat_bgr[mask2d]

    # ── Grid lines every 1 m ──
    cells_per_m = max(1, int(1.0 / Cfg.RES))
    for i in range(0, N, cells_per_m):
        p = i * S
        cv2.line(canvas, (0, p), (sz, p), _PALETTE['grid'], 1)
        cv2.line(canvas, (p, 0), (p, sz), _PALETTE['grid'], 1)

    # ── Central axes ──
    mid = (N // 2) * S
    cv2.line(canvas, (mid, 0), (mid, sz), _PALETTE['axis'], 1)
    cv2.line(canvas, (0, mid), (sz, mid), _PALETTE['axis'], 1)

    # ── Obstacle circles ──
    for (xf, yl, r) in obstacles:
        row  = int((N // 2 - xf / Cfg.RES) * S)
        col  = int((N // 2 + yl / Cfg.RES) * S)
        r_px = max(5, int(r / Cfg.RES * S))

        cv2.circle(canvas, (col, row), r_px + 3, _PALETTE['obs_ring'], 2)
        cv2.circle(canvas, (col, row), 3,         _PALETTE['obs_dot'],  -1)

        txt = f"({xf:.1f}, {yl:+.1f})"
        cv2.putText(canvas, txt, (col + r_px + 4, row + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, _PALETTE['label'], 1, cv2.LINE_AA)

    # ── Robot marker (triangle pointing forward = up) ──
    cx, cy_r = mid, mid
    tri = np.array([[cx, cy_r - 10], [cx - 7, cy_r + 7], [cx + 7, cy_r + 7]])
    cv2.fillPoly(canvas, [tri], _PALETTE['robot'])

    # ── Text overlays ──
    cv2.putText(canvas, "FWD", (mid - 15, 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, _PALETTE['forward'], 1, cv2.LINE_AA)
    footer = f"{N * Cfg.RES:.0f}m x {N * Cfg.RES:.0f}m  |  {Cfg.RES*100:.0f}cm/cell"
    cv2.putText(canvas, footer, (4, sz - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, (100, 100, 100), 1, cv2.LINE_AA)

    n_obs = len(obstacles)
    cv2.putText(canvas, f"obstacles: {n_obs}", (4, 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 180), 1, cv2.LINE_AA)

    return canvas


def draw_height_debug(points, plane, depth_shape):
    """
    Colour-code each pixel by its height above ground.
    Green  = ground,  Blue = crater,  Red/Yellow = obstacle.
    Useful to verify ground removal is working correctly.
    """
    h_img = np.zeros(depth_shape, dtype=np.float32)
    h_img[:] = np.nan

    if plane is not None and len(points) > 0:
        heights = height_above_ground(points, plane)
        # We need to rebuild pixel indices — reconstruct from pointcloud
        # Use a simple height normalisation for display
        h_norm  = np.clip((heights + 0.4) / 1.4, 0.0, 1.0)   # −0.4 → 1.0 m range

    # Colourmap: just show height as a colour overlay on a blank canvas
    vis = np.zeros((depth_shape[0], depth_shape[1], 3), dtype=np.uint8)
    return vis   # placeholder — see notes below


def draw_rgb_overlay(rgb, obstacles, intr):
    """
    Project detected obstacles back onto the RGB frame.
    Each obstacle is drawn as a circle at its estimated image position.
    """
    out = rgb.copy()

    for (xf, yl, r) in obstacles:
        z_cam  =  xf          # camera z = robot forward
        x_cam  = -yl          # camera x = -robot left
        y_cam  =  0.0         # project at ground level (approx)

        if z_cam < Cfg.MIN_DEPTH:
            continue

        u = int(x_cam / z_cam * intr.fx + intr.ppx)
        v = int(y_cam / z_cam * intr.fy + intr.ppy)
        r_px = max(6, int(r / z_cam * intr.fx))

        if 0 <= u < out.shape[1] and 0 <= v < out.shape[0]:
            cv2.circle(out, (u, v), r_px, (0, 80, 255), 2)
            cv2.circle(out, (u, v), 3,    (0, 200, 255), -1)
            dist_txt = f"{xf:.1f}m"
            cv2.putText(out, dist_txt, (u + r_px + 2, v),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 200), 1, cv2.LINE_AA)

    return out


# ─────────────────────────────────────────────
#  MAIN LOOP
# ─────────────────────────────────────────────
def main():
    pipeline, depth_scale, intr, spatial, temporal, hole_fill, align = init_realsense()

    occ_map         = np.zeros((Cfg.GRID_N, Cfg.GRID_N), dtype=np.float32)
    fallback_plane  = None     # estimated once, used when RANSAC fails

    print("Running — press  q  to quit.")
    print("Outputs: (x_fwd [m],  y_lat [m],  radius [m])  — robot frame, CBF-ready\n")

    try:
        while True:
            frames  = pipeline.wait_for_frames()
            aligned = align.process(frames)

            depth_frame = aligned.get_depth_frame()
            color_frame = aligned.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # ── Post-process depth (spatial smoothing + temporal + hole fill) ──
            depth_frame = spatial.process(depth_frame)
            depth_frame = temporal.process(depth_frame)
            depth_frame = hole_fill.process(depth_frame)

            depth = np.asanyarray(depth_frame.get_data())
            rgb   = np.asanyarray(color_frame.get_data())

            # ── 1. De-project depth → point cloud (camera frame) ──
            pts = depth_to_pointcloud(depth, depth_scale, intr)
            if len(pts) < 500:
                continue

            # ── 2. Ground plane estimation (RANSAC) ──
            plane = estimate_ground_plane(pts)

            if plane is not None:
                fallback_plane = plane          # cache last good estimate
            elif fallback_plane is not None:
                plane = fallback_plane          # use cached if RANSAC fails
            else:
                # Last resort: assume flat floor at 85th percentile y
                c = float(np.percentile(pts[:, 1], 85))
                plane = np.array([0.0, 0.0, c])

            # ── 3. Occupancy grid update ──
            occ_map, binary = update_occupancy(occ_map, pts, plane)

            # ── 4. Extract obstacle cluster positions ──
            obstacles = extract_obstacles(binary)

            # At the top of main(), after init:
            VERIFY_MODE = True   # set False when done

            # Inside the loop, after extract_obstacles():
            if VERIFY_MODE and obstacles:
                print("\n--- VERIFY ---")
                for i, (xf, yl, r) in enumerate(obstacles):
                    print(f"  obs {i+1}: x_fwd={xf:.3f}m  y_lat={yl:+.3f}m  r={r:.3f}m")
                print(f"  ground plane: a={plane[0]:.4f} b={plane[1]:.4f} c={plane[2]:.4f}")
                print(f"  inferred cam height ≈ {plane[2]:.3f} m")

            # ── 5. Print CBF-ready obstacle list ──
            if obstacles:
                lines = [f"  [{i+1}] x={xf:.3f} m  y={yl:+.3f} m  r={r:.3f} m"
                         for i, (xf, yl, r) in enumerate(obstacles)]
                print("Obstacles:\n" + "\n".join(lines))
            else:
                print("Obstacles: none")

            # ── 6. Visualise ──
            bev        = draw_bev(occ_map, binary, obstacles)
            rgb_out    = draw_rgb_overlay(rgb, obstacles, intr)

            cv2.imshow("Bird's-Eye View", bev)
            cv2.imshow("RGB + Obstacles", rgb_out)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("Stopped.")


if __name__ == "__main__":
    main()