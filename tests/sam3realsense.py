"""
SAM3 + RealSense D435i — Obstacle Detection with Bird's-Eye View
================================================================
Uses SAM3 text-prompted segmentation to detect obstacles (craters / mounds)
in the RGB stream, then uses the aligned depth frame to back-project each
mask into 3-D camera space and output (x_fwd, y_lat, radius) in robot frame.

Architecture
------------
  Main thread  : RealSense capture + rendering at full frame rate
  SAM3 thread  : runs inference every SAM3_INTERVAL frames on a copy of the
                 latest colour image; writes results into a thread-safe slot.
  This way the display never stalls waiting for the GPU.

Robot frame (right-hand):
    x → forward  (camera z)
    y → left     (camera -x)

Usage
-----
    python obstacle_sam3.py
    python obstacle_sam3.py --height 0.30 --pitch 25 --prompt "crater or mound"
    python obstacle_sam3.py --interval 5 --verify
"""

import argparse
import threading
import time
from dataclasses import dataclass, field
from queue import Empty, Queue

import cv2
import numpy as np
import torch
import pyrealsense2 as rs
from PIL import Image

from sam3.model_builder import build_sam3_image_model
from sam3.model.sam3_image_processor import Sam3Processor


# ─────────────────────────────────────────────────────────────────────────────
#  CONFIG
# ─────────────────────────────────────────────────────────────────────────────
class Cfg:
    # Camera mounting
    CAM_HEIGHT      = 0.30    # m
    CAM_PITCH_DEG   = 25.0    # degrees downward (0 = horizontal)

    # Depth range
    MIN_DEPTH       = 0.20    # m
    MAX_DEPTH       = 5.0     # m

    # SAM3
    SAM3_PROMPT     = "crater or mound"
    SAM3_INTERVAL   = 6       # run SAM3 every N captured frames
    SAM3_MIN_SCORE  = 0.30    # discard detections below this confidence
    SAM3_MIN_PIXELS = 200     # discard tiny masks (noise)

    # Depth sampling inside a mask
    # We take the median of the central DEPTH_SAMPLE_PCT fraction of depth
    # values (by proximity to the mask centroid) to avoid sampling background.
    DEPTH_SAMPLE_PCT = 0.5

    # BEV grid
    GRID_N          = 250
    RES             = 0.03    # m / cell
    DECAY           = 0.75
    BIN_THRESH      = 0.28
    MIN_CLUSTER     = 8

    # Obstacle "memory": keep a detection for this many frames if SAM3 hasn't
    # refreshed yet (prevents flickering).
    OBSTACLE_TTL    = 30      # frames

    # Visualisation
    BEV_SCALE       = 2
    VERIFY_MODE     = False


# ─────────────────────────────────────────────────────────────────────────────
#  DATA STRUCTURES
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class Obstacle:
    x_fwd:  float            # metres forward (robot frame)
    y_lat:  float            # metres left    (robot frame)
    radius: float            # metres
    score:  float            # SAM3 confidence [0-1]
    label:  str = "obstacle"
    ttl:    int = Cfg.OBSTACLE_TTL   # frames remaining before expiry


@dataclass
class Sam3Result:
    """Output from one SAM3 inference pass."""
    masks:  list[np.ndarray]   # each (H, W) bool
    boxes:  list[tuple]        # each (x0, y0, x1, y1) in pixels
    scores: list[float]
    frame_idx: int = 0


# ─────────────────────────────────────────────────────────────────────────────
#  DEPTH  ←→  3-D  UTILITIES
# ─────────────────────────────────────────────────────────────────────────────
def backproject_pixel(u: float, v: float, depth_m: float, intr) -> np.ndarray:
    """
    Back-project a single pixel (u, v) at depth_m into camera-frame 3-D.
    Returns [x_cam, y_cam, z_cam]  (y is DOWN in RealSense convention).
    """
    x = (u - intr.ppx) / intr.fx * depth_m
    y = (v - intr.ppy) / intr.fy * depth_m
    return np.array([x, y, depth_m], dtype=np.float32)


def cam_to_robot(pt_cam: np.ndarray) -> tuple[float, float]:
    """
    Convert a camera-frame point to robot-frame (x_fwd, y_lat).
        x_fwd = z_cam   (forward)
        y_lat = -x_cam  (left)
    """
    return float(pt_cam[2]), float(-pt_cam[0])


def mask_to_obstacle(
    mask:      np.ndarray,
    depth_img: np.ndarray,
    depth_scale: float,
    intr,
    score:     float,
) -> Obstacle | None:
    """
    Given a boolean mask (H, W) and the aligned depth image, compute the
    3-D position and radius of the corresponding obstacle.

    Depth sampling strategy
    -----------------------
    1. Find all in-mask pixels that have valid depth (non-zero, in range).
    2. Sort them by distance to the mask centroid (prefer centre of object).
    3. Take the median depth of the closest DEPTH_SAMPLE_PCT fraction.
       This rejects background depth values that leak in around the edges.
    4. Back-project the mask centroid at that median depth.
    5. Estimate radius from the mask's equivalent circle diameter in pixels,
       scaled by depth and focal length.
    """
    ys, xs = np.where(mask)
    if len(xs) < Cfg.SAM3_MIN_PIXELS:
        return None

    # Centroid in pixel space
    cx = float(xs.mean())
    cy = float(ys.mean())

    # Valid depth pixels inside the mask
    depths = depth_img[ys, xs].astype(np.float32) * depth_scale
    valid  = (depths > Cfg.MIN_DEPTH) & (depths < Cfg.MAX_DEPTH)
    if valid.sum() < 20:
        return None

    xs_v = xs[valid];  ys_v = ys[valid];  d_v = depths[valid]

    # Sort by distance to centroid → prefer centre-of-mask depth values
    dist_to_cx = np.sqrt((xs_v - cx) ** 2 + (ys_v - cy) ** 2)
    order      = np.argsort(dist_to_cx)
    n_sample   = max(20, int(len(order) * Cfg.DEPTH_SAMPLE_PCT))
    d_median   = float(np.median(d_v[order[:n_sample]]))

    if d_median < Cfg.MIN_DEPTH or d_median > Cfg.MAX_DEPTH:
        return None

    # Back-project centroid at median depth
    pt_cam         = backproject_pixel(cx, cy, d_median, intr)
    x_fwd, y_lat   = cam_to_robot(pt_cam)

    if x_fwd < 0.1:   # behind the camera
        return None

    # Radius estimate: pixel radius → metres using pinhole projection
    # pixel_radius ≈ sqrt(mask_area / π)
    px_radius = float(np.sqrt(len(xs) / np.pi))
    # At depth d, one pixel spans d / fx metres in x
    radius_m  = max(0.05, px_radius * d_median / intr.fx)

    return Obstacle(x_fwd=x_fwd, y_lat=y_lat, radius=radius_m, score=score)


# ─────────────────────────────────────────────────────────────────────────────
#  SAM3 INFERENCE THREAD
# ─────────────────────────────────────────────────────────────────────────────
class Sam3Worker(threading.Thread):
    """
    Runs SAM3 inference in a background thread.

    Input  queue: (rgb_np, depth_np, frame_idx)  — numpy arrays
    Output queue: Sam3Result

    Only the most recent frame is processed; older frames are discarded
    so the worker never falls further and further behind.
    """

    def __init__(self, prompt: str):
        super().__init__(daemon=True)
        self.prompt    = prompt
        self.in_q:  Queue = Queue(maxsize=2)
        self.out_q: Queue = Queue(maxsize=2)
        self._stop = threading.Event()

    def submit(self, rgb_np: np.ndarray, frame_idx: int) -> None:
        """Non-blocking submit. Drops old item if queue is full."""
        if self.in_q.full():
            try:
                self.in_q.get_nowait()
            except Empty:
                pass
        self.in_q.put_nowait((rgb_np.copy(), frame_idx))

    def latest_result(self) -> Sam3Result | None:
        """Return the most recent result without blocking."""
        result = None
        while not self.out_q.empty():
            try:
                result = self.out_q.get_nowait()
            except Empty:
                break
        return result

    def stop(self) -> None:
        self._stop.set()

    def run(self) -> None:
        print("[SAM3] loading model …")
        torch.backends.cuda.matmul.allow_tf32 = True
        torch.backends.cudnn.allow_tf32 = True

        with torch.autocast("cuda", dtype=torch.bfloat16):
            model     = build_sam3_image_model()
            processor = Sam3Processor(model)
            print(f"[SAM3] ready  prompt='{self.prompt}'")

            while not self._stop.is_set():
                try:
                    rgb_np, frame_idx = self.in_q.get(timeout=0.5)
                except Empty:
                    continue

                t0    = time.time()
                image = Image.fromarray(cv2.cvtColor(rgb_np, cv2.COLOR_BGR2RGB))

                try:
                    state  = processor.set_image(image)
                    processor.reset_all_prompts(state)
                    output = processor.set_text_prompt(state=state, prompt=self.prompt)

                    masks  = [m.squeeze().cpu().numpy().astype(bool)
                               for m in output["masks"]]
                    boxes  = [tuple(b.cpu().numpy().tolist())
                               for b in output["boxes"]]
                    scores = [float(s) for s in output["scores"]]

                    result = Sam3Result(masks=masks, boxes=boxes,
                                        scores=scores, frame_idx=frame_idx)
                    self.out_q.put_nowait(result)

                    if Cfg.VERIFY_MODE:
                        print(f"[SAM3] frame {frame_idx}  "
                              f"{len(masks)} detections  "
                              f"{(time.time()-t0)*1000:.0f} ms")

                except Exception as exc:
                    print(f"[SAM3] inference error: {exc}")


# ─────────────────────────────────────────────────────────────────────────────
#  OCCUPANCY GRID  (fed from obstacle list, not raw point cloud)
# ─────────────────────────────────────────────────────────────────────────────
def update_occupancy_from_obstacles(
    occ_map:   np.ndarray,
    obstacles: list[Obstacle],
) -> tuple[np.ndarray, np.ndarray]:
    """
    Stamp each obstacle as a Gaussian blob in the occupancy grid.
    This is simpler and more direct than the point-cloud approach since
    SAM3 already tells us where each obstacle is.
    """
    layer = np.zeros_like(occ_map)

    for obs in obstacles:
        row = int(Cfg.GRID_N // 2 - obs.x_fwd / Cfg.RES)
        col = int(Cfg.GRID_N // 2 + obs.y_lat / Cfg.RES)   # y_lat positive = left

        if not (0 <= row < Cfg.GRID_N and 0 <= col < Cfg.GRID_N):
            continue

        # Radius in cells → stamp a disc
        r_cells = max(1, int(obs.radius / Cfg.RES))
        r0, r1  = max(0, row - r_cells), min(Cfg.GRID_N, row + r_cells + 1)
        c0, c1  = max(0, col - r_cells), min(Cfg.GRID_N, col + r_cells + 1)

        rr, cc = np.mgrid[r0:r1, c0:c1]
        d_sq   = (rr - row) ** 2 + (cc - col) ** 2
        blob   = np.exp(-d_sq / (2 * (r_cells * 0.5 + 1) ** 2))
        layer[r0:r1, c0:c1] = np.maximum(layer[r0:r1, c0:c1], blob)

    occ_map = Cfg.DECAY * occ_map + (1.0 - Cfg.DECAY) * layer
    return occ_map, (occ_map > Cfg.BIN_THRESH)


# ─────────────────────────────────────────────────────────────────────────────
#  VISUALISATION
# ─────────────────────────────────────────────────────────────────────────────
_COLS = dict(
    bg       =(18,  18,  18),
    grid     =(35,  35,  35),
    axis     =(55,  55,  55),
    ring     =(48,  48,  48),
    obs_ring =(0,  160, 255),
    obs_dot  =(0,  220, 255),
    label    =(255, 220,  80),
    robot    =(0,  255, 100),
    fwd      =(100, 255, 100),
    stale    =(100, 100, 150),   # faded colour for decaying detections
)

# Colours for mask overlay on the RGB frame (cycle through these)
_MASK_COLOURS = [
    (0,   200, 255),
    (0,   255, 100),
    (255, 180,   0),
    (180,   0, 255),
    (255,  60,  60),
]


def draw_rgb_detections(
    rgb:       np.ndarray,
    sam_result: Sam3Result | None,
    obstacles: list[Obstacle],
    intr,
) -> np.ndarray:
    """
    Overlay SAM3 masks + bounding boxes + 3-D distance labels on the RGB frame.
    """
    out = rgb.copy()

    if sam_result is not None:
        for i, (mask, box, score) in enumerate(
            zip(sam_result.masks, sam_result.boxes, sam_result.scores)
        ):
            if score < Cfg.SAM3_MIN_SCORE:
                continue
            col = _MASK_COLOURS[i % len(_MASK_COLOURS)]

            # Semi-transparent mask overlay
            overlay        = out.copy()
            overlay[mask]  = col
            out            = cv2.addWeighted(overlay, 0.35, out, 0.65, 0)

            # Bounding box
            x0, y0, x1, y1 = [int(v) for v in box]
            cv2.rectangle(out, (x0, y0), (x1, y1), col, 2)
            cv2.putText(out, f"{score:.2f}", (x0 + 2, y0 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)

    # Distance labels from obstacles
    for obs in obstacles:
        z_cam  =  obs.x_fwd
        x_cam  = -obs.y_lat
        if z_cam < Cfg.MIN_DEPTH:
            continue
        u    = int(x_cam / z_cam * intr.fx + intr.ppx)
        v    = int(0.0   / z_cam * intr.fy + intr.ppy)
        r_px = max(6, int(obs.radius / z_cam * intr.fx))

        alpha = min(1.0, obs.ttl / Cfg.OBSTACLE_TTL)
        col   = tuple(int(c * alpha) for c in (0, 200, 255))

        if 0 <= u < out.shape[1] and 0 <= v < out.shape[0]:
            cv2.circle(out, (u, v), r_px, col, 2)
            cv2.putText(out, f"{obs.x_fwd:.2f}m", (u + r_px + 2, v),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)

    return out


def draw_bev(
    occ_map:   np.ndarray,
    obstacles: list[Obstacle],
    frame_idx: int,
    last_sam3_idx: int,
) -> np.ndarray:
    S   = Cfg.BEV_SCALE
    N   = Cfg.GRID_N
    sz  = N * S
    mid = (N // 2) * S

    canvas = np.full((sz, sz, 3), _COLS['bg'], dtype=np.uint8)

    # Occupancy heatmap
    heat     = cv2.resize((occ_map * 255).astype(np.uint8), (sz, sz),
                          interpolation=cv2.INTER_NEAREST)
    heat_bgr = cv2.applyColorMap(heat, cv2.COLORMAP_OCEAN)
    canvas[heat > 15] = heat_bgr[heat > 15]

    # 1 m grid
    cells_m = max(1, int(1.0 / Cfg.RES))
    for i in range(0, N, cells_m):
        p = i * S
        cv2.line(canvas, (0, p), (sz, p), _COLS['grid'], 1)
        cv2.line(canvas, (p, 0), (p, sz), _COLS['grid'], 1)

    # Distance rings
    for d in range(1, int(Cfg.MAX_DEPTH) + 1):
        r_px = int(d / Cfg.RES * S)
        cv2.circle(canvas, (mid, mid), r_px, _COLS['ring'], 1)
        ly = mid - r_px
        if 4 < ly < sz - 4:
            cv2.putText(canvas, f"{d}m", (mid + 4, ly + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.30, (75, 75, 75), 1, cv2.LINE_AA)

    # Axes
    cv2.line(canvas, (mid, 0), (mid, sz), _COLS['axis'], 1)
    cv2.line(canvas, (0, mid), (sz, mid), _COLS['axis'], 1)

    # Obstacles
    for obs in obstacles:
        row   = int((N // 2 - obs.x_fwd / Cfg.RES) * S)
        col_  = int((N // 2 + obs.y_lat / Cfg.RES) * S)
        r_px  = max(5, int(obs.radius / Cfg.RES * S))

        # Fade colour as the detection ages
        alpha  = min(1.0, obs.ttl / Cfg.OBSTACLE_TTL)
        ring_c = tuple(int(c * alpha) for c in _COLS['obs_ring'])
        dot_c  = tuple(int(c * alpha) for c in _COLS['obs_dot'])
        lbl_c  = tuple(int(c * alpha) for c in _COLS['label'])

        cv2.circle(canvas, (col_, row), r_px + 3, ring_c, 2)
        cv2.circle(canvas, (col_, row), 3,         dot_c,  -1)

        txt = f"({obs.x_fwd:.1f},{obs.y_lat:+.1f}) s={obs.score:.2f}"
        cv2.putText(canvas, txt, (col_ + r_px + 4, row + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.28, lbl_c, 1, cv2.LINE_AA)

    # Robot marker
    tri = np.array([[mid, mid - 10], [mid - 7, mid + 7], [mid + 7, mid + 7]])
    cv2.fillPoly(canvas, [tri], _COLS['robot'])

    # Status overlays
    cv2.putText(canvas, "FWD", (mid - 14, 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.36, _COLS['fwd'], 1, cv2.LINE_AA)
    frames_since = frame_idx - last_sam3_idx
    staleness    = f"SAM3 +{frames_since}f ago" if frames_since > 0 else "SAM3 live"
    cv2.putText(canvas, staleness, (4, 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, (140, 140, 140), 1, cv2.LINE_AA)
    cv2.putText(canvas, f"obstacles: {len(obstacles)}", (sz - 100, 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.34, (180, 180, 180), 1, cv2.LINE_AA)

    return canvas


# ─────────────────────────────────────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────────────────────────────────────
def main() -> None:
    parser = argparse.ArgumentParser(
        description="SAM3 + RealSense obstacle BEV",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--height",   type=float, default=Cfg.CAM_HEIGHT)
    parser.add_argument("--pitch",    type=float, default=Cfg.CAM_PITCH_DEG,
                        help="Downward pitch in degrees. 0=horizontal, 25=tilted down 25°")
    parser.add_argument("--prompt",   type=str,   default=Cfg.SAM3_PROMPT)
    parser.add_argument("--interval", type=int,   default=Cfg.SAM3_INTERVAL,
                        help="Run SAM3 every N frames")
    parser.add_argument("--verify",   action="store_true")
    args = parser.parse_args()

    Cfg.CAM_HEIGHT    = args.height
    Cfg.CAM_PITCH_DEG = args.pitch
    Cfg.SAM3_PROMPT   = args.prompt
    Cfg.SAM3_INTERVAL = args.interval
    Cfg.VERIFY_MODE   = args.verify

    # ── RealSense setup ──────────────────────────────────────────────────────
    pipeline = rs.pipeline()
    config   = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,  30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    profile      = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale  = depth_sensor.get_depth_scale()

    depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
    intr          = depth_profile.get_intrinsics()

    # Align depth → colour so pixel coords match
    align         = rs.align(rs.stream.color)

    # Post-processing
    spatial  = rs.spatial_filter()
    spatial.set_option(rs.option.filter_magnitude,    2)
    spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
    spatial.set_option(rs.option.filter_smooth_delta, 20)
    temporal = rs.temporal_filter()
    temporal.set_option(rs.option.filter_smooth_alpha, 0.4)
    temporal.set_option(rs.option.filter_smooth_delta, 20)
    hole_fill = rs.hole_filling_filter()

    print(f"[RealSense] depth scale: {depth_scale:.6f} m/unit")
    print(f"[Config]    height={Cfg.CAM_HEIGHT} m  pitch={Cfg.CAM_PITCH_DEG}°")
    print(f"[SAM3]      prompt='{Cfg.SAM3_PROMPT}'  interval={Cfg.SAM3_INTERVAL} frames")

    # ── SAM3 background worker ───────────────────────────────────────────────
    worker = Sam3Worker(prompt=Cfg.SAM3_PROMPT)
    worker.start()

    # ── State ────────────────────────────────────────────────────────────────
    occ_map       = np.zeros((Cfg.GRID_N, Cfg.GRID_N), dtype=np.float32)
    obstacles:    list[Obstacle] = []
    latest_result: Sam3Result | None = None
    frame_idx     = 0
    last_sam3_idx = 0

    WIN_RGB = "RGB + SAM3 detections"
    WIN_BEV = "Bird's-Eye View"
    cv2.namedWindow(WIN_RGB, cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow(WIN_BEV, cv2.WINDOW_AUTOSIZE)

    print("\nRunning — press  q  to quit.\n")

    try:
        while True:
            t0 = time.time()

            # ── Grab + align frames ──────────────────────────────────────────
            frames  = pipeline.wait_for_frames()
            aligned = align.process(frames)

            depth_frame = aligned.get_depth_frame()
            color_frame = aligned.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            depth_frame = spatial.process(depth_frame)
            depth_frame = temporal.process(depth_frame)
            depth_frame = hole_fill.process(depth_frame)

            depth_img = np.asanyarray(depth_frame.get_data())   # (H, W) uint16
            rgb_img   = np.asanyarray(color_frame.get_data())   # (H, W, 3) BGR

            # ── Submit to SAM3 worker every N frames ─────────────────────────
            if frame_idx % Cfg.SAM3_INTERVAL == 0:
                worker.submit(rgb_img, frame_idx)

            # ── Collect any finished SAM3 results ────────────────────────────
            new_result = worker.latest_result()
            if new_result is not None:
                latest_result = new_result
                last_sam3_idx = new_result.frame_idx

                # Convert SAM3 masks → 3-D obstacles
                new_obstacles: list[Obstacle] = []
                for mask, box, score in zip(
                    latest_result.masks,
                    latest_result.boxes,
                    latest_result.scores,
                ):
                    if score < Cfg.SAM3_MIN_SCORE:
                        continue
                    obs = mask_to_obstacle(mask, depth_img, depth_scale, intr, score)
                    if obs is not None:
                        new_obstacles.append(obs)

                # Merge with existing list (keep non-overlapping old ones at
                # reduced TTL, replace overlapping ones with fresh detections)
                merged: list[Obstacle] = list(new_obstacles)
                for old in obstacles:
                    # Check if any new obstacle is close to this old one
                    covered = any(
                        abs(old.x_fwd - n.x_fwd) < n.radius * 2 and
                        abs(old.y_lat - n.y_lat)  < n.radius * 2
                        for n in new_obstacles
                    )
                    if not covered and old.ttl > 1:
                        old.ttl -= Cfg.SAM3_INTERVAL   # age by one batch
                        if old.ttl > 0:
                            merged.append(old)
                obstacles = merged

                if Cfg.VERIFY_MODE:
                    print(f"\n[Frame {frame_idx}]  SAM3 result from frame "
                          f"{last_sam3_idx}  ({len(obstacles)} obstacles)")
                    for i, obs in enumerate(obstacles):
                        print(f"  [{i+1}] x={obs.x_fwd:.3f} m  "
                              f"y={obs.y_lat:+.3f} m  "
                              f"r={obs.radius:.3f} m  "
                              f"score={obs.score:.2f}  ttl={obs.ttl}")

            else:
                # No new SAM3 result — age TTL by 1
                for obs in obstacles:
                    obs.ttl -= 1
                obstacles = [o for o in obstacles if o.ttl > 0]

            # ── Occupancy grid ───────────────────────────────────────────────
            occ_map, _ = update_occupancy_from_obstacles(occ_map, obstacles)

            # ── Render ───────────────────────────────────────────────────────
            rgb_out = draw_rgb_detections(rgb_img, latest_result, obstacles, intr)
            bev     = draw_bev(occ_map, obstacles, frame_idx, last_sam3_idx)

            dt = time.time() - t0
            cv2.setWindowTitle(WIN_RGB,
                f"RGB + SAM3  |  {1/dt:.0f} FPS  |  "
                f"SAM3 interval: every {Cfg.SAM3_INTERVAL} frames")

            cv2.imshow(WIN_RGB, rgb_out)
            cv2.imshow(WIN_BEV, bev)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            frame_idx += 1

    finally:
        worker.stop()
        worker.join(timeout=3.0)
        pipeline.stop()
        cv2.destroyAllWindows()
        print("Stopped.")


if __name__ == "__main__":
    main()