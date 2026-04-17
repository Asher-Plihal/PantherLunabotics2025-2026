import threading

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


# Max points to render — RTABMap sends the full map each message so
# downsample only for matplotlib performance
MAX_POINTS = 10000


class CloudViewer(Node):
    """
    Subscribes to /rtabmap/cloud_map and renders the full accumulated
    3D map as a live matplotlib scatter plot at ~2 Hz.
    RTABMap already publishes the complete map each message so we just
    display the latest one.
    """

    def __init__(self):
        super().__init__('cloud_viewer')
        self._lock = threading.Lock()
        self._points: np.ndarray | None = None

        self.create_subscription(
            PointCloud2, '/rtabmap/cloud_map',
            self._cloud_callback, 10)

        self.get_logger().info(
            'CloudViewer ready — waiting for /rtabmap/cloud_map...')

    def _cloud_callback(self, msg: PointCloud2) -> None:
        """Replace stored points with the latest full map from RTABMap."""
        try:
            pts = np.array([
                [p[0], p[1], p[2]]
                for p in pc2.read_points(
                    msg, field_names=('x', 'y', 'z'), skip_nans=True)
            ], dtype=np.float32)
        except Exception as e:
            self.get_logger().warn(f'Error reading cloud: {e}')
            return

        if len(pts) == 0:
            return

        # Downsample only for rendering performance
        if len(pts) > MAX_POINTS:
            idx = np.random.choice(len(pts), MAX_POINTS, replace=False)
            pts = pts[idx]

        with self._lock:
            self._points = pts

    def get_points(self) -> np.ndarray | None:
        """Return a copy of the latest full map (thread-safe)."""
        with self._lock:
            return self._points.copy() if self._points is not None else None


def main() -> None:
    """Entry point — spins ROS in a background thread, plots on main thread."""
    rclpy.init()
    node = CloudViewer()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    plt.ion()
    fig = plt.figure(figsize=(8, 6))

    while plt.fignum_exists(fig.number):
        pts = node.get_points()

        fig.clear()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_title('RTABMap 3D Cloud — live')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')

        if pts is None:
            ax.text2D(0.5, 0.5, 'Waiting for /rtabmap/cloud_map...',
                      transform=ax.transAxes, ha='center', va='center',
                      fontsize=12, color='gray')
        else:
            ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2],
                       s=1, c=pts[:, 2], cmap='viridis', alpha=0.6)
            ax.text2D(0.01, 0.99, f'{len(pts)} pts',
                      transform=ax.transAxes, va='top', fontsize=8)

        fig.canvas.draw()
        fig.canvas.flush_events()
        plt.pause(0.5)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
