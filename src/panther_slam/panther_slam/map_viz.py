import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from threading import Thread, Lock

class MapViz(Node):
    def __init__(self):
        super().__init__('map_viz')

        self.cloud_lock = Lock()
        self.points_xyz = None   # Nx3
        self.points_rgb = None   # Nx3 float 0-1

        self.sub = self.create_subscription(
            PointCloud2,
            '/rtabmap/cloud_map',
            self.cloud_callback,
            10
        )
        self.get_logger().info('MapViz waiting for /rtabmap/cloud_map ...')

    def cloud_callback(self, msg: PointCloud2):
        # Extract x, y, z, rgb from the PointCloud2 message
        points = list(pc2.read_points(
            msg,
            field_names=('x', 'y', 'z', 'rgb'),
            skip_nans=True
        ))

        if len(points) == 0:
            return

        xyz = np.array([[p[0], p[1], p[2]] for p in points], dtype=np.float32)

        # Decode packed RGB float → r, g, b (0.0–1.0)
        rgb_packed = np.array([p[3] for p in points], dtype=np.float32)
        rgb_int    = rgb_packed.view(np.int32)
        r = ((rgb_int >> 16) & 0xFF) / 255.0
        g = ((rgb_int >>  8) & 0xFF) / 255.0
        b = ((rgb_int      ) & 0xFF) / 255.0
        rgb = np.stack([r, g, b], axis=1)

        with self.cloud_lock:
            self.points_xyz = xyz
            self.points_rgb = rgb

        self.get_logger().info(f'Cloud updated: {len(xyz)} points')


def start_ros(node):
    rclpy.spin(node)


def main():
    rclpy.init()
    node = MapViz()

    # Run ROS spin in background thread so matplotlib can own the main thread
    ros_thread = Thread(target=start_ros, args=(node,), daemon=True)
    ros_thread.start()

    # --- Matplotlib setup ---
    fig = plt.figure(figsize=(10, 8))
    ax  = fig.add_subplot(111, projection='3d')
    ax.set_title('RTAB-Map Persistent Point Cloud')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    scatter = [None]

    def update(frame):
        with node.cloud_lock:
            if node.points_xyz is None:
                return
            xyz = node.points_xyz.copy()
            rgb = node.points_rgb.copy()

        # Downsample for performance (every 5th point)
        xyz = xyz[::5]
        rgb = rgb[::5]

        ax.cla()
        ax.set_title(f'RTAB-Map Cloud  ({len(xyz)} pts, downsampled 5x)')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')

        ax.scatter(
            xyz[:, 0], xyz[:, 1], xyz[:, 2],
            c=rgb,
            s=0.5,        # dot size
            linewidths=0
        )

        # Top-down view by default (comment out to get full 3D)
        ax.view_init(elev=90, azim=-90)

    ani = animation.FuncAnimation(
        fig, update,
        interval=2000,   # refresh every 2 seconds
        cache_frame_data=False
    )

    plt.tight_layout()
    plt.show()

    rclpy.shutdown()


if __name__ == '__main__':
    main()