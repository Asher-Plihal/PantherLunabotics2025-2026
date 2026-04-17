import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2


class BEVMapper(Node):
    """
    Converts D435i point cloud data to a bird's-eye-view (BEV) occupancy grid.
    Projects 3D points onto a 2D top-down grid and marks cells as occupied/free.
    """

    def __init__(self):
        super().__init__('bev_mapper')

        # Declare parameters
        self.declare_parameter('bev_size', 256)
        self.declare_parameter('bev_range', 3.0)
        self.declare_parameter('height_min', 0.0)
        self.declare_parameter('height_max', 1.0)
        self.declare_parameter('occupancy_threshold', 5)
        self.declare_parameter('pointcloud_topic', '/camera/depth/color/points')
        self.declare_parameter('output_topic', '/bev_map')

        # Get parameters
        self.bev_size = self.get_parameter('bev_size').value
        self.bev_range = self.get_parameter('bev_range').value
        self.height_min = self.get_parameter('height_min').value
        self.height_max = self.get_parameter('height_max').value
        self.occupancy_threshold = self.get_parameter('occupancy_threshold').value
        pointcloud_topic = self.get_parameter('pointcloud_topic').value
        output_topic = self.get_parameter('output_topic').value

        # State
        self.bev_grid = np.zeros((self.bev_size, self.bev_size), dtype=np.int32)
        self.bridge = CvBridge()

        # Subscriptions
        self.pc_sub = self.create_subscription(
            PointCloud2, pointcloud_topic, self.pointcloud_callback, 10)

        # Publishers
        self.bev_occupancy_pub = self.create_publisher(
            OccupancyGrid, output_topic, 10)
        self.bev_image_pub = self.create_publisher(
            Image, '/bev_map_image', 10)

        # Timer to publish BEV grid at regular intervals
        self.create_timer(0.5, self.publish_bev_grid)

        self.get_logger().info(
            f'BEVMapper initialized: {self.bev_size}x{self.bev_size} grid, '
            f'range=±{self.bev_range}m, height=[{self.height_min}, {self.height_max}]m')

    def pointcloud_callback(self, msg: PointCloud2):
        """Convert point cloud to BEV occupancy grid."""
        points = []

        # Extract XYZ points from PointCloud2
        try:
            for point in pc2.read_points(msg, skip_nans=True, field_names=('x', 'y', 'z')):
                x, y, z = point[0], point[1], point[2]
                points.append([x, y, z])
        except Exception as e:
            self.get_logger().warn(f'Error reading point cloud: {e}')
            return

        if not points:
            return

        points = np.array(points)

        # Create fresh BEV grid for this frame
        bev_grid = np.zeros((self.bev_size, self.bev_size), dtype=np.int32)

        # Project points onto BEV grid
        for point in points:
            x, y, z = point

            # Height filter (only ground-level obstacles)
            if z < self.height_min or z > self.height_max:
                continue

            # Convert world coordinates to grid indices
            # X-axis: camera's forward direction
            # Y-axis: camera's left/right direction
            grid_x = int((x + self.bev_range) / (2 * self.bev_range) * self.bev_size)
            grid_y = int((y + self.bev_range) / (2 * self.bev_range) * self.bev_size)

            # Check bounds
            if 0 <= grid_x < self.bev_size and 0 <= grid_y < self.bev_size:
                bev_grid[grid_y, grid_x] += 1

        # Convert to occupancy (0-100 scale)
        # Count > threshold → occupied
        bev_occupancy = np.zeros_like(bev_grid, dtype=np.uint8)
        bev_occupancy[bev_grid > self.occupancy_threshold] = 100

        self.bev_grid = bev_occupancy

    def publish_bev_grid(self):
        """Publish BEV grid as OccupancyGrid and visualization image."""
        if self.bev_grid is None:
            return

        # Publish OccupancyGrid message
        occ_grid = OccupancyGrid()
        occ_grid.header.stamp = self.get_clock().now().to_msg()
        occ_grid.header.frame_id = 'camera_link'
        occ_grid.info.resolution = (2 * self.bev_range) / self.bev_size
        occ_grid.info.width = self.bev_size
        occ_grid.info.height = self.bev_size
        occ_grid.info.origin.position.x = -self.bev_range
        occ_grid.info.origin.position.y = -self.bev_range
        occ_grid.info.origin.orientation.w = 1.0
        occ_grid.data = self.bev_grid.flatten().tolist()

        self.bev_occupancy_pub.publish(occ_grid)

        # Publish visualization image
        img = self._grid_to_image(self.bev_grid)
        ros_img = self.bridge.cv2_to_imgmsg(img, encoding='rgb8')
        ros_img.header.stamp = occ_grid.header.stamp
        ros_img.header.frame_id = 'camera_link'
        self.bev_image_pub.publish(ros_img)

        # Save to disk for debugging
        cv2.imwrite(
            '/tmp/bev_map.png',
            cv2.cvtColor(img, cv2.COLOR_RGB2BGR))

    def _grid_to_image(self, grid):
        """Convert BEV grid to RGB image for visualization."""
        h, w = grid.shape
        img = np.ones((h, w, 3), dtype=np.uint8) * 255

        # White = free (0), Black = occupied (100)
        img[grid > 50] = [0, 0, 0]

        # Add crosshair at center (robot position)
        cx, cy = w // 2, h // 2
        cv2.line(img, (cx - 10, cy), (cx + 10, cy), (0, 255, 0), 1)
        cv2.line(img, (cx, cy - 10), (cx, cy + 10), (0, 255, 0), 1)

        # Flip vertically for proper orientation
        img = cv2.flip(img, 0)

        # Status overlay
        cv2.putText(img, 'BEV MAP', (5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 128, 0), 1)

        return img


def main():
    rclpy.init()
    node = BEVMapper()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
