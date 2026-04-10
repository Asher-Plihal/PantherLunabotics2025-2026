import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
import numpy as np
import heapq
import cv2
from cv_bridge import CvBridge

class SlamPlanner(Node):
    def __init__(self):
        super().__init__('slam_planner')

        self.map_data = None
        self.map_info = None
        self.start    = None
        self.bridge   = CvBridge()

        # Subscriptions
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/rtabmap/localization_pose', self.pose_callback, 10)

        # Publishers
        self.path_pub    = self.create_publisher(Path,  '/planned_path', 10)
        self.mapimg_pub  = self.create_publisher(Image, '/map_image',    10)

        # Publish a fresh map image every second
        self.create_timer(1.0, self.publish_map_image)

        self.get_logger().info('SlamPlanner ready. Waiting for map...')

    # ------------------------------------------------------------------
    # Map callback
    # ------------------------------------------------------------------
    def map_callback(self, msg: OccupancyGrid):
        self.map_info = msg.info
        width  = msg.info.width
        height = msg.info.height
        raw = np.array(msg.data, dtype=np.int8).reshape((height, width))
        self.map_data = raw
        self.get_logger().info(
            f'Map received: {width}x{height}, '
            f'resolution={msg.info.resolution:.3f}m/cell')

    # ------------------------------------------------------------------
    # Convert occupancy grid → colour image and publish + save
    # ------------------------------------------------------------------
    def publish_map_image(self):
        if self.map_data is None:
            return

        h, w = self.map_data.shape
        # Start with a white canvas (RGB)
        img = np.ones((h, w, 3), dtype=np.uint8) * 255

        # Unknown cells → grey
        img[self.map_data == -1] = [128, 128, 128]
        # Occupied cells → black
        img[self.map_data == 100] = [0, 0, 0]
        # Free cells stay white (255,255,255)

        # Draw robot position as a blue dot
        if self.start is not None:
            row, col = self.start
            cv2.circle(img, (col, row), radius=4,
                       color=(0, 0, 255), thickness=-1)

        # Flip vertically so the map is right-side up
        img = cv2.flip(img, 0)

        # Publish as ROS Image topic (viewable in rqt_image_view / RViz)
        ros_img = self.bridge.cv2_to_imgmsg(img, encoding='rgb8')
        ros_img.header.stamp    = self.get_clock().now().to_msg()
        ros_img.header.frame_id = 'map'
        self.mapimg_pub.publish(ros_img)

        # Also save to disk every update (overwrite same file)
        cv2.imwrite('/tmp/slam_map.png',
                    cv2.cvtColor(img, cv2.COLOR_RGB2BGR))

    # ------------------------------------------------------------------
    # Pose callback
    # ------------------------------------------------------------------
    def pose_callback(self, msg: PoseStamped):
        if self.map_info is None:
            return
        self.start = self.world_to_grid(
            msg.pose.position.x, msg.pose.position.y)

    # ------------------------------------------------------------------
    # Coordinate helpers
    # ------------------------------------------------------------------
    def world_to_grid(self, wx, wy):
        res    = self.map_info.resolution
        origin = self.map_info.origin.position
        col = int((wx - origin.x) / res)
        row = int((wy - origin.y) / res)
        return (row, col)

    def grid_to_world(self, row, col):
        res    = self.map_info.resolution
        origin = self.map_info.origin.position
        wx = col * res + origin.x + res / 2.0
        wy = row * res + origin.y + res / 2.0
        return (wx, wy)

    def is_free(self, row, col):
        h, w = self.map_data.shape
        if row < 0 or col < 0 or row >= h or col >= w:
            return False
        return self.map_data[row, col] == 0

    # ------------------------------------------------------------------
    # A* planner
    # ------------------------------------------------------------------
    def plan(self, goal_wx, goal_wy):
        if self.map_data is None or self.start is None:
            return None
        goal  = self.world_to_grid(goal_wx, goal_wy)
        start = self.start
        if not self.is_free(*goal):
            self.get_logger().warn(f'Goal {goal} is occupied or unknown!')
            return None

        def heuristic(a, b):
            return abs(a[0]-b[0]) + abs(a[1]-b[1])

        open_heap = []
        heapq.heappush(open_heap, (0, start))
        came_from = {start: None}
        g_score   = {start: 0}
        neighbours = [(-1,0),(1,0),(0,-1),(0,1),
                      (-1,-1),(-1,1),(1,-1),(1,1)]

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current == goal:
                path_cells = []
                while current is not None:
                    path_cells.append(current)
                    current = came_from[current]
                path_cells.reverse()
                return self.cells_to_path_msg(path_cells)
            for dr, dc in neighbours:
                nb = (current[0]+dr, current[1]+dc)
                if not self.is_free(*nb):
                    continue
                step = 1.414 if dr != 0 and dc != 0 else 1.0
                tentative_g = g_score[current] + step
                if tentative_g < g_score.get(nb, float('inf')):
                    came_from[nb] = current
                    g_score[nb]   = tentative_g
                    f = tentative_g + heuristic(nb, goal)
                    heapq.heappush(open_heap, (f, nb))

        self.get_logger().warn('A*: No path found!')
        return None

    def cells_to_path_msg(self, cells):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp    = self.get_clock().now().to_msg()
        for (row, col) in cells:
            wx, wy = self.grid_to_world(row, col)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def go_to(self, goal_wx, goal_wy):
        path = self.plan(goal_wx, goal_wy)
        if path:
            self.path_pub.publish(path)
            self.get_logger().info(
                f'Path: {len(path.poses)} waypoints → '
                f'({goal_wx:.2f}, {goal_wy:.2f})')
        return path


def main():
    rclpy.init()
    node = SlamPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()