import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import numpy as np
import heapq

class SlamPlanner(Node):
    def __init__(self):
        super().__init__('slam_planner')

        self.map_data = None
        self.map_info = None
        self.start = None  # (row, col) in grid coords

        # Subscribe to rtabmap's occupancy grid
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        # Subscribe to current pose from rtabmap odometry
        self.pose_sub = self.create_subscription(
            PoseStamped, '/rtabmap/localization_pose', self.pose_callback, 10)

        # Publish the planned path
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        self.get_logger().info('SlamPlanner ready. Waiting for map...')

    # ------------------------------------------------------------------
    # Map callback — converts OccupancyGrid to a numpy array
    # ------------------------------------------------------------------
    def map_callback(self, msg: OccupancyGrid):
        self.map_info = msg.info  # resolution, origin, width, height
        width  = msg.info.width
        height = msg.info.height

        # Reshape flat array → 2D grid
        # Values: 0 = free, 100 = occupied, -1 = unknown
        raw = np.array(msg.data, dtype=np.int8).reshape((height, width))
        self.map_data = raw
        self.get_logger().info(
            f'Map received: {width}x{height}, '
            f'resolution={msg.info.resolution:.3f}m/cell')

    # ------------------------------------------------------------------
    # Pose callback — converts world pose → grid cell
    # ------------------------------------------------------------------
    def pose_callback(self, msg: PoseStamped):
        if self.map_info is None:
            return
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.start = self.world_to_grid(x, y)

    # ------------------------------------------------------------------
    # Coordinate helpers
    # ------------------------------------------------------------------
    def world_to_grid(self, wx, wy):
        """Convert world (x, y) in metres → grid (row, col)."""
        res    = self.map_info.resolution
        origin = self.map_info.origin.position
        col = int((wx - origin.x) / res)
        row = int((wy - origin.y) / res)
        return (row, col)

    def grid_to_world(self, row, col):
        """Convert grid (row, col) → world (x, y) in metres."""
        res    = self.map_info.resolution
        origin = self.map_info.origin.position
        wx = col * res + origin.x + res / 2.0
        wy = row * res + origin.y + res / 2.0
        return (wx, wy)

    def is_free(self, row, col):
        """True if cell is within bounds and not occupied/unknown."""
        h, w = self.map_data.shape
        if row < 0 or col < 0 or row >= h or col >= w:
            return False
        return self.map_data[row, col] == 0   # 0 = free

    # ------------------------------------------------------------------
    # A* path planner
    # ------------------------------------------------------------------
    def plan(self, goal_wx, goal_wy):
        """
        Plan a path from current robot pose to (goal_wx, goal_wy).
        Returns a nav_msgs/Path or None if planning fails.
        """
        if self.map_data is None:
            self.get_logger().warn('No map yet!')
            return None
        if self.start is None:
            self.get_logger().warn('No pose yet!')
            return None

        goal = self.world_to_grid(goal_wx, goal_wy)
        start = self.start

        if not self.is_free(*goal):
            self.get_logger().warn(f'Goal cell {goal} is occupied or unknown!')
            return None

        # -- A* --
        def heuristic(a, b):
            return abs(a[0]-b[0]) + abs(a[1]-b[1])  # Manhattan

        open_heap = []
        heapq.heappush(open_heap, (0, start))
        came_from = {start: None}
        g_score   = {start: 0}

        # 8-connected neighbours
        neighbours = [(-1,0),(1,0),(0,-1),(0,1),
                      (-1,-1),(-1,1),(1,-1),(1,1)]

        while open_heap:
            _, current = heapq.heappop(open_heap)

            if current == goal:
                # Reconstruct path
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

    # ------------------------------------------------------------------
    # Convert list of grid cells → nav_msgs/Path
    # ------------------------------------------------------------------
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

    # ------------------------------------------------------------------
    # Public method — call this with a goal to get + publish a path
    # ------------------------------------------------------------------
    def go_to(self, goal_wx, goal_wy):
        path = self.plan(goal_wx, goal_wy)
        if path:
            self.path_pub.publish(path)
            self.get_logger().info(
                f'Path published: {len(path.poses)} waypoints '
                f'→ goal ({goal_wx:.2f}, {goal_wy:.2f})')
        return path


def main():
    rclpy.init()
    node = SlamPlanner()

    # Example: plan to a goal 2 metres ahead once map is ready
    import time
    rclpy.spin_once(node, timeout_sec=2.0)   # wait for first map
    time.sleep(1.0)

    path = node.go_to(goal_wx=2.0, goal_wy=0.0)

    rclpy.spin(node)   # keep running to receive map updates
    rclpy.shutdown()


if __name__ == '__main__':
    main()