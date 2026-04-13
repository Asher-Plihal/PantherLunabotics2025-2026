# BEV Mapper

Bird's-Eye-View (BEV) mapper for converting D435i RealSense point cloud data into a 2D occupancy grid.

## Overview

The BEV mapper subscribes to a D435i point cloud topic and projects 3D points onto a 2D top-down grid. It outputs:
- **OccupancyGrid** message: Standard ROS occupancy grid (0-100 scale)
- **Image** message: Visualization of the BEV map (white=free, black=occupied)
- **PNG file**: Saved to `/tmp/bev_map.png` for debugging

## Running

### Option 1: Direct command
```bash
ros2 run panther_slam bev_mapper
```

### Option 2: Launch file
```bash
ros2 launch panther_slam bev_mapper.launch.py
```

### Option 3: With custom parameters
```bash
ros2 launch panther_slam bev_mapper.launch.py \
  bev_size:=512 \
  bev_range:=5.0 \
  height_min:=0.05 \
  height_max:=0.8
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `bev_size` | 256 | Grid resolution (pixels × pixels) |
| `bev_range` | 3.0 | Coverage area: ±X meters (6m × 6m total) |
| `height_min` | 0.0 | Minimum Z height to include (meters) |
| `height_max` | 1.0 | Maximum Z height to include (meters) |
| `occupancy_threshold` | 5 | Points needed to mark cell occupied |
| `pointcloud_topic` | `/camera/depth/color/points` | Input point cloud topic |
| `output_topic` | `/bev_map` | Output occupancy grid topic |

## Topics

### Subscriptions
- `/camera/depth/color/points` (sensor_msgs/PointCloud2): D435i depth point cloud

### Publications
- `/bev_map` (nav_msgs/OccupancyGrid): Bird's-eye-view occupancy grid
- `/bev_map_image` (sensor_msgs/Image): Visualization image (RGB)

## Coordinate Frame

- **X-axis**: Camera's forward direction (depth)
- **Y-axis**: Camera's left/right direction (horizontal)
- **Grid center (0,0)**: At camera position, looking forward

## Visualization

In RViz:
1. Add an Image display → subscribe to `/bev_map_image`
2. Add a Map display → subscribe to `/bev_map`

Or view the PNG: `cat /tmp/bev_map.png`

## Tuning

### Too much noise?
- Increase `occupancy_threshold` (e.g., 10, 20)
- Narrow `height_min`/`height_max` range
- Reduce `bev_range` for higher resolution

### Not detecting obstacles?
- Decrease `occupancy_threshold` (e.g., 1, 2)
- Widen `height_min`/`height_max` range
- Increase `bev_size` for finer granularity

### Coverage too small/large?
- Decrease/increase `bev_range` (±meters)
