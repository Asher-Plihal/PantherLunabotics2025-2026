#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import pandas as pd

# Read data
trajectory = pd.read_csv('trajectory.csv')
controls = pd.read_csv('controls.csv')
errors = pd.read_csv('errors.csv')
obstacles = pd.read_csv('obstacles.csv')

# Create figure with subplots
fig = plt.figure(figsize=(15, 10))

# 1. Trajectory plot with obstacles
ax1 = plt.subplot(2, 3, 1)
ax1.plot(trajectory['x'], trajectory['y'], 'b-', linewidth=2, label='Trajectory')
ax1.plot(trajectory['x'].iloc[0], trajectory['y'].iloc[0], 'go', markersize=10, label='Start')
ax1.plot(trajectory['x'].iloc[-1], trajectory['y'].iloc[-1], 'r*', markersize=15, label='End')

# Plot obstacles
for _, obs in obstacles.iterrows():
    circle = Circle((obs['x'], obs['y']), obs['radius'], color='red', alpha=0.3)
    ax1.add_patch(circle)
    ax1.plot(obs['x'], obs['y'], 'rx', markersize=8)

ax1.set_xlabel('X Position (m)')
ax1.set_ylabel('Y Position (m)')
ax1.set_title('Robot Trajectory with Obstacles')
ax1.legend()
ax1.grid(True)
ax1.axis('equal')

# 2. X-Y position over time
ax2 = plt.subplot(2, 3, 2)
time = np.arange(len(trajectory)) * 0.05  # dt = 0.05
ax2.plot(time, trajectory['x'], 'b-', label='X position')
ax2.plot(time, trajectory['y'], 'r-', label='Y position')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Position (m)')
ax2.set_title('Position vs Time')
ax2.legend()
ax2.grid(True)

# 3. Theta and velocity over time
ax3 = plt.subplot(2, 3, 3)
ax3_twin = ax3.twinx()
ax3.plot(time, np.rad2deg(trajectory['theta']), 'g-', label='Theta')
ax3_twin.plot(time, trajectory['v'], 'purple', linestyle='--', label='Velocity')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Heading Angle (deg)', color='g')
ax3_twin.set_ylabel('Velocity (m/s)', color='purple')
ax3.set_title('Heading and Velocity vs Time')
ax3.tick_params(axis='y', labelcolor='g')
ax3_twin.tick_params(axis='y', labelcolor='purple')
ax3.grid(True)

# 4. Control inputs - Acceleration
ax4 = plt.subplot(2, 3, 4)
time_control = np.arange(len(controls)) * 0.05
ax4.plot(time_control, controls['acceleration'], 'b-', linewidth=1.5)
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Acceleration (m/s²)')
ax4.set_title('Control Input: Linear Acceleration')
ax4.grid(True)

# 5. Control inputs - Angular velocity
ax5 = plt.subplot(2, 3, 5)
ax5.plot(time_control, controls['angular_velocity'], 'r-', linewidth=1.5)
ax5.set_xlabel('Time (s)')
ax5.set_ylabel('Angular Velocity (rad/s)')
ax5.set_title('Control Input: Angular Velocity')
ax5.grid(True)

# 6. Tracking errors
ax6 = plt.subplot(2, 3, 6)
time_error = np.arange(len(errors)) * 0.05
ax6.plot(time_error, errors['position_error'], 'b-', label='Position Error', linewidth=1.5)
ax6_twin = ax6.twinx()
ax6_twin.plot(time_error, np.rad2deg(errors['angular_error']), 'r-', label='Angular Error', linewidth=1.5)
ax6.set_xlabel('Time (s)')
ax6.set_ylabel('Position Error (m)', color='b')
ax6_twin.set_ylabel('Angular Error (deg)', color='r')
ax6.set_title('Tracking Errors')
ax6.tick_params(axis='y', labelcolor='b')
ax6_twin.tick_params(axis='y', labelcolor='r')
ax6.grid(True)

plt.tight_layout()
plt.savefig('simulation_results.png', dpi=300, bbox_inches='tight')
print("Plot saved as 'simulation_results.png'")
plt.show()