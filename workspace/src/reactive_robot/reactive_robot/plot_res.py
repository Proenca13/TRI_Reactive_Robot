import re
import matplotlib.pyplot as plt

LOG_FILE = 'run_log.txt'

times_wall = []
wall_errors = []
heading_errors = []

times_center = []
dist_to_center = []
angle_to_exit = []

start_time = None

with open(LOG_FILE, 'r') as f:
    for line in f:
        # Extract timestamp
        ts_match = re.search(r'\[(\d+\.\d+)\]', line)
        if not ts_match:
            continue
        ts = float(ts_match.group(1))
        if start_time is None:
            start_time = ts
        t = ts - start_time

        # Wall error
        wall_match = re.search(r'wall_error=([-\d.]+).*heading_error=([-\d.]+)', line)
        if wall_match:
            times_wall.append(t)
            wall_errors.append(float(wall_match.group(1)))
            heading_errors.append(float(wall_match.group(2)))

        # Center distance
        center_match = re.search(r'dist_to_center=([-\d.]+).*angle_to_exit=([-\d.]+)', line)
        if center_match:
            times_center.append(t)
            dist_to_center.append(float(center_match.group(1)))
            angle_to_exit.append(float(center_match.group(2)))

# --- Plot 1: Wall Following Error ---
fig, ax = plt.subplots(figsize=(10, 4))
ax.plot(times_wall, wall_errors, label='Distance Error (m)', color='blue')
ax.plot(times_wall, heading_errors, label='Heading Error (m)', color='orange')
ax.axhline(0, color='red', linestyle='--', label='Target')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Error (m)')
ax.set_title('Wall Following Error over Time')
ax.legend()
ax.grid(True)
plt.tight_layout()
plt.savefig('wall_following_error.png', dpi=150)
plt.show()

# --- Plot 2: Centering Progress ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
ax1.plot(times_center, dist_to_center, color='green', label='Distance to Center (m)')
ax1.axhline(0.15, color='red', linestyle='--', label='Tolerance (0.15m)')
ax1.set_ylabel('Distance (m)')
ax1.set_title('Centering Progress inside Circle')
ax1.legend()
ax1.grid(True)

ax2.plot(times_center, angle_to_exit, color='purple', label='Angle to Exit (rad)')
ax2.axhline(0.2, color='red', linestyle='--', label='Tolerance (0.2 rad)')
ax2.axhline(-0.2, color='red', linestyle='--')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Angle (rad)')
ax2.legend()
ax2.grid(True)

plt.tight_layout()
plt.savefig('centering_progress.png', dpi=150)
plt.show()

print(f'Wall following samples: {len(times_wall)}')
print(f'Centering samples: {len(times_center)}')