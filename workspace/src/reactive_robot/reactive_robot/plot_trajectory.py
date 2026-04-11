import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('trajectory.csv')

x = df['x'].to_numpy()
y = df['y'].to_numpy()

plt.figure(figsize=(8, 8))
plt.plot(x, y, color='blue', linewidth=1.5, label='Robot trajectory')
plt.scatter(x[0], y[0], color='green', s=100, zorder=5, label='Start')
plt.scatter(x[-1], y[-1], color='red', s=100, zorder=5, label='Stop')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Robot Trajectory')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.tight_layout()
plt.savefig('trajectory.png', dpi=150)
plt.show()

print(f'Start: x={x[0]:.2f} y={y[0]:.2f}')
print(f'Stop:  x={x[-1]:.2f} y={y[-1]:.2f}')
print(f'Total points: {len(df)}')