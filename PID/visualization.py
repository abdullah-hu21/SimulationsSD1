import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D

def plot_results(time, states, motor_thrusts):
    phi = states[:, 0]
    theta = states[:, 1]
    psi = states[:, 2]

    plt.figure(figsize=(12, 10))

    # Subplot 1: Roll
    plt.subplot(2, 2, 1)
    plt.plot(time, phi, label="Roll (phi)")
    plt.axhline(y=0, color='r', linestyle='--', label="Desired Roll")
    plt.legend(loc='upper right')  
    plt.ylabel("Angle (rad)")
    plt.title("Roll (phi)")
    plt.xlabel("Time (s)")

    # Subplot 2: Pitch
    plt.subplot(2, 2, 2)
    plt.plot(time, theta, label="Pitch (theta)")
    plt.axhline(y=0, color='r', linestyle='--', label="Desired Pitch")
    plt.legend(loc='upper right')  
    plt.ylabel("Angle (rad)")
    plt.title("Pitch (theta)")
    plt.xlabel("Time (s)")

    # Subplot 3: Yaw
    plt.subplot(2, 2, 3)
    plt.plot(time, psi, label="Yaw (psi)")
    plt.axhline(y=0, color='r', linestyle='--', label="Desired Yaw")
    plt.legend(loc='upper right')  
    plt.ylabel("Angle (rad)")
    plt.title("Yaw (psi)")
    plt.xlabel("Time (s)")

    plt.subplot(2, 2, 4)
    plt.plot(time, motor_thrusts[:, 0], label="Motor 1 Thrust")
    plt.plot(time, motor_thrusts[:, 1], label="Motor 2 Thrust")
    plt.plot(time, motor_thrusts[:, 2], label="Motor 3 Thrust")
    plt.plot(time, motor_thrusts[:, 3], label="Motor 4 Thrust")
    plt.ylabel("Thrust (N)")
    plt.xlabel("Time (s)")
    plt.legend(loc='upper right')
    plt.title("Motor Thrusts (Active)")

    plt.tight_layout()
    plt.savefig("Plots", dpi=150)
    plt.show()


def plot_quadcopter_3d(ax, x, y, z, phi, theta, psi, L, thrusts, current_time, states):
    L_visual = 1.5  # Set a larger visual arm length
    arms = np.array([[L_visual, 0, 0], [-L_visual, 0, 0], [0, L_visual, 0], [0, -L_visual, 0]])
    R_roll = np.array([[1, 0, 0],
                       [0, np.cos(phi), -np.sin(phi)],
                       [0, np.sin(phi), np.cos(phi)]])
    R_pitch = np.array([[np.cos(theta), 0, np.sin(theta)],
                        [0, 1, 0],
                        [-np.sin(theta), 0, np.cos(theta)]])
    R_yaw = np.array([[np.cos(psi), -np.sin(psi), 0],
                      [np.sin(psi), np.cos(psi), 0],
                      [0, 0, 1]])

    R = R_yaw @ R_pitch @ R_roll
    rotated_arms = arms @ R.T + np.array([x, y, z])

    ax.cla()
    ax.plot([rotated_arms[0, 0], rotated_arms[1, 0]],
            [rotated_arms[0, 1], rotated_arms[1, 1]],
            [rotated_arms[0, 2], rotated_arms[1, 2]], 'r', lw=8)
    ax.plot([rotated_arms[2, 0], rotated_arms[3, 0]],
            [rotated_arms[2, 1], rotated_arms[3, 1]],
            [rotated_arms[2, 2], rotated_arms[3, 2]], 'b', lw=8)

    # Add motors and thrust vectors
    motor_positions = [rotated_arms[i] for i in range(4)]
    for pos in motor_positions:
        ax.scatter(pos[0], pos[1], pos[2], color='orange', s=600, edgecolors='k', alpha=0.9)

    for i, pos in enumerate(motor_positions):
        thrust_vector = np.array([0, 0, thrusts[i]]) / 50
        ax.quiver(pos[0], pos[1], pos[2], thrust_vector[0], thrust_vector[1], thrust_vector[2],
                  color='g', arrow_length_ratio=0.3, linewidth=3)

    # Dynamically update z-limits based on altitude
    z_min = np.min(states[:, 8]) - 2  # Add margin for better visualization
    z_max = np.max(states[:, 8]) + 2  # Add margin for better visualization
    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([z_min, z_max])
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')
    ax.set_title(f"Quadcopter 3D Visualization (Time: {current_time:.2f}s)")
