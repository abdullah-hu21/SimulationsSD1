# main.py

import numpy as np
from scipy.integrate import odeint
from dynamics import quadcopter_dynamics, compute_motor_thrust
from control import QuadcopterPID, wrap_angle
from visualization import plot_results, plot_quadcopter_3d
from performance import compute_performance_metrics
from config import *
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from datetime import datetime
import matplotlib
matplotlib.rcParams['animation.ffmpeg_path'] = r'C:\Users\Admin\Desktop\drone detection\test_resultsOnnx\ffmpeg-2024-12-19-git-494c961379-essentials_build\bin\ffmpeg.exe'

# Time settings for simulation
time = np.linspace(0, duration, int(duration / timestep))

# Initialize PID controller
pid_controller = QuadcopterPID()

# User choice: Enable or disable PID control
use_pid = input("Would you like to use PID control? (yes/no): ").strip().lower() == 'yes'
# User choice: Enable or disable wind disturbance
use_wind = input("Would you like to include wind disturbance? (yes/no): ").strip().lower() == 'yes'

# Define simple constant or random wind values
if use_wind:
    wind_forces = np.array([0.5, 0.5, 0.05])  # Constant force in X, Y, Z
    wind_torques = np.array([0.1, 0.01, 0.005])  # Constant torque about roll, pitch, yaw
else:
    wind_forces = np.array([0.0, 0.0, 0.0])
    wind_torques = np.array([0.0, 0.0, 0.0])

# Define acceleration in x-direction (m/s²)
acceleration_x = 0.5 / 3.6  # Convert 0.5 km/h increase to m/s²

# Simulation loop
states = []
motor_thrusts = []
state = np.array(initial_state)  # Ensure state is a NumPy array

for t in time:
    if use_pid:
        # Compute control torques using PID
        u = pid_controller.control(state)  # Get PID control inputs (torques)
    else:
        # Without PID, set control torques to zero
        u = np.zeros(3)  # Zero torques for roll, pitch, yaw

    # Add wind torques to control torques
    u[:3] += wind_torques

    # Calculate additional thrust for acceleration
    additional_thrust_x = m * acceleration_x

    # Adjust the total thrust required (gravity + wind disturbance in Z)
    total_thrust = m * g + additional_thrust_x - wind_forces[2]
    u = np.append(u, total_thrust)

    # Compute motor thrusts (including wind effects)
    thrusts = compute_motor_thrust(u[:3], total_thrust)
    motor_thrusts.append(thrusts)

    # Integrate dynamics with thrust included
    result = odeint(
        quadcopter_dynamics,
        state,
        [t, t + timestep],
        args=(u, Ixx, Iyy, Izz, m, g, wind_forces, wind_torques)
    )

    # Take the last state from the integration results
    state = result[-1]

    # Update velocity in the x, y, and z directions
    state[9] += acceleration_x * timestep  # Update vx (index 9 corresponds to vx)
    state[10] += wind_forces[1] * timestep / m  # Update vy (index 10 corresponds to vy)
    state[11] += (total_thrust / m - g) * timestep  # Update vz (index 11 corresponds to vz)

    # Update position in x, y, and z directions
    state[6] += state[9] * timestep  # Update x (index 6 corresponds to x)
    state[7] += state[10] * timestep  # Update y (index 7 corresponds to y)
    state[8] += state[11] * timestep  # Update z (index 8 corresponds to z)
    
    # Ensure yaw angle is wrapped
    state[2] = wrap_angle(state[2])

    # Append state to the list of states
    states.append(state)

# Convert states and thrusts to numpy array for easier manipulation
states = np.array(states)
motor_thrusts = np.array(motor_thrusts)

# Plot results
plot_results(time, states, motor_thrusts)

# 3D Visualization (unchanged)
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection='3d')

def update_plot_bounds(ax, x, y, z):
    margin = 2  # Extra space around the drone's position
    ax.set_xlim([x - margin, x + margin])
    ax.set_ylim([y - margin, y + margin])
    ax.set_zlim([z - margin, z + margin])

def update(i):
    current_time = time[i]
    x, y, z = states[i, 6], states[i, 7], states[i, 8]
    phi, theta, psi = states[i, 0], states[i, 1], states[i, 2]
    plot_quadcopter_3d(ax, x, y, z, phi, theta, psi, L, motor_thrusts[i], current_time, states)  # Pass states
    update_plot_bounds(ax, x, y, z)

ani = animation.FuncAnimation(
    fig, update, frames=range(0, len(time), 25), interval=10
)
plt.show()

# Save animation as MP4
#output_path = 'C:/Users/Admin/Downloads/quadcopter_simulation.mp4'
#FFMpegWriter = animation.writers['ffmpeg']
#writer = FFMpegWriter(fps=30, metadata={'artist': 'Your Name'}, bitrate=1800)




# Compute performance metrics for roll, pitch, and yaw
roll_metrics = compute_performance_metrics(
    desired_phi, states[:, 0], time, initial_state[0])
pitch_metrics = compute_performance_metrics(
    desired_theta, states[:, 1], time, initial_state[1])
yaw_metrics = compute_performance_metrics(
    desired_psi, states[:, 2], time, initial_state[2])


# Prepare initial conditions and parameters for logging
initial_conditions = (
    "-------------------------------------------------------------------------\n"
    f"Initial Conditions and Parameters:\n"
    "-------------------------------------------------------------------------\n"
    f"Mass (m): {m:.2f} kg\n"
    f"Moments of Inertia: Ixx={Ixx:.3f}, Iyy={Iyy:.3f}, Izz={Izz:.3f} kg*m^2\n"
    f"Gravitational Constant (g): {g:.2f} m/s^2\n"
    f"Arm Length (L): {L:.2f} m\n"
    f"Yaw Control Constant (k): {k:.2f}\n"
    f"Initial State (angles and angular velocities): {initial_state} \n"
    f"Simulation Parameters:\n"
    f"Timestep: {timestep:.6f} s\n"
    f"Duration: {duration:.1f} s\n\n"
)

pid_params = (
    "-------------------------------------------------------------------------\n"
    f"PID Parameters:\n"
    "-------------------------------------------------------------------------\n"
    f"Roll: Kp={pid_controller.pid_roll.Kp}, Ki={pid_controller.pid_roll.Ki}, Kd={pid_controller.pid_roll.Kd}, "
    f"Output Limits={pid_controller.pid_roll.output_limits}\n"
    f"Pitch: Kp={pid_controller.pid_pitch.Kp}, Ki={pid_controller.pid_pitch.Ki}, Kd={pid_controller.pid_pitch.Kd}, "
    f"Output Limits={pid_controller.pid_pitch.output_limits}\n"
    f"Yaw: Kp={pid_controller.pid_yaw.Kp}, Ki={pid_controller.pid_yaw.Ki}, Kd={pid_controller.pid_yaw.Kd}, "
    f"Output Limits={pid_controller.pid_yaw.output_limits}\n\n"
)

# Prepare performance data for logging
performance_data = [
    "-------------------------------------------------------------------------\n"
    f"Performance Parameters:\n"
    "-------------------------------------------------------------------------\n"
    f"Performance Metrics for Roll (phi):\n"
    f"Steady-State Error: {roll_metrics[0]:.4f}, Rise Time: {roll_metrics[1]:.4f}s, "
    f"Settling Time: {roll_metrics[2]:.4f}s, Overshoot: {roll_metrics[3]:.4f}\n",

    f"Performance Metrics for Pitch (theta):\n"
    f"Steady-State Error: {pitch_metrics[0]:.4f}, Rise Time: {pitch_metrics[1]:.4f}s, "
    f"Settling Time: {pitch_metrics[2]:.4f}s, Overshoot: {pitch_metrics[3]:.4f}\n",

    f"Performance Metrics for Yaw (psi):\n"
    f"Steady-State Error: {yaw_metrics[0]:.4f}, Rise Time: {yaw_metrics[1]:.4f}s, "
    f"Settling Time: {yaw_metrics[2]:.4f}s, Overshoot: {yaw_metrics[3]:.4f}\n \n"
]

# Print results
for data in performance_data:
    print(data)

current_datetime = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

# Save performance metrics to log.txt
with open('log.txt', 'a') as log_file:
    log_file.write(f"Log Entry: {current_datetime}\n")
    log_file.write(initial_conditions)
    log_file.write(pid_params)
    log_file.writelines(performance_data)
