# dynamics.py

import numpy as np

def quadcopter_dynamics(state, t, u, Ixx, Iyy, Izz, m, g, wind_forces, wind_torques):
    """
    Quadcopter dynamics with optional wind disturbances.
    """
    # Unpack state variables
    phi, theta, psi, p, q, r, x, y, z, vx, vy, vz = state

    # Torques (roll, pitch, yaw)
    tau_phi = u[0] + wind_torques[0]
    tau_theta = u[1] + wind_torques[1]
    tau_psi = u[2] + wind_torques[2]

    # Angular accelerations
    p_dot = tau_phi / Ixx
    q_dot = tau_theta / Iyy
    r_dot = tau_psi / Izz

    # Angular velocities to Euler angles rates
    phi_dot = p + np.sin(phi) * np.tan(theta) * q + np.cos(phi) * np.tan(theta) * r
    theta_dot = np.cos(phi) * q - np.sin(phi) * r
    psi_dot = (np.sin(phi) / np.cos(theta)) * q + (np.cos(phi) / np.cos(theta)) * r

    # Translational accelerations
    thrust = np.array([
        -np.sin(theta),
        np.sin(phi) * np.cos(theta),
        np.cos(phi) * np.cos(theta)
    ]) * (u[3] / m)

    ax = thrust[0] + wind_forces[0]
    ay = thrust[1] + wind_forces[1]
    az = thrust[2] - g + wind_forces[2]

    # Velocities
    x_dot = vx
    y_dot = vy
    z_dot = vz

    return [phi_dot, theta_dot, psi_dot, p_dot, q_dot, r_dot, x_dot, y_dot, z_dot, ax, ay, az]


def compute_motor_thrust(control_inputs, total_thrust):
    """
    Compute thrust for each motor based on control inputs and total thrust.
    """
    u_phi, u_theta, u_psi = control_inputs
    thrusts = np.array([
        total_thrust * (1 + u_phi + u_theta),  # Motor 1
        total_thrust * (1 - u_phi + u_theta),  # Motor 2
        total_thrust * (1 - u_phi - u_theta),  # Motor 3
        total_thrust * (1 + u_phi - u_theta)   # Motor 4
    ])
    return thrusts
