# config.py

# Quadcopter parameters
Ixx, Iyy, Izz = 0.018, 0.020, 0.025  # Moments of inertia
m = 2.3  # Mass of quadcopter in kg
g = 9.81  # Gravitational constant (m/s^2)
L = 0.25  # Arm length (center to motor)
k = 0.01  # Yaw control constant

# Initial state [phi, theta, psi, p, q, r, x, y, z, vx, vy, vz]
# The quadcopter starts moving at 5 km/h (1.388 m/s) in the x-direction
initial_state = [1.5, 1.5, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0, -10.0, 1.388, 0.0, 0.0]


# Simulation parameters
timestep = 0.0002
duration = 5

# Desired angles (setpoints)
desired_phi = 0
desired_theta = 0
desired_psi = 0
