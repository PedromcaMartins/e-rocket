import numpy as np

def controller(delta_theta, delta_theta_desired, delta_omega):
    """
    Compute the control algorithm.

    Parameters:
        delta_theta (float): Current pitch.
        delta_theta_desired (float): Desired pitch.
        delta_omega (float): Angular velocity.

    Returns:
        delta_gamma (float): Control output.
        zeta_theta (float): Updated integrated pitch tracking error.
    """

    k_p=0.3350 # k_p (float): Proportional gain.
    k_d=0.1616 # k_d (float): Derivative gain.
    k_i=0.3162 # k_i (float): Integral gain.
    dt=0.02 # dt  (float): Time step for integration at 50Hz.

    # Update the integrated error
    global zeta_theta
    zeta_theta = zeta_theta + (delta_theta_desired - delta_theta) * dt

    # State vector
    x_pd = np.array([delta_theta, delta_omega])

    # Gain vector
    K_pd = np.array([k_p, k_d])

    # Compute control input
    delta_gamma = -np.dot(K_pd, x_pd) + k_i * zeta_theta

    return delta_gamma

if __name__ == "__main__":
    # Constants
    m = 2.0     # Mass (kg)
    L = 0.5     # Length (m)
    g = 9.81    # Gravitational acceleration (m/s^2)
    j = 0.3750  # Moment of inertia (kg*m^2)

    # Initial parameters
    delta_theta = 0.0  # Current pitch
    delta_theta_desired = np.pi / 9  # Desired pitch
    delta_omega = 0.0  # Angular velocity

    # Call the controller function
    global zeta_theta
    zeta_theta = 0.0  # Initialize integrated error

    step = 0.02  # Time step (50 Hz)
    import matplotlib.pyplot as plt

    # Initialize lists to store values over time
    time = []
    delta_gamma_values = []
    delta_omega_values = []
    delta_theta_values = []

    # Simulation loop
    for i in range(500):  # Simulate for 500 steps (10 seconds at 50 Hz)
        delta_gamma = controller(delta_theta, delta_theta_desired, delta_omega)
        a = delta_gamma * m * L * g / j
        delta_omega = delta_omega + a * step
        delta_theta = delta_theta + delta_omega * step

        # Store values
        time.append(i * step)
        delta_gamma_values.append(delta_gamma)
        delta_omega_values.append(delta_omega)
        delta_theta_values.append(delta_theta)

    # Plot results
    plt.figure(figsize=(10, 6))
    plt.plot(time, delta_gamma_values, label="delta_gamma")
    plt.plot(time, delta_omega_values, label="delta_omega")
    plt.plot(time, delta_theta_values, label="delta_theta")
    plt.xlabel("Time (s)")
    plt.ylabel("Values")
    plt.title("Controller Outputs Over Time")
    plt.legend()
    plt.grid()
    plt.show()


