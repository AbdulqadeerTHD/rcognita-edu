import numpy as np
import matplotlib.pyplot as plt
import os
from rcognita.controllers import CtrlLQR
from rcognita.systems import Sys3WRobot

# === Create result folder ===
os.makedirs("lqr_results", exist_ok=True)

# === Simulation settings ===
sampling_time = 0.1
Tfinal = 20.0
x0 = np.array([0.0, 0.0, 0.0])
x_goal = np.array([2.0, 2.0, 0.0])

# === Manually define linearized unicycle system matrices (at θ = 0) ===
A = np.array([
    [1.0, 0.0, -sampling_time],
    [0.0, 1.0,  sampling_time],
    [0.0, 0.0,  1.0]
])

B = np.array([
    [sampling_time, 0.0],
    [0.0,           0.0],
    [0.0,    sampling_time]
])

# === Initialize system for consistency ===
system = Sys3WRobot(
    sys_type="diff_drive",
    dim_state=3,
    dim_input=2,
    dim_output=3,
    dim_disturb=0
)
system.dt = sampling_time

# === 10 tuned LQR sets with optimized set10 ===
lqr_sets = {
    "set1":  {"Q": np.diag([1, 1, 0.1]),   "R": np.diag([0.1, 0.1])},
    "set2":  {"Q": np.diag([2, 2, 0.2]),   "R": np.diag([0.2, 0.2])},
    "set3":  {"Q": np.diag([4, 4, 0.4]),   "R": np.diag([0.3, 0.3])},
    "set4":  {"Q": np.diag([6, 6, 0.5]),   "R": np.diag([0.4, 0.3])},
    "set5":  {"Q": np.diag([8, 8, 0.6]),   "R": np.diag([0.5, 0.4])},
    "set6":  {"Q": np.diag([10, 10, 0.7]), "R": np.diag([0.5, 0.5])},
    "set7":  {"Q": np.diag([12, 12, 0.8]), "R": np.diag([0.4, 0.6])},
    "set8":  {"Q": np.diag([14, 14, 1.0]), "R": np.diag([0.3, 0.6])},
    "set9":  {"Q": np.diag([16, 16, 1.2]), "R": np.diag([0.2, 0.5])},
    "set10": {"Q": np.diag([25, 25, 5.0]), "R": np.diag([0.01, 0.01])}  # Optimized
}

# === For final combined plot ===
plt.figure(figsize=(8, 6))

# === Run simulations ===
for label, params in lqr_sets.items():
    ctrl = CtrlLQR(A, B, Q=params["Q"], R=params["R"], sampling_time=sampling_time)

    x = x0.copy()
    t = 0.0
    x_log = [x.copy()]
    u_log = []
    t_log = [t]

    while t < Tfinal:
        state_error = x - x_goal
        u = ctrl.compute_action(state_error)

        v = np.clip(u[0], 0.0, 1.0)
        omega = np.clip(u[1], -1.0, 1.0)
        u = np.array([v, omega])

        theta = x[2]
        x[0] += sampling_time * v * np.cos(theta)
        x[1] += sampling_time * v * np.sin(theta)
        x[2] += sampling_time * omega

        x_log.append(x.copy())
        u_log.append(u.copy())
        t += sampling_time
        t_log.append(t)

    x_log = np.array(x_log)
    u_log = np.array(u_log)
    t_log = np.array(t_log[:-1])

    # === Plot 1: Trajectory ===
    plt.figure()
    plt.plot(x_log[:, 0], x_log[:, 1], label="Trajectory")
    plt.plot(x_goal[0], x_goal[1], "ro", label="Goal")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title(f"LQR Trajectory ({label})")
    plt.legend()
    plt.grid()
    plt.savefig(f"lqr_results/lqr_traj_{label}.png")
    plt.close()

    # Add to combined plot
    plt.figure(1)
    plt.plot(x_log[:, 0], x_log[:, 1], label=label)

    # === Plot 2: Orientation ===
    plt.figure()
    plt.plot(t_log, x_log[:-1, 2])
    plt.xlabel("Time [s]")
    plt.ylabel("Theta [rad]")
    plt.title(f"Orientation Over Time ({label})")
    plt.grid()
    plt.savefig(f"lqr_results/lqr_orient_{label}.png")
    plt.close()

    # === Plot 3: Controls ===
    plt.figure()
    plt.plot(t_log, u_log[:, 0], label="v (linear)")
    plt.plot(t_log, u_log[:, 1], label="omega (angular)")
    plt.xlabel("Time [s]")
    plt.ylabel("Control Inputs")
    plt.title(f"Control Inputs Over Time ({label})")
    plt.legend()
    plt.grid()
    plt.savefig(f"lqr_results/lqr_control_{label}.png")
    plt.close()

# Save combined plot
plt.figure(1)
plt.plot(x_goal[0], x_goal[1], 'ro', label='Goal')
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.title("All LQR Trajectories")
plt.legend()
plt.grid()
plt.savefig("lqr_results/lqr_all_trajectories.png")

print("LQR simulations complete. All 10 result sets and combined plot saved in 'lqr_results/' folder.")
