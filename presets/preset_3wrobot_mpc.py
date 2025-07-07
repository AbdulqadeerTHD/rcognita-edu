import numpy as np
import matplotlib.pyplot as plt
import os
from rcognita.controllers import CtrlMPC
from rcognita.systems import Sys3WRobot

# === Create results folder ===
os.makedirs("mpc_results", exist_ok=True)

# === Simulation Settings ===
sampling_time = 0.1
Tfinal = 20.0
x0 = np.array([0.0, 0.0, 0.0])
x_goal = np.array([2.0, 2.0, 0.0])

# === System Initialization ===
system = Sys3WRobot(
    sys_type="diff_drive",
    dim_state=3,
    dim_input=2,
    dim_output=3,
    dim_disturb=0
)
system.dt = sampling_time

# === MPC parameter sets ===
mpc_sets = {
    "set1":  {"N": 10,  "Q": np.diag([4, 4, 0.2]),   "R": np.diag([0.5, 0.5]),   "Qf": np.diag([6, 6, 0.3])},
    "set2":  {"N": 15,  "Q": np.diag([6, 6, 0.3]),   "R": np.diag([0.3, 0.3]),   "Qf": np.diag([8, 8, 0.4])},
    "set3":  {"N": 20,  "Q": np.diag([8, 8, 0.4]),   "R": np.diag([0.2, 0.2]),   "Qf": np.diag([10, 10, 0.5])},
    "set4":  {"N": 25,  "Q": np.diag([10, 10, 0.5]), "R": np.diag([0.15, 0.15]), "Qf": np.diag([12, 12, 0.6])},
    "set5":  {"N": 30,  "Q": np.diag([12, 12, 0.6]), "R": np.diag([0.1, 0.1]),   "Qf": np.diag([15, 15, 0.8])},
    "set6":  {"N": 35,  "Q": np.diag([14, 14, 0.7]), "R": np.diag([0.08, 0.08]), "Qf": np.diag([18, 18, 1.0])},
    "set7":  {"N": 40,  "Q": np.diag([16, 16, 0.8]), "R": np.diag([0.06, 0.06]), "Qf": np.diag([20, 20, 1.2])},
    "set8":  {"N": 45,  "Q": np.diag([18, 18, 1.0]), "R": np.diag([0.05, 0.05]), "Qf": np.diag([24, 24, 1.4])},
    "set9":  {"N": 50,  "Q": np.diag([20, 20, 1.2]), "R": np.diag([0.03, 0.03]), "Qf": np.diag([28, 28, 1.5])},
    "set10": {"N": 60,  "Q": np.diag([25, 25, 1.5]), "R": np.diag([0.01, 0.01]), "Qf": np.diag([30, 30, 2.0])}
}


# === Combined plot setup ===
plt.figure(figsize=(8, 6))

# === Run simulations ===
for label, params in mpc_sets.items():
    print(f"Running simulation {label}...")
    ctrl = CtrlMPC(
        N=params["N"],
        Q=params["Q"],
        R=params["R"],
        Qf=params["Qf"],
        sampling_time=sampling_time
    )

    x = x0.copy()
    t = 0.0
    x_log = [x.copy()]
    u_log = []
    t_log = [t]

    while t < Tfinal:
        obs = np.concatenate([x, x_goal])
        u = ctrl.compute_action(t, obs)

        v, omega = u
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

    # === Individual Plots ===
    plt.figure()
    plt.plot(x_log[:, 0], x_log[:, 1], label='Trajectory')
    plt.plot(x_goal[0], x_goal[1], 'ro', label='Goal')
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title(f"MPC Trajectory ({label})")
    plt.legend()
    plt.grid()
    plt.savefig(f"mpc_results/mpc_traj_{label}.png")
    plt.close()

    plt.figure()
    plt.plot(t_log, x_log[:-1, 2])
    plt.xlabel("Time [s]")
    plt.ylabel("Theta [rad]")
    plt.title(f"Orientation Over Time ({label})")
    plt.grid()
    plt.savefig(f"mpc_results/mpc_orient_{label}.png")
    plt.close()

    plt.figure()
    plt.plot(t_log, u_log[:, 0], label="v (linear)")
    plt.plot(t_log, u_log[:, 1], label="omega (angular)")
    plt.xlabel("Time [s]")
    plt.ylabel("Control Inputs")
    plt.title(f"Control Inputs Over Time ({label})")
    plt.legend()
    plt.grid()
    plt.savefig(f"mpc_results/mpc_control_{label}.png")
    plt.close()

    # === Add to combined plot ===
    plt.figure(1)
    plt.plot(x_log[:, 0], x_log[:, 1], label=label)

# === Final combined trajectory plot ===
plt.plot(x_goal[0], x_goal[1], 'ro', label='Goal')
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.title("MPC All Trajectories")
plt.legend()
plt.grid()
plt.savefig("mpc_results/mpc_all_trajectories.png")
plt.close()

print("\n MPC simulations complete. All plots saved in 'mpc_results/' folder.")