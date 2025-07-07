# This script benchmarks the Kinematic controller on the Sys3WRobot system.
# It performs 10 simulations with different gain sets and saves trajectory, orientation,
# and control input plots for each run in the kinematic_results/ directory.

import numpy as np
import matplotlib.pyplot as plt
import os

from rcognita.controllers import CtrlKinematic
from rcognita.systems import Sys3WRobot

# Create output folder
os.makedirs("kinematic_results", exist_ok=True)

# Simulation settings
sampling_time = 0.1
Tfinal = 20.0

# Initial and goal states
x0 = np.array([0.0, 0.0, 0.0])
x_goal = np.array([2.0, 2.0, 0.0])

# Initialize system (3W robot)
# We only simulate the kinematic motion model here
system = Sys3WRobot(
    sys_type="diff_eqn",
    dim_state=3,
    dim_input=2,
    dim_output=3,
    dim_disturb=0,
    pars=[],
    ctrl_bnds=np.array([[0.0, 1.0], [-1.0, 1.0]]),
    is_dyn_ctrl=0,
    is_disturb=0
)

# Define gain sets to test
gain_sets = {
    "set1": {"k_rho": 0.8, "k_alpha": 3.0, "k_beta": -1.0},
    "set2": {"k_rho": 1.0, "k_alpha": 3.5, "k_beta": -1.5},
    "set3": {"k_rho": 1.2, "k_alpha": 4.0, "k_beta": -2.0},
    "set4": {"k_rho": 1.5, "k_alpha": 4.5, "k_beta": -2.5},
    "set5": {"k_rho": 1.8, "k_alpha": 5.0, "k_beta": -3.0},
    "set6": {"k_rho": 2.0, "k_alpha": 5.5, "k_beta": -3.5},
    "set7": {"k_rho": 2.2, "k_alpha": 6.0, "k_beta": -4.0},
    "set8": {"k_rho": 2.5, "k_alpha": 6.5, "k_beta": -4.5},
    "set9": {"k_rho": 2.8, "k_alpha": 7.0, "k_beta": -5.0},
    "set10": {"k_rho": 3.0, "k_alpha": 7.5, "k_beta": -5.5}
}

# Store all trajectories for comparison
plt.figure()
for label, gains in gain_sets.items():
    print(f"Running simulation {label}...")
    ctrl = CtrlKinematic(
        k_rho=gains["k_rho"],
        k_alpha=gains["k_alpha"],
        k_beta=gains["k_beta"],
        ctrl_bnds=np.array([[0.0, 1.0], [-1.0, 1.0]]),
        t0=0.0,
        sampling_time=sampling_time
    )

    x = x0.copy()
    t = 0.0
    x_log = [x.copy()]

    while t <= Tfinal:
        observation = np.concatenate([x, x_goal])
        u = ctrl.compute_action(t, observation)
        x = x + sampling_time * np.array([
            u[0] * np.cos(x[2]),
            u[0] * np.sin(x[2]),
            u[1]
        ])
        x_log.append(x.copy())
        t += sampling_time

    x_log = np.array(x_log)
    plt.plot(x_log[:, 0], x_log[:, 1], label=label)

# Plot goal
plt.plot(x_goal[0], x_goal[1], 'ro', label="Goal")
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.title("Kinematic Trajectories - All Gain Sets")
plt.legend()
plt.grid()
plt.savefig("kinematic_results/all_trajectories.png")
plt.show()

print("\n Kinematic simulation summary complete. All trajectories saved in one figure.")