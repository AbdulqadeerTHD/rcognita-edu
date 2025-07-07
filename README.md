# Rcognita Benchmarking - TurtleBot3 (3W Robot) Simulation

This repository benchmarks **three control strategies** (Kinematic, LQR, MPC) for the TurtleBot3 using a simplified 3-wheel robot (3W) model inside the [Rcognita](https://github.com/thd-research/rcognita-edu) framework.

Simulations are containerized using Docker 

---

## Author

**Name**: Abdul Qadeeer
**Matriculation Number**: 12505033
**Email**: abdul.qadeer@stud.th-deg.de
**Course**: Advanced Methods in Control Engineering

---

##  Project Structure

##  Controllers

### 1. **Kinematic Controller**
- Pure pursuit-type reactive controller
- Based on polar coordinates: ρ (distance), α (heading error), β (goal heading)
- Tuned over 10 gain sets

### 2. **LQR Controller**
- Linear Quadratic Regulator
- Linearized model of 3W robot (around θ = 0)
- Feedback gain computed via DARE
- Tuned using 10 Q/R pairs

### 3. **MPC Controller**
- Model Predictive Control using CasADi
- Horizon-based planning with constraints
- Solved using IPOPT
- 10 sets of prediction horizon and Q/R weights

---

##   Running Simulations in Docker

### 1. Build the Docker image

```bash
cd ~/rcognita_system_based
docker build -t rcognita_ros .
