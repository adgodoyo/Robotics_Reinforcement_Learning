# Autonomous Robot Parking: RL and Learning from Demonstrations

A ROS-based system that teaches a simulated HiWonder robot to autonomously park in a target spot inside a Gazebo environment. The project implements two complementary approaches: **Learning from Demonstrations (LfD)** using human teleoperation data, and **Deep Deterministic Policy Gradient (DDPG)** reinforcement learning trained from scratch.

---

## rl_lfd Package

This is the core package (`rl_lfd`). It contains three independent scripts for data collection, imitation learning, and RL-based training:

### 1. `data_lfd.py` — Demonstration Collection (Teleop)

A keyboard teleoperation node that records human demonstrations of the parking manoeuvre. The operator drives the robot from its reset position to the parking spot; each run is saved as a sequence of (state, action) pairs.

**Key bindings:**

| Key | Action |
|-----|--------|
| `W` | Forward |
| `S` | Backward |
| `A` | Turn left |
| `D` | Turn right |
| `ENTER` | Stop current demo, reset robot, start next demo |

The node collects **20 demonstrations** by default and saves them to `scripts/demonstrations.csv` with columns:

```
demo_id, time, state_x, state_y, state_theta, linear_x, angular_z
```

Run with:
```bash
rosrun rl_lfd data_lfd.py
```

---

### 2. `lfd.py` — Imitation Policy Execution

Loads the recorded `demonstrations.csv` and drives the robot to the parking spot using a **nearest-neighbour policy**: at each timestep the node finds the demonstration state closest (in Euclidean distance) to the robot's current pose and executes the associated action.

**State:** `[x, y, θ]`

**Target pose:** `x = 0.9 m`, `y = 1.525 m`, `θ = π rad`

**Success condition:** robot within ±0.3 m in x, ±0.08 m in y, and ±0.3 rad in heading.

Run with:
```bash
rosrun rl_lfd lfd.py
```

---

### 3. `rl.py` — DDPG Reinforcement Learning Training

Trains a **DDPG** agent to autonomously park the robot over 500 episodes. The agent interacts with Gazebo, receives rewards, and updates its policy via the actor–critic architecture defined in `actor.py`.

**State (7D):** `[x, y, θ, Δx, Δy, Δθ, Euclidean distance to target]`

**Action (2D):** `[linear velocity v, angular velocity ω]` clipped to `v ∈ [-0.6, 0.6] m/s`, `ω ∈ [-1.5, 1.5] rad/s`

**Reward function:**
- Penalises displacement from the target in x, y, and heading (cubic penalty on x and y)
- Rewards progress toward the target (ratio of previous to current distance)
- Adds an overlap bonus proportional to the intersection of the robot's footprint with the parking rectangle (Sutherland-Hodgman clipping)
- Grants a large terminal bonus `10 000 × 0.1 × remaining_steps` when the robot parks successfully

**Training hyper-parameters:**

| Parameter | Value |
|-----------|-------|
| Episodes | 500 |
| Max steps / episode | 50 |
| Step duration | 0.1 s |
| Batch size | 128 |
| Replay buffer | 100 000 |
| Actor / Critic LR | 1 × 10⁻⁴ / 1 × 10⁻³ |
| Discount γ | 0.99 |
| Soft-update τ | 0.005 |
| OU noise (θ, σ) | 0.15, 0.2 |

Training metrics (episode, steps, cumulative reward, final pose, success flag) are logged to a timestamped CSV file in the same directory.

Run with:
```bash
rosrun rl_lfd rl.py
```

---

### 4. `actor.py` — Neural Network Definitions

Provides the building blocks used by `rl.py`:

| Class | Description |
|-------|-------------|
| `Actor` | 3-layer MLP (128 → 128 → action_dim), Tanh output scaled by `max_action` |
| `Critic` | 3-layer MLP on concatenated state-action input, outputs scalar Q-value |
| `ReplayBuffer` | Circular deque of capacity 100 000; uniform random sampling |
| `DDPG` | Wraps Actor + Critic with target networks, soft updates, and OU noise for exploration |
| `OUNoise` | Ornstein-Uhlenbeck process for temporally-correlated exploration noise |

---

## Getting Started

### Prerequisites

- [VS Code](https://code.visualstudio.com/) with the **Dev Containers** extension
- [Docker Desktop](https://www.docker.com/products/docker-desktop/)

### Running with VS Code Dev Container

1. **Open the project** in VS Code:

   ```
   File > Open Folder > select the docker-container/ directory
   ```

2. **Reopen in Container** — VS Code will detect `.devcontainer/devcontainer.json` and prompt you. Click **"Reopen in Container"** (or use the command palette: `Dev Containers: Reopen in Container`). This builds and starts the Docker services (X server, Gazebo simulator, and ROS workspace).

3. **Build the workspace** inside the container terminal:

   ```bash
   catkin build
   ```

4. **Source the workspace:**

   ```bash
   source devel/setup.bash
   ```

5. **View the simulator GUI** by opening `http://localhost:3000` in your browser.

### Workflow

#### Step 1 — Collect demonstrations (LfD)
```bash
rosrun rl_lfd data_lfd.py
```
Drive the robot to the parking spot 20 times using W/A/S/D. Press ENTER after each run to save and reset.

#### Step 2 — Run the imitation policy
```bash
rosrun rl_lfd lfd.py
```
The nearest-neighbour policy replays the closest recorded action for the current robot pose.

#### Step 3 — Train with DDPG
```bash
rosrun rl_lfd rl.py
```
The agent runs 500 episodes, logs progress to a CSV, and prints the cumulative reward and parking success after each episode.

---

## Project Structure

```
docker-container/
├── .devcontainer/          # VS Code Dev Container configuration
│   ├── devcontainer.json
│   ├── Dockerfile
│   └── docker-compose.yml
├── docker-compose.yml      # Multi-service orchestration (xserver, simulator, workspace)
└── src/
    └── rl_lfd/             # Main package (ROS name: lab_2)
        ├── scripts/
        │   ├── actor.py            # DDPG networks: Actor, Critic, ReplayBuffer, OUNoise
        │   ├── rl.py               # DDPG training loop (ROS node: car_parking)
        │   ├── lfd.py              # Nearest-neighbour LfD execution (ROS node: lfd_parking)
        │   ├── data_lfd.py         # Teleoperation + demonstration recorder (ROS node: lfd_teleop)
        │   └── demonstrations.csv  # Recorded human demonstrations
        ├── CMakeLists.txt
        └── package.xml
```

---

## Acknowledgements

This project was prepared based on various online resources, including the Docker image and simulation environment originally developed by Dr Sen Wang.
