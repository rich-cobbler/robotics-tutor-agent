# Project Progress Log: PyBullet Robot Simulation

## 1. Project Goal
Implement a PyBullet physics simulation of a differential-drive robot that uses an emulated ultrasonic sensor to detect an obstacle $1.0\text{ m}$ ahead and stops smoothly using a tuned PID controller.

---

## 2. Completed Milestones

### Phase 1: Environment Setup & Package Issues
- [x] **Virtual Environment creation**: Set up an isolated Python 3.8 environment inside the workspace using `uv`.
- [x] **PyBullet Windows Installation Resolution**: Resolved compiler failures (lack of MSVC build tools) by downloading a precompiled Windows CPython 3.8 package from `conda-forge` and manually extracting it to the local virtual environment.
- [x] **Dependency Installation**: Installed `numpy` and `matplotlib` packages to enable physics vector math and telemetry plotting.

### Phase 2: Model Design & Physics Tuning
- [x] **URDF Design (`robot.urdf`)**: Designed a differential-drive chassis with active wheel joints, passive caster wheel, and ultrasonic sensor visuals.
- [x] **Physics Stability Improvements**:
  - Replaced cylinder wheel collisions with **spheres** to stabilize plane contact and rolling dynamics.
  - Widened wheel base offset to prevent clipping into the chassis collision box.
  - Changed the caster joint to **continuous** and set its lateral friction to **0.0** to allow frictionless forward rolling/sliding.
- [x] **Control Joint Alignment**: Realigned the motor driving directions (applied negative target velocity) to drive the robot forward in global coordinates.

### Phase 3: Initial Control Loop & Telemetry Verification
- [x] **Ray Casting Sensor Emulation**: Programmed a forward-pointing ray cast in `simulation.py` to calculate distances to the red cylinder obstacle.
- [x] **Stop Trigger logic**: Wrote feedback motor control to halt wheel rotations instantly when obstacle distance falls $\le 1.0\text{ m}$.
- [x] **Headless Frame Capture**: Configured Tiny Renderer to capture simulation frames step-by-step and compile them into an animated GIF.
- [x] **Telemetry Graphing**: Plotted distance-to-obstacle and velocity curves over time.
- [x] **Execution Verification**: Confirmed the robot stops at **3.504 seconds** at a distance of **0.999 meters**.
- [x] **Simulation Report**: Formatted and published [simulation_report.md](file:///C:/Users/user/.gemini/antigravity-cli/brain/b53e6b17-e9ba-496d-9949-d152ae286e6b/simulation_report.md) with visual telemetry embeddings.

### Phase 4: Multi-Ray Sensor & Comparative PID Tuning
- [x] **Multi-Ray Sensor Array (FOV)**: Programmed a 5-ray diverging array (spanning $-15.0^\circ$ to $15.0^\circ$) via `rayTestBatch` to realistically emulate ultrasonic sensor cone coverage.
- [x] **Smooth PID Deceleration**: Replaced the instant wheel-lock logic with a pure feedback control loop. Added integral control ($K_i$) to successfully overcome simulation friction and eliminate the steady-state deadband.
- [x] **PID Response Parameter Comparison**: Implemented `--compare-pid` command-line option to compare Underdamped, Overdamped, and Critically Damped configurations.
- [x] **Plot Comparison generation**: Outputted `pid_comparison.png` visualizing distance-to-obstacle and robot speed trajectories.
- [x] **Simulation Telemetry & Report Update**: Updated [simulation_report.md](file:///C:/Users/user/.gemini/antigravity-cli/brain/b53e6b17-e9ba-496d-9949-d152ae286e6b/simulation_report.md) to document Multi-Ray implementation, PID comparison metrics, and updated plots.

---

## 3. Telemetry Results Summary (Critically Damped / Tuned PID)
- **Cruising Speed**: $0.3\text{ m/s}$ (linear velocity)
- **Time of Stopping Trigger**: $4.604\text{ s}$
- **Detection Distance at Stop**: $1.003\text{ m}$ (within 3mm of target)
- **Final Status**: Successful smooth stop with $0.0\text{ m/s}$ residual speed, preventing collision.

---

## 4. Next Steps & Optional Features
- [ ] **Dynamic Obstacles**: Introduce moving obstacles to test the PID controller's capability of tracking and stopping in dynamic environments.
- [ ] **Path Planning / Obstacle Avoidance**: Extend the robot's control system to steer around obstacles rather than just stopping in front of them.
