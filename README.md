# surgical-arm-motor-control

PID control of a robot joint's motor, built from scratch in MATLAB —
closing the gap between
[`surgical-arm-kinematics`](https://github.com/AmlElmotiem/surgical-arm-kinematics)
(*which* joint angle gets the tip to a target) and this project
(*how* a real motor actually gets there: with inertia, damping, and a
finite-time response, not instantly).

## What it does

- **`joint_plant_params.m`** — illustrative parameters for one robot
  joint driven by a DC motor (inertia, damping, motor constant,
  winding resistance, voltage limit) — same teaching-model spirit as
  the example arm in `surgical-arm-kinematics`.
- **`motor_derivative.m`** — the joint's equation of motion
  (`J*theta'' + (b + K^2/R)*theta' = (K/R)*V - disturbance_torque`),
  derived from the motor's electrical and mechanical equations with
  the electrical dynamics neglected (a standard, documented
  simplification). Supports an optional constant disturbance torque
  (e.g. gravity on an unbalanced joint).
- **`rk4_step.m`** — a classic 4th-order Runge-Kutta integrator,
  implemented from scratch rather than calling a built-in ODE solver,
  so the simulation runs as a fixed-step loop the same way a real
  embedded controller would (a fixed control-loop rate).
- **`pid_step.m`** / **`pid_state.m`** — a discrete-time PID
  controller with anti-windup: while the actuator output is clamped at
  its physical limit, the integral term stops accumulating, preventing
  the runaway overshoot that an un-clamped integral would otherwise
  cause. See "What we found" below.
- **`simulate_closed_loop.m`** — wires the controller and the plant
  together into a full closed-loop simulation.
- **`step_response_metrics.m`** — rise time, overshoot %, settling
  time, and steady-state error from a simulated response.
- **`analytical_second_order_response.m`** — the closed-form step
  response of a standard underdamped 2nd-order system. A pure-P
  controller with no disturbance and no saturation reduces exactly to
  this system, so it serves as **ground truth**: the entire simulation
  pipeline is checked against a known, provable textbook formula
  instead of only against itself — see "What we found" below.
- **`scripts/run_demo.m`** — simulates one tuned PID controller
  reaching a target angle, plots the response and control voltage.
- **`scripts/windup_demo.m`** — the integrator-windup bug, side by
  side with the anti-windup fix, same gains and target.
- **`scripts/controller_comparison.m`** — P vs. PI vs. PID on the same
  joint under a constant disturbance torque, with a metrics table.

## What we found (the honest part)

To fill in after running the test suite and demo scripts.

## Limitations

- Single joint only — does not model coupling between multiple joints
  of a full arm (e.g. one joint's motion affecting another's load).
- Electrical dynamics (motor inductance) are neglected, a standard
  simplification valid when the electrical time constant is much
  faster than the mechanical one — not modeled explicitly or verified
  against a full electromechanical model.
- Gains in the demo scripts were tuned manually by inspection, not via
  a systematic method (e.g. Ziegler-Nichols or pole placement).

## Roadmap

- Multi-joint coupled simulation, feeding real DH parameters and
  target trajectories from `surgical-arm-kinematics`
- Systematic gain tuning (Ziegler-Nichols or pole-placement) instead
  of manual tuning
- Trajectory tracking (following a moving target over time), not just
  single-step commands
