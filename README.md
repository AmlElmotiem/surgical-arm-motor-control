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

**The analytical ground-truth test first exposed my own mistake, not a
code bug.** `testPureProportionalMatchesAnalyticalSecondOrderSystem`
compares the full simulation against the closed-form 2nd-order step
response formula, which is only valid for an *underdamped* system
(zeta &lt; 1). My first choice of gain, Kp = 0.6, actually produces
zeta &approx; 2.05 — heavily overdamped — so the test errored out
immediately instead of comparing anything. Solving zeta(Kp) &lt; 1 for
this plant shows Kp must exceed &approx; 2.53; raising it to Kp = 8
(zeta &approx; 0.56) made the comparison valid, and the full pipeline
(motor model + RK4 integrator + PID controller + closed-loop
simulation) matched the textbook formula to within 5&times;10&#8315;&sup3;
rad over a 3-second simulation.

**The first version of `windup_demo.m` didn't actually demonstrate
anti-windup at all.** With the original gains and target, the
commanded voltage never once reached the 12V actuator limit —
0% of the simulation was saturated — so running with and without
anti-windup produced bit-for-bit identical results. The fix wasn't a
code bug but a demo-parameter bug: the target had to be large enough
that proportional action *alone* would demand more than 12V
(`Kp * target > voltage_limit`), so the actuator saturates from t = 0
and the integral actually has room to run away. With Kp = 5, Ki = 15,
target = 3 rad, the difference is large and real:

| | No anti-windup | With anti-windup |
|---|---|---|
| Overshoot | 51.28% | 27.52% |
| Settling time | 1.546 s | 1.477 s |

**The P-vs-PI-vs-PID comparison (under a constant disturbance torque)
reproduced two classic, real control-theory facts** — not fabricated
results, genuine simulation output:

| Controller | Overshoot | Settling time | Rise time | Steady-state error |
|---|---|---|---|---|
| P | 0.00% | never (ran out of time) | never reached 90% | 0.300 rad |
| PI | 8.97% | 1.443 s | 0.362 s | 0.00015 rad |
| PID | 10.29% | 1.874 s | 0.430 s | 0.00147 rad |

The P-only row is the honest, slightly dramatic-looking one: under a
constant disturbance it settles with a 0.3 rad permanent offset and
never even reaches 90% of the target, so rise time is undefined
(`NaN`) — exactly the textbook prediction that P-only control cannot
reject a constant disturbance. Less expected: PID shows *more*
overshoot than PI (10.29% vs. 8.97%), not less. The cause is
**derivative kick** — because the derivative term here acts on the
raw error, a step change in the target produces one huge instantaneous
spike in the derivative term at t = 0, momentarily overdriving the
motor. This is a well-documented real PID pitfall, not a bug in this
implementation — the standard fix ("derivative on measurement"
instead of "derivative on error") is noted in the roadmap below.

## Limitations

- Single joint only — does not model coupling between multiple joints
  of a full arm (e.g. one joint's motion affecting another's load).
- Electrical dynamics (motor inductance) are neglected, a standard
  simplification valid when the electrical time constant is much
  faster than the mechanical one — not modeled explicitly or verified
  against a full electromechanical model.
- Gains in the demo scripts were tuned manually by inspection, not via
  a systematic method (e.g. Ziegler-Nichols or pole placement).
- The derivative term acts on the raw error ("derivative on error"),
  which causes the derivative-kick effect described above on step
  changes in the target.

## Roadmap

- Switch to "derivative on measurement" to eliminate the derivative
  kick documented above
- Multi-joint coupled simulation, feeding real DH parameters and
  target trajectories from `surgical-arm-kinematics`
- Systematic gain tuning (Ziegler-Nichols or pole-placement) instead
  of manual tuning
- Trajectory tracking (following a moving target over time), not just
  single-step commands
