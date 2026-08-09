function next_state = rk4_step(derivative_fn, state, dt)
%RK4_STEP One classic 4th-order Runge-Kutta integration step for
%   state' = DERIVATIVE_FN(state).
%
%   Used instead of a built-in ODE solver so the simulation runs as a
%   fixed-step loop -- matching how a real embedded controller actually
%   operates (a fixed control-loop rate, e.g. 1 kHz) -- and so the
%   integration itself is not a black box.
    k1 = derivative_fn(state);
    k2 = derivative_fn(state + dt / 2 * k1);
    k3 = derivative_fn(state + dt / 2 * k2);
    k4 = derivative_fn(state + dt * k3);
    next_state = state + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
end
