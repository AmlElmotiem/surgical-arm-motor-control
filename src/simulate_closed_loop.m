function result = simulate_closed_loop(target, gains, p, opts)
%SIMULATE_CLOSED_LOOP Simulate a PID-controlled joint driving from rest
%   toward TARGET (rad), at a fixed control-loop rate.
%
%   result = SIMULATE_CLOSED_LOOP(target, gains, p, opts) returns a
%   struct with time, theta, and voltage (all column vectors). OPTS
%   fields (all optional): dt (default 0.001s, i.e. a 1kHz control
%   loop), duration (default 2s), anti_windup (default true),
%   disturbance_torque (default 0, a constant opposing load such as
%   gravity on an unbalanced joint).
    if nargin < 4
        opts = struct();
    end
    if ~isfield(opts, 'dt'), opts.dt = 0.001; end
    if ~isfield(opts, 'duration'), opts.duration = 2.0; end
    if ~isfield(opts, 'anti_windup'), opts.anti_windup = true; end
    if ~isfield(opts, 'disturbance_torque'), opts.disturbance_torque = 0; end

    n = round(opts.duration / opts.dt);
    t = (0:n)' * opts.dt;

    theta = zeros(n + 1, 1);
    voltage_log = zeros(n + 1, 1);

    state = [0; 0];  % [theta; omega], starts at rest at 0 rad
    pid = pid_state(opts.anti_windup);
    limits = [-p.voltage_limit, p.voltage_limit];

    for i = 1:n
        error = target - state(1);
        [voltage, pid] = pid_step(error, opts.dt, gains, pid, limits);
        voltage_log(i) = voltage;

        derivative_fn = @(s) motor_derivative(s, voltage, p, opts.disturbance_torque);
        state = rk4_step(derivative_fn, state, opts.dt);
        theta(i + 1) = state(1);
    end
    voltage_log(end) = voltage_log(max(end - 1, 1));

    result.t = t;
    result.theta = theta;
    result.voltage = voltage_log;
    result.target = target;
end
