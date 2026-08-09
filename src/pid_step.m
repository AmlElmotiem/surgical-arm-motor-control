function [output, state] = pid_step(error, dt, gains, state, output_limits)
%PID_STEP One discrete-time PID control-loop update, with anti-windup.
%
%   [output, state] = PID_STEP(error, dt, gains, state, output_limits)
%   GAINS is a struct with fields Kp, Ki, Kd. STATE carries the
%   controller's memory between calls (integral accumulator and the
%   previous error, for the derivative term) -- get a fresh one from
%   PID_STATE() before the first call. OUTPUT_LIMITS = [min max] clamps
%   the output to what the actuator can physically produce.
%
%   Anti-windup: when the clamped output differs from what the raw P+I+D
%   sum would have been, the actuator is already saturated, so this
%   step's error is NOT added to the integral. Without this, the
%   integral keeps accumulating while the actuator can't do anything
%   more about it, and the controller way overshoots once the error
%   finally shrinks enough to leave saturation -- see "What we found"
%   in the README for a real, simulated example of this.
    proportional = gains.Kp * error;
    derivative = gains.Kd * (error - state.prev_error) / dt;

    candidate_integral = state.integral + error * dt;
    integral_term = gains.Ki * candidate_integral;

    unclamped_output = proportional + integral_term + derivative;
    output = min(max(unclamped_output, output_limits(1)), output_limits(2));

    is_saturated = (output ~= unclamped_output);
    if ~(state.anti_windup && is_saturated)
        state.integral = candidate_integral;
    end

    state.prev_error = error;
end
