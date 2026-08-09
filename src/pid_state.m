function state = pid_state(anti_windup)
%PID_STATE Fresh, zeroed state for PID_STEP.
%
%   state = PID_STATE(anti_windup) -- ANTI_WINDUP (default true) turns
%   on the anti-windup clamp described in pid_step.m. Call this once
%   before the first PID_STEP call in a simulation.
    if nargin < 1
        anti_windup = true;
    end
    state.integral = 0;
    state.prev_error = 0;
    state.anti_windup = anti_windup;
end
