function dstate = motor_derivative(state, voltage, p, disturbance_torque)
%MOTOR_DERIVATIVE State derivative for a DC-motor-driven joint, with an
%   optional constant disturbance torque (e.g. gravity acting on an
%   unbalanced joint, or an external payload).
%
%   dstate = MOTOR_DERIVATIVE(state, voltage, p, disturbance_torque)
%   with STATE = [theta; omega] (rad, rad/s). Electrical dynamics are
%   neglected -- a standard simplification, valid because the
%   electrical time constant (L/R) is normally much faster than the
%   mechanical one, so it settles effectively instantly compared to the
%   joint's motion. That leaves:
%
%     J*theta'' + (b + K^2/R)*theta' = (K/R)*voltage - disturbance_torque
%
%   where the K^2/R term is the motor's back-EMF acting as extra
%   damping (a spinning motor generates a voltage that opposes its own
%   drive current), and DISTURBANCE_TORQUE (default 0) is a constant
%   opposing load the controller has to actively fight to hold
%   position against.
    if nargin < 4
        disturbance_torque = 0;
    end
    theta_dot = state(2);
    b_eff = p.b + p.K^2 / p.R;
    omega_dot = (p.K / p.R * voltage - b_eff * state(2) - disturbance_torque) / p.J;
    dstate = [theta_dot; omega_dot];
end
