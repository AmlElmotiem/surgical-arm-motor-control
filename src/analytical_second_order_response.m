function theta = analytical_second_order_response(t, target, wn, zeta)
%ANALYTICAL_SECOND_ORDER_RESPONSE Closed-form step response of a
%   standard underdamped 2nd-order system, scaled to TARGET.
%
%   A joint driven by a pure-P controller (Ki = Kd = 0), with no
%   voltage saturation, reduces exactly to this kind of system -- so
%   it serves as ground truth: the full simulation pipeline
%   (motor_derivative + rk4_step + pid_step + simulate_closed_loop) can
%   be checked against a known, provable textbook formula instead of
%   only against itself.
    if zeta >= 1
        error('analytical_second_order_response:overdamped', ...
            'Formula implemented for the underdamped case (zeta < 1) only.');
    end
    wd = wn * sqrt(1 - zeta^2);
    phi = acos(zeta);
    theta = target * (1 - exp(-zeta * wn * t) / sqrt(1 - zeta^2) .* sin(wd * t + phi));
end
