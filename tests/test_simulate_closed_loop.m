classdef test_simulate_closed_loop < matlab.unittest.TestCase
    methods (Test)
        function testPureProportionalMatchesAnalyticalSecondOrderSystem(testCase)
            % With Ki = Kd = 0, no disturbance, and no voltage
            % saturation, the closed loop is exactly a standard
            % 2nd-order system -- this checks the whole simulation
            % pipeline (motor_derivative + rk4_step + pid_step +
            % simulate_closed_loop) against a known, provable formula
            % instead of only against itself.
            p = joint_plant_params();
            Kp = 8;  % chosen so the closed loop is comfortably underdamped (zeta < 1)
            gains = struct('Kp', Kp, 'Ki', 0, 'Kd', 0);
            target = 1.0;

            opts = struct('dt', 0.0005, 'duration', 3, 'anti_windup', false);
            result = simulate_closed_loop(target, gains, p, opts);

            a2 = (p.K / p.R) / p.J;
            b_eff = p.b + p.K^2 / p.R;
            a1 = b_eff / p.J;
            wn = sqrt(a2 * Kp);
            zeta = a1 / (2 * wn);

            theta_expected = analytical_second_order_response(result.t, target, wn, zeta);

            testCase.verifyEqual(result.theta, theta_expected, 'AbsTol', 5e-3);
        end

        function testWellTunedPidReachesTargetWithNegligibleSteadyStateError(testCase)
            p = joint_plant_params();
            gains = struct('Kp', 5, 'Ki', 2, 'Kd', 0.1);
            result = simulate_closed_loop(0.5, gains, p, struct('duration', 4));
            m = step_response_metrics(result.t, result.theta, 0.5);
            testCase.verifyLessThan(m.steady_state_error, 0.01);
        end

        function testVoltageNeverExceedsActuatorLimit(testCase)
            p = joint_plant_params();
            gains = struct('Kp', 50, 'Ki', 20, 'Kd', 1);  % deliberately aggressive
            result = simulate_closed_loop(1.0, gains, p);
            testCase.verifyLessThanOrEqual(max(abs(result.voltage)), p.voltage_limit + 1e-9);
        end

        function testProportionalOnlyLeavesSteadyStateErrorUnderDisturbance(testCase)
            % Classic, honest control-theory fact: under a constant
            % disturbance (e.g. gravity on an unbalanced joint), a
            % P-only controller always settles with an offset --
            % holding position against the disturbance needs a
            % sustained nonzero voltage, which a P controller can only
            % produce from a sustained nonzero error.
            p = joint_plant_params();
            gains = struct('Kp', 2, 'Ki', 0, 'Kd', 0);
            opts = struct('duration', 5, 'disturbance_torque', 0.3);
            result = simulate_closed_loop(1.0, gains, p, opts);
            m = step_response_metrics(result.t, result.theta, 1.0);
            testCase.verifyGreaterThan(m.steady_state_error, 0.05);
        end

        function testIntegralTermEliminatesDisturbanceSteadyStateError(testCase)
            % Adding Ki to the same disturbed scenario should drive the
            % steady-state error to (near) zero -- the integral term
            % keeps winding up until it alone supplies the voltage
            % needed to hold against the disturbance at zero error.
            p = joint_plant_params();
            gains = struct('Kp', 2, 'Ki', 3, 'Kd', 0);
            opts = struct('duration', 8, 'disturbance_torque', 0.3);
            result = simulate_closed_loop(1.0, gains, p, opts);
            m = step_response_metrics(result.t, result.theta, 1.0);
            testCase.verifyLessThan(m.steady_state_error, 0.01);
        end
    end
end
