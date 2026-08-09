classdef test_analytical_second_order_response < matlab.unittest.TestCase
    methods (Test)
        function testStartsAtZero(testCase)
            theta = analytical_second_order_response(0, 3, 5, 0.5);
            testCase.verifyEqual(theta, 0, 'AbsTol', 1e-9);
        end

        function testReachesTargetAtSteadyState(testCase)
            theta = analytical_second_order_response(20, 3, 5, 0.5);
            testCase.verifyEqual(theta, 3, 'AbsTol', 1e-3);
        end

        function testOvershootMatchesKnownTextbookFormula(testCase)
            target = 1; wn = 10; zeta = 0.3;
            t = (0:0.0005:3)';
            theta = analytical_second_order_response(t, target, wn, zeta);
            m = step_response_metrics(t, theta, target);
            expected_overshoot = 100 * exp(-zeta * pi / sqrt(1 - zeta^2));
            testCase.verifyEqual(m.overshoot_pct, expected_overshoot, 'RelTol', 0.02);
        end

        function testErrorsForOverdampedZeta(testCase)
            testCase.verifyError( ...
                @() analytical_second_order_response(1, 1, 5, 1.2), ...
                'analytical_second_order_response:overdamped');
        end
    end
end
