classdef test_step_response_metrics < matlab.unittest.TestCase
    methods (Test)
        function testInstantStepHasNoOvershootAndNoSteadyStateError(testCase)
            t = (0:0.01:2)';
            theta = ones(size(t)) * 5;
            theta(1) = 0;
            m = step_response_metrics(t, theta, 5);
            testCase.verifyEqual(m.overshoot_pct, 0, 'AbsTol', 1e-9);
            testCase.verifyEqual(m.steady_state_error, 0, 'AbsTol', 1e-9);
        end

        function testSettlingTimeZeroWhenAlwaysWithinBand(testCase)
            t = (0:0.01:1)';
            theta = 5 * ones(size(t));  % exactly on target the whole time
            m = step_response_metrics(t, theta, 5);
            testCase.verifyEqual(m.settling_time_s, 0);
        end

        function testDetectsOvershootAboveTarget(testCase)
            t = (0:0.01:2)';
            target = 1;
            theta = target * (1 - exp(-3 * t)) .* (1 + 0.3 * exp(-2 * t) .* cos(8 * t));
            m = step_response_metrics(t, theta, target);
            testCase.verifyGreaterThan(m.overshoot_pct, 0);
        end
    end
end
