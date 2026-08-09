classdef test_pid_step < matlab.unittest.TestCase
    methods (Test)
        function testProportionalOnlyOutput(testCase)
            gains = struct('Kp', 2, 'Ki', 0, 'Kd', 0);
            state = pid_state();
            [output, ~] = pid_step(3, 0.01, gains, state, [-100, 100]);
            testCase.verifyEqual(output, 6, 'AbsTol', 1e-9);
        end

        function testIntegralAccumulates(testCase)
            gains = struct('Kp', 0, 'Ki', 1, 'Kd', 0);
            state = pid_state();
            [out1, state] = pid_step(2, 0.1, gains, state, [-100, 100]);
            [out2, ~] = pid_step(2, 0.1, gains, state, [-100, 100]);
            testCase.verifyEqual(out1, 0.2, 'AbsTol', 1e-9);
            testCase.verifyEqual(out2, 0.4, 'AbsTol', 1e-9);
        end

        function testDerivativeRespondsToChange(testCase)
            gains = struct('Kp', 0, 'Ki', 0, 'Kd', 1);
            state = pid_state();
            [~, state] = pid_step(0, 0.1, gains, state, [-100, 100]);
            [output, ~] = pid_step(5, 0.1, gains, state, [-100, 100]);
            testCase.verifyEqual(output, 50, 'AbsTol', 1e-9);
        end

        function testAntiWindupStopsIntegratingWhenSaturated(testCase)
            gains = struct('Kp', 0, 'Ki', 10, 'Kd', 0);
            state = pid_state(true);
            [out1, state] = pid_step(100, 1, gains, state, [-5, 5]);
            [out2, state] = pid_step(100, 1, gains, state, [-5, 5]);
            testCase.verifyEqual(out1, 5, 'AbsTol', 1e-9);
            testCase.verifyEqual(out2, 5, 'AbsTol', 1e-9);
            testCase.verifyEqual(state.integral, 0, 'AbsTol', 1e-9);
        end

        function testWithoutAntiWindupIntegralKeepsGrowing(testCase)
            gains = struct('Kp', 0, 'Ki', 10, 'Kd', 0);
            state = pid_state(false);
            [~, state] = pid_step(100, 1, gains, state, [-5, 5]);
            [~, state] = pid_step(100, 1, gains, state, [-5, 5]);
            testCase.verifyGreaterThan(state.integral, 100);
        end
    end
end
