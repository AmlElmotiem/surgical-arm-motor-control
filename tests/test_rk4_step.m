classdef test_rk4_step < matlab.unittest.TestCase
    methods (Test)
        function testMatchesKnownExponentialDecaySolution(testCase)
            % state' = -state has the exact solution state(t) = state0*exp(-t)
            f = @(s) -s;
            state = 1;
            dt = 0.01;
            for i = 1:100
                state = rk4_step(f, state, dt);
            end
            expected = exp(-1);  % t = 100*0.01 = 1
            testCase.verifyEqual(state, expected, 'AbsTol', 1e-6);
        end

        function testConstantDerivativeGivesLinearMotion(testCase)
            f = @(s) 2;  % state' = 2 -> state(t) = state0 + 2t
            state = rk4_step(f, 0, 0.5);
            testCase.verifyEqual(state, 1, 'AbsTol', 1e-9);
        end
    end
end
