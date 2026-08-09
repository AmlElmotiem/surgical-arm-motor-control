classdef test_motor_derivative < matlab.unittest.TestCase
    methods (Test)
        function testZeroVoltageZeroVelocityGivesZeroAcceleration(testCase)
            p = joint_plant_params();
            d = motor_derivative([0.5; 0], 0, p);
            testCase.verifyEqual(d(2), 0, 'AbsTol', 1e-9);
        end

        function testPositiveVoltageAccelerates(testCase)
            p = joint_plant_params();
            d = motor_derivative([0; 0], p.voltage_limit, p);
            testCase.verifyGreaterThan(d(2), 0);
        end

        function testThetaDotEqualsOmega(testCase)
            p = joint_plant_params();
            d = motor_derivative([0.3; 1.7], 0, p);
            testCase.verifyEqual(d(1), 1.7, 'AbsTol', 1e-9);
        end

        function testDampingOpposesMotion(testCase)
            p = joint_plant_params();
            d = motor_derivative([0; 5], 0, p);
            testCase.verifyLessThan(d(2), 0);
        end

        function testDisturbanceTorqueOpposesMotionAtRest(testCase)
            p = joint_plant_params();
            d = motor_derivative([0; 0], 0, p, 0.3);
            testCase.verifyLessThan(d(2), 0);
        end

        function testDisturbanceDefaultsToZero(testCase)
            p = joint_plant_params();
            d_no_arg = motor_derivative([0.2; 1.0], 2, p);
            d_zero_arg = motor_derivative([0.2; 1.0], 2, p, 0);
            testCase.verifyEqual(d_no_arg, d_zero_arg);
        end
    end
end
