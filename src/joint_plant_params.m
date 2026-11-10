function p = joint_plant_params()
%JOINT_PLANT_PARAMS Illustrative parameters for one robot joint driven
%   by a DC motor. Same spirit as the example arm in
%   surgical-arm-kinematics: a realistic teaching-model set of numbers,
    p.J = 0.02;             % kg*m^2, joint + rotor inertia
    p.b = 0.1;               % N*m*s, mechanical damping (friction)
    p.K = 0.5;                % N*m/A (torque constant) = V*s/rad (back-EMF constant)
    p.R = 2.0;                % Ohm, motor winding resistance
    p.voltage_limit = 12;     % V, actuator saturation
end
 
