% Simulate a single joint driven by a tuned PID controller toward a
% target angle, plot the response, and print step-response metrics.
%
% Run: scripts/run_demo
project_root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(project_root));

p = joint_plant_params();
gains = struct('Kp', 8, 'Ki', 4, 'Kd', 0.3);
target = 1.0;  % rad, ~57 degrees

result = simulate_closed_loop(target, gains, p, struct('duration', 4));
m = step_response_metrics(result.t, result.theta, target);

fprintf('Target: %.3f rad\n', target);
fprintf('Rise time (10-90%%): %.3f s\n', m.rise_time_s);
fprintf('Overshoot: %.2f %%\n', m.overshoot_pct);
fprintf('Settling time (2%% band): %.3f s\n', m.settling_time_s);
fprintf('Steady-state error: %.5f rad\n', m.steady_state_error);

results_dir = fullfile(project_root, 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

fig = plot_response(result, 'PID joint control (Kp=8, Ki=4, Kd=0.3)');
saveas(fig, fullfile(results_dir, 'pid_demo.png'));
fprintf('Wrote %s\n', fullfile(results_dir, 'pid_demo.png'));
