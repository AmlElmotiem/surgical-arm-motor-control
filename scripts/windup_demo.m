% Demonstrate integrator windup: a large step command with an
% aggressive integral gain, first WITHOUT anti-windup (large
% overshoot) and then WITH it (much better behaved) -- same gains,
% same target, only the anti-windup clamp differs.
%
% Run: scripts/windup_demo
project_root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(project_root));

p = joint_plant_params();
gains = struct('Kp', 3, 'Ki', 15, 'Kd', 0);
target = 2.0;  % a large step, deliberately chosen to saturate the 12V actuator

opts_no_aw = struct('anti_windup', false, 'duration', 4);
opts_aw = struct('anti_windup', true, 'duration', 4);

result_no_aw = simulate_closed_loop(target, gains, p, opts_no_aw);
result_aw = simulate_closed_loop(target, gains, p, opts_aw);

m_no_aw = step_response_metrics(result_no_aw.t, result_no_aw.theta, target);
m_aw = step_response_metrics(result_aw.t, result_aw.theta, target);

fprintf('%-20s %15s %15s\n', '', 'No anti-windup', 'Anti-windup');
fprintf('%-20s %15.2f %15.2f\n', 'Overshoot (%)', m_no_aw.overshoot_pct, m_aw.overshoot_pct);
fprintf('%-20s %15.3f %15.3f\n', 'Settling time (s)', m_no_aw.settling_time_s, m_aw.settling_time_s);

results_dir = fullfile(project_root, 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

fig = figure('Color', 'w');
plot(result_no_aw.t, result_no_aw.theta, 'r', 'LineWidth', 1.5);
hold on;
plot(result_aw.t, result_aw.theta, 'b', 'LineWidth', 1.5);
yline(target, 'k--');
xlabel('Time (s)');
ylabel('Joint angle (rad)');
legend('Without anti-windup', 'With anti-windup', 'Target', 'Location', 'best');
title('Integrator windup: same gains, same target, only anti-windup differs');
grid on;
saveas(fig, fullfile(results_dir, 'windup_comparison.png'));
fprintf('Wrote %s\n', fullfile(results_dir, 'windup_comparison.png'));
