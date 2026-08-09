% Compare P, PI, and PID control of the same joint/target: overlaid
% step responses plus a metrics table, showing the classic trade-offs
% (P alone leaves a steady-state error under load; adding I removes it
% but can add overshoot; adding D damps that overshoot back down).
%
% Run: scripts/controller_comparison
project_root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(project_root));

p = joint_plant_params();
target = 1.0;
opts = struct('duration', 3, 'disturbance_torque', 0.3);

controllers = struct( ...
    'name', {'P', 'PI', 'PID'}, ...
    'gains', { ...
        struct('Kp', 4, 'Ki', 0, 'Kd', 0), ...
        struct('Kp', 4, 'Ki', 6, 'Kd', 0), ...
        struct('Kp', 4, 'Ki', 6, 'Kd', 0.2) ...
    });

results_dir = fullfile(project_root, 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

fig = figure('Color', 'w');
hold on;
colors = {'r', 'b', 'g'};

fprintf('%-6s %12s %12s %12s %12s\n', 'Ctrl', 'Overshoot%', 'Settle(s)', 'Rise(s)', 'SS error');
for i = 1:numel(controllers)
    result = simulate_closed_loop(target, controllers(i).gains, p, opts);
    m = step_response_metrics(result.t, result.theta, target);
    fprintf('%-6s %12.2f %12.3f %12.3f %12.5f\n', controllers(i).name, ...
        m.overshoot_pct, m.settling_time_s, m.rise_time_s, m.steady_state_error);
    plot(result.t, result.theta, colors{i}, 'LineWidth', 1.5, 'DisplayName', controllers(i).name);
end
yline(target, 'k--', 'DisplayName', 'Target');
xlabel('Time (s)');
ylabel('Joint angle (rad)');
title('P vs. PI vs. PID under a constant disturbance torque');
legend('Location', 'best');
grid on;

saveas(fig, fullfile(results_dir, 'controller_comparison.png'));
fprintf('Wrote %s\n', fullfile(results_dir, 'controller_comparison.png'));
