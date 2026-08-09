function fig = plot_response(result, title_str)
%PLOT_RESPONSE Plot a closed-loop simulation result: joint angle vs.
%   target (top) and control voltage (bottom).
    fig = figure('Color', 'w');

    subplot(2, 1, 1);
    plot(result.t, result.theta, 'b', 'LineWidth', 1.5);
    hold on;
    yline(result.target, 'k--');
    xlabel('Time (s)');
    ylabel('Joint angle (rad)');
    legend('Response', 'Target', 'Location', 'best');
    if nargin >= 2
        title(title_str);
    end
    grid on;

    subplot(2, 1, 2);
    plot(result.t, result.voltage, 'r');
    xlabel('Time (s)');
    ylabel('Motor voltage (V)');
    grid on;
end
