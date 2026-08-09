function m = step_response_metrics(t, theta, target)
%STEP_RESPONSE_METRICS Classic step-response characteristics from a
%   simulated (or measured) response THETA(T) driving toward TARGET:
%     rise_time_s         - time from 10% to 90% of TARGET
%     overshoot_pct       - how far past TARGET the response peaks,
%                           as a percentage of TARGET
%     settling_time_s     - time after which THETA stays within 2% of
%                           TARGET for the rest of the run
%     steady_state_error  - |TARGET - final THETA|
    frac = theta / target;

    idx10 = find(frac >= 0.1, 1, 'first');
    idx90 = find(frac >= 0.9, 1, 'first');
    if isempty(idx10) || isempty(idx90)
        m.rise_time_s = NaN;
    else
        m.rise_time_s = t(idx90) - t(idx10);
    end

    peak = max(theta);
    m.overshoot_pct = max(0, (peak - target) / target * 100);

    band = 0.02 * abs(target);
    within_band = abs(theta - target) <= band;
    outside_idx = find(~within_band, 1, 'last');
    if isempty(outside_idx)
        m.settling_time_s = 0;
    else
        m.settling_time_s = t(min(outside_idx + 1, numel(t)));
    end

    m.steady_state_error = abs(target - theta(end));
end
