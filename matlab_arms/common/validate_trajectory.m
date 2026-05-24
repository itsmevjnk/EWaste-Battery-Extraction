function report = validate_trajectory(traj, tau, params, safety_margin)
% Check a trajectory against the arm's position, velocity, and torque
% limits. Prints a summary table and returns a struct.
%
% I treat any joint above (1 - safety_margin) * limit as a WARNING and
% any joint above the limit itself as a FAIL.

    if nargin < 4
        safety_margin = 0.05;   % i.e. flag at 95% of limit
    end

    verdict = 0;   % 0 = PASS, 1 = WARNING, 2 = FAIL
    msgs = {};

    % Position
    over_upper = max(traj.q, [], 1) - params.joint_upper;
    over_lower = params.joint_lower - min(traj.q, [], 1);
    pos_excess = max(0, max([over_upper; over_lower], [], 1));
    if any(pos_excess > 0)
        idx = find(pos_excess > 0);
        msgs{end+1} = sprintf('FAIL: joint(s) %s exceed position limits', num2str(idx));
        verdict = 2;
    end

    % Velocity
    vel_peak = max(abs(traj.qd), [], 1);
    vel_util = vel_peak ./ params.vel_limit;
    if any(vel_util >= 1)
        idx = find(vel_util >= 1);
        msgs{end+1} = sprintf('FAIL: joint(s) %s exceed velocity limit', num2str(idx));
        verdict = max(verdict, 2);
    elseif any(vel_util >= 1 - safety_margin)
        idx = find(vel_util >= 1 - safety_margin);
        msgs{end+1} = sprintf('WARNING: joint(s) %s near velocity limit', num2str(idx));
        verdict = max(verdict, 1);
    end

    % Torque - this is the main check I care about
    tau_peak = max(abs(tau), [], 1);
    tau_util = tau_peak ./ params.effort_limit;
    for j = 1:numel(tau_peak)
        if tau_util(j) >= 1
            msgs{end+1} = sprintf('FAIL: joint %d torque %.2f Nm exceeds %.0f Nm limit', ...
                j, tau_peak(j), params.effort_limit(j));
            verdict = max(verdict, 2);
        elseif tau_util(j) >= 1 - safety_margin
            msgs{end+1} = sprintf('WARNING: joint %d torque %.2f Nm = %.0f%% of limit', ...
                j, tau_peak(j), tau_util(j) * 100);
            verdict = max(verdict, 1);
        end
    end

    report.verdict      = pickVerdict(verdict);
    report.position_excess = pos_excess;
    report.velocity_peak   = vel_peak;
    report.torque_peak     = tau_peak;
    report.torque_util     = tau_util;
    [~, report.worst_joint] = max(tau_util);
    report.messages = msgs;

    % Print a summary
    fprintf('\n--- Trajectory Validation ---\n');
    fprintf('Verdict: %s\n\n', report.verdict);
    fprintf('Joint | Peak vel (rad/s) | Peak torque (Nm) | Limit (Nm) | Util %%\n');
    fprintf('------+------------------+------------------+------------+-------\n');
    for j = 1:numel(tau_peak)
        fprintf('  %d   |      %6.3f      |       %6.2f     |     %3.0f    |  %5.1f\n', ...
            j, vel_peak(j), tau_peak(j), params.effort_limit(j), tau_util(j) * 100);
    end
    if ~isempty(msgs)
        fprintf('\nNotes:\n');
        for k = 1:numel(msgs)
            fprintf('  - %s\n', msgs{k});
        end
    end
    fprintf('\n');
end


function s = pickVerdict(v)
    switch v
        case 0, s = 'PASS';
        case 1, s = 'WARNING';
        case 2, s = 'FAIL';
    end
end
