function traj = trajectory_generate(robot, waypoints_T, t_segment, ee_name, q_init)
% Build a joint-space trajectory from Cartesian waypoints.
%
% robot         rigidBodyTree (already built)
% waypoints_T   4x4xM array of end-effector poses
% t_segment     M-1 segment durations in seconds (scalar = same for all)
% ee_name       end-effector body name (e.g. 'end_effector_link', 'link_eef')
% q_init        1x7 initial guess for the first IK solve
%
% Returns a struct with t, q, qd, qdd, ee_pos, waypoint_q.
%
% Steps: solve IK at each waypoint (chaining the solutions for continuity),
% then interpolate with a quintic polynomial so velocity and acceleration
% are smooth.

    M = size(waypoints_T, 3);

    if isscalar(t_segment)
        t_segment = t_segment * ones(M - 1, 1);
    end
    t_segment = t_segment(:);

    n_dof = numel(homeConfiguration(robot));

    if nargin < 5 || isempty(q_init)
        q_init = homeConfiguration(robot);
    end
    if iscolumn(q_init), q_init = q_init.'; end

    % Solve IK at each waypoint, using the previous solution as the guess
    waypoint_q = zeros(M, n_dof);
    ik = inverseKinematics('RigidBodyTree', robot);
    ik.SolverParameters.MaxIterations = 1500;
    weights = [1 1 1 1 1 1];

    guess = q_init;
    for i = 1:M
        waypoint_q(i, :) = ik(ee_name, waypoints_T(:, :, i), weights, guess);
        guess = waypoint_q(i, :);
    end

    % Time-parameterise with quintic polynomial between waypoints
    t_breaks = [0; cumsum(t_segment)];
    N        = 300;
    t_samp   = linspace(0, t_breaks(end), N).';

    [q_mat, qd_mat, qdd_mat] = quinticpolytraj(waypoint_q.', t_breaks.', t_samp.');

    traj.t   = t_samp;
    traj.q   = q_mat.';
    traj.qd  = qd_mat.';
    traj.qdd = qdd_mat.';
    traj.waypoint_q = waypoint_q;
    traj.waypoint_T = waypoints_T;

    % Sample end-effector positions for plotting
    ee_pos = zeros(N, 3);
    for k = 1:N
        T = getTransform(robot, traj.q(k, :), ee_name);
        ee_pos(k, :) = T(1:3, 4).';
    end
    traj.ee_pos = ee_pos;
end
