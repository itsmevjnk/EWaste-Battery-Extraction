% demo_kinova_gen3_battery_extraction.m
%
% Battery extraction trajectory + torque check for the Kinova Gen3.
% The arm goes home -> over the phone -> down to grasp -> lift -> deposit.

clear; clc; close all;

% Add the project paths
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'kinova_gen3'));
addpath(fullfile(here, '..', 'common'));

% Build the arm
fprintf('Building Gen3...\n');
robot  = kinova_gen3_build();
params = kinova_gen3_params();

% Cartesian waypoints. Z is up, base of the arm is the origin.
% I picked these for a phone lying flat on the lab bench.
z_table    = 0.05;
z_approach = z_table + 0.08;
z_lift     = z_table + 0.15;

% End-effector pointing straight down at the phone
R_down = eul2rotm([0, pi, 0], 'ZYX');
make_pose = @(xyz) [R_down, xyz(:); 0 0 0 1];

waypoints = zeros(4, 4, 5);
waypoints(:, :, 1) = make_pose([0.40, 0.00, z_lift]);      % start
waypoints(:, :, 2) = make_pose([0.45, 0.00, z_approach]);  % above phone
waypoints(:, :, 3) = make_pose([0.45, 0.00, z_table+0.005]); % grasp
waypoints(:, :, 4) = make_pose([0.45, 0.00, z_lift]);      % lift
waypoints(:, :, 5) = make_pose([0.40, 0.30, z_lift]);      % move to bin

% Segment durations (seconds). Slow on the grasp.
t_seg = [1.5; 1.5; 1.0; 2.0];

fprintf('Planning trajectory...\n');
traj = trajectory_generate(robot, waypoints, t_seg, 'end_effector_link', params.q_ready);

% Compute torques with a 50 g payload (battery + a bit for the vacuum gripper)
fprintf('Computing torques...\n');
[tau, tau_info] = kinova_gen3_torques(traj.q, traj.qd, traj.qdd, robot, 0.05);

% Check against the manufacturer limits
report = validate_trajectory(traj, tau, params, 0.05);

% Save plots into results/
results_dir = fullfile(here, '..', 'results', 'kinova_gen3');
if ~isfolder(results_dir), mkdir(results_dir); end

figure('Name', 'Joint positions');
plot(traj.t, rad2deg(traj.q), 'LineWidth', 1.4);
grid on; xlabel('Time (s)'); ylabel('Joint angle (deg)');
title('Gen3 joint positions');
legend('J1','J2','J3','J4','J5','J6','J7');
saveas(gcf, fullfile(results_dir, 'joint_positions.png'));

figure('Name', 'Joint torques');
plot(traj.t, tau, 'LineWidth', 1.4); hold on;
for j = 1:7
    yline( params.effort_limit(j), '--', 'Color', [0.7 0.2 0.2]);
    yline(-params.effort_limit(j), '--', 'Color', [0.7 0.2 0.2]);
end
grid on; xlabel('Time (s)'); ylabel('Torque (Nm)');
title('Gen3 joint torques vs manufacturer limits');
legend('J1','J2','J3','J4','J5','J6','J7');
saveas(gcf, fullfile(results_dir, 'joint_torques.png'));

figure('Name', 'End-effector path');
plot3(traj.ee_pos(:,1), traj.ee_pos(:,2), traj.ee_pos(:,3), 'b-', 'LineWidth', 1.5);
hold on;
wp = squeeze(waypoints(1:3, 4, :)).';
plot3(wp(:,1), wp(:,2), wp(:,3), 'ro', 'MarkerFaceColor', 'r');
axis equal; grid on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('End-effector path');
saveas(gcf, fullfile(results_dir, 'ee_path.png'));

% CSV summary
T = table((1:7).', params.effort_limit.', tau_info.peak_abs_tau.', ...
          (tau_info.utilisation * 100).', ...
          'VariableNames', {'Joint', 'Limit_Nm', 'Peak_Nm', 'Util_pct'});
writetable(T, fullfile(results_dir, 'torque_summary.csv'));

fprintf('Done. Results in %s\n', results_dir);
