% demo_xarm7_battery_extraction.m
%
% Same battery extraction trajectory as the Gen3 demo, but for the xArm 7.
% Useful for comparing torque profiles between the two arms.

clear; clc; close all;

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'xarm7'));
addpath(fullfile(here, '..', 'common'));

fprintf('Building xArm 7...\n');
robot  = xarm7_build();
params = xarm7_params();

% Same Cartesian waypoints I used for the Gen3
z_table    = 0.05;
z_approach = z_table + 0.08;
z_lift     = z_table + 0.15;

R_down = eul2rotm([0, pi, 0], 'ZYX');
make_pose = @(xyz) [R_down, xyz(:); 0 0 0 1];

waypoints = zeros(4, 4, 5);
waypoints(:, :, 1) = make_pose([0.40, 0.00, z_lift]);
waypoints(:, :, 2) = make_pose([0.45, 0.00, z_approach]);
waypoints(:, :, 3) = make_pose([0.45, 0.00, z_table+0.005]);
waypoints(:, :, 4) = make_pose([0.45, 0.00, z_lift]);
waypoints(:, :, 5) = make_pose([0.40, 0.30, z_lift]);

t_seg = [1.5; 1.5; 1.0; 2.0];

fprintf('Planning trajectory...\n');
traj = trajectory_generate(robot, waypoints, t_seg, 'link_eef', params.q_ready);

fprintf('Computing torques...\n');
[tau, tau_info] = xarm7_torques(traj.q, traj.qd, traj.qdd, robot, 0.05);

report = validate_trajectory(traj, tau, params, 0.05);

results_dir = fullfile(here, '..', 'results', 'xarm7');
if ~isfolder(results_dir), mkdir(results_dir); end

figure('Name', 'xArm7 joint positions');
plot(traj.t, rad2deg(traj.q), 'LineWidth', 1.4);
grid on; xlabel('Time (s)'); ylabel('Joint angle (deg)');
title('xArm 7 joint positions');
legend('J1','J2','J3','J4','J5','J6','J7');
saveas(gcf, fullfile(results_dir, 'joint_positions.png'));

figure('Name', 'xArm7 joint torques');
plot(traj.t, tau, 'LineWidth', 1.4); hold on;
for j = 1:7
    yline( params.effort_limit(j), '--', 'Color', [0.7 0.2 0.2]);
    yline(-params.effort_limit(j), '--', 'Color', [0.7 0.2 0.2]);
end
grid on; xlabel('Time (s)'); ylabel('Torque (Nm)');
title('xArm 7 joint torques vs manufacturer limits');
legend('J1','J2','J3','J4','J5','J6','J7');
saveas(gcf, fullfile(results_dir, 'joint_torques.png'));

figure('Name', 'xArm7 EE path');
plot3(traj.ee_pos(:,1), traj.ee_pos(:,2), traj.ee_pos(:,3), 'b-', 'LineWidth', 1.5);
hold on;
wp = squeeze(waypoints(1:3, 4, :)).';
plot3(wp(:,1), wp(:,2), wp(:,3), 'ro', 'MarkerFaceColor', 'r');
axis equal; grid on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('End-effector path');
saveas(gcf, fullfile(results_dir, 'ee_path.png'));

T = table((1:7).', params.effort_limit.', tau_info.peak_abs_tau.', ...
          (tau_info.utilisation * 100).', ...
          'VariableNames', {'Joint', 'Limit_Nm', 'Peak_Nm', 'Util_pct'});
writetable(T, fullfile(results_dir, 'torque_summary.csv'));

fprintf('Done. Results in %s\n', results_dir);
