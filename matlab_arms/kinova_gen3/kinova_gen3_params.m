function params = kinova_gen3_params()
% Parameters for the Kinova Gen3 7-DoF arm.
% I pulled all the numbers from the official URDF in the ros_kortex
% repo (GEN3-7DOF-VISION_ARM_URDF_V12.urdf) and the Kinova spec sheet.

    params.dof        = 7;
    params.reach_m    = 0.902;
    params.payload_kg = 4.0;
    params.weight_kg  = 8.2;

    % Joint position limits (rad). Joints 1, 3, 5, 7 are continuous in
    % the URDF, but I'm clamping them to +/- pi so trajectory planning
    % doesn't try to wind them up.
    params.joint_lower = [-pi, -2.24, -pi, -2.57, -pi, -2.09, -pi];
    params.joint_upper = [ pi,  2.24,  pi,  2.57,  pi,  2.09,  pi];

    % Joint velocity limits (rad/s) - large actuators 1-4, small 5-7
    params.vel_limit = [1.3963 1.3963 1.3963 1.3963 1.2218 1.2218 1.2218];

    % Joint torque limits (Nm). Large actuators: 39 Nm. Small: 9 Nm.
    params.effort_limit = [39 39 39 39 9 9 9];

    % Link masses (kg) - from URDF inertial blocks
    params.link_mass = [1.697 1.3773 1.1636 1.1636 0.9302 0.6781 0.6781 0.5006];

    % Where to find the URDF
    params.local_urdf = fullfile(fileparts(mfilename('fullpath')), '..', ...
                                 'urdf', 'gen3_7dof.urdf');

    % Default starting pose for the battery extraction task.
    % These angles put the arm in a "ready over the table" position.
    % Tweak after running the demo if it doesn't fit your setup.
    params.q_home  = zeros(1, 7);
    params.q_ready = wrapToPi(deg2rad([0, 15, 0, -130, 0, 55, 90]));
end
