function [tau, info] = kinova_gen3_torques(q, qd, qdd, robot, payload_kg)
% Compute joint torques along a trajectory using inverse dynamics.
%
% q, qd, qdd    Nx7 trajectory (positions, velocities, accelerations)
% robot         pre-built rigidBodyTree (optional)
% payload_kg    point mass at the end-effector, in kg (optional, default 0)
%
% Returns:
%   tau    Nx7 joint torques (Nm)
%   info   struct with peak torques and utilisation vs the manufacturer
%          limits. I use this to check if the trajectory stays in the
%          safe envelope.

    if nargin < 4 || isempty(robot)
        robot = kinova_gen3_build();
    end
    if nargin < 5
        payload_kg = 0;
    end

    if payload_kg > 0
        attach_payload(robot, payload_kg);
    end

    N = size(q, 1);
    tau = zeros(N, 7);

    % No external wrench for free-space motion. If the gripper is in
    % contact with the phone, this would need to change.
    fext = externalForce(robot, 'end_effector_link', zeros(6, 1));

    for k = 1:N
        tau(k, :) = inverseDynamics(robot, q(k, :), qd(k, :), qdd(k, :), fext);
    end

    params = kinova_gen3_params();
    info.peak_abs_tau  = max(abs(tau), [], 1);
    info.effort_limit  = params.effort_limit;
    info.utilisation   = info.peak_abs_tau ./ info.effort_limit;
    [info.max_util_pct, info.worst_joint] = max(info.utilisation * 100);
end


function attach_payload(robot, mass_kg)
    body = rigidBody('payload');
    jnt  = rigidBodyJoint('payload_joint', 'fixed');
    setFixedTransform(jnt, trvec2tform([0 0 0]));
    body.Joint = jnt;
    body.Mass = mass_kg;
    body.CenterOfMass = [0 0 0.02];
    body.Inertia = [1e-4 1e-4 1e-4 0 0 0];
    addBody(robot, body, 'end_effector_link');
end
