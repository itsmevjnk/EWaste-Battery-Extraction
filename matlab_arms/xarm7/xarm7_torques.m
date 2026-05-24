function [tau, info] = xarm7_torques(q, qd, qdd, robot, payload_kg)
% Inverse dynamics for the xArm 7. Same interface as the Gen3 version.

    if nargin < 4 || isempty(robot)
        robot = xarm7_build();
    end
    if nargin < 5
        payload_kg = 0;
    end

    if payload_kg > 0
        attach_payload(robot, payload_kg);
    end

    N = size(q, 1);
    tau = zeros(N, 7);
    fext = externalForce(robot, 'link_eef', zeros(6, 1));

    for k = 1:N
        tau(k, :) = inverseDynamics(robot, q(k, :), qd(k, :), qdd(k, :), fext);
    end

    params = xarm7_params();
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
    addBody(robot, body, 'link_eef');
end
