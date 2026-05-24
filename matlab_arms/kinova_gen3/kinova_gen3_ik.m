function [q_sol, info] = kinova_gen3_ik(T_target, q_init, robot)
% Inverse kinematics for the Gen3 using MATLAB's numerical solver.
%
% T_target  4x4 target pose for end_effector_link
% q_init    1x7 initial guess (optional - defaults to the ready pose).
%           For trajectory continuity, pass in the previous waypoint's
%           joint config.
% robot     pre-built rigidBodyTree (optional)
%
% The Gen3 is redundant (7-DoF), so the solution isn't unique. A good
% initial guess matters a lot.

    if nargin < 3 || isempty(robot)
        robot = kinova_gen3_build();
    end
    if nargin < 2 || isempty(q_init)
        p = kinova_gen3_params();
        q_init = p.q_ready;
    end
    if iscolumn(q_init), q_init = q_init.'; end

    ik = inverseKinematics('RigidBodyTree', robot);
    ik.SolverParameters.MaxIterations = 1500;

    weights = [1 1 1 1 1 1];   % equal weight to orientation and position
    [q_sol, info] = ik('end_effector_link', T_target, weights, q_init);

    if info.PoseErrorNorm > 1e-3
        warning('IK pose error %.4f - solution might be off', info.PoseErrorNorm);
    end
end
