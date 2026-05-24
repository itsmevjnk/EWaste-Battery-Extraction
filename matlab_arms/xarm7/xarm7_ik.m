function [q_sol, info] = xarm7_ik(T_target, q_init, robot, ee_name)
% IK for the xArm 7. Same idea as the Gen3 version - pass in the
% previous waypoint as q_init for trajectory continuity.

    if nargin < 3 || isempty(robot)
        robot = xarm7_build();
    end
    if nargin < 4 || isempty(ee_name)
        ee_name = 'link_eef';
    end
    if nargin < 2 || isempty(q_init)
        p = xarm7_params();
        q_init = p.q_ready;
    end
    if iscolumn(q_init), q_init = q_init.'; end

    ik = inverseKinematics('RigidBodyTree', robot);
    ik.SolverParameters.MaxIterations = 1500;

    weights = [1 1 1 1 1 1];
    [q_sol, info] = ik(ee_name, T_target, weights, q_init);

    if info.PoseErrorNorm > 1e-3
        warning('IK pose error %.4f - check the target pose', info.PoseErrorNorm);
    end
end
