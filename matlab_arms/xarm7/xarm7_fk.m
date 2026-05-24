function T = xarm7_fk(q, robot, ee_name)
% Forward kinematics for the xArm 7. Default end-effector frame is
% link_eef (tool flange). Pass 'link_tcp' for the gripper tip.

    if nargin < 2 || isempty(robot)
        robot = xarm7_build();
    end
    if nargin < 3 || isempty(ee_name)
        ee_name = 'link_eef';
    end
    if iscolumn(q), q = q.'; end

    T = getTransform(robot, q, ee_name, robot.BaseName);
end
