function T = kinova_gen3_fk(q, robot)
% Forward kinematics for the Gen3. Returns the base -> end_effector_link
% transform as a 4x4 matrix.
%
% If you're calling this in a loop, pass in a pre-built robot to avoid
% rebuilding it every time.

    if nargin < 2 || isempty(robot)
        robot = kinova_gen3_build();
    end
    if iscolumn(q), q = q.'; end

    T = getTransform(robot, q, 'end_effector_link', robot.BaseName);
end
