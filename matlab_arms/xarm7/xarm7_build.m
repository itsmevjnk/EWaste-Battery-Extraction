function robot = xarm7_build()
% Build the xArm 7 from the URDF in the urdf folder.

    params = xarm7_params();

    if ~isfile(params.local_urdf)
        error('Cannot find xArm 7 URDF at:\n  %s', params.local_urdf);
    end

    robot = importrobot(params.local_urdf);
    robot.DataFormat = 'row';
    robot.Gravity = [0 0 -9.80665];
end
