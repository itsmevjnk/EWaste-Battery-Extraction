function robot = kinova_gen3_build()
% Build the Kinova Gen3 as a MATLAB rigidBodyTree from the URDF.

    params = kinova_gen3_params();

    if ~isfile(params.local_urdf)
        error('Cannot find Gen3 URDF at:\n  %s', params.local_urdf);
    end

    robot = importrobot(params.local_urdf);
    robot.DataFormat = 'row';
    robot.Gravity = [0 0 -9.80665];
end
