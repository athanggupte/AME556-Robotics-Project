function params = get_gait_params()

    params = struct('N', 0, 'dt', 0, 'gait_length', 0, 't_start', 0);
    params.N = 20;
    params.dt = 0.015;
    params.gait_length = params.N * params.dt/2;
    params.t_start = 2 - mod(2, params.gait_length);
end

