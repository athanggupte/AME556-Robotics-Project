function mpcTable = running_gait(t, N, dt, gait_length)
    phase0 = [0;1;1;0];
    phase1 = [1;0;0;1];
    flightphase = [0;0;0;0];
    mpcTable = [];
    coder.varsize("mpcTable");
    t = t + dt;
    [phase, phase_start] = get_current_phase_running(t, gait_length);
    next_phase_start = phase_start + gait_length;
    for n=1:N
        if t >= next_phase_start
            [phase, ~] = get_current_phase(t, gait_length);
            currentphase = flightphase;
            next_phase_start = next_phase_start + gait_length;
        elseif phase == 0
            currentphase = phase0;
        else %if phase == 1
            currentphase = phase1;
        end
        mpcTable = [mpcTable; currentphase];
        t = t + dt;
    end
end