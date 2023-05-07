function mpcTable = running_gait(t, N, dt, gait_length, flight_length)
    phase0 = [0;1;1;0];
    phase1 = [1;0;0;1];
    flightphase = [0;0;0;0];
    %mpcTable = zeros(4 * N);
    mpcTable = [];
    coder.varsize("mpcTable");
    [phase, phase_start] = get_current_phase(t, gait_length);
    next_phase_start = phase_start + gait_length;
    t = t + dt;
    for n=0:N-1
%         if t < phase_start + flight_length * dt
%             currentphase = flightphase;
        if t >= next_phase_start - flight_length * dt + dt
            [phase, ~] = get_current_phase(t, gait_length);
            currentphase = flightphase;
            if t >= next_phase_start
                next_phase_start = next_phase_start + gait_length;
            end
        elseif phase == 0
            currentphase = phase0;
        else %if phase == 1
            currentphase = phase1;
        end
        mpcTable = [mpcTable; currentphase];
        % mpcTable(4*n+1:4*n+4) = currentphase;
        t = t + dt;
    end
end