function [phase, phase_start] = get_current_phase(t, gait_length)

    phase = 0;
    phase_start = 0;
    while t >= phase_start
        phase_start = phase_start + gait_length;
        phase = 1-phase;
    end
    phase_start = phase_start - gait_length;

end