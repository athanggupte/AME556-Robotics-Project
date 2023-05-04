function [phase, phase_start] = get_current_phase_running(t, gait_length)

    phase = 1;
    phase_start = 0;
    while t >= phase_start
        phase_start = phase_start + gait_length;
        phase = mod(phase+1,3);
%         fprintf("%d\n", phase);
    end
    phase_start = phase_start - gait_length;    

end