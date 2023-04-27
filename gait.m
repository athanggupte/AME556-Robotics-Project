function mpcTable = gait(t,N,dt,gaitname)
    if isequal(gaitname, 'standing')
        offsets = [0,0,0,0];
        duration = [N,N,N,N];
    elseif isequal(gaitname, 'trotting')
        offsets = [0,N/2,N/2,0];
        duration = [N/2,N/2,N/2,N/2];
    elseif isequal(gaitname, 'bounding')
        offsets = [N/2,N/2,0,0];
        duration = [N/2,N/2,N/2,N/2];
    end
    mpcTable = zeros(N*4,1);
    iteration = floor(mod(t/dt,N));
    for i = 0:N-1
        iter = mod((i + iteration), N);
        progress = iter - offsets;
        for j = 1:4
            if progress(j) < 0
                progress(j) = progress(j) + N;
            end
            if progress(j) < duration(j)
                mpcTable(i*4+j) = 1;
            else
                mpcTable(i*4+j) = 0;
            end
        end
    end
end