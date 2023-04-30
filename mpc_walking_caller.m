function u = mpc_walking_caller(t,q,r1,r2,r3,r4,xd,dt,N,gait_length)
     Q_mpc = diag([ ...
            0, ...  % px
            80, ... % py
            100, ... % pz
            90, ... % roll
            25, ... % pitch
            100, ...% yaw
            80, ... % vx
            40, ... % vy
            4, ...  % vz
            90, ...  % wx
            60, ...  % wy
            70 ...  % wz
        ]);
     R_mpc = 0.001*eye(12);
%      gaitname = 'standing';
     gaitname = 'trotting';
%      gaitname = 'trotting_FR';
     u = mpc_soln(t,q,r1,r2,r3,r4,xd,Q_mpc,R_mpc,dt,N,gait_length,gaitname);
end