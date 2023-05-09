function u = mpc_climbing_caller(t,q,r1,r2,r3,r4,xd,dt,N,gait_length)
     Q_mpc = diag([ ...
         0, ... % px
         80, ... % py
         150, ... % pz
         50, ... % roll
         150, ... % pitch
         50, ... % yaw
         100, ...  % vx
         80, ...  % vy
         4, ...  % vz
         50, ... % wx
         10, ...  % wy
         50 ...  % wz
        ]);
     R_mpc = 0.00001*eye(12);
%      gaitname = 'standing';
     gaitname = 'trotting';
%      gaitname = 'trotting_FR';
     u = mpc_soln(t,q,r1,r2,r3,r4,xd,Q_mpc,R_mpc,dt,N,gait_length,gaitname);
end