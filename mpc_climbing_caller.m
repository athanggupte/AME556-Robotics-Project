function u = mpc_climbing_caller(t,q,r1,r2,r3,r4,xd,dt,N,gait_length)
     Q_mpc = diag([ ...
         0, ... % px
         35, ... % py
         0, ... % pz
         350, ... % roll
         350, ... % pitch
         35, ... % yaw
         100, ...  % vx
         45, ...  % vy
         4, ...  % vz
         700, ... % wx
         10, ...  % wy
         70 ...  % wz
        ]);
     R_mpc = 0.0001*eye(12);
%      gaitname = 'standing';
     gaitname = 'trotting';
%      gaitname = 'trotting_FR';
     u = mpc_soln(t,q,r1,r2,r3,r4,xd,Q_mpc,R_mpc,dt,N,gait_length,gaitname);
end