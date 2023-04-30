function u = mpc_standing_caller(t,q,r1,r2,r3,r4,xd,dt,N,gait_length)
     Q_mpc = diag([ ...
         50, ... % px
         60, ... % py
         70, ... % pz
         35, ... % roll
         10, ... % pitch
         35, ... % yaw
         4, ...  % vx
         4, ...  % vy
         4, ...  % vz
         70, ... % wx
         1, ...  % wy
         70 ...  % wz
     ]);
     R_mpc = 0.00001*eye(12);
%      gaitname = 'standing';
     gaitname = 'trotting';
%      gaitname = 'trotting_FR';
     u = mpc_soln(t,q,r1,r2,r3,r4,xd,Q_mpc,R_mpc,dt,N,gait_length,gaitname);
end