function u = mpc_turning_caller(t,q,r1,r2,r3,r4,xd,dt,N,gait_length)
     Q_mpc = diag([ ...
         20, ... % px
         20, ... % py
         70, ... % pz
         300, ... % roll
         300, ... % pitch
         0, ... % yaw
         4, ...  % vx
         4, ...  % vy
         4, ...  % vz
         300, ... % wx
         300, ...  % wy
         100 ...  % wz
     ]);
     R_mpc = 0.00001*eye(12);
%      gaitname = 'standing';
     gaitname = 'trotting';
%      gaitname = 'trotting_FR';
     u = mpc_soln(t,q,r1,r2,r3,r4,xd,Q_mpc,R_mpc,dt,N,gait_length,gaitname);
end