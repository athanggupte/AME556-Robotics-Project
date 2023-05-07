function u = mpc_running_caller(t,q,r1,r2,r3,r4,xd,dt,N,gait_length)
     Q_mpc = diag([ ...
         0, ... % px
         65, ... % py
         170, ... % pz
         150, ... % roll
         40, ... % pitch
         60, ... % yaw
         180, ...  % vx
         45, ...  % vy
         4, ...  % vz
         150, ... % wx
         10, ...  % wy
         70 ...  % wz
        ]);
     R_mpc = 0.0001*eye(12);
%      gaitname = 'standing';
     gaitname = 'running';
%      gaitname = 'trotting_FR';
     u = mpc_soln(t,q,r1,r2,r3,r4,xd,Q_mpc,R_mpc,dt,N,gait_length,gaitname);
end