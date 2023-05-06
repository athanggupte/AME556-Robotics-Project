function u = mpc_turning_caller(t,q,r1,r2,r3,r4,xd,dt,N,gait_length)
%      Q_mpc = diag([ ...
%          20, ... % px
%          20, ... % py
%          70, ... % pz
%          100, ... % roll
%          100, ... % pitch
%          0, ... % yaw
%          40, ...  % vx
%          40, ...  % vy
%          40, ...  % vz
%          150, ... % wx
%          75, ...  % wy
%          350 ...  % wz
%      ]);
    Q_mpc = diag([ ...
             35, ... % px
             35, ... % py
             80, ... % pz
             10, ... % roll
             10, ... % pitch
             0, ... % yaw
             5, ...  % vx
             5, ...  % vy
             50, ...  % vz
             30, ... % wx
             30, ...  % wy
             70 ...  % wz
            ]);
     R_mpc = 0.0000001*eye(12);
%      gaitname = 'standing';
     gaitname = 'trotting';
%      gaitname = 'trotting_FR';
     u = mpc_soln(t,q,r1,r2,r3,r4,xd,Q_mpc,R_mpc,dt,N,gait_length,gaitname);
end