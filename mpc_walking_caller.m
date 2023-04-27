function u = mpc_walking_caller(t,q,r1,r2,r3,r4,xd,dt,N)
     Q_mpc = diag([45,80,35, 5,45,150, 50,55,4, 1,1,150]);
     R_mpc = 0.001*eye(12);
%      gaitname = 'standing';
     gaitname = 'trotting';
%      gaitname = 'trotting_FR';
     u = mpc_soln(t,q,r1,r2,r3,r4,xd,Q_mpc,R_mpc,dt,N,gaitname);
end