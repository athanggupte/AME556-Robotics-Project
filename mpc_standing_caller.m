function u = mpc_standing_caller(t,q,r1,r2,r3,r4,xd,dt,N)
     Q_mpc = diag([40,50,60,10,10,10,4,4,4,1,1,1]);
     R_mpc = 0.00001*eye(12);
%      gaitname = 'standing';
     gaitname = 'trotting';
     u = mpc_soln(t,q,r1,r2,r3,r4,xd,Q_mpc,R_mpc,dt,N,gaitname);
end