function u = mpc_soln(t,q,r1,r2,r3,r4,xd,Q_mpc,R_mpc,dt,N,gaitname)
    m = 12;
    Ib = diag([0.0168, 0.0565, 0.064]);
    state_num = 13;
    controller_num = 12;
    p = q(1:3);
    R = reshape(q(4:12),[3,3]);
    dp = q(13:15);
    wb = q(16:18);
    eul = rotm2eul(R);
    q0 = [p;eul(3);eul(2);eul(1);dp;wb];
%     Rd = eul2rotm(params.rpy);
    yaw = eul(1);
    Rz_yaw = [cos(yaw) sin(yaw) 0;
        -sin(yaw) cos(yaw) 0;
        0 0 1];
    Act = [zeros(3) zeros(3) eye(3) zeros(3) zeros(3,1);
        zeros(3) zeros(3) zeros(3) Rz_yaw zeros(3,1);
        zeros(3) zeros(3) zeros(3) zeros(3) [0;0;-1];
        zeros(4,13)];
    Iw = R'*Ib*R;
    D = [m*eye(3) zeros(3);
        zeros(3) Iw];
    A = [eye(3), eye(3), eye(3), eye(3);
        Vec2Skewmat(r1),Vec2Skewmat(r2),Vec2Skewmat(r3),Vec2Skewmat(r4)];
    Bct = [zeros(6,12);
        D\A;
        zeros(1,12)];
    At = eye(state_num)+dt*Act;
    Bt = dt*Bct;
    Q_mpc = blkdiag(Q_mpc,0);
    xd = [xd;9.8];
    H=[];
    coder.varsize("H");
    for i = 1:N
        H =  blkdiag(H,Q_mpc);
    end
    for i = 1:N
        H =  blkdiag(H,R_mpc);
    end
%     H = H(2:end, 2:end);
    f_blk = -Q_mpc*xd;
    f=[];
    coder.varsize("f");
    for i = 1:N
        f = [f;f_blk];
    end
%     f = f(2:end);
    f = [f;zeros(N*controller_num,1)];
    Aeq_bl1 = eye(state_num*N);
    Aeq_bl2 = [];
    coder.varsize("Aeq_bl2");
    for i = 1:N-1
        Aeq_bl2 = blkdiag(Aeq_bl2,-At);
    end
%     Aeq_bl2 = Aeq_bl2(2:end, 2:end);
    Aeq_bl2 = [zeros(state_num, state_num*N);
        Aeq_bl2, zeros(state_num*(N-1),state_num)];
    Aeq_bl3 =[];
    coder.varsize("Aeq_bl3");
    for i = 1:N
        Aeq_bl3 = blkdiag(Aeq_bl3,-Bt);
    end
%     Aeq_bl3 = Aeq_bl3(2:end, 2:end);
    Aeq = [Aeq_bl1+Aeq_bl2 Aeq_bl3];
    Beq = [At*[q0;9.8]; zeros((N-1)*state_num,1)];
    mu = 1;
    Aineq_blk = [1 0 -mu;
        -1 0 -mu;
        0 1 -mu;
        0 -1 -mu;
        0 0 1;
        0 0 -1];
    A_ineq = [];
    coder.varsize("A_ineq");
    for i = 1:(N*4)
        A_ineq = blkdiag(A_ineq,Aineq_blk);
    end
%     A_ineq = A_ineq(2:end, 2:end);
    A_ineq = [zeros(6*N*4,N*state_num) A_ineq];
    b_ineq = [];
    coder.varsize("b_ineq");
    mpctable = gait(t,N,dt,gaitname);
    for i = 0:N-1
        for j = 1:4
            b_ineq_blk = [0;0;0;0;500*mpctable(4*i + j);-10*mpctable(4*i + j)];
            b_ineq =[b_ineq;b_ineq_blk];
        end
    end
%     b_ineq = bineq(2:end);
    coder.extrinsic('quadprog');
%     options = optimoptions('quadprog','Display','off');
    X_star = quadprog(H,f,A_ineq,b_ineq,Aeq,Beq,[],[],[]);
    u = X_star(N*state_num+1:N*state_num+controller_num);
end

% function mpcTable = gait(t,N,dt,gaitname)
%     if isequal(gaitname, 'standing')
%         offsets = [0,0,0,0];
%         duration = [N,N,N,N];
%     elseif isequal(gaitname, 'trotting')
%         offsets = [0,N/2,N/2,0];
%         duration = [N/2,N/2,N/2,N/2];
%     elseif isequal(gaitname, 'bounding')
%         offsets = [N/2,N/2,0,0];
%         duration = [N/2,N/2,N/2,N/2];
%     end
%     mpcTable = zeros(N*4,1);
%     iteration = floor(mod(t/dt,N));
%     for i = 0:N-1
%         iter = mod((i +1 + iteration),N);
%         progress = iter - offsets;
%         for j = 1:4
%             if progress(j) < 0
%                 progress(j) = progress(j) + N;
%             end
%             if progress(j) < duration(j)
%                 mpcTable(i*4+j) = 1;
%             else
%                 mpcTable(i*4+j) = 0;
%             end
%         end
%     end
% end

function S = Vec2Skewmat(a)
    S = [0 -a(3) a(2);
        a(3) 0 -a(1);
        -a(2) a(1) 0];
end
