function u = mpc(q, Rot, I, r1, r2, r3, r4, phase, t, c)
    % Same MPC code as 2b except for constraints
    N = 50;
    dt = 0.01;
    pd = [0; 0; 0.3];
    pddot = [0; 0; 0];
    m = 12;
    thetad = [0; 0; 0];
    wd = [0; 0; 0;];
    xd = [pd; thetad; pddot; wd; -9.81];
    Q = diag([40000, 50000, 60000, 10, 10, 10, 4, 4, 4, 1, 1, 1]);
    Qg = blkdiag(Q, 0);
    R = 0.01*eye(12);
    H = zeros(250, 250);
    for i=1:10
        H(13*(i-1)+1:13*i, 13*(i-1)+1:13*i) = Qg;
        H(130+12*(i-1)+1:130+12*i, 130+12*(i-1)+1:130+12*i) = R;
    end
    H = 2*H;
    f = zeros(250, 1);
    for i=1:10
        f(13*(i-1)+1:13*i) = -2*Qg*xd;
    end

    mu = 0.5;
    Aineq = zeros(240, 250);
    for n=1:10
        for i=1:4
            j = 1+3*(i-1);
            Aineq(24*(n-1)+i, 130+12*(n-1)+j:130+12*(n-1)+j+2) = [0,0,1];
            Aineq(24*(n-1)+i+4, 130+12*(n-1)+j:130+12*(n-1)+j+2) = [0,0,-1];
            Aineq(24*(n-1)+i+8, 130+12*(n-1)+j:130+12*(n-1)+j+2) = [1,0,-mu];
            Aineq(24*(n-1)+i+12, 130+12*(n-1)+j:130+12*(n-1)+j+2) = [-1,0,-mu];
            Aineq(24*(n-1)+i+16, 130+12*(n-1)+j:130+12*(n-1)+j+2) = [0,1,-mu];
            Aineq(24*(n-1)+i+20, 130+12*(n-1)+j:130+12*(n-1)+j+2) = [0,-1,-mu];
        end
    end
    bineq = zeros(240, 1);
    for i=1:10
        t = t + dt;
        % If t crosses the time limit (c) for the phase, then we update c
        % and change the phase
        if t >= c
            c = c + 0.2;
            phase = 1-phase;
        end
        if phase == 0
            % For phase = 0, Front left and Rear right are in contact, so
            % their constraints are non-zero
            bineq(24*(i-1)+1:24*i) = [500 * [1; 0; 0; 1];
                                      -10 * [1; 0; 0; 1];
                                      zeros(16, 1)];
%         bineq(24*(i-1)+1:24*i) = [500 * [1; 1; 1; 1];
%                                               -10 * [1; 1; 1; 1];
%                                               zeros(16, 1)];
        else
            % For phase = 1, Front right and Rear left are in contact, so
            % their constraints are non-zero
            bineq(24*(i-1)+1:24*i) = [500 * [0; 1; 1; 0];
                                      -10 * [0; 1; 1; 0];
                                      zeros(16, 1)];
% bineq(24*(i-1)+1:24*i) = [500 * [1; 1; 1; 1];
%                                               -10 * [1; 1; 1; 1];
%                                               zeros(16, 1)];
        end
    end

    A = zeros(13, 13);
    B = zeros(13, 12);
    A(1:3, :) = [zeros(3,6), eye(3), zeros(3,4)];
    yaw = q(6);
    T = [cos(yaw), -sin(yaw), 0;
         sin(yaw), cos(yaw), 0;
         0, 0, 1];
    A(4:6, :) = [zeros(3,9), inv(T), zeros(3,1)];
    A(7:9, :) = [zeros(3,12), [0; 0; 1]];
    Abar = A*dt + eye(13);
    B(7:9, :) = [eye(3), eye(3), eye(3), eye(3)] / m;
    Iw = Rot*I*Rot';
    rxF = [0, -r1(3), r1(2), 0, -r2(3), r2(2), 0, -r3(3), r3(2), 0, -r4(3), r4(2);
           r1(3), 0, -r1(1), r2(3), 0, -r2(1), r3(3), 0, -r3(1), r4(3), 0, -r4(1);
          -r1(2), r1(1), 0, -r2(2), r2(1), 0, -r3(2), r3(1), 0, -r4(2), r4(1), 0];
    B(10:12, :) = Iw \ rxF;
    Bbar = B*dt;
    Aeq = zeros(130, 250);
    Aeq(1:13, :) = [eye(13), zeros(13, 117), -Bbar, zeros(13, 108)];
    for i=2:10
        Aeq(13*(i-1)+1:13*i, 13*(i-2)+1:13*(i-1)) = -Abar;
        Aeq(13*(i-1)+1:13*i, 13*(i-1)+1:13*i) = eye(13, 13);
        Aeq(13*(i-1)+1:13*i, 130+12*(i-1)+1:130+12*i) = -Bbar;
    end
    beq = zeros(130, 1);
    beq(1:13) = Abar * q;

    coder.extrinsic('quadprog')
    umpc = quadprog(H, f, Aineq, bineq, Aeq, beq);
    u = umpc(131:142);
end