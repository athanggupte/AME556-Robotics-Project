function [tau, F, z] = mpc_caller(p, v, w, R, q, vd, p1, p2, p3, p4, t, task)

params = get_gait_params();
N = params.N; dt = params.dt;
gait_length = params.gait_length;
tau = zeros(12,1);
F = zeros(12,1);
z=0;
if t < params.t_start
    return;
end

R = R';

eul_angles = rotm2eul(R);
% X = [p; eul_angles(3); eul_angles(2); eul_angles(1); v; w; -9.81];
I = diag([0.0168, 0.0565, 0.064]);

r1 = p1 - p;
r2 = p2 - p;
r3 = p3 - p;
r4 = p4 - p;

J_FL = foot_jacobian([q(12); q(8); q(4)], 1);
J_FR = foot_jacobian([q(11); q(7); q(3)], 2);
J_RL = foot_jacobian([q(10); q(6); q(2)], 3);
J_RR = foot_jacobian([q(9); q(5); q(1)], 4);

X = [p; reshape(R, [9 1]); v; R'*w];

pd = [0; 0; 0.28];
pddot = [0; 0; 0];
% m = 12;
thetad = [0; 0; 0];
wd = [0; 0; 0;];

if task == 0 % standing
    xd = [pd; thetad; pddot; wd];
    F = mpc_standing_caller(t, X, r1, r2, r3, r4, xd, dt, N, gait_length);
    z=0;
elseif task == 1 % walking
    pddot = vd;
    xd = [pd; thetad; pddot; wd];
    F = mpc_walking_caller(t, X, r1, r2, r3, r4, xd, dt, N, gait_length);
    z = 0;
elseif task == 2 % turning
    wd = [0; 0; 0.6];
    xd = [pd; thetad; pddot; wd];
    F = mpc_turning_caller(t, X, r1, r2, r3, r4, xd, dt, N, gait_length);
    z=0;
elseif task == 5 % climbing
    pddot = vd;
%     if t > 2.7
%         thetad = [0; max(-pi/6, (-pi/6)*t/2.7); 0;];
%     end
    W = [1, p1(1), p1(2);
        1, p2(1), p2(2);
        1, p3(1), p3(2);
        1, p4(1), p4(2);];
    a = pinv(W'*W)*W'*[p1(3); p2(3); p3(3); p4(3)];
    z = a(1) + a(2)*p(1) + a(3)*p(2) - 0.02;
%     fprintf("%f\n", sqrt(1 + a(2)^2 + a(3)^2));
    thetad(2) =  -pi/6;% - acos(1/sqrt(1 + a(2)^2 + a(3)^2));
    fprintf("t %f theta %f cos(theta) %f pcos %f z %f\n", t, ...
        acos(1/sqrt(1 + a(2)^2 + a(3)^2)), cos(acos(1/sqrt(1 + a(2)^2 + a(3)^2))), ...
        pd(3)*cos(acos(1/sqrt(1 + a(2)^2 + a(3)^2))), z);
    pd(3) = pd(3)*cos(acos(1/sqrt(1 + a(2)^2 + a(3)^2))) + z;
%     if p(1) - min(p3(1), p4(1)) > 0.2
%         fprintf("t %f pd(1) %f p3(1) %f p4(1) %f\n", t, p(1), p3(1), p4(1));
%         pddot = [0; 0; 0;];
%     end
    
%     pd(3) = pd(3)*cos(acos(1/sqrt(1 + a(2)^2 + a(3)^2)));
    xd = [pd; thetad; pddot; wd];
    F = mpc_climbing_caller(t, X, r1, r2, r3, r4, xd, dt, N, gait_length);
end

% Convert GRF to Torque
tau_FL = zeros(3,1);
tau_FR = zeros(3,1);
tau_RL = zeros(3,1);
tau_RR = zeros(3,1);

tau_FL = - J_FL' * R' * F(1:3);
tau_FR = - J_FR' * R' * F(4:6);
tau_RL = - J_RL' * R' * F(7:9);
tau_RR = - J_RR' * R' * F(10:12);

tau(1:3) = tau_FL(1:3);
tau(4:6) = tau_FR(1:3);
tau(7:9) = tau_RL(1:3);
tau(10:12) = tau_RR(1:3);
tau = [tau(12); tau(9); tau(6); tau(3); tau(11); tau(8); tau(5); tau(2); tau(10); tau(7); tau(4); tau(1)];


end

