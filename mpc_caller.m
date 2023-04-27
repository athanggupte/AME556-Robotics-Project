function [tau, F] = mpc_caller(p, v, w, R, q, p1, p2, p3, p4, t)

N = 20; dt = 0.02;
gait_length = 0.2;
tau = zeros(12,1);
F = zeros(12,1);
if t < 2
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

% [phase, phase_start] = get_current_phase(t, gait_length);
% Now phase is for the next interval, so we should go back
% phase = 1-phase;

J_FL = foot_jacobian([q(12); q(8); q(4)], 1);
J_FR = foot_jacobian([q(11); q(7); q(3)], 2);
J_RL = foot_jacobian([q(10); q(6); q(2)], 3);
J_RR = foot_jacobian([q(9); q(5); q(1)], 4);

% F = mpc(X, R, I, r1, r2, r3, r4, phase, t, c);
pd = [0; 0; 0.25];
pddot = [0; 0; 0];
% m = 12;
thetad = [0; 0; 0];
wd = [0; 0; 0;];
xd = [pd; thetad; pddot; wd];
X = [p; reshape(R, [9 1]); v; R'*w];
F = mpc_standing_caller(t, X, r1, r2, r3, r4, xd, dt, N);
%F = -F;

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

