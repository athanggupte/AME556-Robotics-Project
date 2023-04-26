function [tau, pd_foot, vd_foot, Fswing] = foot_placement_walking(p, R, v, p_hip, p_foot, v_foot, q, t)

gait_length = 0.15;
Fswing = zeros(12, 1);
tau = zeros(12, 1);
pd_foot = p_hip + [0.05; 0; 0; 
                    0.05; 0; 0; 
                    0.05; 0; 0;
                    0.05; 0; 0;];
vd_foot = zeros(12, 1);
if t < 2
    return;
end

vd_com = [0; 0; 0;
      0; 0; 0;
      0; 0; 0;
      0; 0; 0];

v_com = [v; v; v; v];


R = R';

t_stance = gait_length;
K_step = sqrt(p(3) / 9.81);

pfinal_foot = p_hip + t_stance / 2 * v_com + K_step * (v_com - vd_com);
pfinal_foot(3) = 0;
pfinal_foot(6) = 0;
pfinal_foot(9) = 0;
pfinal_foot(12) = 0;

Kp = 100; Kd = 0.5;

phase = 0;
c = 0;
while t >= c
    c = c + gait_length;
    phase = 1-phase;
end
c = c - gait_length;
tswing = gait_length;
[pd_foot, vd_foot] = calculate_foot_trajectory(t, c, tswing, phase, pd_foot, vd_foot);

Fswing = Kp * (pd_foot - p_foot) + Kd * (vd_foot - v_foot);
% Fswing = -Fswing;

J_FL = foot_jacobian([q(12); q(8); q(4)], 1);
J_FR = foot_jacobian([q(11); q(7); q(3)], 2);
J_RL = foot_jacobian([q(10); q(6); q(2)], 3);
J_RR = foot_jacobian([q(9); q(5); q(1)], 4);

tau_FL = zeros(3,1);
tau_FR = zeros(3,1);
tau_RL = zeros(3,1);
tau_RR = zeros(3,1);

tau_FL = - J_FL' * R' * Fswing(1:3);
tau_FR = - J_FR' * R' * Fswing(4:6);
tau_RL = - J_RL' * R' * Fswing(7:9);
tau_RR = - J_RR' * R' * Fswing(10:12);


tau(1:3) = tau_FL(1:3);
tau(4:6) = tau_FR(1:3);
tau(7:9) = tau_RL(1:3);
tau(10:12) = tau_RR(1:3);
tau = [tau(12); tau(9); tau(6); tau(3); tau(11); tau(8); tau(5); tau(2); tau(10); tau(7); tau(4); tau(1)];


tau = tau.*[phase; 1-phase; 1-phase; phase;
            phase; 1-phase; 1-phase; phase;
            phase; 1-phase; 1-phase; phase];


end


function [pd_foot, vd_foot] = calculate_foot_trajectory(t, c, tswing, phase, pd_foot, vd_foot)

h = 0.0;
v = 0;

if phase
    if t < (c + tswing/2)
        fprintf("%f FL, RR, up\n", t);
        pd_foot(3) = (t-c)*2*h/tswing;
        pd_foot(12) = (t-c)*2*h/tswing;
        vd_foot(3) = (t-c)*2*v/tswing;
        vd_foot(12) = (t-c)*2*v/tswing;
    else
        fprintf("%f FL, RR, down\n", t);
        pd_foot(3) = h - (t-c-tswing/2)*2*h/tswing;
        pd_foot(12) = h - (t-c-tswing/2)*2*h/tswing;
        vd_foot(3) = v - (t-c-tswing/2)*2*v/tswing;
        vd_foot(12) = v - (t-c-tswing/2)*2*v/tswing;
    end
%     vd_foot(3) = v - (t-c)*2*v/tswing;
%     vd_foot(12) = v - (t-c)*2*v/tswing;
else
    if t < (c + tswing/2)
        fprintf("%f RL, FR, up\n", t);
        pd_foot(6) = (t-c)*2*h/tswing;
        pd_foot(9) = (t-c)*2*h/tswing;
        vd_foot(6) = (t-c)*2*v/tswing;
        vd_foot(9) = (t-c)*2*v/tswing;
    else
        fprintf("%f RL, FR, down\n", t);
        pd_foot(6) = h - (t-c-tswing/2)*2*h/tswing;
        pd_foot(9) = h - (t-c-tswing/2)*2*h/tswing;
        vd_foot(6) = v - (t-c-tswing/2)*2*v/tswing;
        vd_foot(9) = v - (t-c-tswing/2)*2*v/tswing;
    end
%     vd_foot(6) = v - (t-c)*2*v/tswing;
%     vd_foot(9) = v - (t-c)*2*v/tswing;
end

end

