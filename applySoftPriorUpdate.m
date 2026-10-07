function [x, P] = applySoftPriorUpdate(x, P, z, H, R)
% APPLYSOFTPRIORUPDATE  EKF update using pseudo-measurement z = Hx + n.

S = ensurePD(H * P * H' + R);
innov = z - H * x;
K = P * H' / S;
x = x + K * innov;
I = eye(size(P));
P = (I - K*H) * P * (I - K*H)' + K*R*K';
P = ensurePD(P);
end
