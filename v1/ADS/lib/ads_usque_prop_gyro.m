function [q0, x, Pm, qs, chi] = ads_usque_prop_gyro(q, x, Pm, w_meas, P)
%ADS_USQUE_PROP_GYRO Gyro-based USQUE propagation, Eqs. (5.21)-(5.27).
% State x = [dp; beta]. Process noise enters before the sigma points (Crassidis & Markley).
%#codegen
n = 6;
dt = P.dt;
sv2 = P.gyr.sig_v^2;
su2 = P.gyr.sig_u^2;
Qb = dt/2*blkdiag((sv2 - su2*dt^2/6)*eye(3), su2*eye(3));
[qs, chi] = ads_usque_sigma(q, x, (n + P.lambda)*(Pm + Qb), P);
for i = 1:2*n+1
    qs(:,i) = ads_qprop(qs(:,i), w_meas(:) - chi(4:6,i), dt);
end
[q0, x, Pm, chi] = ads_usque_moments(qs, chi, Qb, P);
end
