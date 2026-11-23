function [q0, xm, Pm, chi] = ads_usque_moments(qs, chi, Q, P)
%ADS_USQUE_MOMENTS Propagated attitude errors relative to sigma point 0, mean and
% covariance, Eqs. (5.25)-(5.27).
%#codegen
n = 6;
W = ads_usque_weights(P);
q0 = qs(:,1);
q0i = ads_qinv(q0);
for i = 1:2*n+1
    chi(1:3,i) = ads_dq2grp(ads_qmult(qs(:,i), q0i), P.a, P.f);
end
chi(1:3,1) = 0;
xm = chi*W;
Dx = chi - xm*ones(1, 2*n+1);
Pm = Dx*diag(W)*Dx.' + Q;
Pm = 0.5*(Pm + Pm.');
end
