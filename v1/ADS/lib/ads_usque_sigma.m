function [qs, chi, ok] = ads_usque_sigma(q, x, A, P)
%ADS_USQUE_SIGMA Sigma points and sigma-point quaternions, Eqs. (5.21), (5.23).
% A is the matrix to factorise: (n+lambda)*P (dynamics USQUE) or
% (n+lambda)*(P + Qbar) (gyro-based USQUE).
%#codegen
n = 6;
A = 0.5*(A + A.');
[L, p] = chol(A, 'lower');
ok = p == 0;
if ~ok
    A = A + 1e-9*trace(A)/n*eye(n);
    [L, p] = chol(A, 'lower');
    ok = p == 0;
    if ~ok
        L = diag(sqrt(max(diag(A), 0)));
    end
end
chi = [x, x*ones(1,n) + L, x*ones(1,n) - L];
qs = zeros(4, 2*n+1);
for i = 1:2*n+1
    qs(:,i) = ads_qnorm(ads_qmult(ads_grp2dq(chi(1:3,i), P.a, P.f), q));
end
end
