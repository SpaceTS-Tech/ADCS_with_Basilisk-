function [x, Pm, e, acc] = ads_usque_update(x, Pm, qs, chi, y, rv, R, kind, P)
%ADS_USQUE_UPDATE Joint USQUE measurement update with per-block gating, Eqs. (5.28)-(5.31).
% Up to three 3-row blocks; kind(j): 0 unused, 1 unit vector (reference rv),
% 2 attitude GRP (sigma-point attitude error). Unused or rejected blocks are
% neutralised (zero deviation, zero residual, unit covariance) to keep fixed sizes.
%#codegen
n = 6;
W = ads_usque_weights(P);
e = zeros(3,1);
acc = false(3,1);
G = zeros(9, 2*n+1);
for j = 1:3
    r = 3*j-2:3*j;
    for i = 1:2*n+1
        if kind(j) == 1
            G(r,i) = ads_q2A(qs(:,i))*rv(r);
        elseif kind(j) == 2
            G(r,i) = chi(1:3,i);
        end
    end
end
yh = G*W;
D = G - yh*ones(1, 2*n+1);
X = chi - x*ones(1, 2*n+1);
nu = y(:) - yh;
for j = 1:3
    if kind(j) == 0
        r = 3*j-2:3*j;
        D(r,:) = 0;  nu(r) = 0;  R(r,:) = 0;  R(:,r) = 0;  R(r,r) = eye(3);
    end
end
Wd = diag(W);
Pvv = D*Wd*D.' + R;
rejected = false;
for j = 1:3
    if kind(j) > 0
        r = 3*j-2:3*j;
        e(j) = nu(r).'*(Pvv(r,r)\nu(r));
        if kind(j) == 2
            acc(j) = e(j) <= P.gate(3);
        else
            acc(j) = e(j) <= P.gate(2);
        end
        if ~acc(j)
            D(r,:) = 0;  nu(r) = 0;  R(r,:) = 0;  R(:,r) = 0;  R(r,r) = eye(3);
            rejected = true;
        end
    end
end
if rejected
    Pvv = D*Wd*D.' + R;
end
Pxy = X*Wd*D.';
K = Pxy/Pvv;
x = x + K*nu;
Pm = Pm - K*Pvv*K.';
Pm = 0.5*(Pm + Pm.');
end
