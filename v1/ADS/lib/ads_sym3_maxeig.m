function lmax = ads_sym3_maxeig(M)
%ADS_SYM3_MAXEIG Largest eigenvalue of a symmetric 3x3 matrix (closed form).
% Used for 3*sigma_att = 3*sqrt(lambda_max(P_att)), Section 5.4.
%#codegen
M = 0.5*(M + M.');
p1 = M(1,2)^2 + M(1,3)^2 + M(2,3)^2;
if p1 < 1e-30
    lmax = max([M(1,1), M(2,2), M(3,3)]);
    return
end
qm = (M(1,1) + M(2,2) + M(3,3))/3;
p2 = (M(1,1) - qm)^2 + (M(2,2) - qm)^2 + (M(3,3) - qm)^2 + 2*p1;
p = sqrt(p2/6);
B = (M - qm*eye(3))/p;
r = det(B)/2;
r = min(max(r, -1), 1);
phi = acos(r)/3;
lmax = qm + 2*p*cos(phi);
end
