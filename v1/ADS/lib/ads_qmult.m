function r = ads_qmult(p, q)
%ADS_QMULT Quaternion product p (x) q, scalar last, A(p(x)q) = A(p)A(q). Eq. (2.5).
%#codegen
p = p(:); q = q(:);
r = [p(4)*q(1:3) + q(4)*p(1:3) - cross(p(1:3), q(1:3)); ...
     p(4)*q(4) - p(1:3).'*q(1:3)];
end
