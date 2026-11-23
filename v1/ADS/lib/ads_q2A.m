function A = ads_q2A(q)
%ADS_Q2A Attitude matrix (inertial -> body) from quaternion, Eq. (2.3).
%#codegen
v = [q(1); q(2); q(3)];
A = (q(4)^2 - v.'*v)*eye(3) + 2*(v*v.') - 2*q(4)*ads_skew(v);
end
