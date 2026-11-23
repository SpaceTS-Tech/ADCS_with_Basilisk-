function dq = ads_dq_rotvec(dth)
%ADS_DQ_ROTVEC Exact error quaternion of a rotation vector, Eq. (2.13).
%#codegen
a = norm(dth);
if a < 1e-12
    dq = ads_qnorm([0.5*dth(:); 1]);
else
    dq = [dth(:)/a*sin(0.5*a); cos(0.5*a)];
end
end
