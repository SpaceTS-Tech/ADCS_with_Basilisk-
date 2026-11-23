function [q, x] = ads_usque_reset(q0, x, P)
%ADS_USQUE_RESET Quaternion reset about sigma point 0, Eq. (5.32).
%#codegen
q = ads_qnorm(ads_qmult(ads_grp2dq(x(1:3), P.a, P.f), q0));
x(1:3) = 0;
end
