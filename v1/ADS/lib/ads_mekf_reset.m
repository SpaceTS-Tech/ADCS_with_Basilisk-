function [q, b] = ads_mekf_reset(q, b, dx)
%ADS_MEKF_RESET Move the error estimate into the global state, Eq. (5.18).
%#codegen
q = ads_qnorm(ads_qmult(ads_dq_rotvec(dx(1:3)), q));
b = b(:) + dx(4:6);
end
