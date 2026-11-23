function [dx, Pm, e, acc] = ads_mekf_update_st(dx, Pm, q, q_meas, R, P)
%ADS_MEKF_UPDATE_ST Sequential MEKF update with a star-tracker quaternion,
% Eqs. (5.13)-(5.17). Gate with 3 degrees of freedom (Section 5.4).
%#codegen
dq = ads_qmult(q_meas, ads_qinv(q));
if dq(4) < 0
    dq = -dq;
end
y = 2*dq(1:3);
H = [eye(3), zeros(3)];
nu = y - H*dx;
S = H*Pm*H.' + R;
e = nu.'*(S\nu);
acc = e <= P.gate(3);
if acc
    K = (Pm*H.')/S;
    dx = dx + K*nu;
    IKH = eye(6) - K*H;
    Pm = IKH*Pm*IKH.' + K*R*K.';
    Pm = 0.5*(Pm + Pm.');
end
end
