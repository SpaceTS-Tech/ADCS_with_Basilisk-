function [dx, Pm, e, acc] = ads_mekf_update_vec(dx, Pm, q, b_meas, r_ref, sig, P)
%ADS_MEKF_UPDATE_VEC Sequential MEKF update with a unit-vector observation,
% Eqs. (5.12), (5.14)-(5.17). Gate with 2 degrees of freedom (Section 5.4).
%#codegen
bh = ads_q2A(q)*r_ref(:);
H = [ads_skew(bh), zeros(3)];
R = sig^2*eye(3);
nu = b_meas(:) - bh - H*dx;
S = H*Pm*H.' + R;
e = nu.'*(S\nu);
acc = e <= P.gate(2);
if acc
    K = (Pm*H.')/S;
    dx = dx + K*nu;
    IKH = eye(6) - K*H;
    Pm = IKH*Pm*IKH.' + K*R*K.';
    Pm = 0.5*(Pm + Pm.');
end
end
