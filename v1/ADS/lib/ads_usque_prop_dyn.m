function [q0, x, Pm, qs, chi] = ads_usque_prop_dyn(q, x, Pm, tau_c, h_w, r_I, P)
%ADS_USQUE_PROP_DYN GYROLESS USQUE propagation with Euler dynamics and RK4,
% Eqs. (11.1)-(11.5). State x = [dp; omega]; additive process noise after propagation.
%#codegen
n = 6;
dt = P.dt;
[qs, chi] = ads_usque_sigma(q, x, (n + P.lambda)*Pm, P);
h = dt/P.gl.N_sub;
for i = 1:2*n+1
    s = [qs(:,i); chi(4:6,i)];
    for k = 1:P.gl.N_sub
        k1 = dyn(s, tau_c, h_w, r_I, P);
        k2 = dyn(s + 0.5*h*k1, tau_c, h_w, r_I, P);
        k3 = dyn(s + 0.5*h*k2, tau_c, h_w, r_I, P);
        k4 = dyn(s + h*k3, tau_c, h_w, r_I, P);
        s = s + h/6*(k1 + 2*k2 + 2*k3 + k4);
        s(1:4) = ads_qnorm(s(1:4));
    end
    qs(:,i) = s(1:4);
    chi(4:6,i) = s(5:7);
end
Sa = P.gl.Jinv*(P.gl.sig_tau^2*eye(3))*P.gl.Jinv.';
Qg = [dt^3/3*Sa + P.gl.sig_theta^2*dt*eye(3), dt^2/2*Sa; dt^2/2*Sa, dt*Sa];
[q0, x, Pm, chi] = ads_usque_moments(qs, chi, Qg, P);
end

function ds = dyn(s, tau, hw, rI, P)
% Rigid body with reaction wheels and gravity gradient, Eqs. (11.1)-(11.2).
q = s(1:4);
w = s(5:7);
J = P.gl.J;
tg = zeros(3,1);
nr = norm(rI);
if nr > 1000
    nb = ads_q2A(q)*rI(:)/nr;
    tg = 3*P.mu/nr^3*cross(nb, J*nb);
end
Om = [-ads_skew(w), w; -w.', 0];
ds = [0.5*Om*q; P.gl.Jinv*(-cross(w, J*w + hw(:)) + tau(:) + tg)];
end
