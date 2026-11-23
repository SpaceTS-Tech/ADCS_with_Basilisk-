function q = ads_qprop(q, w, dt)
%ADS_QPROP Discrete quaternion propagation with constant rate, Eqs. (2.9)-(2.10).
%#codegen
wn = norm(w);
if wn*dt < 1e-8
    psi = 0.5*dt*w(:);
    c = 1;
else
    psi = sin(0.5*wn*dt) / wn * w(:);
    c = cos(0.5*wn*dt);
end
Om = [c*eye(3) - ads_skew(psi), psi; -psi.', c];
q = ads_qnorm(Om*q(:));
end
