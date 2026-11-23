function [q, Pm] = ads_mekf_propagate(q, b, Pm, w_meas, P)
%ADS_MEKF_PROPAGATE MEKF propagation, design document Eqs. (5.7)-(5.11).
%#codegen
dt = P.dt;
I3 = eye(3);
w = w_meas(:) - b(:);
q = ads_qprop(q, w, dt);
wn = norm(w);
W = ads_skew(w);
if wn*dt < 1e-6
    Phi11 = I3 - W*dt;
    Phi12 = -I3*dt + 0.5*W*dt^2;
else
    s = sin(wn*dt);
    c = cos(wn*dt);
    Phi11 = I3 - W*s/wn + W*W*(1 - c)/wn^2;
    Phi12 = W*(1 - c)/wn^2 - I3*dt - W*W*(wn*dt - s)/wn^3;
end
Phi = [Phi11, Phi12; zeros(3), I3];
sv2 = P.gyr.sig_v^2;
su2 = P.gyr.sig_u^2;
Q = [(sv2*dt + su2*dt^3/3)*I3, -0.5*su2*dt^2*I3; -0.5*su2*dt^2*I3, su2*dt*I3];
Pm = Phi*Pm*Phi.' + Q;
Pm = 0.5*(Pm + Pm.');
end
