%TEST_ADS_LIBRARY Unit checks of the ADS library (design document Section 14.3).
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'lib'));
addpath(fullfile(here, '..', '..'));
P = ads_params();
tol = 1e-12;

% Quaternion product composes like attitude matrices
rng(1);
for k = 1:100
    p = ads_qnorm(randn(4,1));  q = ads_qnorm(randn(4,1));
    assert(norm(ads_q2A(ads_qmult(p, q)) - ads_q2A(p)*ads_q2A(q)) < 1e-12);
    qq = ads_A2q(ads_q2A(q));
    assert(min(norm(qq - q), norm(qq + q)) < 1e-12);
end
% Passive convention: rotation of 90 deg about z maps inertial x to body -y
q = [0; 0; sin(pi/4); cos(pi/4)];
assert(norm(ads_q2A(q)*[1;0;0] - [0;-1;0]) < tol);
% Kinematics: propagate 1 s at 0.1 rad/s about x, compare with exact rotation
q1 = ads_qprop([0;0;0;1], [0.1;0;0], 1);
assert(norm(q1 - [sin(0.05);0;0;cos(0.05)]) < tol);
% GRP round trip
dq = ads_dq_rotvec([0.3; -0.2; 0.1]);
assert(norm(ads_grp2dq(ads_dq2grp(dq, 1, 4), 1, 4) - dq) < tol);

% Reference models: field magnitude and sun vector norm at 500 km
jd = juliandate(datetime(2026, 10, 23, 0, 0, 0));
r = [6878; 0; 0];  v = [0; 7.61*cosd(97.4); 7.61*sind(97.4)];
ref = ads_reference_models(jd, r, v, true, P);
fprintf('|B_I| = %.1f nT, |s_I| = %.6f, d_AU = %.4f, eclipse = %d\n', ...
    norm(ref.B_I)*1e9, norm(ref.s_I), ref.d_AU, ref.eclipse);
assert(norm(ref.B_I) > 15e-6 && norm(ref.B_I) < 60e-6);
assert(abs(norm(ref.s_I) - 1) < 1e-12);
% Sun near the autumn equinox+1 month: declination about -11 deg
fprintf('sun declination = %.2f deg\n', asind(ref.s_I(3)));

% IGRF dipole check: field at equator points north (+z)
G = ads_igrf_default();  G.N = 1;  G.g(2,2) = 0;  G.h(2,2) = 0;
B = ads_igrf([7000;0;0], G.epoch, G);
assert(B(3) > 0 && abs(B(1)) < 1e-12);

% TRIAD inversion through the estimator's sensor geometry
qt = ads_qnorm([0.2; -0.4; 0.1; 0.9]);
A = ads_q2A(qt);
r1 = ref.B_I/norm(ref.B_I);  r2 = ref.s_I;
b1 = A*r1;  b2 = A*r2;
v2 = cross(b1, b2)/norm(cross(b1, b2));  w2 = cross(r1, r2)/norm(cross(r1, r2));
At = [b1, v2, cross(b1, v2)]*[r1, w2, cross(r1, w2)].';
assert(norm(At - A) < 1e-12);
disp('ADS library tests passed');
