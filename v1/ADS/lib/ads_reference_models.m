function ref = ads_reference_models(jd, r_I, v_I, eph_valid, P)
%ADS_REFERENCE_MODELS Sun, eclipse, geomagnetic field and LVLH frame (Sections 3.7, 2.8).
% jd: Julian date (UTC ~ UT1); r_I, v_I: J2000 position [km] / velocity [km/s].
%#codegen
[ref, ~, ~, ~] = ads_bus_templates();
r_I = r_I(:);  v_I = v_I(:);
if ~eph_valid || norm(r_I) < 1000 || ~all(isfinite(r_I)) || ~isfinite(jd)
    return
end
d2r = pi/180;
as2r = d2r/3600;
T = (jd - 2451545.0)/36525;

% Earth rotation (GMST) and IAU-76 precession
th_s = 67310.54841 + 3164400184.812866*T + 0.093104*T^2 - 6.2e-6*T^3;
th_g = mod(th_s, 86400)*2*pi/86400;
zeta = (2306.2181*T + 0.30188*T^2 + 0.017998*T^3)*as2r;
thp  = (2004.3109*T - 0.42665*T^2 - 0.041833*T^3)*as2r;
zz   = (2306.2181*T + 1.09468*T^2 + 0.018203*T^3)*as2r;
A_IMOD = R3(zeta)*R2(-thp)*R3(zz);
A_IE = A_IMOD*R3(-th_g);

% Low-precision sun (mean of date -> J2000) with parallax
lamM = (280.460 + 36000.771*T)*d2r;
M = (357.5291092 + 35999.05034*T)*d2r;
lam = lamM + (1.914666471*sin(M) + 0.019994643*sin(2*M))*d2r;
eps0 = (23.439291 - 0.0130042*T)*d2r;
rs = 1.000140612 - 0.016708617*cos(M) - 0.000139589*cos(2*M);
r_sun_I = A_IMOD*(rs*[cos(lam); cos(eps0)*sin(lam); sin(eps0)*sin(lam)])*P.AU;
d = r_sun_I - r_I;
ref.s_I = d/norm(d);
ref.d_AU = norm(d)/P.AU;

% Cylindrical eclipse with margin
u = r_sun_I/norm(r_sun_I);
ru = r_I.'*u;
ref.eclipse = (ru < 0) && (norm(r_I - ru*u) < P.Re + P.h_margin);

% Geomagnetic field
year = 2000.0 + (jd - 2451545.0)/365.25;
B_E = ads_igrf(A_IE.'*r_I, year, P.igrf);
ref.B_I = A_IE*B_E;

% LVLH
h = cross(r_I, v_I);
o3 = -r_I/norm(r_I);
o2 = -h/max(norm(h), 1e-9);
o1 = cross(o2, o3);
ref.A_OI = [o1.'; o2.'; o3.'];
ref.w_o = norm(h)/(r_I.'*r_I);
ref.r_I = r_I;
ref.valid = true;
end

function R = R2(a)
R = [cos(a) 0 -sin(a); 0 1 0; sin(a) 0 cos(a)];
end

function R = R3(a)
R = [cos(a) sin(a) 0; -sin(a) cos(a) 0; 0 0 1];
end
