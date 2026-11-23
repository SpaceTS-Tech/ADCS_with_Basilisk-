function meas = ads_sensor_processing(gyro_w, gyro_ok, mag_B, mag_ok, mtq_active, ...
    css_I, css_ok, fss_ab, fss_present, st_q, st_tracking, st_nstars, st_age, ...
    inhibit, ref, est, P)
%ADS_SENSOR_PROCESSING Calibration, frame transformation and validity checks (Sections 3, 12.1).
% inhibit bit mask (ground command ADS_SENSOR_INHIBIT): 1 GYR, 2 MAG, 4 CSS, 8 FSS, 16 ST.
% est is the previous-cycle estimator output (used for ST latency and exclusion checks).
%#codegen
persistent g_prev stuck_cnt
if isempty(g_prev)
    g_prev = zeros(3,1);
    stuck_cnt = 0;
end
[~, meas, ~, ~] = ads_bus_templates();
inh = uint8(inhibit);
inhG = bitand(inh, uint8(1)) ~= 0;
inhM = bitand(inh, uint8(2)) ~= 0;
inhC = bitand(inh, uint8(4)) ~= 0;
inhF = bitand(inh, uint8(8)) ~= 0;
inhS = bitand(inh, uint8(16)) ~= 0;
dark = ref.valid && ref.eclipse;

%% Gyroscope (Section 3.2)
w = P.gyr.A_GB.'*gyro_w(:);
if all(w == g_prev)
    stuck_cnt = stuck_cnt + 1;
else
    stuck_cnt = 0;
end
meas.w_B = w;
meas.gyr_valid = gyro_ok && ~inhG && all(isfinite(w)) && all(abs(gyro_w(:)) < P.gyr.w_sat) ...
    && norm(w - g_prev) < P.gyr.dw_max && stuck_cnt < P.gyr.N_stuck;
g_prev = w;

%% Magnetometer (Section 3.3)
B = P.mag.A_MB.'*(P.mag.D*(mag_B(:) - P.mag.b));
nB = norm(B);
v = mag_ok && ~inhM && ~mtq_active && all(isfinite(B)) && all(abs(mag_B(:)) < P.mag.B_sat) && nB > 1e-7;
if v && ref.valid
    v = abs(nB - norm(ref.B_I)) < P.mag.dB_max;
end
meas.B_B = B;
if nB > 0
    meas.b_mag = B/nB;
end
meas.mag_valid = v;
meas.sig_mag = P.mag.sig_min;
if ref.valid && norm(ref.B_I) > 0
    meas.sig_mag = max(sqrt(P.mag.sig_sens^2 + P.mag.sig_model^2)/norm(ref.B_I), P.mag.sig_min);
end

%% Coarse sun sensors, opposite-pair method (Section 3.4)
c = css_I(:)*ref.d_AU^2./P.css.I0;
c(c < P.css.c_min) = 0;
dvec = c(P.css.pair_plus) - c(P.css.pair_minus);
Mp = P.css.n(P.css.pair_plus, :);
s = (Mp.'*Mp)\(Mp.'*dvec);
ns = norm(s);
v = css_ok && ~inhC && ~dark && any(c > 0) && all(c <= 1.2) && ...
    ns > 1 - P.css.delta && ns < 1 + P.css.delta;
if ns > 0
    meas.s_css = s/ns;
end
meas.css_valid = v;

%% Fine sun sensors (Section 3.5)
for j = 1:2
    al = fss_ab(2*j-1);
    be = fss_ab(2*j);
    vS = [tan(al); tan(be); 1]/sqrt(1 + tan(al)^2 + tan(be)^2);
    sB = P.fss.A_FB(:,:,j).'*vS;
    ok = fss_present(j) && ~inhF && ~dark && abs(al) <= P.fss.fov && abs(be) <= P.fss.fov && all(isfinite(sB));
    if ok && meas.css_valid
        ok = acos(min(max(sB.'*meas.s_css, -1), 1)) < P.fss.zeta;
    end
    meas.s_fss(:, j) = sB;
    meas.fss_valid(j) = ok;
end

%% Best sun vector: FSS before CSS
if meas.fss_valid(1)
    meas.s_sun = meas.s_fss(:,1);  meas.sig_sun = P.fss.sig;  meas.sun_valid = true;
elseif meas.fss_valid(2)
    meas.s_sun = meas.s_fss(:,2);  meas.sig_sun = P.fss.sig;  meas.sun_valid = true;
elseif meas.css_valid
    meas.s_sun = meas.s_css;       meas.sig_sun = P.css.sig;  meas.sun_valid = true;
end

%% Star tracker (Section 3.6)
qn = norm(st_q(:));
q_SI = ads_qnorm(st_q(:));
q_BI = ads_qmult(ads_qinv(P.st.q_SB), q_SI);
if meas.gyr_valid
    w_c = w - est.bias;
else
    w_c = est.w;
end
age = max(st_age, 0);
q_BI = ads_qprop(q_BI, w_c, age);
if q_BI(4) < 0
    q_BI = -q_BI;
end
v = st_tracking && ~inhS && st_nstars >= P.st.N_star_min && abs(qn - 1) < 1e-3 && ...
    st_age >= 0 && st_age < P.st.age_max && norm(w_c) < P.st.w_max;
if v && est.att_init && ref.valid
    zS = P.st.A_SB(3,:).';                       % ST boresight in body
    sB = ads_q2A(est.q)*ref.s_I;
    v = acos(min(max(zS.'*sB, -1), 1)) > P.st.excl_sun;
end
meas.q_st = q_BI;
meas.st_valid = v;

%% Health bits
h = uint32(0);
h = bitor(h, uint32(meas.gyr_valid));
h = bitor(h, bitshift(uint32(meas.mag_valid), 1));
h = bitor(h, bitshift(uint32(meas.css_valid), 2));
h = bitor(h, bitshift(uint32(meas.fss_valid(1)), 3));
h = bitor(h, bitshift(uint32(meas.fss_valid(2)), 4));
h = bitor(h, bitshift(uint32(meas.st_valid), 5));
h = bitor(h, bitshift(uint32(dark), 6));
meas.health = h;
end
