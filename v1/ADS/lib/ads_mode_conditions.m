function cond = ads_mode_conditions(meas, ref, est, eph_valid, mode_req, P)
%ADS_MODE_CONDITIONS Persistence-filtered guard conditions for the mode manager (Table 6, Section 4.4).
% The Stateflow chart combines these booleans exactly as in transition table T1-T15.
% est is the previous-cycle estimator output; estimator-related flags are tagged
% with the mode in which the estimator ran, so that a flag raised by one filter
% never triggers a fallback of the next mode in a cascade.
%#codegen
persistent C
if isempty(C)
    C = struct('gyr_v', 0, 'gyr_i', 0, 'mag_v', 0, 'mag_i', 0, 'both_i', 0, ...
        'eph_v', 0, 'eph_i', 0, 'sun_v', 0, 'fss_v', 0, 'fss_i', 0, 'st_v', 0, 'st_i', 0, ...
        'tri_v', 0, 'wCin', 0, 'wCout', 0, 'wFSin', 0, 'wFSout', 0, 'wSTin', 0, 'wSTout', 0, ...
        'wGLin', 0, 'wGLout', 0, 'conv_t', 0, 'rec', 0);
end
[~, ~, cond, ~] = ads_bus_templates();
dt = P.dt;
tol = 1e-9;

%% Requested mode rank: IDLE 0, RATE 1, GYROLESS 2, COARSE 3, FINE_SUN 4, FINE_STELLAR 5
rank_tab = [0 1 3 4 5 2];
m = double(min(mode_req, uint8(5)));
cond.req_rank = rank_tab(m + 1);

%% Sensor validity with persistence
C.gyr_v = up(C.gyr_v, meas.gyr_valid);       C.gyr_i = tm(C.gyr_i, ~meas.gyr_valid, dt);
C.mag_v = up(C.mag_v, meas.mag_valid);       C.mag_i = tm(C.mag_i, ~meas.mag_valid, dt);
C.both_i = tm(C.both_i, ~meas.gyr_valid && ~meas.mag_valid, dt);
eph = eph_valid && ref.valid;
C.eph_v = up(C.eph_v, eph);                  C.eph_i = tm(C.eph_i, ~eph, dt);
C.sun_v = up(C.sun_v, meas.sun_valid);
fss = meas.fss_valid(1) || meas.fss_valid(2);
C.fss_v = up(C.fss_v, fss);                  C.fss_i = tm(C.fss_i, ~fss, dt);
C.st_v = up(C.st_v, meas.st_valid);          C.st_i = tm(C.st_i, ~meas.st_valid, dt);

cond.gyr_ok = C.gyr_v >= P.N_up;             cond.gyr_lost = C.gyr_i >= P.T_GYR - tol;
cond.mag_ok = C.mag_v >= P.N_up;             cond.mag_lost = C.mag_i >= P.T_MAG - tol;
cond.rate_src_lost = C.both_i >= P.T_rate_lost - tol;
cond.eph_ok = C.eph_v >= P.N_up;             cond.eph_lost = C.eph_i >= P.T_EPH - tol;
cond.sun_ok = C.sun_v >= P.N_up;
cond.fss_ok = C.fss_v >= P.N_up;             cond.fss_lost = C.fss_i >= P.T_FSS - tol;
cond.st_ok = C.st_v >= P.N_ST;               cond.st_lost = C.st_i >= P.T_ST - tol;

%% TRIAD geometry and attitude-independent consistency, Eq. (5.3)
tri = meas.sun_valid && meas.mag_valid && eph;
if tri
    r1 = ref.B_I/norm(ref.B_I);
    ab = acos(min(max(meas.b_mag.'*meas.s_sun, -1), 1));
    ar = acos(min(max(r1.'*ref.s_I, -1), 1));
    tri = ab >= P.th_TRIAD && abs(ab - ar) < 3*sqrt(meas.sig_mag^2 + meas.sig_sun^2);
end
C.tri_v = up(C.tri_v, tri);
cond.triad_ok = C.tri_v >= P.N_up;
cond.gl_init_ok = meas.st_valid || (meas.sun_valid && meas.mag_valid && eph);
cond.gyroless_en = P.gyroless_enable;

%% Rate thresholds with hysteresis (entry: N_up, exit: N_down)
wn = est.w_norm;
rk = est.rate_ok;
C.wCin = up(C.wCin, rk && wn < P.w_C_in);      C.wCout = up(C.wCout, rk && wn > P.w_C_out);
C.wFSin = up(C.wFSin, rk && wn < P.w_FS_in);   C.wFSout = up(C.wFSout, rk && wn > P.w_FS_out);
C.wSTin = up(C.wSTin, rk && wn < P.w_ST_in);   C.wSTout = up(C.wSTout, rk && wn > P.w_ST_out);
C.wGLin = up(C.wGLin, rk && wn < P.w_GL_in);   C.wGLout = up(C.wGLout, rk && wn > P.w_GL_out);
cond.w_lt_C_in = C.wCin >= P.N_up;     cond.w_gt_C_out = C.wCout >= P.N_down;
cond.w_lt_FS_in = C.wFSin >= P.N_up;   cond.w_gt_FS_out = C.wFSout >= P.N_down;
cond.w_lt_ST_in = C.wSTin >= P.N_up;   cond.w_gt_ST_out = C.wSTout >= P.N_down;
cond.w_lt_GL_in = C.wGLin >= P.N_up;   cond.w_gt_GL_out = C.wGLout >= P.N_down;

%% Estimator health, tagged with the mode the estimator ran in
em = est.mode;
C.conv_t = tm(C.conv_t, em == uint8(2) && est.att_init && est.sig_max < P.th_C_conv && ~est.div, dt);
cond.coarse_conv = C.conv_t >= P.T_conv - tol;
cond.sig_lt_FS_in = est.att_init && em >= uint8(2) && est.sig_max < P.th_FS_in && ~est.div;
cond.coarse_fault = em == uint8(2) && (est.div || (est.was_conv && est.sig_max > P.th_C_max));
cond.fine_sun_fault = em == uint8(3) && (est.div || est.sig_max > P.th_FS_max);
cond.stellar_fault = em == uint8(4) && est.div;
cond.gl_fault = em == uint8(5) && (est.div || (est.was_conv && est.sig_max > P.th_GL_max));
cond.gl_meas_lost = em == uint8(5) && est.no_meas_t >= P.T_GL - tol;
cond.st_suspect = em == uint8(4) && est.st_suspect;
cond.num_fault = est.num_fault;

%% Gyro recovery check (Section 11.6)
rec = meas.gyr_valid;
if rec && em == uint8(5)
    dw_chk = max(3*est.sig_w, 0.05*pi/180);
    rec = norm(meas.w_B - est.beta_NV - est.w) < dw_chk;
end
C.rec = up(C.rec, rec);
cond.gyr_recovered = C.rec >= P.N_GYR_rec;
end

function n = up(n, c)
if c
    n = n + 1;
else
    n = 0;
end
end

function t = tm(t, c, dt)
if c
    t = t + dt;
else
    t = 0;
end
end
