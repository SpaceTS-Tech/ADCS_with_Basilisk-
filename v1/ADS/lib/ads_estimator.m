function est = ads_estimator(mode, meas, ref, h_w, tau_c, cmd_reset, P)
%ADS_ESTIMATOR Runs the estimator of the active ADS mode (Sections 5-11).
%   mode 0 ADS_IDLE          no attitude estimation
%   mode 1 ADS_RATE          gyro rate or MAG-rate Kalman filter
%   mode 2 ADS_COARSE        USQUE, MAG + sun vector + gyro
%   mode 3 ADS_FINE_SUN      MEKF, FSS + MAG + gyro (CSS backup)
%   mode 4 ADS_FINE_STELLAR  MEKF, ST + gyro (FSS/MAG aiding during ST outages)
%   mode 5 ADS_GYROLESS      USQUE with Euler dynamics
% The MAG-rate filter and the gyro rate path always run (Section 7).
% On a mode change the new filter is initialised with the hand-over rules of Table 8.
% Each cycle: propagate with the previous gyro sample, update with the current
% measurements, reset; the output is the a-posteriori estimate at the current time.
%#codegen
persistent S
if isempty(S) || cmd_reset
    S = initState(P);
end
[~, ~, ~, est] = ads_bus_templates();
dt = P.dt;
numFault = false;

%% Rate path (always active)
S = magRateKF(S, meas, P);
B = S.mz(1:3);
Bd = S.mz(4:6);
w_perp = zeros(3,1);
if S.minit && B.'*B > 1e-14
    w_perp = cross(Bd, B)/(B.'*B);                       % Eq. (7.5)
end
mkf_valid = S.minit && S.mt >= P.mkf.T_valid;
if mkf_valid                                             % 600 s window maximum
    S.blk(S.bi) = max(S.blk(S.bi), norm(w_perp));
    S.bt = S.bt + dt;
    if S.bt >= P.mkf.T_blk
        S.bi = mod(S.bi, P.mkf.N_blk) + 1;
        S.blk(S.bi) = 0;
        S.bt = 0;
    end
end
if meas.gyr_valid
    w_rate = meas.w_B - S.beta_NV;                       % Eq. (7.1)
    S.wlp = S.wlp + dt/(P.tau_LP + dt)*(w_rate - S.wlp);
    w_rate_norm = norm(S.wlp);
    rate_ok = true;
    S.w_last = meas.w_B;
    S.w_last_ok = true;
elseif mkf_valid
    w_rate = w_perp;
    w_rate_norm = max(S.blk);
    rate_ok = true;
else
    w_rate = zeros(3,1);
    w_rate_norm = 0;
    rate_ok = false;
end

%% Hand-over on mode change (Table 8)
if mode ~= S.mode
    S = handover(S, mode, meas, ref, w_perp, P);
    S.mode = mode;
end

%% Active estimator
switch mode
    case 2
        S = runCoarse(S, meas, ref, P);
    case 3
        S = runFineSun(S, meas, ref, P);
    case 4
        S = runStellar(S, meas, ref, P);
    case 5
        S = runGyroless(S, meas, ref, h_w, tau_c, P);
    otherwise
        S.att_init = false;
        S.has_prev = false;
end

%% Numerical safeguards (Section 5.5)
if any(~isfinite(S.q)) || any(~isfinite(S.x)) || any(~isfinite(S.Pm(:)))
    numFault = true;
    keepNV = S.beta_NV;
    S = initState(P);
    S.beta_NV = keepNV;
    S.mode = mode;
end
if mode >= uint8(2) && mode <= uint8(4)
    nb = norm(S.x(4:6));
    if nb > P.gyr.beta_max
        S.x(4:6) = S.x(4:6)*P.gyr.beta_max/nb;
    end
end

%% Outputs
est.mode = mode;
est.q = S.q;
if mode >= uint8(2) && mode <= uint8(4) && S.att_init
    est.w = S.w_last - S.x(4:6);
    est.bias = S.x(4:6);
elseif mode == uint8(5) && S.att_init
    est.w = S.x(4:6);
    est.bias = S.beta_NV;
    est.sig_w = 3*sqrt(max(ads_sym3_maxeig(S.Pm(4:6,4:6)), 0));
else
    est.w = w_rate;
    est.bias = S.beta_NV;
end
est.beta_NV = S.beta_NV;
if S.att_init
    est.sig_att = 3*sqrt(max(diag(S.Pm(1:3,1:3)), 0));
    est.sig_max = 3*sqrt(max(ads_sym3_maxeig(S.Pm(1:3,1:3)), 0));
    est.w_norm = norm(est.w);
    est.rate_ok = rate_ok || mode == uint8(5);
else
    est.sig_att = pi*ones(3,1);
    est.sig_max = pi;
    est.w_norm = w_rate_norm;
    est.rate_ok = rate_ok;
end
est.w_perp = w_perp;
est.B_hat = B;
est.Bdot = Bd;
est.att_init = S.att_init;
est.div = S.div;
est.was_conv = S.was_conv;
est.no_meas_t = S.no_meas_t;
est.st_suspect = S.st_suspect;
est.num_fault = numFault;
if est.rate_ok && est.w_norm < P.w_det
    S.det_t = S.det_t + dt;
else
    S.det_t = 0;
end
est.detumbled = S.det_t >= P.T_det;
if S.nis_n > 0
    est.nis_mean = sum(S.nis_e)/max(sum(S.nis_m), 1);
end
h = uint32(0);
h = bitor(h, uint32(S.div));
h = bitor(h, bitshift(uint32(numFault), 1));
h = bitor(h, bitshift(uint32(S.st_reinit), 2));
h = bitor(h, bitshift(uint32(S.nv_init), 3));
h = bitor(h, bitshift(uint32(mkf_valid), 4));
est.health = h;
end

%% ------------------------------------------------------------------ state
function S = initState(P)
S.mode = uint8(0);
S.q = [0;0;0;1];
S.x = zeros(6,1);
S.Pm = eye(6);
S.att_init = false;
S.has_prev = false;
S.w_last = zeros(3,1);
S.w_last_ok = false;
S.w_prev = zeros(3,1);
S.tau_prev = zeros(3,1);
S.hw_prev = zeros(3,1);
S.beta_NV = zeros(3,1);
S.nv_t = 0;
S.nv_init = false;
S.mz = zeros(6,1);
S.mP = eye(6);
S.minit = false;
S.mt = 0;
S.blk = zeros(P.mkf.N_blk, 1);
S.bi = 1;
S.bt = 0;
S.wlp = zeros(3,1);
S.det_t = 0;
S.nis_e = zeros(P.W, 1);
S.nis_m = zeros(P.W, 1);
S.nis_n = 0;
S.nis_i = 1;
S.rej = 0;
S.div = false;
S.was_conv = false;
S.no_meas_t = 0;
S.st_chk = 0;
S.st_suspect = false;
S.st_reinit = false;
S.sun_rej = 0;
end

function S = resetHealth(S)
S.nis_e(:) = 0;
S.nis_m(:) = 0;
S.nis_n = 0;
S.nis_i = 1;
S.rej = 0;
S.div = false;
S.was_conv = false;
S.no_meas_t = 0;
S.st_chk = 0;
S.st_suspect = false;
S.st_reinit = false;
S.sun_rej = 0;
end

function S = updHealth(S, e, dof, acc, used, P)
% Windowed NIS and consecutive-rejection divergence tests (Section 5.4).
if ~used
    return
end
if acc
    S.nis_e(S.nis_i) = e;
    S.nis_m(S.nis_i) = dof;
    S.nis_i = mod(S.nis_i, P.W) + 1;
    S.nis_n = min(S.nis_n + 1, P.W);
    S.rej = 0;
else
    S.rej = S.rej + 1;
end
if S.rej >= P.N_rej
    S.div = true;
end
if S.nis_n >= P.W
    M = sum(S.nis_m);
    g = M*(1 - 2/(9*M) + P.z_p*sqrt(2/(9*M)))^3;      % Eq. (5.33)
    if sum(S.nis_e) > g
        S.div = true;
    end
end
end

%% ------------------------------------------------------------------ rate path
function S = magRateKF(S, meas, P)
% Linear Kalman filter on [B; Bdot] (Section 7.3, Eqs. (7.2)-(7.4)).
dt = P.dt;
I3 = eye(3);
R = P.mag.sig_sens^2*I3;
if ~S.minit
    if meas.mag_valid
        nb = norm(meas.B_B);
        S.mz = [meas.B_B; zeros(3,1)];
        S.mP = blkdiag(R, (P.mkf.w_max*nb)^2*I3);
        S.minit = true;
        S.mt = 0;
    end
    return
end
F = [I3, dt*I3; zeros(3), I3];
Q = P.mkf.q_B*[dt^3/3*I3, dt^2/2*I3; dt^2/2*I3, dt*I3];
S.mz = F*S.mz;
S.mP = F*S.mP*F.' + Q;
S.mt = S.mt + dt;
if meas.mag_valid
    H = [I3, zeros(3)];
    Sk = H*S.mP*H.' + R;
    K = (S.mP*H.')/Sk;
    S.mz = S.mz + K*(meas.B_B - H*S.mz);
    IKH = eye(6) - K*H;
    S.mP = IKH*S.mP*IKH.' + K*R*K.';
    S.mP = 0.5*(S.mP + S.mP.');
end
end

%% ------------------------------------------------------------------ initialisation
function [ok, q, Pa] = triadInit(meas, ref, P)
% TRIAD with the more accurate vector as anchor, Eqs. (5.1)-(5.4).
q = [0;0;0;1];
Pa = (pi/2)^2*eye(3);
ok = meas.sun_valid && meas.mag_valid && ref.valid;
if ~ok
    return
end
b1 = meas.b_mag;  r1 = ref.B_I/norm(ref.B_I);  s1 = meas.sig_mag;
b2 = meas.s_sun;  r2 = ref.s_I;                s2 = meas.sig_sun;
if s2 < s1
    t = b1; b1 = b2; b2 = t;
    t = r1; r1 = r2; r2 = t;
    ts = s1; s1 = s2; s2 = ts;
end
cb = cross(b1, b2);
cr = cross(r1, r2);
if norm(cb) < 1e-6 || norm(cr) < 1e-6
    ok = false;
    return
end
v2 = cb/norm(cb);  w2 = cr/norm(cr);
A = [b1, v2, cross(b1, v2)]*[r1, w2, cross(r1, w2)].';
q = ads_A2q(A);
I3 = eye(3);
Pa = inv((I3 - b1*b1.')/s1^2 + (I3 - b2*b2.')/s2^2);
end

function S = handover(S, mode, meas, ref, w_perp, P)
prev = S.mode;
had = S.att_init && prev >= uint8(2);
S = resetHealth(S);
S.has_prev = false;
I3 = eye(3);
sb2 = P.gyr.sig_b0^2;
switch mode
    case 2
        if had && (prev == uint8(3) || prev == uint8(4))
            T = diag([sqrt(P.k_inf)*ones(3,1); ones(3,1)]);
            S.Pm = T*S.Pm*T.';
        elseif had && prev == uint8(5)
            S.Pm = blkdiag(S.Pm(1:3,1:3), sb2*I3);
            S.x = [zeros(3,1); S.beta_NV];
        else
            [~, q, Pa] = triadInit(meas, ref, P);
            S.q = q;
            S.Pm = blkdiag(P.kP0*Pa, sb2*I3);
            S.x = [zeros(3,1); S.beta_NV];
        end
        S.att_init = true;
    case 3
        if had && prev == uint8(5)
            S.Pm = blkdiag(S.Pm(1:3,1:3), sb2*I3);
            S.x = [zeros(3,1); S.beta_NV];
        elseif ~had
            [~, q, Pa] = triadInit(meas, ref, P);
            S.q = q;
            S.Pm = blkdiag(P.kP0*Pa, sb2*I3);
            S.x = [zeros(3,1); S.beta_NV];
        end
        S.att_init = true;
    case 4
        sig = 3*sqrt(max(ads_sym3_maxeig(S.Pm(1:3,1:3)), 0));
        if had && prev ~= uint8(5) && sig < P.th_ST_reinit
            S.st_chk = P.N_ST;                       % entry consistency check (Section 10.4)
        elseif meas.st_valid
            if had && (prev == uint8(2) || prev == uint8(3))
                Pb = S.Pm(4:6,4:6);
            else
                S.x(4:6) = S.beta_NV;
                Pb = sb2*I3;
            end
            S.q = meas.q_st;
            S.Pm = blkdiag(P.kP0*P.st.R_B, Pb);
        elseif had && prev == uint8(5)
            S.Pm = blkdiag(S.Pm(1:3,1:3), sb2*I3);
            S.x = [zeros(3,1); S.beta_NV];
        elseif ~had
            [~, q, Pa] = triadInit(meas, ref, P);
            S.q = q;
            S.Pm = blkdiag(P.kP0*Pa, sb2*I3);
            S.x = [zeros(3,1); S.beta_NV];
        end
        S.att_init = true;
    case 5
        if had
            Pa = S.Pm(1:3,1:3);
            if S.w_last_ok
                w0 = S.w_last - S.x(4:6);
                Pw = S.Pm(4:6,4:6) + P.gl.sig_w0^2*I3;
            else
                w0 = w_perp;
                Pw = P.gl.sig_w0_mag^2*I3;
            end
        else
            if meas.st_valid
                S.q = meas.q_st;
                Pa = P.kP0*P.st.R_B;
            else
                [~, q, Pt] = triadInit(meas, ref, P);
                S.q = q;
                Pa = P.kP0*Pt;
            end
            w0 = w_perp;
            Pw = P.gl.sig_w0_mag^2*I3;
        end
        S.x = [zeros(3,1); w0];
        S.Pm = blkdiag(Pa, Pw);
        S.att_init = true;
    otherwise
        S.att_init = false;
end
S.x(1:3) = 0;
end

%% ------------------------------------------------------------------ mode filters
function S = runCoarse(S, meas, ref, P)
% ADS_COARSE: USQUE with MAG and sun vector (Section 8).
if S.has_prev
    [S.q, S.x, S.Pm, qs, chi] = ads_usque_prop_gyro(S.q, S.x, S.Pm, S.w_prev, P);
else
    [qs, chi] = ads_usque_sigma(S.q, S.x, (6 + P.lambda)*S.Pm, P);
end
[y, rv, R, kind] = vectorBlocks(meas, ref, P);
[S.x, S.Pm, e, acc] = ads_usque_update(S.x, S.Pm, qs, chi, y, rv, R, kind, P);
[S.q, S.x] = ads_usque_reset(qs(:,1), S.x, P);
S = updHealth(S, e(1), 2, acc(1), kind(1) > 0, P);
% Eclipse exit: repeated sun rejection with a consistent sensor pair -> in-mode TRIAD reset (Section 8.4)
if kind(2) > 0 && ~acc(2)
    S.sun_rej = S.sun_rej + 1;
else
    S.sun_rej = 0;
end
if S.sun_rej >= P.N_rej
    [ok, q, Pa] = triadInit(meas, ref, P);
    if ok
        S.q = q;
        S.Pm(1:3,1:3) = P.kP0*Pa;
        S.Pm(1:3,4:6) = 0;
        S.Pm(4:6,1:3) = 0;
    end
    S.sun_rej = 0;
end
if 3*sqrt(max(ads_sym3_maxeig(S.Pm(1:3,1:3)), 0)) < P.th_C_conv
    S.was_conv = true;
end
S.w_prev = S.w_last;
S.has_prev = true;
end

function S = runFineSun(S, meas, ref, P)
% ADS_FINE_SUN: MEKF, FSS heads then MAG, CSS if no FSS (Section 9).
b = S.x(4:6);
dx = zeros(6,1);
if S.has_prev
    [S.q, S.Pm] = ads_mekf_propagate(S.q, b, S.Pm, S.w_prev, P);
end
e_p = 0;  acc_p = false;  used_p = false;  nf = 0;
if ref.valid
    for j = 1:2
        if meas.fss_valid(j)
            sig = P.fss.sig;
            if nf > 0
                sig = sig*sqrt(2);
            end
            [dx, S.Pm, e, a] = ads_mekf_update_vec(dx, S.Pm, S.q, meas.s_fss(:,j), ref.s_I, sig, P);
            if nf == 0
                e_p = e;  acc_p = a;  used_p = true;
            end
            nf = nf + 1;
        end
    end
    if nf == 0 && meas.css_valid
        [dx, S.Pm, e_p, acc_p] = ads_mekf_update_vec(dx, S.Pm, S.q, meas.s_css, ref.s_I, P.css.sig, P);
        used_p = true;
    end
    if meas.mag_valid
        [dx, S.Pm] = ads_mekf_update_vec(dx, S.Pm, S.q, meas.b_mag, ref.B_I/norm(ref.B_I), meas.sig_mag, P);
    end
end
[S.q, b] = ads_mekf_reset(S.q, b, dx);
S.x = [zeros(3,1); b];
S = updHealth(S, e_p, 2, acc_p, used_p, P);
if 3*sqrt(max(ads_sym3_maxeig(S.Pm(1:3,1:3)), 0)) < P.th_FS_in
    S.was_conv = true;
end
S.w_prev = S.w_last;
S.has_prev = true;
end

function S = runStellar(S, meas, ref, P)
% ADS_FINE_STELLAR: MEKF with ST, optional FSS/MAG aiding during ST outages (Section 10).
b = S.x(4:6);
dx = zeros(6,1);
if S.has_prev
    [S.q, S.Pm] = ads_mekf_propagate(S.q, b, S.Pm, S.w_prev, P);
end
if meas.st_valid
    [dx1, Pm1, e, a] = ads_mekf_update_st(dx, S.Pm, S.q, meas.q_st, P.st.R_B, P);
    reinit = false;
    if S.st_chk > 0
        % Entry consistency check (Section 10.4): the ST has already been valid for
        % N_ST samples (cond.st_ok), so a prior that disagrees with it - by angle or
        % by failing the NIS gate - is replaced by the ST attitude.
        S.st_chk = S.st_chk - 1;
        dq = ads_qmult(meas.q_st, ads_qinv(S.q));
        ang = 2*acos(min(abs(dq(4)), 1));
        thr = max(3*3*sqrt(max(ads_sym3_maxeig(S.Pm(1:3,1:3)), 0)), P.th_ST_reinit);
        reinit = ~a || ang > thr;
    end
    if reinit
        S.q = meas.q_st;
        S.Pm(1:3,1:3) = P.kP0*P.st.R_B;
        S.Pm(1:3,4:6) = 0;
        S.Pm(4:6,1:3) = 0;
        S.st_reinit = true;
        S.st_chk = 0;
    else
        dx = dx1;
        S.Pm = Pm1;
        S = updHealth(S, e, 3, a, true, P);
    end
elseif P.st_aiding && ref.valid
    for j = 1:2
        if meas.fss_valid(j)
            [dx, S.Pm] = ads_mekf_update_vec(dx, S.Pm, S.q, meas.s_fss(:,j), ref.s_I, P.fss.sig, P);
        end
    end
    if meas.mag_valid
        [dx, S.Pm] = ads_mekf_update_vec(dx, S.Pm, S.q, meas.b_mag, ref.B_I/norm(ref.B_I), meas.sig_mag, P);
    end
end
[S.q, b] = ads_mekf_reset(S.q, b, dx);
S.x = [zeros(3,1); b];
% Gyro bias storage (Section 10.3)
if 3*sqrt(max(ads_sym3_maxeig(S.Pm(4:6,4:6)), 0)) < P.sig_beta_NV
    S.nv_t = S.nv_t + P.dt;
    if ~S.nv_init || S.nv_t >= P.T_NV
        S.beta_NV = b;
        S.nv_t = 0;
        S.nv_init = true;
    end
end
S.was_conv = true;
S.w_prev = S.w_last;
S.has_prev = true;
end

function S = runGyroless(S, meas, ref, h_w, tau_c, P)
% ADS_GYROLESS: USQUE with Euler dynamics, ST or vector measurements (Section 11).
rI = zeros(3,1);
if ref.valid
    rI = ref.r_I;
end
if S.has_prev
    [S.q, S.x, S.Pm, qs, chi] = ads_usque_prop_dyn(S.q, S.x, S.Pm, S.tau_prev, S.hw_prev, rI, P);
else
    [qs, chi] = ads_usque_sigma(S.q, S.x, (6 + P.lambda)*S.Pm, P);
end
if meas.st_valid
    y = zeros(9,1);  rv = zeros(9,1);  R = zeros(9);  kind = zeros(3,1);
    y(7:9) = ads_dq2grp(ads_qmult(meas.q_st, ads_qinv(qs(:,1))), P.a, P.f);   % Eq. (11.6)
    R(7:9,7:9) = P.st.R_B;
    kind(3) = 2;
    prim = 3;  dof = 3;
else
    [y, rv, R, kind] = vectorBlocks(meas, ref, P);
    prim = 1;  dof = 2;
end
[S.x, S.Pm, e, acc] = ads_usque_update(S.x, S.Pm, qs, chi, y, rv, R, kind, P);
[S.q, S.x] = ads_usque_reset(qs(:,1), S.x, P);
S = updHealth(S, e(prim), dof, acc(prim), kind(prim) > 0, P);
if any(acc)
    S.no_meas_t = 0;
else
    S.no_meas_t = S.no_meas_t + P.dt;
end
if 3*sqrt(max(ads_sym3_maxeig(S.Pm(1:3,1:3)), 0)) < P.th_GL_conv
    S.was_conv = true;
end
S.tau_prev = tau_c(:);
S.hw_prev = h_w(:);
S.has_prev = true;
end

function [y, rv, R, kind] = vectorBlocks(meas, ref, P)
% Block 1: magnetometer, block 2: best sun vector (Eq. (8.1)).
y = zeros(9,1);  rv = zeros(9,1);  R = zeros(9);  kind = zeros(3,1);
if ~ref.valid
    return
end
if meas.mag_valid
    kind(1) = 1;
    y(1:3) = meas.b_mag;
    rv(1:3) = ref.B_I/norm(ref.B_I);
    R(1:3,1:3) = meas.sig_mag^2*eye(3);
end
if meas.sun_valid
    kind(2) = 1;
    y(4:6) = meas.s_sun;
    rv(4:6) = ref.s_I;
    R(4:6,4:6) = meas.sig_sun^2*eye(3);
end
end
