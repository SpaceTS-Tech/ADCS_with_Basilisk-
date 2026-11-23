function P = ads_params()
%ADS_PARAMS Default ADS parameter set (design document ADCS-ADS-DD-001, Table 24).
% Values are defaults for a LEO satellite with example sensors; replace them
% with data-sheet and calibration values before use.
d2r = pi/180;
as2r = d2r/3600;

%% Timing
P.dt = 0.1;                         % ADS cycle [s] (10 Hz, same as ts in parameters.m)

%% Mode manager (Section 4, Table 24)
P.N_up = 10;                        % upgrade persistence [cycles]
P.N_down = 5;                       % rate-exit persistence [cycles]
P.T_retry = 60;                     % upgrade inhibit after fault-driven fallback [s]
P.N_retry = round(P.T_retry/P.dt);  % same in cycles (Stateflow counter)
P.T_rate_lost = 10;  P.T_GYR = 0.5;  P.T_MAG = 60;  P.T_FSS = 30;
P.T_ST = 120;        P.T_EPH = 300;  P.T_GL = 60;   P.T_conv = 60;
P.N_ST = 5;          P.N_GYR_rec = 100;
P.w_C_in  = 3*d2r;   P.w_C_out  = 5*d2r;
P.w_FS_in = 1*d2r;   P.w_FS_out = 1.5*d2r;
P.w_ST_in = 0.5*d2r; P.w_ST_out = 0.8*d2r;
P.w_GL_in = 2*d2r;   P.w_GL_out = 4*d2r;
P.w_det = 0.2*d2r;   P.T_det = 60;
P.th_TRIAD = 15*d2r;
P.th_C_conv = 10*d2r;  P.th_C_max = 30*d2r;
P.th_FS_in = 5*d2r;    P.th_FS_max = 10*d2r;
P.th_ST_reinit = 1*d2r;
P.th_GL_conv = 10*d2r; P.th_GL_max = 30*d2r;
P.th_fine = 0.05*d2r;  P.th_valid = 20*d2r;
P.gyroless_enable = true;
P.fine_sun_fine_ok = false;
P.st_aiding = true;                 % FSS/MAG aiding in FINE_STELLAR during ST outages

%% Gyroscope (Section 3.2, Appendix A)
P.gyr.A_GB = eye(3);                % body -> gyro frame
P.gyr.sig_v = 0.15*d2r/60;          % ARW 0.15 deg/sqrt(h) -> rad/sqrt(s)
P.gyr.sig_u = 0.5*d2r/(3600*60);    % RRW 0.5 deg/h/sqrt(h) -> rad/s^1.5
P.gyr.sig_b0 = 0.05*d2r;            % initial bias uncertainty [rad/s]
P.gyr.w_sat = 100*d2r;              % measurement range [rad/s]
P.gyr.dw_max = 5*d2r;               % max change per cycle [rad/s]
P.gyr.N_stuck = 50;                 % identical samples -> stuck
P.gyr.beta_max = 1*d2r;             % bias bound [rad/s]

%% Magnetometer (Section 3.3)
P.mag.A_MB = eye(3);                % body -> magnetometer frame
P.mag.D = eye(3);                   % soft-iron / scale correction
P.mag.b = zeros(3,1);               % hard-iron bias [T]
P.mag.sig_sens = 50e-9;  P.mag.sig_model = 200e-9;  P.mag.sig_min = 1*d2r;
P.mag.B_sat = 100e-6;    P.mag.dB_max = 3e-6;

%% Coarse sun sensors: six-cell cube, opposite pairs (Section 3.4)
P.css.n = [eye(3); -eye(3)];        % cell normals (rows): +x +y +z -x -y -z
P.css.I0 = ones(6,1);               % current at 1 AU, normal incidence
P.css.pair_plus = [1; 2; 3];
P.css.pair_minus = [4; 5; 6];
P.css.c_min = 0.15;  P.css.delta = 0.35;  P.css.sig = 5*d2r;

%% Fine sun sensors (Section 3.5): head 1 boresight +z_B, head 2 boresight +x_B
P.fss.A_FB = cat(3, eye(3), [0 0 -1; 0 1 0; 1 0 0]);   % body -> FSS_j
P.fss.fov = 60*d2r;  P.fss.sig = 0.1*d2r;  P.fss.zeta = 15*d2r;

%% Star tracker (Section 3.6): boresight -z_B
P.st.A_SB = [1 0 0; 0 -1 0; 0 0 -1];                     % body -> ST
P.st.q_SB = ads_A2q(P.st.A_SB);
P.st.sig_cb = 5*as2r;  P.st.sig_roll = 40*as2r;
P.st.R_B = P.st.A_SB.'*diag([P.st.sig_cb^2, P.st.sig_cb^2, P.st.sig_roll^2])*P.st.A_SB;
P.st.N_star_min = 4;  P.st.age_max = 0.5;  P.st.w_max = 1*d2r;  P.st.excl_sun = 30*d2r;

%% Filters (Section 5)
P.lambda = 1;  P.a = 1;  P.f = 4;          % USQUE
P.kP0 = 4;     P.k_inf = 4;
P.W = 100;     P.N_rej = 20;
P.gate = [10.828; 13.816; 16.266];         % chi2(1-1e-3), dof 1..3 (Appendix B)
% Windowed-NIS divergence bound (Eq. (5.33)). The document default z = 3.090
% (p = 1e-3) is evaluated every cycle on overlapping windows and gave false
% divergence alarms in simulation; z = 6 (bound ~1.57 x dof for W = 100 ST
% samples) only flags gross inconsistency.
P.z_p = 6.0;
P.sig_beta_NV = 0.002*d2r;  P.T_NV = 600;

%% MAG-rate Kalman filter (Section 7.3)
P.mkf.w_max = 5*d2r;
P.mkf.q_B = (P.mkf.w_max^2*40e-6)^2*P.dt;
P.mkf.T_valid = 10;                        % settling time before the estimate is used [s]
P.mkf.T_blk = 60;  P.mkf.N_blk = 10;       % window max = 10 blocks of 60 s = 600 s
P.tau_LP = 1;

%% GYROLESS (Section 11)
J = load_inertia();
P.gl.J = J;
P.gl.Jinv = inv(J);
P.gl.sig_tau = 1e-4;  P.gl.sig_theta = 1e-5;  P.gl.N_sub = 2;
P.gl.sig_w0 = 0.05*d2r;  P.gl.sig_w0_mag = 1*d2r;

%% Constants and reference models (Section 3.7)
P.Re = 6378.137;  P.mu = 398600.4418;  P.AU = 149597870.7;  P.h_margin = 50;
P.igrf = ads_igrf_default();
end

function J = load_inertia()
% Spacecraft inertia from ../satellite_params.m when it is on the path.
J = diag([202.691, 115.7325, 161.4387]);
if exist('satellite_params', 'file') == 2
    S = satellite_params();
    J = S.I0;
end
end
