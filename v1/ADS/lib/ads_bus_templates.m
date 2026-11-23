function [ref, meas, cond, est] = ads_bus_templates()
%ADS_BUS_TEMPLATES Default values of the ADS internal buses.
% These structs define the Simulink bus objects (ads_define_buses) and are
% used by the MATLAB Function blocks to initialise their outputs.
%#codegen
z3 = zeros(3,1);

ref.valid = false;
ref.s_I = z3;            % sun unit vector, inertial
ref.d_AU = 1;            % sun distance [AU]
ref.eclipse = false;
ref.B_I = z3;            % geomagnetic field, inertial [T]
ref.r_I = z3;            % position [km]
ref.A_OI = eye(3);       % inertial -> orbit (LVLH)
ref.w_o = 0;             % orbit rate [rad/s]

meas.w_B = z3;           % gyro rate in body (not bias compensated)
meas.gyr_valid = false;
meas.B_B = z3;           % calibrated field in body [T]
meas.b_mag = z3;         % unit field vector
meas.mag_valid = false;
meas.sig_mag = 0;
meas.s_css = z3;
meas.css_valid = false;
meas.s_fss = zeros(3,2);
meas.fss_valid = false(2,1);
meas.s_sun = z3;         % best sun vector (FSS > CSS)
meas.sun_valid = false;
meas.sig_sun = 0;
meas.q_st = [0;0;0;1];   % ST attitude q_B/I (latency compensated)
meas.st_valid = false;
meas.health = uint32(0);

cond.req_rank = 0;
cond.gyr_ok = false;        cond.gyr_lost = false;
cond.mag_ok = false;        cond.mag_lost = false;
cond.rate_src_lost = false;
cond.eph_ok = false;        cond.eph_lost = false;
cond.sun_ok = false;
cond.fss_ok = false;        cond.fss_lost = false;
cond.st_ok = false;         cond.st_lost = false;
cond.triad_ok = false;      cond.gl_init_ok = false;
cond.gyr_recovered = false; cond.gyroless_en = false;
cond.w_lt_C_in = false;     cond.w_gt_C_out = false;
cond.w_lt_FS_in = false;    cond.w_gt_FS_out = false;
cond.w_lt_ST_in = false;    cond.w_gt_ST_out = false;
cond.w_lt_GL_in = false;    cond.w_gt_GL_out = false;
cond.coarse_conv = false;   cond.sig_lt_FS_in = false;
cond.coarse_fault = false;  cond.fine_sun_fault = false;
cond.stellar_fault = false; cond.gl_fault = false;
cond.gl_meas_lost = false;  cond.st_suspect = false;
cond.num_fault = false;

est.mode = uint8(0);        % mode in which the estimator ran
est.q = [0;0;0;1];          % q_B/I
est.w = z3;                 % body rate estimate [rad/s]
est.bias = z3;
est.beta_NV = z3;
est.sig_att = z3;           % 3-sigma per axis [rad]
est.sig_max = 0;            % 3*sqrt(lambda_max(P_att)) [rad]
est.sig_w = 0;              % 3-sigma rate (GYROLESS) [rad/s]
est.w_norm = 0;             % rate magnitude used by the mode manager
est.w_perp = z3;
est.B_hat = z3;
est.Bdot = z3;
est.att_init = false;
est.rate_ok = false;
est.div = false;
est.was_conv = false;
est.no_meas_t = 0;
est.st_suspect = false;
est.num_fault = false;
est.detumbled = false;
est.nis_mean = 0;
est.health = uint32(0);
end
