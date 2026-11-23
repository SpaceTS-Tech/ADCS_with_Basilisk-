function [ds, truth] = ads_demo_scenario(P, T)
%ADS_DEMO_SCENARIO Truth model and sensor signals for the ADS_v1 root inports.
% Circular 500 km noon-midnight orbit (eclipse from about t = 1770 s), initial
% tumble decaying to a slow residual rate, body +z kept near the Sun.
% Injected events:
%   ST outage       900-1200 s (sunlit)  -> FINE_STELLAR -> FINE_SUN   (T8)
%   Gyro failure   2200-2700 s (eclipse) -> GYROLESS (T11), recovery (T14)
%   ST outage      3000-3300 s (eclipse) -> FINE_STELLAR -> COARSE     (T9)
if nargin < 2
    T = 3600;
end
rng(7);
dt = P.dt;
t = (0:dt:T).';
N = numel(t);
d2r = pi/180;
mu = P.mu;
jd0 = juliandate(datetime(2026, 10, 23, 0, 0, 0));

% Orbit: node along the Sun's right ascension -> noon-midnight plane
a = 6878;  inc = 97.4*d2r;  n = sqrt(mu/a^3);
ref0 = ads_reference_models(jd0, [a;0;0], [0;7.6;0], true, P);
RAAN = atan2(ref0.s_I(2), ref0.s_I(1));
u0 = 0;

J = P.gl.J;
w0 = [0.1; -0.1; 2.7]*d2r;          % initial tumble [rad/s]
wres = [0.005; -0.005; 0.03]*d2r;   % residual rate after "detumbling"
tauDecay = 120;
beta = [0.05; -0.03; 0.04]*d2r;     % true gyro bias
Bmodel_err = 150e-9*[1; -1; 0.5]/norm([1; -1; 0.5]);

% Initial attitude: body +z toward the Sun
s = ref0.s_I;
x = cross([0;0;1], s);  x = x/norm(x);
y = cross(s, x);
q = ads_A2q([x.'; y.'; s.']);

U = struct();
U.jd = jd0 + t/86400;
U.r_I = zeros(N,3);  U.v_I = zeros(N,3);  U.eph_valid = true(N,1);
U.gyro_w = zeros(N,3);  U.gyro_ok = true(N,1);
U.mag_B = zeros(N,3);  U.mag_ok = true(N,1);  U.mtq_active = false(N,1);
U.css_I = zeros(N,6);  U.css_ok = true(N,1);
U.fss_ab = zeros(N,4);  U.fss_present = false(N,2);
U.st_q = zeros(N,4);  U.st_tracking = false(N,1);  U.st_nstars = 12*ones(N,1);  U.st_age = zeros(N,1);
U.h_w = zeros(N,3);  U.tau_c = zeros(N,3);
U.mode_req = uint8(4)*ones(N,1,'uint8');
U.autonomy = true(N,1);
U.sensor_inhibit = zeros(N,1,'uint8');
U.cmd_reset = false(N,1);
U.cmd_clear_fault = false(N,1);
truth.t = t;  truth.q = zeros(N,4);  truth.w = zeros(N,3);  truth.eclipse = false(N,1);

for k = 1:N
    tk = t(k);
    u = u0 + n*tk;
    cO = cos(RAAN);  sO = sin(RAAN);  ci = cos(inc);  si = sin(inc);
    r = a*[cos(u)*cO - sin(u)*ci*sO; cos(u)*sO + sin(u)*ci*cO; sin(u)*si];
    v = sqrt(mu/a)*[-sin(u)*cO - cos(u)*ci*sO; -sin(u)*sO + cos(u)*ci*cO; cos(u)*si];
    ref = ads_reference_models(U.jd(k), r, v, true, P);
    w = w0*exp(-tk/tauDecay) + wres;
    wdot = -w0/tauDecay*exp(-tk/tauDecay);
    A = ads_q2A(q);
    nb = A*r/norm(r);
    tgg = 3*mu/norm(r)^3*cross(nb, J*nb);

    U.r_I(k,:) = r.';  U.v_I(k,:) = v.';
    U.tau_c(k,:) = (J*wdot + cross(w, J*w) - tgg).';
    truth.q(k,:) = q.';  truth.w(k,:) = w.';  truth.eclipse(k) = ref.eclipse;

    % Gyro
    U.gyro_w(k,:) = (w + beta + P.gyr.sig_v/sqrt(dt)*randn(3,1)).';
    U.gyro_ok(k) = ~(tk >= 2200 && tk < 2700);
    % Magnetometer
    U.mag_B(k,:) = (A*(ref.B_I + Bmodel_err) + P.mag.sig_sens*randn(3,1)).';
    % Sun sensors
    sB = A*ref.s_I;
    lit = ~ref.eclipse;
    alb = 0.03*max(0, P.css.n*(-nb))*lit;
    U.css_I(k,:) = (lit*max(0, P.css.n*sB)/ref.d_AU^2 + alb + 0.003*randn(6,1)).';
    for j = 1:2
        sF = P.fss.A_FB(:,:,j)*sB;
        if lit && sF(3) > cos(P.fss.fov)
            U.fss_present(k,j) = true;
            U.fss_ab(k,2*j-1) = atan(sF(1)/sF(3)) + P.fss.sig*randn;
            U.fss_ab(k,2*j) = atan(sF(2)/sF(3)) + P.fss.sig*randn;
        end
    end
    % Star tracker
    q_SI = ads_qmult(P.st.q_SB, q);
    dth = [P.st.sig_cb; P.st.sig_cb; P.st.sig_roll].*randn(3,1);
    U.st_q(k,:) = ads_qmult(ads_dq_rotvec(dth), q_SI).';
    zS = P.st.A_SB(3,:).';
    outage = (tk >= 900 && tk < 1200) || (tk >= 3000 && tk < 3300);
    U.st_tracking(k) = norm(w) < P.st.w_max && acos(zS.'*sB) > P.st.excl_sun && ~outage;

    q = ads_qprop(q, w, dt);
end

names = {'jd','r_I','v_I','eph_valid','gyro_w','gyro_ok','mag_B','mag_ok','mtq_active', ...
    'css_I','css_ok','fss_ab','fss_present','st_q','st_tracking','st_nstars','st_age', ...
    'h_w','tau_c','mode_req','autonomy','sensor_inhibit','cmd_reset','cmd_clear_fault'};
ds = Simulink.SimulationData.Dataset;
for k = 1:numel(names)
    ts = timeseries(U.(names{k}), t);
    ts.Name = names{k};
    ds = ds.addElement(ts, names{k});
end
end
