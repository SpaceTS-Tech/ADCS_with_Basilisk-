function P = satellite_params()

% ==========================================
% INITIAL INERTIA
% ==========================================
Lxx = 202.6910;
Lyx = 0.6765;
Lzx = 0.2248;
Lxy = 0.6765;
Lyy = 115.7325;
Lzy = -16.5856;
Lxz = 0.2248;
Lyz = -16.5856;
Lzz = 161.4387;

P.I0 = ([Lxx Lxy Lxz; ...
            Lyx Lyy Lyz; ...
            Lzx Lzy Lzz]);

% ==========================================
% INERTIA UNCERTAINTY
% ==========================================

P.uncertainty.Ixy = 0;
P.uncertainty.Ixz = 0;
P.uncertainty.Iyz = 0;

% ==========================================
% MASS
% ==========================================

P.mass_dry = 386;      % kg

P.fuel0 = 114;         % kg

P.mass0 = P.mass_dry + P.fuel0;

P.g0 = 9.81;
P.Isp = 290;

% ==========================================
% ATTITUDE CONTROL THRUSTERS
% ==========================================

P.acs.F = 1;          % N

P.acs.arm = 0.5;      % m

P.acs.tau_max = P.acs.F * P.acs.arm;

P.acs.MIB = 0.059;    % N s

P.acs.tmin = P.acs.MIB / P.acs.F;

% ==========================================
% CONTROLLER GAINS
% ==========================================

P.control.Kp = 5;

P.control.Kd = 40;

% ==========================================
% SCHMITT TRIGGER
% ==========================================

P.acs.on_threshold  = 0.3;
P.acs.off_threshold = 0.15;

% SIMULATION MODE
% ==========================================

% 'frame'  -> full-frame ON/OFF
% 'pwm'    -> intra-frame pulse width

P.acs.mode = 'pwm';

% ==========================================
% TIMING
% ==========================================

P.sim.Ts = 0.25;

% ==========================================
% THRUSTER HARDWARE
% ==========================================

P.acs.F = 1;

P.acs.arm = 0.5;

P.acs.tau_max = P.acs.F * P.acs.arm;

P.acs.MIB = 0.059;

P.acs.tmin = P.acs.MIB / P.acs.F;

% ==========================================
% PWPF MODULATOR for thrusters
% ==========================================

P.pwpf.Km = 0.5;

P.pwpf.Tm = 0.1;

% ==========================================
% REACTION WHEELS
% ==========================================

% P.rw.torque_max = 0.310;      % Nm (??)
% P.rw.h_max = 7.83;             % Nms
% P.rw.h_nom = 6;
% P.rw.Kp = 5;
% P.rw.Kd = P.rw.Kp*1;
% P.rw.Kh = 0.1;

P.rw.torque_max = 0.09;      % Nm (??)
P.rw.h_max = 1.47;             % Nms
P.rw.h_nom = 1;
P.rw.Kp = 5;
P.rw.Kd = P.rw.Kp*5;
P.rw.Kh = 0.2;

P.rw.Kp = P.I0/P.I0(1,1).*eye(3)*0.5;
P.rw.Kd = 2*sqrt((P.I0.*eye(3))*P.rw.Kp);

P.rw.Kd = 100;

P.rw.A = eye(3);            % orinetation of reaction wheels

% P.rw.Kp = P.I0/P.I0(1,1)*1;
% P.rw.Kd = 2*sqrt((P.I0)*P.rw.Kp);

% % ==========================================
% % REACTION WHEELS DAMPING
% % ==========================================
% P.rw.kw = 100*eye(3);
% P.rw.kh = 0.5*eye(3);


% Guidance
% Circle plane n'*z + d = 0 for the body z-axis on the unit sphere.
% Leave empty to build the great-circle plane from q0 and q_ref.
P.M2_V = [];
P.M3_V = [];
P.guidance.circle_speed = 0.0096*2;
P.guidance.plane_gain = 1;
P.guidance.H_I = [];

end
