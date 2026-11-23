clear
clc
ts = 0.1;

t0 = 2*pi/0.0011;
P = satellite_params();

% Initial conditions
q0 = [1 0 0 0 ];
omega_0 = [0 0 0.0011];

% Reference
theta_ref = deg2rad(1);      % rotation angle

axis_ref = [1; 2; 3];        % rotation axis

axis_ref = axis_ref / norm(axis_ref);

% Quaternion [qw qx qy qz]
q_ref = [ cos(theta_ref/2);
          axis_ref*sin(theta_ref/2) ];

q0 = q_ref;
q0 = [sqrt(0.5) -sqrt(0.5) 0 0];

omega_ref = [0 0 0];

omega_thr = 0.1;
omega_zero = 0.001;
B = [1 0 0];


%% Thrusters
% Detumble mode
kd_detumble = 1;

% Slew_fast mode:
kp_slew_fast = 1;
kd_slew_fast = 1;


%% Reaction Wheels
I_RW = 1;
h_max = 1.47;
h0 = [1 1 1]*0.5;
% Point mode:
kp_point = 2;
kd_point = 20;

% Hold mode:
kp_hold = 1;
kd_hold = 1;

% Slew_slow mode:
kp_slew_slow = 1;
kd_slew_slow = 1;

% Slew_slow mode:
kp_spin = 1;
kd_spin = 0;

%% Magnetorquers
% Desaturation mode
k_desat = 1;

input = [6	;           % mission cmd
    1	;               % gps_valid
    0	;               % tle_error_high
    1	;               % ads_fine_allowed
    1	;               % ads_fault
    1	;               % reference avaiable
    0	;               % energy low
    1	;               % recovery ready
    1	;               % spin_feasible
    1	;               % spin axis availbale
    1	;               % optical target available
    1	;               % sun sensor valid
    1];                 % interst sensor tracking

%% 2. Create Dataset for 16 root-level Inports
ds = Simulink.SimulationData.Dataset;
t = [0 t0];
for i = 1:13
    if i == 1
        sig = timeseries(uint8(input(i)*ones(size(t))), t);
    else
        sig = timeseries(boolean(input(i)*ones(size(t))), t);
    end
    sig.Name = ['u' num2str(i)];   % optional but useful

    ds = ds.addElement(sig);
end

% sim('ADCS_FSM_v1.slx')