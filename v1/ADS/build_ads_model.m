function build_ads_model(overwrite)
%BUILD_ADS_MODEL Generate ADS_v1.slx: ADS mode manager (Stateflow FSM) and estimators.
% Implements design document ADCS-ADS-DD-001.
%   ADS mode IDs:  0 ADS_IDLE, 1 ADS_RATE, 2 ADS_COARSE, 3 ADS_FINE_SUN,
%                  4 ADS_FINE_STELLAR, 5 ADS_GYROLESS
%   build_ads_model          % refuses to overwrite an existing model
%   build_ads_model(true)    % regenerates ADS_v1.slx
if nargin < 1
    overwrite = false;
end
here = fileparts(mfilename('fullpath'));
evalin('base', sprintf('clear ADS_P; run(''%s'');', fullfile(here, 'ads_setup.m')));

modelName = 'ADS_v1';
modelFile = fullfile(here, [modelName '.slx']);
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
if isfile(modelFile)
    if ~overwrite
        error('%s already exists. Call build_ads_model(true) to regenerate it.', modelFile);
    end
    delete(modelFile);
end

new_system(modelName);
load_system(modelName);
setupCmd = 'run(fullfile(fileparts(get_param(''ADS_v1'', ''FileName'')), ''ads_setup.m''));';
set_param(modelName, 'Solver', 'FixedStepDiscrete', 'FixedStep', 'ADS_P.dt', 'StopTime', '3600', ...
    'PostLoadFcn', setupCmd, 'InitFcn', setupCmd, ...
    'SaveOutput', 'on', 'SaveFormat', 'Dataset', 'SignalLogging', 'on');

%% Root inputs
inputs = {
    'jd',              'double',  1;   % Julian date (UTC)
    'r_I',             'double',  3;   % J2000 position [km]
    'v_I',             'double',  3;   % J2000 velocity [km/s]
    'eph_valid',       'boolean', 1;   % EPHEM valid (EPHEM mode ~= EPHEM_IDLE)
    'gyro_w',          'double',  3;   % gyro rate, gyro frame [rad/s]
    'gyro_ok',         'boolean', 1;   % gyro communication / BIT OK
    'mag_B',           'double',  3;   % raw field, MAG frame [T]
    'mag_ok',          'boolean', 1;
    'mtq_active',      'boolean', 1;   % magnetorquers driving (MAG blanking)
    'css_I',           'double',  6;   % CSS cell outputs
    'css_ok',          'boolean', 1;
    'fss_ab',          'double',  4;   % [alpha1 beta1 alpha2 beta2] [rad]
    'fss_present',     'boolean', 2;   % sun-presence flags
    'st_q',            'double',  4;   % ST quaternion q_S/I, scalar last
    'st_tracking',     'boolean', 1;
    'st_nstars',       'double',  1;
    'st_age',          'double',  1;   % ST measurement age [s]
    'h_w',             'double',  3;   % wheel momentum, body [N m s]
    'tau_c',           'double',  3;   % applied external control torque, body [N m]
    'mode_req',        'uint8',   1;   % requested ADS mode 0..5
    'autonomy',        'boolean', 1;   % ADS_AUTONOMY
    'sensor_inhibit',  'uint8',   1;   % ADS_SENSOR_INHIBIT mask: 1 GYR 2 MAG 4 CSS 8 FSS 16 ST
    'cmd_reset',       'boolean', 1;   % ADS_RESET (T15)
    'cmd_clear_fault', 'boolean', 1};
for k = 1:size(inputs, 1)
    y = 40 + 34*(k - 1);
    blk = [modelName '/' inputs{k,1}];
    add_block('simulink/Sources/In1', blk, 'Position', [30 y 60 y+14]);
    set_param(blk, 'OutDataTypeStr', inputs{k,2}, 'PortDimensions', num2str(inputs{k,3}));
end

%% Algorithm blocks
mlfb(modelName, 'Reference_Models', [260 40 420 160], ...
    {'function ref = Reference_Models(jd, r_I, v_I, eph_valid, ADS_P)'
     '% Sun, eclipse, IGRF field and LVLH frame (Section 3.7).'
     'ref = ads_reference_models(jd, r_I, v_I, eph_valid, ADS_P);'}, ...
    {'ref', 'Bus: ADS_RefBus'}, {});

mlfb(modelName, 'Sensor_Processing', [520 140 720 760], ...
    {'function meas = Sensor_Processing(gyro_w, gyro_ok, mag_B, mag_ok, mtq_active, css_I, css_ok, fss_ab, fss_present, st_q, st_tracking, st_nstars, st_age, sensor_inhibit, ref, est_prev, ADS_P)'
     '% Calibration, frame transformation and validity checks (Sections 3, 12.1).'
     'meas = ads_sensor_processing(gyro_w, gyro_ok, mag_B, mag_ok, mtq_active, css_I, css_ok, fss_ab, fss_present, st_q, st_tracking, st_nstars, st_age, sensor_inhibit, ref, est_prev, ADS_P);'}, ...
    {'meas', 'Bus: ADS_MeasBus'}, {'ref', 'Bus: ADS_RefBus'; 'est_prev', 'Bus: ADS_EstBus'});

mlfb(modelName, 'Mode_Conditions', [840 300 1010 440], ...
    {'function cond = Mode_Conditions(meas, ref, est_prev, eph_valid, mode_req, ADS_P)'
     '% Persistence-filtered guard conditions for transitions T1-T15 (Section 4.3).'
     'cond = ads_mode_conditions(meas, ref, est_prev, eph_valid, mode_req, ADS_P);'}, ...
    {'cond', 'Bus: ADS_CondBus'}, {'meas', 'Bus: ADS_MeasBus'; 'ref', 'Bus: ADS_RefBus'; 'est_prev', 'Bus: ADS_EstBus'});

buildModeManager(modelName, [1110 300 1300 460]);

mlfb(modelName, 'ADS_Estimators', [1400 300 1580 480], ...
    {'function est = ADS_Estimators(ads_mode, meas, ref, h_w, tau_c, cmd_reset, ADS_P)'
     '% Estimator of the active mode with hand-over rules (Sections 5-11, Table 8).'
     'est = ads_estimator(ads_mode, meas, ref, h_w, tau_c, cmd_reset, ADS_P);'}, ...
    {'est', 'Bus: ADS_EstBus'}, {'meas', 'Bus: ADS_MeasBus'; 'ref', 'Bus: ADS_RefBus'});

mlfb(modelName, 'ADS_Outputs', [1700 200 1880 760], ...
    {'function [q_BI, q_BO, w_B, bias, sig_att, s_B, sun_valid, B_B, Bdot_B, ads_att_valid, ads_rate_valid, ads_fine_allowed, ads_health, detumbled] = ADS_Outputs(ads_mode, ads_fault, est, meas, ref, ADS_P)'
     '% ADS outputs and status flags (Table 9).'
     '[q_BI, q_BO, w_B, bias, sig_att, s_B, sun_valid, B_B, Bdot_B, ads_att_valid, ads_rate_valid, ads_fine_allowed, ads_health, detumbled] = ads_outputs(ads_mode, ads_fault, est, meas, ref, ADS_P);'}, ...
    {}, {'est', 'Bus: ADS_EstBus'; 'meas', 'Bus: ADS_MeasBus'; 'ref', 'Bus: ADS_RefBus'});

add_block('simulink/Discrete/Unit Delay', [modelName '/Est_Delay'], 'Position', [1420 560 1470 600], ...
    'InitialCondition', '0', 'SampleTime', '-1', 'Orientation', 'left');
add_block('simulink/Sources/Constant', [modelName '/N_retry'], 'Position', [960 520 1030 545], ...
    'Value', 'ADS_P.N_retry', 'OutDataTypeStr', 'double', 'SampleTime', '-1');

%% Wiring
c = @(s, sp, d, dp) add_line(modelName, sprintf('%s/%d', s, sp), sprintf('%s/%d', d, dp), 'autorouting', 'smart');
c('jd', 1, 'Reference_Models', 1);
c('r_I', 1, 'Reference_Models', 2);
c('v_I', 1, 'Reference_Models', 3);
c('eph_valid', 1, 'Reference_Models', 4);
spIn = {'gyro_w', 'gyro_ok', 'mag_B', 'mag_ok', 'mtq_active', 'css_I', 'css_ok', 'fss_ab', 'fss_present', ...
        'st_q', 'st_tracking', 'st_nstars', 'st_age', 'sensor_inhibit'};
for k = 1:numel(spIn)
    c(spIn{k}, 1, 'Sensor_Processing', k);
end
c('Reference_Models', 1, 'Sensor_Processing', 15);
c('Est_Delay', 1, 'Sensor_Processing', 16);

c('Sensor_Processing', 1, 'Mode_Conditions', 1);
c('Reference_Models', 1, 'Mode_Conditions', 2);
c('Est_Delay', 1, 'Mode_Conditions', 3);
c('eph_valid', 1, 'Mode_Conditions', 4);
c('mode_req', 1, 'Mode_Conditions', 5);

c('Mode_Conditions', 1, 'ADS_Mode_Manager', 1);
c('mode_req', 1, 'ADS_Mode_Manager', 2);
c('autonomy', 1, 'ADS_Mode_Manager', 3);
c('cmd_reset', 1, 'ADS_Mode_Manager', 4);
c('cmd_clear_fault', 1, 'ADS_Mode_Manager', 5);
c('N_retry', 1, 'ADS_Mode_Manager', 6);

c('ADS_Mode_Manager', 1, 'ADS_Estimators', 1);
c('Sensor_Processing', 1, 'ADS_Estimators', 2);
c('Reference_Models', 1, 'ADS_Estimators', 3);
c('h_w', 1, 'ADS_Estimators', 4);
c('tau_c', 1, 'ADS_Estimators', 5);
c('cmd_reset', 1, 'ADS_Estimators', 6);
c('ADS_Estimators', 1, 'Est_Delay', 1);

c('ADS_Mode_Manager', 1, 'ADS_Outputs', 1);
c('ADS_Mode_Manager', 2, 'ADS_Outputs', 2);
c('ADS_Estimators', 1, 'ADS_Outputs', 3);
c('Sensor_Processing', 1, 'ADS_Outputs', 4);
c('Reference_Models', 1, 'ADS_Outputs', 5);

%% Root outputs
outs = {'ads_mode', 'ADS_Mode_Manager', 1;  'ads_fault', 'ADS_Mode_Manager', 2;
        'ads_last_transition', 'ADS_Mode_Manager', 3;
        'q_BI', 'ADS_Outputs', 1;  'q_BO', 'ADS_Outputs', 2;  'w_B', 'ADS_Outputs', 3;
        'bias', 'ADS_Outputs', 4;  'sig_att', 'ADS_Outputs', 5;  's_B', 'ADS_Outputs', 6;
        'sun_valid', 'ADS_Outputs', 7;  'B_B', 'ADS_Outputs', 8;  'Bdot_B', 'ADS_Outputs', 9;
        'ads_att_valid', 'ADS_Outputs', 10;  'ads_rate_valid', 'ADS_Outputs', 11;
        'ads_fine_allowed', 'ADS_Outputs', 12;  'ads_health', 'ADS_Outputs', 13;
        'detumbled', 'ADS_Outputs', 14;  'est', 'ADS_Estimators', 1};
for k = 1:size(outs, 1)
    y = 40 + 40*(k - 1);
    blk = [modelName '/' outs{k,1}];
    add_block('simulink/Sinks/Out1', blk, 'Position', [2000 y 2030 y+14]);
    if strcmp(outs{k,1}, 'est')
        set_param(blk, 'OutDataTypeStr', 'Bus: ADS_EstBus');
    end
    c(outs{k,2}, outs{k,3}, outs{k,1}, 1);
end

add_block('built-in/Note', [modelName '/ADS Mode IDs: 0 IDLE, 1 RATE, 2 COARSE, 3 FINE_SUN, 4 FINE_STELLAR, 5 GYROLESS'], ...
    'Position', [1110 250]);

save_system(modelName, modelFile);
fprintf('Created %s\n', modelFile);
end

%% ------------------------------------------------------------------------
function mlfb(modelName, name, position, code, outTypes, inTypes)
% MATLAB Function block calling the ADS library; ADS_P is a non-tunable parameter.
blk = [modelName '/' name];
add_block('simulink/User-Defined Functions/MATLAB Function', blk, 'Position', position);
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', blk);
ch.Script = strjoin(code, newline);
p = ch.find('-isa', 'Stateflow.Data', 'Name', 'ADS_P');
p.Scope = 'Parameter';
p.Tunable = false;
for k = 1:size(outTypes, 1)
    d = ch.find('-isa', 'Stateflow.Data', 'Name', outTypes{k,1});
    d.DataType = outTypes{k,2};
end
for k = 1:size(inTypes, 1)
    d = ch.find('-isa', 'Stateflow.Data', 'Name', inTypes{k,1});
    d.DataType = inTypes{k,2};
end
end

function buildModeManager(modelName, position)
% ADS mode FSM, document Section 4 (Figure 1, Table 6).
chartPath = [modelName '/ADS_Mode_Manager'];
add_block('sflib/Chart', chartPath, 'Position', position);
rt = sfroot;
chart = rt.find('-isa', 'Stateflow.Chart', 'Path', chartPath);
chart.ActionLanguage = 'MATLAB';
chart.EnableNonTerminalStates = true;      % super-step: cascaded fallback within one cycle (Section 4.4)
chart.NonTerminalMaxCounts = 10;

addData(chart, 'cond', 'Input', 'Bus: ADS_CondBus');
addData(chart, 'mode_req', 'Input', 'uint8');
addData(chart, 'autonomy', 'Input', 'boolean');
addData(chart, 'cmd_reset', 'Input', 'boolean');
addData(chart, 'cmd_clear_fault', 'Input', 'boolean');
addData(chart, 'N_retry', 'Input', 'double');
addData(chart, 'ads_mode', 'Output', 'uint8');
addData(chart, 'ads_fault', 'Output', 'boolean');
addData(chart, 'last_transition', 'Output', 'uint8');
addData(chart, 'retry_cnt', 'Local', 'double');
addData(chart, 'clear_cnt', 'Local', 'double');

%% Chart functions: bookkeeping, transition marking and one guard per document transition
x0Grid = 1330;
fx = x0Grid;  fy = 40;  fw = 170;  fh = 46;  ncol = 4;  slot = 0;
    function place(f)
        col = mod(slot, ncol);  row = floor(slot/ncol);
        f.Position = [fx + col*(fw + 20), fy + row*(fh + 18), fw, fh];
        slot = slot + 1;
    end
emf(chart, 'bookkeeping', '', {
    '% Retry timer and ads_fault latch (Sections 4.4, 12.4); runs every cycle.'
    'retry_cnt = max(retry_cnt - 1, 0);'
    'if cmd_clear_fault'
    '    ads_fault = false;  clear_cnt = 0;'
    'elseif ads_fault && ads_mode == mode_req && retry_cnt == 0'
    '    clear_cnt = clear_cnt + 1;'
    '    if clear_cnt >= N_retry'
    '        ads_fault = false;  clear_cnt = 0;'
    '    end'
    'else'
    '    clear_cnt = 0;'
    'end'}, @place);
emf(chart, 'mark', 'id', {
    '% Record the document transition ID (16 = commanded downgrade).'
    'last_transition = uint8(id);'
    'if id == 15 && cond.num_fault'
    '    ads_fault = true;'
    'end'}, @place);
emf(chart, 'fault_fb', 'id', {
    '% Fault-driven fallback: latch ads_fault and inhibit upgrades for N_retry cycles.'
    'last_transition = uint8(id);'
    'ads_fault = true;'
    'retry_cnt = N_retry;'}, @place);
guards = {
    'up',      'Upgrades allowed: autonomy enabled and retry timer expired', 'autonomy && retry_cnt == 0';
    'T1',      'IDLE -> RATE',                    '~cmd_reset && autonomy && mode_req ~= uint8(0) && (cond.gyr_ok || cond.mag_ok)';
    'T2',      'RATE -> IDLE (fault)',            'cond.rate_src_lost';
    'T3',      'RATE -> COARSE',                  'up() && cond.req_rank >= 3 && cond.gyr_ok && cond.mag_ok && cond.eph_ok && cond.w_lt_C_in && cond.triad_ok';
    'T4f',     'COARSE -> RATE (fault)',          'cond.gyr_lost || cond.mag_lost || cond.eph_lost || cond.coarse_fault';
    'T4g',     'COARSE -> RATE (rate too high)',  'cond.w_gt_C_out';
    'T5',      'COARSE -> FINE_SUN',              'up() && cond.req_rank >= 4 && cond.coarse_conv && cond.sig_lt_FS_in && cond.fss_ok && cond.mag_ok && cond.w_lt_FS_in';
    'T6f',     'FINE_SUN -> COARSE (fault)',      'cond.gyr_lost || cond.mag_lost || cond.eph_lost || cond.fine_sun_fault';
    'T6g',     'FINE_SUN -> COARSE (no FSS / rate)', 'cond.fss_lost || cond.w_gt_FS_out';
    'T7',      'FINE_SUN -> FINE_STELLAR',        'up() && cond.req_rank >= 5 && cond.st_ok && cond.w_lt_ST_in && ~cond.st_suspect';
    'T8',      'FINE_STELLAR -> FINE_SUN',        '(cond.st_lost || cond.w_gt_ST_out || cond.st_suspect) && cond.fss_ok && cond.mag_ok && cond.eph_ok';
    'T9f',     'FINE_STELLAR -> COARSE (fault)',  'cond.gyr_lost || cond.stellar_fault';
    'T9g',     'FINE_STELLAR -> COARSE (ST out)', 'cond.st_lost || cond.w_gt_ST_out || cond.st_suspect';
    'T10',     'RATE/COARSE -> FINE_STELLAR',     'up() && cond.req_rank >= 5 && cond.gyr_ok && cond.st_ok && cond.w_lt_ST_in';
    'T11',     'COARSE/FINE_SUN/FINE_STELLAR -> GYROLESS', 'cond.gyr_lost && cond.gyroless_en && cond.gl_init_ok';
    'T12',     'RATE -> GYROLESS',                'up() && cond.req_rank >= 2 && cond.gyr_lost && cond.gyroless_en && cond.mag_ok && cond.w_lt_GL_in && cond.gl_init_ok';
    'T13f',    'GYROLESS -> RATE (fault)',        'cond.gl_meas_lost || cond.gl_fault';
    'T13g',    'GYROLESS -> RATE (rate too high)', 'cond.w_gt_GL_out';
    'T14',     'GYROLESS -> gyro modes (gyro recovered)', 'cond.gyr_recovered && cond.req_rank >= 3';
    'T14_FST', 'T14 target FINE_STELLAR',         'cond.req_rank >= 5 && cond.st_ok && cond.w_lt_ST_in';
    'T14_FSUN','T14 target FINE_SUN',             'cond.req_rank >= 4 && cond.fss_ok && cond.mag_ok && cond.eph_ok && cond.sig_lt_FS_in && cond.w_lt_FS_in';
    'T14_CRS', 'T14 target COARSE',               'cond.mag_ok && cond.eph_ok';
    'T15',     'Any -> IDLE (command / numerical fault)', 'cmd_reset || mode_req == uint8(0) || cond.num_fault'};
for k = 1:size(guards, 1)
    emf(chart, guards{k,1}, 'y', {['% ' guards{k,2} ' (document Table 6)']; ['y = ' guards{k,3} ';']}, @place);
end

%% States (mode IDs 0..5). Modes 1-5 sit in ADS_ACTIVE so that T15 is drawn once
% and, as an outer transition, has priority over every mode transition.
W = 250;  Hs = 70;  x0 = 260;  dy = 210;  y0 = 60;
lbl = @(n, id) sprintf('%s\nentry: ads_mode = uint8(%d);\nduring: bookkeeping();', n, id);
ACT = Stateflow.State(chart);
ACT.Name = 'ADS_ACTIVE';
ACT.Position = [20, 15, x0 + 980, y0 + 3*dy + Hs + 40];
ACT.LabelString = 'ADS_ACTIVE';
IDLE = makeState(chart, 'ADS_IDLE',        [x0, y0 + 3*dy + Hs + 130, W, Hs], lbl('ADS_IDLE', 0));
RATE = makeState(ACT, 'ADS_RATE',          [x0, y0 + 3*dy, W, Hs], lbl('ADS_RATE', 1));
CRS  = makeState(ACT, 'ADS_COARSE',        [x0, y0 + 2*dy, W, Hs], lbl('ADS_COARSE', 2));
FSUN = makeState(ACT, 'ADS_FINE_SUN',      [x0, y0 + 1*dy, W, Hs], lbl('ADS_FINE_SUN', 3));
FST  = makeState(ACT, 'ADS_FINE_STELLAR',  [x0, y0,        W, Hs], lbl('ADS_FINE_STELLAR', 4));
GL   = makeState(ACT, 'ADS_GYROLESS',      [x0 + 680, y0 + 2*dy, W, Hs], lbl('ADS_GYROLESS', 5));

dt = Stateflow.Transition(chart);
dt.Destination = IDLE;
dt.SourceEndPoint = [IDLE.Position(1) - 40, IDLE.Position(2) + 35];
dt.DestinationEndPoint = [IDLE.Position(1), IDLE.Position(2) + 35];
da = Stateflow.Transition(ACT);           % required default path inside ADS_ACTIVE
da.Destination = RATE;
da.SourceEndPoint = [RATE.Position(1) - 40, RATE.Position(2) + 35];
da.DestinationEndPoint = [RATE.Position(1), RATE.Position(2) + 35];

J = Stateflow.Junction(ACT);              % downgrade on request (Section 4.4 step 5)
J.Position.Center = [x0 + 560, y0 + 3*dy + 35];
J.Position.Radius = 8;
J14 = Stateflow.Junction(ACT);            % T14 target selection
J14.Position.Center = [x0 + 805, y0 + 0.5*dy];
J14.Position.Radius = 8;

% Execution order: T15 (outer) > gyro failure > fault fallback > geometric fallback > downgrade > upgrade
tr(chart, ACT, IDLE, 1, '[T15()]{mark(15)}', 7.5, 10.5);
tr(chart, IDLE, RATE, 1, '[T1()]{mark(1)}', 1, 5);
tr(chart, RATE, IDLE, 2, '[T2()]{fault_fb(2)}', 5, 1);

tr(chart, RATE, FST, 1, '[T10()]{mark(10)}', 9.5, 8.5);
tr(chart, RATE, CRS, 2, '[T3()]{mark(3)}', 11.3, 6.7);
tr(chart, RATE, GL, 3, '[T12()]{mark(12)}', 2.5, 7);

tr(chart, CRS, GL, 1, '[T11()]{fault_fb(11)}', 2.7, 9.5);
tr(chart, CRS, RATE, 2, '[T4f()]{fault_fb(4)}', 5.2, 0.8);
tr(chart, CRS, RATE, 3, '[T4g()]{mark(4)}', 5.7, 0.3);
tr(chart, CRS, J, 4, '[cond.req_rank < 3]{mark(16)}', 4, 12);
tr(chart, CRS, FST, 5, '[T10()]{mark(10)}', 9, 9);
tr(chart, CRS, FSUN, 6, '[T5()]{mark(5)}', 11.3, 6.7);

tr(chart, FSUN, GL, 1, '[T11()]{fault_fb(11)}', 2.5, 11);
tr(chart, FSUN, CRS, 2, '[T6f()]{fault_fb(6)}', 5.2, 0.8);
tr(chart, FSUN, CRS, 3, '[T6g()]{mark(6)}', 5.7, 0.3);
tr(chart, FSUN, J, 4, '[cond.req_rank < 4]{mark(16)}', 4, 11);
tr(chart, FSUN, FST, 5, '[T7()]{mark(7)}', 11.3, 6.7);

tr(chart, FST, GL, 1, '[T11()]{fault_fb(11)}', 2, 12);
tr(chart, FST, CRS, 2, '[T9f()]{fault_fb(9)}', 3.4, 2.4);
tr(chart, FST, FSUN, 3, '[T8()]{mark(8)}', 5.5, 0.5);
tr(chart, FST, CRS, 4, '[T9g()]{mark(9)}', 3.8, 2);
tr(chart, FST, J, 5, '[cond.req_rank < 5]{mark(16)}', 4.3, 10);

tr(chart, GL, RATE, 1, '[T13f()]{fault_fb(13)}', 7, 3.5);
tr(chart, GL, RATE, 2, '[T13g()]{mark(13)}', 6.3, 4);
tr(chart, GL, J, 3, '[cond.req_rank < 2]{mark(16)}', 8, 3);
tr(chart, GL, J14, 4, '[T14()]', 1, 6);
tr(chart, J14, FST, 1, '[T14_FST()]{mark(14)}', 9, 3);
tr(chart, J14, FSUN, 2, '[T14_FSUN()]{mark(14)}', 8, 3);
tr(chart, J14, CRS, 3, '[T14_CRS()]{mark(14)}', 7, 2.4);

tr(chart, J, RATE, 1, '[mode_req == uint8(1)]', 9, 3);
tr(chart, J, GL, 2, '[mode_req == uint8(5)]', 2, 7.7);
tr(chart, J, CRS, 3, '[mode_req == uint8(2)]', 11, 4.4);
tr(chart, J, FSUN, 4, '[mode_req == uint8(3)]', 12, 4.6);

note = Stateflow.Annotation(chart);
note.Position = [x0 + 330, y0 + 3*dy + Hs + 120, 600, 120];
note.Text = sprintf(['ADS mode manager (ADCS-ADS-DD-001, Section 4)\n' ...
    'Mode IDs: 0 IDLE, 1 RATE, 2 COARSE, 3 FINE_SUN, 4 FINE_STELLAR, 5 GYROLESS\n' ...
    'Guard functions T1..T15 = document Table 6 (f = fault-driven, g = geometric).\n' ...
    'Execution order: T15 > gyro failure > fault > geometric > downgrade > upgrade.\n' ...
    'Super-step semantics give the same-cycle fallback cascade.\n' ...
    'last_transition = document transition ID, 16 = commanded downgrade.']);
end

function emf(chart, name, arg, body, placeFcn)
% MATLAB function inside the chart. arg: '' (none), 'id' (input) or 'y' (boolean output).
f = Stateflow.EMFunction(chart);
switch arg
    case 'y'
        sig = sprintf('y = %s()', name);
    case 'id'
        sig = sprintf('%s(id)', name);
    otherwise
        sig = sprintf('%s()', name);
end
f.LabelString = sig;
placeFcn(f);
f.Script = sprintf('function %s\n%s\nend', sig, strjoin(body, newline));
if strcmp(arg, 'y')
    d = f.find('-isa', 'Stateflow.Data', 'Name', 'y');
    d.DataType = 'boolean';
elseif strcmp(arg, 'id')
    d = f.find('-isa', 'Stateflow.Data', 'Name', 'id');
    d.DataType = 'double';
end
end

function addData(chart, name, scope, type)
d = Stateflow.Data(chart);
d.Name = name;
d.Scope = scope;
d.DataType = type;
if ~strcmp(scope, 'Input') && ~startsWith(type, 'Bus')
    d.Props.InitialValue = '0';
end
end

function s = makeState(chart, name, position, label)
s = Stateflow.State(chart);
s.Name = name;
s.Position = position;
s.LabelString = label;
end

function t = tr(parent, src, dst, order, label, srcClock, dstClock)
t = Stateflow.Transition(parent);
t.Source = src;
t.Destination = dst;
t.LabelString = label;
t.ExecutionOrder = order;
if nargin > 5
    t.SourceOClock = srcClock;
    t.DestinationOClock = dstClock;
end
end
