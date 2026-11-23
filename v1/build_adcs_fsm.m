%% Build the structural ADCS Stateflow/Simulink model.
% Mission_Mode_Manager is the outer mission layer.  It requests a bundle of
% primary modes.  Each primary Stateflow chart owns its actual mode and can
% fall back/retry without changing the commanded mission mode.

modelName = 'ADCS_FSM';
modelFile = fullfile(pwd, [modelName '.slx']);

if isfile(modelFile)
    error('%s already exists. This generator will not overwrite a model.', modelFile);
end

new_system(modelName);
load_system(modelName);
set_param(modelName, 'Solver', 'FixedStepDiscrete', 'FixedStep', '0.1', 'StopTime', '10');

rootInputs = {
    'mission_cmd',              'uint8';
    'gps_valid',                'boolean';
    'tle_error_high',           'boolean';
    'ads_fine_allowed',         'boolean';
    'ads_fault',                'boolean';
    'detumbled',                'boolean';
    'reference_available',      'boolean';
    'energy_low',               'boolean';
    'recovery_ready',           'boolean';
    'slew_feasible',            'boolean';
    'wheels_saturated',         'boolean';
    'desaturated',              'boolean';
    'spin_feasible',            'boolean';
    'spin_axis_available',      'boolean';
    'optical_target_available', 'boolean';
    'sun_sensor_valid',         'boolean';
    'drag_use_vnb',             'boolean';
    'intersat_sensor_tracking', 'boolean';
    'mission_reference_frame',  'uint8'};

for k = 1:size(rootInputs, 1)
    addRootInport(modelName, rootInputs{k, 1}, rootInputs{k, 2}, 110 + 27 * (k - 1));
end

buildMissionChart(modelName);
buildEphemerisChart(modelName);
buildADSChart(modelName);
buildFrameChart(modelName);
buildACSChart(modelName);
buildActuatorChart(modelName, 'Magnetorquer_Actuator', 'desired_mtq_enable', 'actual_mtq_enable', ...
    'desired_mtq_enable || actual_acs == uint8(7)', [970 380 1150 450]);
buildActuatorChart(modelName, 'Reaction_Wheel_Actuator', 'desired_rw_enable', 'actual_rw_enable', ...
    'desired_rw_enable && actual_acs ~= uint8(7)', [970 485 1150 555]);
buildActuatorChart(modelName, 'Thruster_Actuator', 'desired_thruster_enable', 'actual_thruster_enable', ...
    'desired_thruster_enable', [970 590 1150 660]);
buildStatusChart(modelName);

% Connect external guard inputs to the appropriate Stateflow chart inputs.
connect(modelName, 'mission_cmd', 1, 'Mission_Mode_Manager', 1);
connect(modelName, 'spin_feasible', 1, 'Mission_Mode_Manager', 2);
connect(modelName, 'slew_feasible', 1, 'Mission_Mode_Manager', 3);
connect(modelName, 'intersat_sensor_tracking', 1, 'Mission_Mode_Manager', 4);
connect(modelName, 'drag_use_vnb', 1, 'Mission_Mode_Manager', 5);
connect(modelName, 'mission_reference_frame', 1, 'Mission_Mode_Manager', 6);

connect(modelName, 'Mission_Mode_Manager', 1, 'Ephemeris_Manager', 1);
connect(modelName, 'gps_valid', 1, 'Ephemeris_Manager', 2);
connect(modelName, 'tle_error_high', 1, 'Ephemeris_Manager', 3);

connect(modelName, 'Mission_Mode_Manager', 2, 'ADS_Manager', 1);
connect(modelName, 'ads_fine_allowed', 1, 'ADS_Manager', 2);
connect(modelName, 'ads_fault', 1, 'ADS_Manager', 3);

connect(modelName, 'Mission_Mode_Manager', 3, 'Frame_Manager', 1);

connect(modelName, 'Mission_Mode_Manager', 4, 'ACS_Manager', 1);
connect(modelName, 'Ephemeris_Manager', 1, 'ACS_Manager', 2);
connect(modelName, 'detumbled', 1, 'ACS_Manager', 3);
connect(modelName, 'reference_available', 1, 'ACS_Manager', 4);
connect(modelName, 'energy_low', 1, 'ACS_Manager', 5);
connect(modelName, 'recovery_ready', 1, 'ACS_Manager', 6);
connect(modelName, 'wheels_saturated', 1, 'ACS_Manager', 7);
connect(modelName, 'desaturated', 1, 'ACS_Manager', 8);
connect(modelName, 'spin_feasible', 1, 'ACS_Manager', 9);
connect(modelName, 'spin_axis_available', 1, 'ACS_Manager', 10);
connect(modelName, 'optical_target_available', 1, 'ACS_Manager', 11);
connect(modelName, 'sun_sensor_valid', 1, 'ACS_Manager', 12);

connect(modelName, 'Mission_Mode_Manager', 5, 'Magnetorquer_Actuator', 1);
connect(modelName, 'ACS_Manager', 1, 'Magnetorquer_Actuator', 2);
connect(modelName, 'Mission_Mode_Manager', 6, 'Reaction_Wheel_Actuator', 1);
connect(modelName, 'ACS_Manager', 1, 'Reaction_Wheel_Actuator', 2);
connect(modelName, 'Mission_Mode_Manager', 7, 'Thruster_Actuator', 1);

% Degraded is true whenever a requested primary mode is not its actual mode.
connect(modelName, 'Mission_Mode_Manager', 1, 'Status_Manager', 1);
connect(modelName, 'Mission_Mode_Manager', 2, 'Status_Manager', 2);
connect(modelName, 'Mission_Mode_Manager', 3, 'Status_Manager', 3);
connect(modelName, 'Mission_Mode_Manager', 4, 'Status_Manager', 4);
connect(modelName, 'Ephemeris_Manager', 1, 'Status_Manager', 5);
connect(modelName, 'ADS_Manager', 1, 'Status_Manager', 6);
connect(modelName, 'Frame_Manager', 1, 'Status_Manager', 7);
connect(modelName, 'ACS_Manager', 1, 'Status_Manager', 8);

rootOutputs = {
    'desired_ephem',           'Mission_Mode_Manager', 1;
    'desired_ads',             'Mission_Mode_Manager', 2;
    'desired_frame',           'Mission_Mode_Manager', 3;
    'desired_acs',             'Mission_Mode_Manager', 4;
    'desired_mtq_enable',      'Mission_Mode_Manager', 5;
    'desired_rw_enable',       'Mission_Mode_Manager', 6;
    'desired_thruster_enable', 'Mission_Mode_Manager', 7;
    'actual_mission_mode',     'Mission_Mode_Manager', 8;
    'actual_ephem',            'Ephemeris_Manager', 1;
    'actual_ads',              'ADS_Manager', 1;
    'actual_frame',            'Frame_Manager', 1;
    'actual_acs',              'ACS_Manager', 1;
    'actual_mtq_enable',       'Magnetorquer_Actuator', 1;
    'actual_rw_enable',        'Reaction_Wheel_Actuator', 1;
    'actual_thruster_enable',  'Thruster_Actuator', 1;
    'mode_degraded',           'Status_Manager', 1};

for k = 1:size(rootOutputs, 1)
    addRootOutport(modelName, rootOutputs{k, 1}, 110 + 27 * (k - 1));
    connect(modelName, rootOutputs{k, 2}, rootOutputs{k, 3}, rootOutputs{k, 1}, 1);
end

save_system(modelName, modelFile);
close_system(modelName);
fprintf('Created %s\n', modelFile);

%% Local functions
function chart = makeChart(modelName, blockName, position, inputDefinitions, outputDefinitions)
    chartPath = [modelName '/' blockName];
    add_block('sflib/Chart', chartPath, 'Position', position);
    rt = sfroot;
    chart = rt.find('-isa', 'Stateflow.Chart', 'Path', chartPath);
    chart.ActionLanguage = 'MATLAB';
    for k = 1:size(inputDefinitions, 1)
        addData(chart, inputDefinitions{k, 1}, 'Input', inputDefinitions{k, 2});
    end
    for k = 1:size(outputDefinitions, 1)
        addData(chart, outputDefinitions{k, 1}, 'Output', outputDefinitions{k, 2});
    end
end

function addData(chart, name, scope, type)
    data = Stateflow.Data(chart);
    data.Name = name;
    data.Scope = scope;
    data.DataType = type;
    if strcmp(scope, 'Input')
        data.Props.InitialValue = '0';
    else
        data.Props.InitialValue = '0';
    end
end

function state = makeState(chart, name, position, entryActions)
    state = Stateflow.State(chart);
    state.Name = name;
    state.Position = position;
    if isempty(entryActions)
        state.LabelString = name;
    else
        state.LabelString = sprintf('%s\nentry:\n%s', name, strjoin(entryActions, newline));
    end
end

function addDefaultTransition(chart, destination)
    transition = Stateflow.Transition(chart);
    transition.Destination = destination;
    position = destination.Position;
    transition.SourceEndPoint = [position(1) - 20 position(2) + position(4) / 2];
    transition.DestinationEndPoint = [position(1) position(2) + position(4) / 2];
end

function addTransition(chart, source, destination, label)
    transition = Stateflow.Transition(chart);
    transition.Source = source;
    transition.Destination = destination;
    transition.LabelString = label;
end

function junction = makeJunction(chart, center)
    junction = Stateflow.Junction(chart);
    junction.Position.Center = center;
    junction.Position.Radius = 8;
end

function buildSelectorChart(chart, requestName, modes, selectorCenter)
    states = cell(1, numel(modes));
    for k = 1:numel(modes)
        states{k} = makeState(chart, modes(k).name, modes(k).position, modes(k).actions);
    end
    addDefaultTransition(chart, states{1});
    selector = makeJunction(chart, selectorCenter);
    for k = 1:numel(modes)
        addTransition(chart, states{k}, selector, sprintf('[%s ~= uint8(%d)]', requestName, modes(k).id));
        addTransition(chart, selector, states{k}, sprintf('[%s == uint8(%d)]', requestName, modes(k).id));
    end
end

function mode = modeDefinition(name, id, position, actions)
    mode = struct('name', name, 'id', id, 'position', position, 'actions', {actions});
end

function buildMissionChart(modelName)
    inputs = {
        'mission_cmd', 'uint8';
        'spin_feasible', 'boolean';
        'slew_feasible', 'boolean';
        'intersat_sensor_tracking', 'boolean';
        'drag_use_vnb', 'boolean';
        'mission_reference_frame', 'uint8'};
    outputs = {
        'desired_ephem', 'uint8'; 'desired_ads', 'uint8'; 'desired_frame', 'uint8'; 'desired_acs', 'uint8';
        'desired_mtq_enable', 'boolean'; 'desired_rw_enable', 'boolean'; 'desired_thruster_enable', 'boolean';
        'actual_mission_mode', 'uint8'};
    chart = makeChart(modelName, 'Mission_Mode_Manager', [300 100 510 200], inputs, outputs);

    modes = repmat(modeDefinition('', 0, [], {}), 1, 12);
    modes(1) = modeDefinition('M_IDLE', 0, [30 25 135 50], missionActions(0, 0, 0, 0, 0, false, false, false));
    modes(2) = modeDefinition('M_DETUMBLE', 1, [190 25 135 50], missionActions(1, 0, 1, 0, 1, true, false, true));
    modes(3) = modeDefinition('M_SAFE_SUN', 2, [350 25 135 50], missionActions(2, 0, 0, 0, 9, false, true, false));
    modes(4) = modeDefinition('M_Y_SPIN', 3, [510 25 135 50], missionActions(3, 2, 1, 3, 6, false, true, '~spin_feasible'));
    modes(5) = modeDefinition('M_POINT_NADIR', 4, [30 105 135 50], missionActions(4, 2, 2, 3, 2, false, true, false));
    modes(6) = modeDefinition('M_POINT_SUN', 5, [190 105 135 50], missionActions(5, 2, 2, 2, 3, false, true, false));
    modes(7) = modeDefinition('M_POINT_MOON', 6, [350 105 135 50], missionActions(6, 2, 2, 2, 2, false, true, false));
    modes(8) = modeDefinition('M_SLEW_FIXED', 7, [510 105 135 50], missionActions(7, 2, 2, 'mission_reference_frame', 4, false, 'slew_feasible', '~slew_feasible'));
    modes(9) = modeDefinition('M_SLEW_FASTEST', 8, [30 185 135 50], missionActions(8, 2, 2, 'mission_reference_frame', 5, false, true, false));
    modes(10) = modeDefinition('M_INTERSAT_TRACK', 9, [190 185 135 50], missionActions(9, 2, 2, 'mission_reference_frame', 'uint8(2 + 6 * intersat_sensor_tracking)', false, true, false));
    modes(11) = modeDefinition('M_TDI', 10, [350 185 135 50], missionActions(10, 2, 2, 6, 2, false, true, false));
    modes(12) = modeDefinition('M_MINMAX_DRAG', 11, [510 185 135 50], missionActions(11, 2, 2, 'uint8(3 + 2 * drag_use_vnb)', 2, false, true, false));
    buildSelectorChart(chart, 'mission_cmd', modes, [690 145]);
end

function actions = missionActions(missionId, ephem, ads, frame, acs, mtq, rw, thruster)
    actions = { ...
        sprintf('actual_mission_mode = uint8(%d);', missionId), ...
        assignment('desired_ephem', ephem), assignment('desired_ads', ads), assignment('desired_frame', frame), assignment('desired_acs', acs), ...
        assignment('desired_mtq_enable', mtq), assignment('desired_rw_enable', rw), assignment('desired_thruster_enable', thruster)};
end

function line = assignment(name, value)
    if islogical(value)
        if value
            valueText = 'true';
        else
            valueText = 'false';
        end
    elseif isnumeric(value)
        valueText = sprintf('uint8(%d)', value);
    else
        valueText = value;
    end
    line = sprintf('%s = %s;', name, valueText);
end

function buildEphemerisChart(modelName)
    chart = makeChart(modelName, 'Ephemeris_Manager', [560 80 710 140], ...
        {'desired_ephem', 'uint8'; 'gps_valid', 'boolean'; 'tle_error_high', 'boolean'}, {'actual_ephem', 'uint8'});
    idle = makeState(chart, 'EPHEM_IDLE', [30 70 120 50], {'actual_ephem = uint8(0);'});
    tle = makeState(chart, 'EPHEM_TLE', [190 70 120 50], {'actual_ephem = uint8(1);'});
    gps = makeState(chart, 'EPHEM_GPS', [350 70 120 50], {'actual_ephem = uint8(2);'});
    addDefaultTransition(chart, idle);
    addTransition(chart, idle, tle, '[desired_ephem == uint8(1) || (desired_ephem == uint8(2) && ~gps_valid && ~tle_error_high)]');
    addTransition(chart, idle, gps, '[desired_ephem == uint8(2) && gps_valid]');
    addTransition(chart, tle, idle, '[desired_ephem == uint8(0) || tle_error_high]');
    addTransition(chart, tle, gps, '[desired_ephem == uint8(2) && gps_valid]');
    addTransition(chart, gps, idle, '[desired_ephem == uint8(0)]');
    addTransition(chart, gps, tle, '[desired_ephem == uint8(1) || ~gps_valid]');
end

function buildADSChart(modelName)
    chart = makeChart(modelName, 'ADS_Manager', [560 180 710 240], ...
        {'desired_ads', 'uint8'; 'ads_fine_allowed', 'boolean'; 'ads_fault', 'boolean'}, {'actual_ads', 'uint8'});
    idle = makeState(chart, 'ADS_IDLE', [30 70 120 50], {'actual_ads = uint8(0);'});
    coarse = makeState(chart, 'ADS_UKF_COARSE', [190 70 120 50], {'actual_ads = uint8(1);'});
    fine = makeState(chart, 'ADS_UKF_FINE', [350 70 120 50], {'actual_ads = uint8(2);'});
    addDefaultTransition(chart, idle);
    addTransition(chart, idle, coarse, '[desired_ads == uint8(1) || (desired_ads == uint8(2) && ~ads_fine_allowed)]');
    addTransition(chart, idle, fine, '[desired_ads == uint8(2) && ads_fine_allowed && ~ads_fault]');
    addTransition(chart, coarse, idle, '[desired_ads == uint8(0) || ads_fault]');
    addTransition(chart, coarse, fine, '[desired_ads == uint8(2) && ads_fine_allowed && ~ads_fault]');
    addTransition(chart, fine, idle, '[desired_ads == uint8(0) || ads_fault]');
    addTransition(chart, fine, coarse, '[desired_ads == uint8(1) || ~ads_fine_allowed]');
end

function buildFrameChart(modelName)
    chart = makeChart(modelName, 'Frame_Manager', [560 280 710 340], {'desired_frame', 'uint8'}, {'actual_frame', 'uint8'});
    names = {'FRAME_IDLE', 'FRAME_BODY', 'FRAME_INERTIAL', 'FRAME_LVLH', 'FRAME_EARTH_TARGET', 'FRAME_VNB', 'FRAME_IMAGING_TDI'};
    modes = repmat(modeDefinition('', 0, [], {}), 1, numel(names));
    for k = 1:numel(names)
        row = floor((k - 1) / 4);
        column = mod(k - 1, 4);
        modes(k) = modeDefinition(names{k}, k - 1, [30 + 165 * column 45 + 95 * row 140 48], {sprintf('actual_frame = uint8(%d);', k - 1)});
    end
    buildSelectorChart(chart, 'desired_frame', modes, [680 160]);
end

function buildACSChart(modelName)
    inputs = {
        'desired_acs', 'uint8'; 'actual_ephem', 'uint8'; 'detumbled', 'boolean'; 'reference_available', 'boolean';
        'energy_low', 'boolean'; 'recovery_ready', 'boolean'; 'wheels_saturated', 'boolean'; 'desaturated', 'boolean';
        'spin_feasible', 'boolean'; 'spin_axis_available', 'boolean'; 'optical_target_available', 'boolean'; 'sun_sensor_valid', 'boolean'};
    chart = makeChart(modelName, 'ACS_Manager', [560 380 710 460], inputs, {'actual_acs', 'uint8'});
    names = {'ACS_IDLE', 'ACS_DETUMBLE', 'ACS_POINT', 'ACS_HOLD', 'ACS_SLEW_SLOW', 'ACS_SLEW_FAST', 'ACS_SPIN', 'ACS_DESAT', 'ACS_OPTICAL', 'ACS_SAFE'};
    states = cell(1, numel(names));
    for k = 1:numel(names)
        row = floor((k - 1) / 5);
        column = mod(k - 1, 5);
        states{k} = makeState(chart, names{k}, [25 + 150 * column 40 + 140 * row 130 55], {sprintf('actual_acs = uint8(%d);', k - 1)});
    end
    addDefaultTransition(chart, states{1});
    selector = makeJunction(chart, [790 290]);
    guards = {
        '[desired_acs ~= uint8(0)]';
        '[desired_acs ~= uint8(1)]';
        '[desired_acs ~= uint8(2) && ~(desired_acs == uint8(8) && ~optical_target_available)]';
        '[desired_acs ~= uint8(3)]';
        '[desired_acs ~= uint8(4)]';
        '[desired_acs ~= uint8(5)]';
        '[desired_acs ~= uint8(6)]';
        '[desired_acs ~= uint8(7) && desaturated && recovery_ready && ~energy_low]';
        '[desired_acs ~= uint8(8) && optical_target_available]';
        ['[desired_acs ~= uint8(9) && ~energy_low && recovery_ready && sun_sensor_valid && ' ...
         '~((desired_acs == uint8(2)) && ~reference_available) && ' ...
         '~((desired_acs == uint8(6)) && (~spin_feasible || ~spin_axis_available))]']};
    for k = 1:numel(states)
        addTransition(chart, states{k}, selector, guards{k});
        addTransition(chart, selector, states{k}, sprintf('[desired_acs == uint8(%d)]', k - 1));
    end
    addTransition(chart, states{3}, states{10}, '[energy_low || ~reference_available || ~detumbled || actual_ephem == uint8(0)]');
    addTransition(chart, states{4}, states{10}, '[energy_low || ~detumbled || actual_ephem == uint8(0)]');
    addTransition(chart, states{5}, states{10}, '[energy_low || ~detumbled || actual_ephem == uint8(0)]');
    addTransition(chart, states{5}, states{8}, '[wheels_saturated && ~energy_low]');
    addTransition(chart, states{6}, states{10}, '[energy_low || ~detumbled || actual_ephem == uint8(0)]');
    addTransition(chart, states{6}, states{8}, '[wheels_saturated && ~energy_low]');
    addTransition(chart, states{7}, states{10}, '[energy_low || ~spin_feasible || ~spin_axis_available]');
    addTransition(chart, states{8}, states{10}, '[energy_low]');
    addTransition(chart, states{9}, states{10}, '[energy_low]');
    addTransition(chart, states{9}, states{3}, '[~optical_target_available && ~energy_low]');
end

function buildActuatorChart(modelName, blockName, desiredInput, actualOutput, enableExpression, position)
    inputs = {desiredInput, 'boolean'};
    if contains(enableExpression, 'actual_acs')
        inputs = [inputs; {'actual_acs', 'uint8'}];
    end
    chart = makeChart(modelName, blockName, position, inputs, {actualOutput, 'boolean'});
    off = makeState(chart, 'OFF', [50 65 100 45], {[actualOutput ' = false;']});
    on = makeState(chart, 'ON', [220 65 100 45], {[actualOutput ' = true;']});
    addDefaultTransition(chart, off);
    addTransition(chart, off, on, ['[' enableExpression ']']);
    addTransition(chart, on, off, ['[~(' enableExpression ')]']);
end

function buildStatusChart(modelName)
    inputs = {
        'desired_ephem', 'uint8'; 'desired_ads', 'uint8'; 'desired_frame', 'uint8'; 'desired_acs', 'uint8';
        'actual_ephem', 'uint8'; 'actual_ads', 'uint8'; 'actual_frame', 'uint8'; 'actual_acs', 'uint8'};
    chart = makeChart(modelName, 'Status_Manager', [1180 265 1340 335], inputs, {'mode_degraded', 'boolean'});
    status = Stateflow.State(chart);
    status.Name = 'STATUS';
    status.Position = [70 70 250 65];
    status.LabelString = sprintf(['STATUS\nduring:\nmode_degraded = (actual_ephem ~= desired_ephem) || ' ...
        '(actual_ads ~= desired_ads) || (actual_frame ~= desired_frame) || (actual_acs ~= desired_acs);']);
    addDefaultTransition(chart, status);
end

function addRootInport(modelName, name, dataType, y)
    blockPath = [modelName '/' name];
    add_block('simulink/Sources/In1', blockPath, 'Position', [40 y 70 y + 14]);
    set_param(blockPath, 'OutDataTypeStr', dataType, 'PortDimensions', '1');
end

function addRootOutport(modelName, name, y)
    add_block('simulink/Sinks/Out1', [modelName '/' name], 'Position', [1400 y 1430 y + 14]);
end

function connect(modelName, sourceBlock, sourcePort, destinationBlock, destinationPort)
    add_line(modelName, [sourceBlock '/' num2str(sourcePort)], [destinationBlock '/' num2str(destinationPort)], 'autorouting', 'on');
end
