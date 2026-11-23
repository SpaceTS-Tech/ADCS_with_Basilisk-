%% ADCS FSM root-I/O test runner
% Run this script from the folder that contains ADCS_FSM_test.slx.
% It exercises only the root-I/O executable cases and writes
% ADCS_FSM_test_Report.xlsx.

omega_thr = 1;
runADCSFSMTestSuite();

function runADCSFSMTestSuite()

modelName = 'ADCS_FSM_test';
reportFile = fullfile(pwd, 'ADCS_FSM_test_Report.xlsx');
sampleTime = 0.1;

expectedInports = [ ...
    "mission_cmd", "gps_valid", "tle_error_high", "ads_fine_allowed", ...
    "ads_fault", "detumbled", "reference_available", "energy_low", ...
    "recovery_ready", "wheels_saturated", "desaturated", "spin_feasible", ...
    "spin_axis_available", "optical_target_available", "sun_sensor_valid", ...
    "intersat_sensor_tracking"];

expectedOutports = [ ...
    "actual_ephem", "actual_ads", "actual_frame", "actual_acs", ...
    "actual_mtq_enable", "actual_rw_enable", "actual_thruster_enable", ...
    "mode_degraded"];

if ~isfile([modelName '.slx'])
    error('Model file %s.slx was not found in %s.', modelName, pwd);
end

if isfile(reportFile)
    delete(reportFile);
end

load_system(modelName);
cleanModel = onCleanup(@()closeModelWithoutSaving(modelName)); %#ok<NASGU>

auxiliaryVariables = modeManagerAuxVariables();
cleanBase = provisionAuxiliaryBaseVariables(auxiliaryVariables); %#ok<NASGU>

[inportNames, inportTypes] = orderedRootPorts(modelName, 'Inport');
[outportNames, ~] = orderedRootPorts(modelName, 'Outport');

portCheck = strings(0,1);

if ~isequal(inportNames(:), expectedInports(:))
    portCheck(end+1) = "Input-port order/name mismatch. Expected: " + strjoin(expectedInports, ', '); %#ok<AGROW>
    portCheck(end+1) = "Found: " + strjoin(inportNames, ', '); %#ok<AGROW>
end

if ~isequal(outportNames(:), expectedOutports(:))
    portCheck(end+1) = "Output-port order/name mismatch. Expected: " + strjoin(expectedOutports, ', '); %#ok<AGROW>
    portCheck(end+1) = "Found: " + strjoin(outportNames, ', '); %#ok<AGROW>
end

cases = buildRootIOCases();

preflightError = strjoin(portCheck, newline);

if strlength(preflightError) == 0
    try
        set_param(modelName, 'SimulationCommand', 'update');
    catch ME
        preflightError = compactException(ME);
    end
end

nCases = numel(cases);
nOutputs = numel(outportNames);

actual = nan(nCases, nOutputs);
status = strings(nCases,1);
failureDetail = strings(nCases,1);
simulationError = strings(nCases,1);

if strlength(preflightError) > 0

    for k = 1:nCases
        status(k) = "ERROR";
        failureDetail(k) = "Model failed preflight compilation; root-I/O test was not simulated.";
        simulationError(k) = preflightError;
    end

else

    for k = 1:nCases
        try
            actual(k,:) = simulateCase( ...
                modelName, cases(k), inportNames, inportTypes, ...
                outportNames, sampleTime, auxiliaryVariables);

            [status(k), failureDetail(k)] = compareOutputs( ...
                cases(k).expected, actual(k,:), outportNames);

        catch ME
            status(k) = "ERROR";
            failureDetail(k) = "Simulation did not complete.";
            simulationError(k) = compactException(ME);
        end
    end

end

results = resultsTable( ...
    cases, inportNames, outportNames, actual, ...
    status, failureDetail, simulationError);

writeReport( ...
    reportFile, modelName, sampleTime, inportNames, ...
    outportNames, portCheck, auxiliaryVariables, results);

formatReport(reportFile);

nPass = nnz(status == "PASS");
nFail = nnz(status == "FAIL");
nError = nnz(status == "ERROR");

fprintf('ADCS FSM test report written: %s\n', reportFile);
fprintf('Total executable: %d | PASS: %d | FAIL: %d | ERROR: %d\n', ...
    nCases, nPass, nFail, nError);

end

function cases = buildRootIOCases()

% Root-port order:
%  1 mission_cmd, 2 gps_valid, 3 tle_error_high, 4 ads_fine_allowed,
%  5 ads_fault, 6 detumbled, 7 reference_available, 8 energy_low,
%  9 recovery_ready, 10 wheels_saturated, 11 desaturated,
% 12 spin_feasible, 13 spin_axis_available, 14 optical_target_available,
% 15 sun_sensor_valid, 16 intersat_sensor_tracking.

I = struct( ...
    'cmd',1, ...
    'gps',2, ...
    'tle',3, ...
    'fine',4, ...
    'adsFault',5, ...
    'det',6, ...
    'ref',7, ...
    'energy',8, ...
    'recovery',9, ...
    'wheelSat',10, ...
    'desat',11, ...
    'spinFeasible',12, ...
    'spinAxis',13, ...
    'target',14, ...
    'sun',15, ...
    'intersat',16);

B0 = [0 1 0 1 0 1 1 0 1 0 1 1 1 1 1 0];

e = @(ephem,ads,frame,acs,mtq,rw,thr,degraded) ...
    [ephem ads frame acs mtq rw thr degraded];

cases = repmat(emptyCase(), 0, 1);

% Nominal mission commands 0..11.
nominal = [ ...
    e(0,0,0,0,0,0,0,0); ... % M_IDLE
    e(0,1,0,1,1,0,1,0); ... % M_DETUMBLE
    e(0,0,0,9,0,1,0,0); ... % M_SAFE_SUN
    e(2,1,3,6,0,1,0,0); ... % M_Y_SPIN
    e(2,2,3,2,0,1,0,0); ... % M_POINT_NADIR
    e(2,2,2,3,0,1,0,0); ... % M_POINT_SUN
    e(2,2,2,2,0,1,0,0); ... % M_POINT_MOON
    e(2,2,2,4,0,1,0,0); ... % M_SLEW_FIXED
    e(2,2,2,5,0,1,0,0); ... % M_SLEW_FASTEST
    e(2,2,2,2,0,1,0,0); ... % M_INTERSAT_TRACK
    e(2,2,6,2,0,1,0,0); ... % M_TDI
    e(2,2,3,2,0,1,0,0)];    % M_MINMAX_DRAG

missionNames = [ ...
    "M_IDLE", ...
    "M_DETUMBLE", ...
    "M_SAFE_SUN", ...
    "M_Y_SPIN", ...
    "M_POINT_NADIR", ...
    "M_POINT_SUN", ...
    "M_POINT_MOON", ...
    "M_SLEW_FIXED", ...
    "M_SLEW_FASTEST", ...
    "M_INTERSAT_TRACK", ...
    "M_TDI", ...
    "M_MINMAX_DRAG"];

for cmd = 0:11

    u = B0;
    u(I.cmd) = cmd;

    if cmd == 9
        u(I.intersat) = 0;
    end

    cases(end+1) = makeCase( ...
        sprintf('TC-MM-%03d',cmd+1), ...
        missionNames(cmd+1), ...
        'Nominal root-input mission selection.', ...
        0, u, nominal(cmd+1,:)); %#ok<AGROW>

end

% Root-visible fallback, recovery, and actuator-interlock cases.

u = setValues(B0, [I.cmd I.spinFeasible], [3 0]);
cases(end+1) = makeCase( ...
    'TC-MM-013', ...
    'Y-spin infeasible', ...
    'Spin feasibility is lost before M_Y_SPIN is requested.', ...
    0, u, e(2,1,3,9,0,1,0,1));

u = setValues(B0, [I.cmd I.spinAxis], [3 0]);
cases(end+1) = makeCase( ...
    'TC-MM-014', ...
    'Y-spin axis unavailable', ...
    'Spin-axis availability is lost before M_Y_SPIN is requested.', ...
    0, u, e(2,1,3,9,0,1,0,1));

u = setValues(B0, [I.cmd I.gps], [4 0]);
cases(end+1) = makeCase( ...
    'TC-E2E-001', ...
    'Pointing with TLE fallback', ...
    'GPS is invalid while TLE is valid.', ...
    0, u, e(1,2,3,2,0,1,0,1));

u = setValues(B0, [I.cmd I.gps I.tle], [4 0 1]);
cases(end+1) = makeCase( ...
    'TC-E2E-002', ...
    'Pointing with no ephemeris', ...
    'GPS is invalid and TLE error is high.', ...
    0, u, e(0,2,3,9,0,1,0,1));

u = setValues(B0, [I.cmd I.fine], [4 0]);
cases(end+1) = makeCase( ...
    'TC-E2E-003', ...
    'Pointing with coarse ADS', ...
    'Fine ADS is not allowed.', ...
    0, u, e(2,1,3,2,0,1,0,1));

u = setValues(B0, [I.cmd I.adsFault], [4 1]);
cases(end+1) = makeCase( ...
    'TC-E2E-004', ...
    'Pointing with ADS fault', ...
    'ADS fault is asserted before M_POINT_NADIR is requested.', ...
    0, u, e(2,0,3,2,0,1,0,1));

u = setValues(B0, [I.cmd I.ref], [4 0]);
cases(end+1) = makeCase( ...
    'TC-ACS-012', ...
    'POINT reference fallback', ...
    'Reference availability is false.', ...
    0, u, e(2,2,3,9,0,1,0,1));

u = setValues(B0, [I.cmd I.det], [4 0]);
cases(end+1) = makeCase( ...
    'TC-ACS-013', ...
    'POINT detumble fallback', ...
    'Detumble completion is false.', ...
    0, u, e(2,2,3,9,0,1,0,1));

u = setValues(B0, [I.cmd I.energy], [4 1]);
cases(end+1) = makeCase( ...
    'TC-ACS-011', ...
    'POINT energy fallback', ...
    'Energy-low guard is asserted.', ...
    0, u, e(2,2,3,9,0,1,0,1));

u = setValues(B0, [I.cmd I.wheelSat], [8 1]);
cases(end+1) = makeCase( ...
    'TC-ACS-027', ...
    'FAST slew wheel saturation', ...
    'Reaction wheels are saturated in M_SLEW_FASTEST.', ...
    0, u, e(2,2,2,7,1,0,0,1));

u = setValues(B0, [I.cmd I.target I.intersat], [9 0 1]);
cases(end+1) = makeCase( ...
    'TC-ACS-037', ...
    'OPTICAL target unavailable', ...
    'Intersatellite optical mode is requested with no target.', ...
    0, u, e(2,2,2,2,0,1,0,1));

u = setValues(B0, [I.cmd I.sun], [2 0]);
cases(end+1) = makeCase( ...
    'TC-ACS-050', ...
    'SAFE with invalid sun sensor', ...
    'Safe sun mode is requested with sun sensor invalid.', ...
    0, u, e(0,0,0,9,0,1,0,0));

% Recovery sequences. Inputs are held long enough for the Stateflow charts
% to leave the fallback and re-enter the originally requested mode.

u1 = setValues(B0, I.cmd, 4);
u2 = setValues(u1, I.ref, 0);
u3 = setValues(u1, I.ref, 1);
cases(end+1) = makeSequenceCase( ...
    'TC-E2E-005', ...
    'POINT reference recovery', ...
    'M_POINT_NADIR, lose reference, then restore it.', ...
    [0 0.4 0.8], [u1;u2;u3], e(2,2,3,2,0,1,0,0));

u1 = setValues(B0, I.cmd, 4);
u2 = setValues(u1, [I.gps I.tle], [0 1]);
u3 = setValues(u1, [I.gps I.tle], [1 0]);
cases(end+1) = makeSequenceCase( ...
    'TC-E2E-006', ...
    'POINT ephemeris recovery', ...
    'M_POINT_NADIR, make both ephemeris sources unavailable, then restore GPS.', ...
    [0 0.4 0.8], [u1;u2;u3], e(2,2,3,2,0,1,0,0));

u1 = setValues(B0, I.cmd, 8);
u2 = setValues(u1, I.wheelSat, 1);
u3 = setValues(u1, I.wheelSat, 0);
cases(end+1) = makeSequenceCase( ...
    'TC-E2E-007', ...
    'FAST slew desaturation recovery', ...
    'M_SLEW_FASTEST, wheel saturation, then wheel recovery.', ...
    [0 0.4 0.8], [u1;u2;u3], e(2,2,2,5,0,1,0,0));

u1 = setValues(B0, I.cmd, 4);
u2 = setValues(B0, I.cmd, 2);
cases(end+1) = makeSequenceCase( ...
    'TC-MM-017', ...
    'Mission transition: NADIR to SAFE', ...
    'Change mission command from M_POINT_NADIR to M_SAFE_SUN.', ...
    [0 0.6], [u1;u2], e(0,0,0,9,0,1,0,0));

u1 = setValues(B0, I.cmd, 4);
u2 = setValues(B0, I.cmd, 12);
cases(end+1) = makeSequenceCase( ...
    'TC-MM-018', ...
    'Unsupported mission command: 12', ...
    'Change M_POINT_NADIR to unsupported command 12.', ...
    [0 0.6], [u1;u2], e(0,0,0,0,0,0,0,0));

u1 = setValues(B0, I.cmd, 4);
u2 = setValues(B0, I.cmd, 255);
cases(end+1) = makeSequenceCase( ...
    'TC-MM-019', ...
    'Unsupported mission command: 255', ...
    'Change M_POINT_NADIR to unsupported command 255.', ...
    [0 0.6], [u1;u2], e(0,0,0,0,0,0,0,0));

% Important:
% Do not merge with wordCaseCatalogue().
% That catalogue contains many INTERNAL_HARNESS cases which are not
% executable through the ADCS_FSM_test root Inport/Outport interface.

end

function c = emptyCase()

c = struct( ...
    'id',"", ...
    'name',"", ...
    'description',"", ...
    'times',0, ...
    'stages',zeros(1,16), ...
    'expected',nan(1,8), ...
    'runnable',true, ...
    'scope',"ROOT_IO", ...
    'executionNote',"Executable through the ADCS_FSM_test root Inport/Outport interface.");

end

function c = makeCase(id, name, description, time, inputVector, expected)

c = emptyCase();
c.id = string(id);
c.name = string(name);
c.description = string(description);
c.times = time;
c.stages = inputVector;
c.expected = expected;
c.runnable = true;
c.scope = "ROOT_IO";
c.executionNote = "Executable through the ADCS_FSM_test root Inport/Outport interface.";

end

function c = makeSequenceCase(id, name, description, times, stages, expected)

c = emptyCase();
c.id = string(id);
c.name = string(name);
c.description = string(description);
c.times = times;
c.stages = stages;
c.expected = expected;
c.runnable = true;
c.scope = "ROOT_IO";
c.executionNote = "Executable through the ADCS_FSM_test root Inport/Outport interface.";

end

function values = setValues(values, index, replacement)

values(index) = replacement;

end

function [names, types] = orderedRootPorts(modelName, blockType)

blocks = find_system(modelName, 'SearchDepth', 1, 'BlockType', blockType);

portNumber = cellfun(@(b)str2double(get_param(b,'Port')), blocks);
[~,idx] = sort(portNumber);
blocks = blocks(idx);

names = string(cellfun(@(b)get_param(b,'Name'), blocks, 'UniformOutput', false));

if strcmp(blockType,'Inport')
    types = string(cellfun(@(b)get_param(b,'OutDataTypeStr'), blocks, 'UniformOutput', false));
else
    types = strings(size(names));
end

end

function y = simulateCase( ...
    modelName, testCase, inportNames, inportTypes, ...
    outportNames, Ts, auxiliaryVariables)

stopTime = max(testCase.times(end) + 0.8, 1.0);

time = (0:Ts:stopTime)';

u = zeros(numel(time), numel(inportNames));

for k = 1:numel(time)
    stage = find(testCase.times <= time(k), 1, 'last');
    u(k,:) = testCase.stages(stage,:);
end

inputData = Simulink.SimulationData.Dataset;

for p = 1:numel(inportNames)

    data = u(:,p);

    if strcmpi(inportTypes(p),'boolean')
        data = logical(data);
    elseif contains(lower(inportTypes(p)),'uint8')
        data = uint8(data);
    end

    signal = timeseries(data, time, 'Name', char(inportNames(p)));
    inputData = inputData.addElement(signal, char(inportNames(p)));

end

simIn = Simulink.SimulationInput(modelName);
simIn = simIn.setExternalInput(inputData);
simIn = applyAuxiliaryVariables(simIn, auxiliaryVariables);

simIn = simIn.setModelParameter( ...
    'StartTime','0', ...
    'StopTime',num2str(stopTime), ...
    'SaveOutput','on', ...
    'OutputSaveName','yout', ...
    'SaveFormat','Dataset', ...
    'ReturnWorkspaceOutputs','on');

simOut = sim(simIn);

yout = simOut.get('yout');

y = zeros(1,numel(outportNames));

for p = 1:numel(outportNames)

    % Root Outport logging does not preserve propagated signal names in
    % every Simulink release, so read by verified root-port order.
    element = yout.getElement(p);
    data = element.Values.Data;
    y(p) = double(data(end));

end

end

function [status, details] = compareOutputs(expected, actual, outportNames)

mask = ~isnan(expected);
bad = find(mask & abs(expected - actual) > 1e-9);

if isempty(bad)

    status = "PASS";
    details = "";

else

    status = "FAIL";
    parts = strings(1,numel(bad));

    for k = 1:numel(bad)
        p = bad(k);
        parts(k) = outportNames(p) + ": expected " + expected(p) + ", actual " + actual(p);
    end

    details = strjoin(parts, '; ');

end

end

function t = resultsTable( ...
    cases, inportNames, outportNames, actual, status, detail, simError)

n = numel(cases);

ids = reshape(string({cases.id}),[],1);
names = reshape(string({cases.name}),[],1);
descriptions = reshape(string({cases.description}),[],1);
scopes = reshape(string({cases.scope}),[],1);
executionNotes = reshape(string({cases.executionNote}),[],1);

finalInputs = zeros(n,numel(inportNames));
expected = zeros(n,numel(outportNames));

for k = 1:n
    finalInputs(k,:) = cases(k).stages(end,:);
    expected(k,:) = cases(k).expected;
end

inputTable = array2table( ...
    finalInputs, ...
    'VariableNames', cellstr("in_" + inportNames));

expectedTable = array2table( ...
    expected, ...
    'VariableNames', cellstr("expected_" + outportNames));

actualTable = array2table( ...
    actual, ...
    'VariableNames', cellstr("actual_" + outportNames));

t = [ ...
    table( ...
        ids, names, scopes, descriptions, status, ...
        executionNotes, detail, simError, ...
        'VariableNames', { ...
        'Test_ID', ...
        'Test_Name', ...
        'Execution_Scope', ...
        'Description', ...
        'Status', ...
        'Execution_Note', ...
        'Failure_Detail', ...
        'Simulation_Error'}) ...
    inputTable expectedTable actualTable];

end

function writeReport( ...
    reportFile, modelName, sampleTime, inportNames, ...
    outportNames, portCheck, auxiliaryVariables, results)

isPass = results.Status == "PASS";
isFail = results.Status == "FAIL";
isError = results.Status == "ERROR";

summary = table( ...
    ["Total executable root-I/O tests";"Passed";"Failed";"Errors"], ...
    [height(results);nnz(isPass);nnz(isFail);nnz(isError)], ...
    'VariableNames', {'Metric','Count'});

config = table( ...
    ["Model"; ...
     "Sample time (s)"; ...
     "Input port order"; ...
     "Output port order"; ...
     "Auxiliary Mode Manager variables"; ...
     "Preflight result"; ...
     "Port check notes"], ...
    [string(modelName); ...
     string(sampleTime); ...
     strjoin(inportNames, ', '); ...
     strjoin(outportNames, ', '); ...
     auxiliaryVariablesText(auxiliaryVariables); ...
     ternary(strlength(strjoin(portCheck,newline)) == 0, "PASS", "ERROR"); ...
     ternary(strlength(strjoin(portCheck,newline)) == 0, ...
        "Port order matches expected interface.", ...
        strjoin(portCheck,newline))], ...
    'VariableNames', {'Item','Value'});

writecell( ...
    {"ADCS FSM Root-I/O Test Report"; ...
     "Generated"; ...
     string(datetime('now','Format','yyyy-MM-dd HH:mm:ss'))}, ...
    reportFile, ...
    'Sheet','Summary', ...
    'Range','A1');

writetable(summary, reportFile, 'Sheet','Summary', 'Range','A6');
writetable(config, reportFile, 'Sheet','Configuration', 'Range','A1');

writecell({"Test metadata"}, reportFile, 'Sheet','Results', 'Range','A1');
writecell({"Input values (final test stage)"}, reportFile, 'Sheet','Results', 'Range','I1');
writecell({"Expected outputs"}, reportFile, 'Sheet','Results', 'Range','Y1');
writecell({"Obtained outputs"}, reportFile, 'Sheet','Results', 'Range','AG1');

writetable(results, reportFile, 'Sheet','Results', 'Range','A2');

end

function auxiliaryVariables = modeManagerAuxVariables()

% These Stateflow symbols are referenced by the model but are not root
% Inports. The values mirror the nominal configuration used by the test plan
% and are injected only for the programmatic simulation.

auxiliaryVariables = struct( ...
    'slew_feasible', true, ...
    'mission_reference_frame', uint8(2), ...
    'drag_use_vnb', false);

end

function cleaner = provisionAuxiliaryBaseVariables(auxiliaryVariables)

names = fieldnames(auxiliaryVariables);

hadValue = false(numel(names),1);
oldValue = cell(numel(names),1);

for k = 1:numel(names)

    hadValue(k) = evalin('base', sprintf('exist(''%s'',''var'') == 1', names{k}));

    if hadValue(k)
        oldValue{k} = evalin('base', names{k});
    end

    assignin('base', names{k}, auxiliaryVariables.(names{k}));

end

cleaner = onCleanup(@()restoreAuxiliaryBaseVariables(names, hadValue, oldValue));

end

function restoreAuxiliaryBaseVariables(names, hadValue, oldValue)

for k = 1:numel(names)

    if hadValue(k)
        assignin('base', names{k}, oldValue{k});
    else
        evalin('base', sprintf('clear %s', names{k}));
    end

end

end

function simIn = applyAuxiliaryVariables(simIn, auxiliaryVariables)

names = fieldnames(auxiliaryVariables);

for k = 1:numel(names)
    simIn = simIn.setVariable(names{k}, auxiliaryVariables.(names{k}));
end

end

function text = auxiliaryVariablesText(auxiliaryVariables) %#ok<INUSD>

text = "slew_feasible=true; mission_reference_frame=uint8(2); drag_use_vnb=false";

end

function formatReport(reportFile)

% Excel formatting is optional for execution portability.
% The report content is still written if Excel automation is unavailable.

try

    excel = actxserver('Excel.Application');
    cleaner = onCleanup(@()quitExcel(excel)); %#ok<NASGU>

    excel.Visible = false;
    excel.DisplayAlerts = false;

    book = excel.Workbooks.Open(reportFile);

    reportSheet = book.Worksheets.Item('Results');
    summarySheet = book.Worksheets.Item('Summary');
    configSheet = book.Worksheets.Item('Configuration');

    styleHeader(summarySheet.Range('A6:B6'));
    styleHeader(configSheet.Range('A1:B1'));

    lastRow = reportSheet.UsedRange.Rows.Count;
    lastCol = reportSheet.UsedRange.Columns.Count;
    lastColLetter = excelColumn(lastCol);

    reportSheet.Range('A1:H1').Merge;
    reportSheet.Range('I1:X1').Merge;
    reportSheet.Range('Y1:AF1').Merge;
    reportSheet.Range('AG1:AN1').Merge;

    styleHeader(reportSheet.Range('A1:H1'));
    styleHeader(reportSheet.Range('I1:X1'));
    styleHeader(reportSheet.Range('Y1:AF1'));
    styleHeader(reportSheet.Range('AG1:AN1'));
    styleHeader(reportSheet.Range(sprintf('A2:%s2',lastColLetter)));

    reportSheet.Range(sprintf('A1:%s%d',lastColLetter,lastRow)).WrapText = true;
    reportSheet.Range(sprintf('A1:%s%d',lastColLetter,lastRow)).VerticalAlignment = -4160; % xlTop
    reportSheet.Range(sprintf('A2:%s%d',lastColLetter,lastRow)).AutoFilter;

    reportSheet.Columns.AutoFit;
    reportSheet.Columns.Item(3).ColumnWidth = 22;
    reportSheet.Columns.Item(4).ColumnWidth = 32;
    reportSheet.Columns.Item(5).ColumnWidth = 16;
    reportSheet.Columns.Item(6).ColumnWidth = 38;
    reportSheet.Columns.Item(7).ColumnWidth = 45;
    reportSheet.Columns.Item(8).ColumnWidth = 55;

    statusColumnLetter = 'E';

    statusValues = string(reportSheet.Range(sprintf('%s3:%s%d', ...
        statusColumnLetter, statusColumnLetter, lastRow)).Value2);

    statusValues = statusValues(:);

    startIndex = 1;

    while startIndex <= numel(statusValues)

        endIndex = startIndex;

        while endIndex < numel(statusValues) && ...
                statusValues(endIndex+1) == statusValues(startIndex)
            endIndex = endIndex + 1;
        end

        firstRow = startIndex + 2;
        finalRow = endIndex + 2;

        rowRange = reportSheet.Range(sprintf('A%d:%s%d', ...
            firstRow, lastColLetter, finalRow));

        statusRange = reportSheet.Range(sprintf('%s%d:%s%d', ...
            statusColumnLetter, firstRow, statusColumnLetter, finalRow));

        if statusValues(startIndex) == "PASS"

            rowRange.Interior.Color = excelRgb(226,239,218);

        elseif statusValues(startIndex) == "FAIL"

            rowRange.Interior.Color = excelRgb(255,199,206);
            statusRange.Font.Bold = true;

        elseif statusValues(startIndex) == "ERROR"

            rowRange.Interior.Color = excelRgb(255,235,156);
            statusRange.Font.Bold = true;

        end

        startIndex = endIndex + 1;

    end

    summarySheet.Range('A1:B1').Merge;
    summarySheet.Range('A1').Font.Bold = true;
    summarySheet.Range('A1').Font.Size = 16;
    summarySheet.Range('A1:B1').Interior.Color = excelRgb(31,78,121);
    summarySheet.Range('A1').Font.Color = excelRgb(255,255,255);

    summarySheet.Columns.AutoFit;
    configSheet.Columns.AutoFit;

    book.Save();
    book.Close(false);

catch ME

    warning('ADCS:FSMTest:ExcelFormatting', ...
        'Report data was written, but Excel highlighting was not applied: %s', ME.message);

end

end

function styleHeader(range)

range.Font.Bold = true;
range.Font.Color = excelRgb(255,255,255);
range.Interior.Color = excelRgb(31,78,121);
range.HorizontalAlignment = -4108; % xlCenter

end

function color = excelRgb(r,g,b)

color = r + 256*g + 65536*b;

end

function column = excelColumn(number)

column = '';

while number > 0

    rem = mod(number-1,26);
    column = [char(65+rem) column]; %#ok<AGROW>
    number = floor((number-1)/26);

end

end

function quitExcel(excel)

try
    excel.Quit();
catch
end

end

function message = compactException(ME)

message = string(ME.message);

for k = 1:numel(ME.cause)
    message = message + newline + "Caused by: " + string(ME.cause{k}.message);
end

end

function value = ternary(condition, whenTrue, whenFalse)

if condition
    value = whenTrue;
else
    value = whenFalse;
end

end

function closeModelWithoutSaving(modelName)

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end

end