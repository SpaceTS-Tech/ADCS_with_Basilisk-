% Compile ADS_v1 and write all diagnostic-viewer messages to tests/diag_log.txt.
here = fileparts(fileparts(mfilename('fullpath')));
cd(here);
evalin('base', 'clear ADS_P; ads_setup');
load_system('ADS_v1');
logFile = fullfile(here, 'tests', 'diag_log.txt');
if isfile(logFile)
    delete(logFile);
end
sldiagviewer.diary(logFile, 'UTF-8');
try
    set_param('ADS_v1', 'SimulationCommand', 'update');
    disp('COMPILE OK');
catch ME
    disp(ME.message);
end
sldiagviewer.diary('off');
