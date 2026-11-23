%ADS_SETUP Add the ADS library to the path and create ADS_P and the bus objects.
% Called automatically by ADS_v1.slx (PreLoadFcn/InitFcn).
ads_root_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(ads_root_dir, 'lib'));
if exist('satellite_params', 'file') ~= 2
    addpath(fileparts(ads_root_dir));            % ../satellite_params.m (inertia)
end
if ~exist('ADS_P', 'var')
    ADS_P = ads_params();
end
ads_define_buses();
clear ads_root_dir
