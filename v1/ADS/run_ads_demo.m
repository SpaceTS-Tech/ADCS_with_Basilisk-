%RUN_ADS_DEMO Closed-loop check of ADS_v1: mode sequence and estimation accuracy.
% Generates the truth/sensor scenario (tests/ads_demo_scenario.m), simulates
% ADS_v1.slx and prints the mode transitions; plots are saved to tests/ads_demo.png.
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, 'tests'));
evalin('base', sprintf('clear ADS_P; run(''%s'');', fullfile(here, 'ads_setup.m')));
P = evalin('base', 'ADS_P');
if ~isfile(fullfile(here, 'ADS_v1.slx'))
    build_ads_model();
end
T = 3600;
fprintf('Generating scenario...\n');
[ds, truth] = ads_demo_scenario(P, T);

load_system(fullfile(here, 'ADS_v1.slx'));
in = Simulink.SimulationInput('ADS_v1');
in = in.setExternalInput(ds);
in = in.setModelParameter('StopTime', num2str(T));
fprintf('Simulating %g s...\n', T);
tic;
out = sim(in);
fprintf('Simulation time %.1f s\n', toc);

y = out.yout;
portNames = cell(y.numElements, 1);
for k = 1:y.numElements
    [~, portNames{k}] = fileparts(y{k}.BlockPath.getBlock(1));
end
get1 = @(n) squeeze(y{find(strcmp(portNames, n), 1)}.Values.Data);
tt = y{find(strcmp(portNames, 'ads_mode'), 1)}.Values.Time;
mode = double(get1('ads_mode'));
trn = double(get1('ads_last_transition'));
fault = logical(get1('ads_fault'));
qe = get1('q_BI');
if size(qe, 1) == 4 && size(qe, 2) ~= 4
    qe = qe.';
end
sig = get1('sig_att');
if size(sig, 1) == 3 && size(sig, 2) ~= 3
    sig = sig.';
end
names = {'IDLE', 'RATE', 'COARSE', 'FINE_SUN', 'FINE_STELLAR', 'GYROLESS'};

%% Mode transition log
fprintf('\n   time [s]   transition                         T#   ads_fault\n');
fprintf('   %8.1f   start in %s\n', tt(1), names{mode(1)+1});
for k = 2:numel(mode)
    if mode(k) ~= mode(k-1)
        fprintf('   %8.1f   %-14s -> %-16s  T%-3d %d\n', tt(k), names{mode(k-1)+1}, names{mode(k)+1}, trn(k), fault(k));
    end
end

%% Attitude error versus truth
qt = interp1(truth.t, truth.q, tt, 'previous');
err = nan(numel(tt), 1);
for k = 1:numel(tt)
    if mode(k) >= 2
        dq = ads_qmult(qe(k,:).', ads_qinv(qt(k,:).'));
        err(k) = 2*acosd(min(abs(dq(4)), 1));
    end
end
fprintf('\nAttitude error (truth vs estimate), last 60 s of each mode segment:\n');
seg = [1; find(diff(mode) ~= 0) + 1; numel(mode) + 1];
for s = 1:numel(seg) - 1
    idx = seg(s):seg(s+1) - 1;
    m = mode(idx(1));
    if m >= 2
        tail = idx(tt(idx) >= tt(idx(end)) - 60);
        fprintf('   %-13s %7.1f-%7.1f s : error %8.4f deg, 3-sigma %8.4f deg\n', names{m+1}, ...
            tt(idx(1)), tt(idx(end)), mean(err(tail)), mean(max(sig(tail,:), [], 2))*180/pi);
    end
end

%% Plots
fig = figure('Visible', 'off', 'Position', [100 100 1100 800]);
subplot(3,1,1);
stairs(tt, mode, 'LineWidth', 1.5); hold on;
area(truth.t, 5.6*truth.eclipse, 'FaceColor', [0.85 0.85 0.85], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
stairs(tt, mode, 'b', 'LineWidth', 1.5);
yticks(0:5); yticklabels(names); ylim([-0.3 5.6]); grid on;
set(gca, 'TickLabelInterpreter', 'none');
title('ADS mode (grey: eclipse)'); xlabel('t [s]');
subplot(3,1,2);
semilogy(tt, err, 'LineWidth', 1.2); hold on;
semilogy(tt, max(sig, [], 2)*180/pi, '--', 'LineWidth', 1.0);
grid on; legend('attitude error', '3\sigma (max axis)'); ylabel('deg'); xlabel('t [s]');
title('Attitude estimation error');
subplot(3,1,3);
wB = get1('w_B');
if size(wB, 1) == 3 && size(wB, 2) ~= 3
    wB = wB.';
end
plot(tt, vecnorm(wB, 2, 2)*180/pi, truth.t, vecnorm(truth.w, 2, 2)*180/pi, '--');
grid on; legend('|\omega| estimate', '|\omega| truth'); ylabel('deg/s'); xlabel('t [s]');
title('Body rate magnitude');
exportgraphics(fig, fullfile(here, 'tests', 'ads_demo.png'), 'Resolution', 110);
fprintf('\nPlot saved to %s\n', fullfile(here, 'tests', 'ads_demo.png'));
