%% 1. Define Model and Input Data
modelName = 'ADCS_FSM_v1_test';
parameters;
P = satellite_params();
t = (0:0.1:10)';

input = [1	1	0	1	1	1	1	1	1	1	1	1	1];

%% 2. Create Dataset for 16 root-level Inports
ds = Simulink.SimulationData.Dataset;

for i = 1:13
    if i == 1
        sig = timeseries(uint8(input(i)*ones(size(t))), t);
    else
        sig = timeseries(boolean(input(i)*ones(size(t))), t);
    end
    sig.Name = ['u' num2str(i)];   % optional but useful

    ds = ds.addElement(sig);
end

%% 3. Configure Simulation
simIn = Simulink.SimulationInput(modelName);
simIn = simIn.setExternalInput(ds);

%% 4. Run Simulation
out = sim(simIn);

%% 5. Extract 8 outputs
y_out = zeros(1,8);

for i = 1:8
    y_out(i) = out.yout{i}.Values.Data(end);
end

y_out