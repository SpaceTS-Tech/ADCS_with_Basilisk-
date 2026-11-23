function ads_define_buses()
%ADS_DEFINE_BUSES Create ADS_RefBus, ADS_MeasBus, ADS_CondBus, ADS_EstBus in the base workspace.
[ref, meas, cond, est] = ads_bus_templates();
assignin('base', 'ADS_RefBus', makeBus(ref));
assignin('base', 'ADS_MeasBus', makeBus(meas));
assignin('base', 'ADS_CondBus', makeBus(cond));
assignin('base', 'ADS_EstBus', makeBus(est));
end

function bus = makeBus(s)
names = fieldnames(s);
elems = Simulink.BusElement.empty;
for k = 1:numel(names)
    v = s.(names{k});
    e = Simulink.BusElement;
    e.Name = names{k};
    e.Dimensions = size(v);
    if islogical(v)
        e.DataType = 'boolean';
    else
        e.DataType = class(v);
    end
    elems(end+1) = e; %#ok<AGROW>
end
bus = Simulink.Bus;
bus.Elements = elems;
end
