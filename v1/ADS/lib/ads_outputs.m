function [q_BI, q_BO, w_B, bias, sig_att, s_B, sun_valid, B_B, Bdot_B, ...
    att_valid, rate_valid, fine_allowed, health, detumbled] = ads_outputs(mode, ads_fault, est, meas, ref, P)
%ADS_OUTPUTS ADS outputs and status flags (Table 9).
%#codegen
q_BI = est.q;
q_BO = [0;0;0;1];
if ref.valid
    q_BO = ads_A2q(ads_q2A(est.q)*ref.A_OI.');
end
w_B = est.w;
bias = est.bias;
sig_att = est.sig_att;
s_B = meas.s_sun;
sun_valid = meas.sun_valid;
B_B = meas.B_B;
Bdot_B = est.Bdot;
att_valid = mode >= uint8(2) && est.mode == mode && est.att_init && ~est.div && est.sig_max < P.th_valid;
rate_valid = mode ~= uint8(0) && est.rate_ok;
fine_allowed = att_valid && est.sig_max < P.th_fine && ...
    (mode == uint8(4) || (mode == uint8(3) && P.fine_sun_fine_ok));
health = bitor(meas.health, bitshift(est.health, 8));
health = bitor(health, bitshift(uint32(ads_fault), 16));
health = bitor(health, bitshift(uint32(mode), 24));
detumbled = est.detumbled;
end
