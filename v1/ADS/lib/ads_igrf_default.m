function G = ads_igrf_default()
%ADS_IGRF_DEFAULT Placeholder IGRF coefficient set (degree 2, epoch 2020.0).
% Load the full official table with ads_load_igrf('igrf14coeffs.txt') before
% flight use (ground command ADS_LOAD_IGRF, Table 7). Arrays are indexed (n+1, m+1)
% and sized for degree 13. Units: nT and nT/year.
G.N = 2;
G.epoch = 2020.0;
G.g = zeros(14); G.h = zeros(14); G.gd = zeros(14); G.hd = zeros(14);
G.g(2,1) = -29404.8;  G.gd(2,1) = 5.7;
G.g(2,2) = -1450.9;   G.gd(2,2) = 7.4;   G.h(2,2) = 4652.5;  G.hd(2,2) = -25.9;
G.g(3,1) = -2499.6;   G.gd(3,1) = -11.0;
G.g(3,2) = 2982.0;    G.gd(3,2) = -7.0;  G.h(3,2) = -2991.6; G.hd(3,2) = -30.2;
G.g(3,3) = 1677.0;    G.gd(3,3) = -2.1;  G.h(3,3) = -734.6;  G.hd(3,3) = -22.4;
end
