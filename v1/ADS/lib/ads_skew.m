function S = ads_skew(a)
%ADS_SKEW Cross-product matrix [a x], design document Eq. (2.1).
%#codegen
S = [0, -a(3), a(2); a(3), 0, -a(1); -a(2), a(1), 0];
end
