function dq = ads_grp2dq(dp, a, f)
%ADS_GRP2DQ Generalised Rodrigues parameters -> error quaternion, Eq. (2.15).
%#codegen
n2 = dp(:).'*dp(:);
dq4 = (-a*n2 + f*sqrt(max(0, f^2 + (1 - a^2)*n2))) / (f^2 + n2);
dq = [(a + dq4)/f*dp(:); dq4];
end
