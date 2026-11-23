function dp = ads_dq2grp(dq, a, f)
%ADS_DQ2GRP Error quaternion -> generalised Rodrigues parameters, Eq. (2.14).
% The quaternion sign is normalised first so that the shortest rotation is used.
%#codegen
if dq(4) < 0
    dq = -dq;
end
dp = f*dq(1:3)/(a + dq(4));
dp = dp(:);
end
