function qi = ads_qinv(q)
%ADS_QINV Quaternion inverse (conjugate of a unit quaternion). Eq. (2.6).
%#codegen
qi = [-q(1); -q(2); -q(3); q(4)];
end
