function W = ads_usque_weights(P)
%ADS_USQUE_WEIGHTS Unscented weights for n = 6, Eq. (5.20).
%#codegen
n = 6;
lam = P.lambda;
W = [lam/(n + lam); repmat(1/(2*(n + lam)), 2*n, 1)];
end
