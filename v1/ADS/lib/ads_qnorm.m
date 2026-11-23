function q = ads_qnorm(q)
%ADS_QNORM Normalise a quaternion; returns identity for a zero/invalid input.
%#codegen
q = q(:);
n = norm(q);
if n > 1e-12 && isfinite(n)
    q = q / n;
else
    q = [0; 0; 0; 1];
end
end
