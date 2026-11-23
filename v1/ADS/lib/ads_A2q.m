function q = ads_A2q(A)
%ADS_A2Q Quaternion from attitude matrix with Shepperd's method (Table 3).
%#codegen
tr = A(1,1) + A(2,2) + A(3,3);
[~, k] = max([tr, A(1,1), A(2,2), A(3,3)]);
q = zeros(4, 1);
switch k
    case 1
        q(4) = 0.5*sqrt(max(0, 1 + tr));
        q(1) = (A(2,3) - A(3,2)) / (4*q(4));
        q(2) = (A(3,1) - A(1,3)) / (4*q(4));
        q(3) = (A(1,2) - A(2,1)) / (4*q(4));
    case 2
        q(1) = 0.5*sqrt(max(0, 1 + 2*A(1,1) - tr));
        q(2) = (A(1,2) + A(2,1)) / (4*q(1));
        q(3) = (A(1,3) + A(3,1)) / (4*q(1));
        q(4) = (A(2,3) - A(3,2)) / (4*q(1));
    case 3
        q(2) = 0.5*sqrt(max(0, 1 + 2*A(2,2) - tr));
        q(1) = (A(1,2) + A(2,1)) / (4*q(2));
        q(3) = (A(2,3) + A(3,2)) / (4*q(2));
        q(4) = (A(3,1) - A(1,3)) / (4*q(2));
    otherwise
        q(3) = 0.5*sqrt(max(0, 1 + 2*A(3,3) - tr));
        q(1) = (A(1,3) + A(3,1)) / (4*q(3));
        q(2) = (A(2,3) + A(3,2)) / (4*q(3));
        q(4) = (A(1,2) - A(2,1)) / (4*q(3));
end
q = ads_qnorm(q);
if q(4) < 0
    q = -q;
end
end
