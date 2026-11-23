function B_E = ads_igrf(r_E, year, G)
%ADS_IGRF Geomagnetic field in ECEF [T], design document Eqs. (3.20)-(3.29).
% r_E: ECEF position [km]; year: decimal year; G: coefficient struct.
%#codegen
a = 6371.2;
r = norm(r_E);
th = acos(min(max(r_E(3)/r, -1), 1));
ph = atan2(r_E(2), r_E(1));
th = min(max(th, 1e-6), pi - 1e-6);       % pole guard (Section 3.7.3)
c = cos(th);  s = sin(th);
Pn = zeros(14);  dP = zeros(14);
Pn(1,1) = 1;
N = min(G.N, 13);
for n = 1:N
    for m = 0:n
        if n == m
            if n == 1
                Pn(2,2) = s;  dP(2,2) = c;
            else
                k = sqrt(1 - 1/(2*n));
                Pn(n+1,n+1) = k*s*Pn(n,n);
                dP(n+1,n+1) = k*(c*Pn(n,n) + s*dP(n,n));
            end
        else
            t2 = 0;  d2 = 0;
            if n - 2 >= m
                kk = sqrt((n-1)^2 - m^2);
                t2 = kk*Pn(n-1,m+1);
                d2 = kk*dP(n-1,m+1);
            end
            den = sqrt(n^2 - m^2);
            Pn(n+1,m+1) = ((2*n-1)*c*Pn(n,m+1) - t2)/den;
            dP(n+1,m+1) = ((2*n-1)*(c*dP(n,m+1) - s*Pn(n,m+1)) - d2)/den;
        end
    end
end
dt = year - G.epoch;
Br = 0;  Bt = 0;  Bp = 0;
for n = 1:N
    ar = (a/r)^(n+2);
    for m = 0:n
        g = G.g(n+1,m+1) + G.gd(n+1,m+1)*dt;
        h = G.h(n+1,m+1) + G.hd(n+1,m+1)*dt;
        cm = cos(m*ph);  sm = sin(m*ph);
        Br = Br + ar*(n+1)*(g*cm + h*sm)*Pn(n+1,m+1);
        Bt = Bt - ar*(g*cm + h*sm)*dP(n+1,m+1);
        Bp = Bp - ar*m*(-g*sm + h*cm)*Pn(n+1,m+1)/s;
    end
end
B_E = 1e-9*[(Br*s + Bt*c)*cos(ph) - Bp*sin(ph); ...
            (Br*s + Bt*c)*sin(ph) + Bp*cos(ph); ...
            Br*c - Bt*s];
end
