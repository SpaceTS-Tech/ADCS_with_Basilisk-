function G = ads_load_igrf(fileName, Nmax)
%ADS_LOAD_IGRF Read an official IAGA IGRF coefficient file (igrfNNcoeffs.txt).
% Uses the last main-field epoch column and the secular-variation column.
%   ADS_P.igrf = ads_load_igrf('igrf14coeffs.txt', 10);
if nargin < 2
    Nmax = 13;
end
G.N = Nmax;
G.epoch = NaN;
G.g = zeros(14); G.h = zeros(14); G.gd = zeros(14); G.hd = zeros(14);
lines = readlines(fileName);
for k = 1:numel(lines)
    t = strtrim(lines(k));
    if startsWith(t, "g/h")
        tok = split(t);
        G.epoch = str2double(tok(end-1));
    elseif startsWith(t, "g ") || startsWith(t, "h ")
        tok = split(t);
        n = str2double(tok(2));
        m = str2double(tok(3));
        if n > Nmax
            continue
        end
        val = str2double(tok(end-1));
        sv = str2double(tok(end));
        if tok(1) == "g"
            G.g(n+1, m+1) = val;  G.gd(n+1, m+1) = sv;
        else
            G.h(n+1, m+1) = val;  G.hd(n+1, m+1) = sv;
        end
    end
end
if isnan(G.epoch)
    error('ads_load_igrf: no "g/h" header line found in %s', fileName);
end
end
