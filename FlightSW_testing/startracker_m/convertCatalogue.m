function convertCatalogue()

binFile = 'catalogue.bin';

cat = readCatalogue(binFile);
cat = buildIndex(cat);

save('catalogue.mat', 'cat', '-v7.3');

end

%% functions
function cam = cameraFromHeader(headerJson)
%CAMERAFROMHEADER  Camera parameters from the catalogue's JSON header.
%
%   The catalogue and the camera belong together -- the pair table only holds
%   pairs closer than that camera's frame diagonal -- so the parameters travel
%   inside the file rather than being set here.  The defaults are the shipped
%   camera, used if the header is unreadable.

cam = struct('widthPx', 1024, 'heightPx', 1024, ...
    'focalLengthPx', 2903.696291644267, ...
    'principalPointPx', [512.0, 512.0], ...
    'detectionSigma', 5.0, 'centroidSigmaPx', 0.1357);

try
    header = jsondecode(headerJson);
catch
    return;                      % defaults: the shipped camera
end
if ~isfield(header, 'camera')
    return;
end

raw = header.camera;
names = {'widthPx', 'width_px'; 'heightPx', 'height_px'; ...
    'focalLengthPx', 'focal_length_px'; ...
    'principalPointPx', 'principal_point_px'; ...
    'detectionSigma', 'detection_sigma'; ...
    'centroidSigmaPx', 'centroid_sigma_px'};
for i = 1:size(names, 1)
    if isfield(raw, names{i, 2})
        cam.(names{i, 1}) = reshape(raw.(names{i, 2}), 1, []);
    end
end
end

function cat = readCatalogue(path)
%READCATALOGUE  Parse the binary catalogue.  Layout, all little-endian:
%
%   magic     8 bytes   "STARCAT1", or "STARCAT2" when region tables follow
%   n_stars   uint32
%   n_pairs   uint32
%   max_sep   float64   frame diagonal in radians, the pair table's extent
%   hdr_len   uint32
%   header    JSON      provenance and camera parameters
%   ids       int32  [n_stars]
%   vectors   float64[n_stars*3]  unit vectors, J2000 equatorial
%   mags      float32[n_stars]
%   pair_a    int32  [n_pairs]
%   pair_b    int32  [n_pairs]
%   pair_sep  float32[n_pairs]    ascending
%
%   STARCAT2 only: n_regions uint32, radius float64, centres, and two
%   compressed-sparse-row tables (adjacency, membership).

fid = fopen(path, 'r', 'ieee-le');
if fid < 0
    error('startracker:catalogue', 'cannot open %s', path);
end
closeFile = onCleanup(@() fclose(fid));

magic = fread(fid, 8, '*char')';
if ~any(strcmp(magic, {'STARCAT1', 'STARCAT2'}))
    error('startracker:catalogue', '%s is not a star catalogue file', path);
end

cat.nStars = fread(fid, 1, 'uint32=>double');
cat.nPairs = fread(fid, 1, 'uint32=>double');
cat.maxSeparationRad = fread(fid, 1, 'float64');

headerLength = fread(fid, 1, 'uint32=>double');
headerJson = fread(fid, headerLength, '*char')';

cat.starId = fread(fid, cat.nStars, 'int32=>double');
cat.vectors = reshape(fread(fid, cat.nStars * 3, 'float64'), 3, cat.nStars)';
cat.magnitude = fread(fid, cat.nStars, 'float32=>double');

% +1 throughout: the file stores 0-based star indices, MATLAB counts from 1.
cat.pairA = fread(fid, cat.nPairs, 'int32=>double') + 1;
cat.pairB = fread(fid, cat.nPairs, 'int32=>double') + 1;
cat.pairSep = fread(fid, cat.nPairs, 'float32=>double');

cat.camera = cameraFromHeader(headerJson);

cat.hasRegions = strcmp(magic, 'STARCAT2');
if cat.hasRegions
    nRegions = fread(fid, 1, 'uint32=>double');
    radiusRad = fread(fid, 1, 'float64');
    cat.nRegions = nRegions;
    cat.regionCosRadius = cos(radiusRad);
    cat.regionCentres = reshape(fread(fid, nRegions * 3, 'float64'), 3, nRegions)';

    adjStart = fread(fid, nRegions + 1, 'int32=>double');
    adjId = fread(fid, adjStart(end), 'int32=>double') + 1;
    memStart = fread(fid, nRegions + 1, 'int32=>double');
    memStar = fread(fid, memStart(end), 'int32=>double') + 1;

    cat.regionAdjacent = splitRows(adjStart, adjId);
    cat.regionMembers = splitRows(memStart, memStar);
else
    cat.nRegions = 0;
    cat.regionCentres = zeros(0, 3);
    cat.regionAdjacent = {};
    cat.regionMembers = {};
    cat.regionCosRadius = 1;
end
end

function rows = splitRows(offsets, flat)
%SPLITROWS  Compressed sparse rows to a cell array of index vectors.
n = numel(offsets) - 1;
rows = cell(n, 1);
for i = 1:n
    rows{i} = flat(offsets(i) + 1 : offsets(i + 1))';
end
end

function cat = buildIndex(cat)
%BUILDINDEX  The k-vector over the pair table, and the per-star neighbour lists.
%
%   Both are derived from the pair table, so they are built at load rather
%   than shipped: recomputing costs a moment, storing them would grow the file.

% ---- Mortari's k-vector: constant-time range search over the pair table ----
% A straight line is fitted through the sorted separations end to end;
% kvector(j) counts how many fall at or below the line's value at j, so a query
% interval maps to an index interval by two divisions instead of two binary
% searches.  The span returned is a superset, filtered exactly by the caller.
n = cat.nPairs;
if n < 2
    cat.kSlope = 1;
    cat.kIntercept = 0;
    cat.kVector = zeros(max(n, 2), 1);
else
    low = cat.pairSep(1);
    high = cat.pairSep(end);
    margin = max((high - low) * 1e-9, 1e-300);
    cat.kSlope = (high - low + 2 * margin) / (n - 1);
    cat.kIntercept = low - margin - cat.kSlope;

    % One merge pass: both the thresholds and the separations ascend, so the
    % scan pointer only ever moves forward.
    kv = zeros(n, 1);
    scan = 0;
    for j = 1:n
        threshold = cat.kSlope * j + cat.kIntercept;
        while scan < n && cat.pairSep(scan + 1) <= threshold
            scan = scan + 1;
        end
        kv(j) = scan;
    end
    cat.kVector = kv;
end

% ---- per-star neighbour lists, sorted by separation ----
% Each pair is stored once, so it is read from both ends.
starColumn = [cat.pairA; cat.pairB];
otherColumn = [cat.pairB; cat.pairA];
sepColumn = [cat.pairSep; cat.pairSep];

sorted = sortrows([starColumn, sepColumn, otherColumn], [1 2]);
cat.neighSep = sorted(:, 2);
cat.neighId = sorted(:, 3);

counts = accumarray(starColumn, 1, [cat.nStars, 1]);
cat.neighStart = [0; cumsum(counts)];      % 0-based offsets into neighSep

% ---- which regions each star is in; derived, used to seed the region walk ----
if cat.hasRegions
    starRegions = cell(cat.nStars, 1);
    for region = 1:cat.nRegions
        for star = cat.regionMembers{region}
            starRegions{star}(end + 1) = region; %#ok<AGROW>
        end
    end
    cat.starRegions = starRegions;
else
    cat.starRegions = cell(cat.nStars, 1);
end
end
