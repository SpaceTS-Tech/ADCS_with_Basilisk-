function [quaternion, attitude_deg, valid, info] = startracker(image, use_previous, previous_quaternion)
%STARTRACKER  Star image in, attitude quaternion out.
%
%   q = STARTRACKER(image)
%   q = STARTRACKER(image, use_previous, previous_quaternion)
%   [q, attitude_deg, valid, info] = STARTRACKER(...)
%
%   Inputs, the same three the Simulink C Function block takes:
%
%     image                a 1024x1024 uint8 frame, or a path to one
%     use_previous         0 = solve lost in space, input 3 ignored
%                          1 = solve in tracking mode around input 3
%     previous_quaternion  [w x y z], scalar-first, the previous attitude
%
%   Outputs:
%
%     q             1x4 [w x y z], scalar-first, describing [BN] -- the
%                   rotation carrying inertial components into body
%                   components.  NaN(1,4) when the frame was not solved.
%     attitude_deg  1x3 boresight right ascension, declination, roll
%     valid         true when q is a fix
%     info          struct: mode, nDetections, nMatched, nSearched,
%                   residualArcsec, seconds
%
%   VALID = FALSE IS AN ANSWER.  The tracker refuses rather than return a
%   low-confidence guess: a wrong quaternion is worse than no quaternion.
%   Hold the last good attitude, or coast on gyros, when it is false.
%
%   Mode.  With use_previous = 1 the search is restricted to the sky regions
%   around the previous attitude.  If that finds nothing it falls back to a
%   full lost-in-space search on the same frame, so a stale previous
%   quaternion costs time, never a solution.
%
%   The pipeline:
%
%     detect -> take the brightest 15 -> project to body vectors -> angular
%     invariants -> pyramid identification -> grow the match -> QUEST
%
%   Needs catalogue.bin beside this file: 1102 stars, 35178 star pairs and
%   264 sky regions, for the 1024x1024 camera it was validated on.  It is
%   read and indexed on the first call and cached after that, so the first
%   call costs about a second and later ones far less.
%
%   Base MATLAB only -- no toolboxes.
%
%   Example:
%       q = startracker('frame_0000.png');          % lost in space
%       q2 = startracker('frame_0001.png', 1, q);   % tracking from it
%
%   Ported from startracker_lib_v2 (Python) and startracker_c.  Same
%   algorithms, same catalogue, same conventions.

if nargin < 2 || isempty(use_previous)
    use_previous = 0;
end
if nargin < 3
    previous_quaternion = [];
end

started = tic;

quaternion = NaN(1, 4);
attitude_deg = NaN(1, 3);
valid = false;
info = struct('mode', 'lost-in-space', 'nDetections', 0, 'nConsidered', 0, ...
              'nMatched', 0, 'nSearched', 0, 'residualArcsec', NaN, ...
              'reason', '', 'fallbackReason', '', 'regionIds', [], ...
              'seconds', 0);

cat = catalogue();
cam = cat.camera;

% Settings, matching the Python library's defaults.
kDetections = 15;        % brightest detections handed to identification
minMatched = 4;          % fewest correspondences worth a quaternion
maxTriads = 400;         % triad budget before giving up
expandRadiusPx = 3.0;    % match growth radius
trackRings = 1;          % how far the tracking region walk spreads

% Two centroids contribute to an inter-star angle, hence the sqrt(2); the
% safety factor covers PSF asymmetry, saturation and calibration residue.
tolerance = 3.5 * sqrt(2) * cam.centroidSigmaPx / cam.focalLengthPx;

% ---------------------------------------------------------------- detect ---
frame = readFrame(image);
[u, v, flux] = detectStars(frame, cam.detectionSigma);
info.nDetections = numel(u);

if numel(u) < minMatched
    info.reason = 'too few detections';
    info.seconds = toc(started);
    return;
end

% Brightness decides which detections are worked with and nothing else.  It
% never decides which star a detection is: two stars of similar magnitude are
% not distinguishable by brightness.
[~, order] = sort(flux, 'descend');
order = order(1:min(kDetections, numel(order)));
body = pixelToCamera(cam, u(order), v(order));
info.nConsidered = size(body, 1);

% -------------------------------------------------------------- identify ---
% Tracking searches only the stars of the regions around the previous
% boresight.  The subset is a mask over the shared index: star indices stay
% those of the full catalogue, nothing is copied or re-sorted.
match = [];
priorRegions = [];
if use_previous ~= 0
    if isempty(previous_quaternion) || numel(previous_quaternion) ~= 4
        info.fallbackReason = 'previous quaternion missing or malformed';
    else
        previousDcm = quatToDcm(previous_quaternion);
        priorRegions = regionsContaining(cat, previousDcm(3, :), []);
    end
end

if ~isempty(priorRegions)
    searchRegions = regionNeighbourhood(cat, priorRegions, trackRings);
    mask = subsetMask(cat, searchRegions);
    match = identifyPyramid(body, cat, tolerance, maxTriads, mask);
    if match.success
        info.mode = 'tracking';
        info.nSearched = sum(mask);
    else
        info.fallbackReason = sprintf('tracking found no match (%s)', match.reason);
        match = [];
    end
end

if isempty(match)
    mask = [];                                  % empty mask = whole catalogue
    match = identifyPyramid(body, cat, tolerance, maxTriads, mask);
    info.mode = 'lost-in-space';
    info.nSearched = cat.nStars;
end

info.reason = match.reason;
if ~match.success
    info.seconds = toc(started);
    return;
end

% ---------------------------------------------------------------- attitude -
% Provisional attitude from the confirmed pyramid, then grow the match set so
% QUEST is not estimating three axes from four stars.
provisional = quest(body(match.detectionIndex, :), ...
                    cat.vectors(match.catalogueIndex, :));

candidates = [];
if strcmp(info.mode, 'tracking')
    regionIds = regionsContaining(cat, provisional(3, :), priorRegions);
    candidates = find(subsetMask(cat, regionNeighbourhood(cat, regionIds, 1)));
end

expandTolerance = expandRadiusPx / cam.focalLengthPx;
[detectionIndex, catalogueIndex] = expandMatches( ...
    provisional, body, cat, expandTolerance, candidates);

if numel(detectionIndex) < minMatched
    detectionIndex = match.detectionIndex;
    catalogueIndex = match.catalogueIndex;
end

matchedBody = body(detectionIndex, :);
matchedInertial = cat.vectors(catalogueIndex, :);

q = questQuaternion(matchedBody, matchedInertial);
dcm = quatToDcm(q);

[ra, dec, roll] = dcmToRadecRoll(dcm);
residuals = residualAngles(dcm, matchedBody, matchedInertial);

quaternion = q;
attitude_deg = [ra, dec, roll];
valid = true;
info.nMatched = numel(detectionIndex);
info.residualArcsec = mean(residuals) * 3600;
info.regionIds = regionsContaining(cat, dcm(3, :), priorRegions);
info.seconds = toc(started);
end


%% ======================================================== catalogue =======

function cat = catalogue()
%CATALOGUE  Read catalogue.bin once and keep it, with its index, in memory.
%
%   Loading and indexing costs about a second; a solve costs far less, so the
%   cost has to be paid once rather than per frame.  Clear it with
%   `clear functions` if you swap the file.

persistent cached
if ~isempty(cached)
    cat = cached;
    return;
end

here = fileparts(mfilename('fullpath'));
path = fullfile(here, 'catalogue.bin');
if exist(path, 'file') ~= 2
    error('startracker:catalogue', ...
          ['catalogue.bin is missing from %s. Copy it from ' ...
           'startracker_lib_v2/startracker/data/catalogue.bin.'], here);
end

cat = readCatalogue(path);
cat = buildIndex(cat);
cached = cat;
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


%% ======================================================== index queries ===

function [a, b] = queryPairs(cat, separation, tolerance, mask)
%QUERYPAIRS  Catalogue pairs whose separation is within tolerance.
%
%   MASK, when given, is a logical vector over the stars: only pairs with both
%   ends inside it are returned.  That is the whole of tracking's subset --
%   the index itself is shared, never rebuilt.

low = separation - tolerance;
high = separation + tolerance;
[start, stop] = pairSpan(cat, low, high);
if stop <= start
    a = zeros(0, 1);
    b = zeros(0, 1);
    return;
end

slice = (start + 1 : stop)';
keep = cat.pairSep(slice) >= low & cat.pairSep(slice) <= high;
slice = slice(keep);

a = cat.pairA(slice);
b = cat.pairB(slice);

if ~isempty(mask)
    inside = mask(a) & mask(b);
    a = a(inside);
    b = b(inside);
end
end


function [start, stop] = pairSpan(cat, low, high)
%PAIRSPAN  Index range of the pair table that may hold separations in [low, high].
n = cat.nPairs;
if n < 2 || high < cat.pairSep(1) || low > cat.pairSep(end)
    start = 0;
    stop = 0;
    return;
end

jLow = min(max(floor((low - cat.kIntercept) / cat.kSlope), 1), n);
jHigh = min(max(ceil((high - cat.kIntercept) / cat.kSlope), 1), n);
start = cat.kVector(jLow);
stop = cat.kVector(jHigh);
end


function out = neighbours(cat, star, separation, tolerance, mask)
%NEIGHBOURS  Catalogue stars lying at SEPARATION +/- TOLERANCE from STAR.
first = cat.neighStart(star) + 1;
last = cat.neighStart(star + 1);
if last < first
    out = zeros(1, 0);
    return;
end

span = first:last;
seps = cat.neighSep(span);
keep = seps >= separation - tolerance & seps <= separation + tolerance;
out = cat.neighId(span(keep))';

if ~isempty(mask) && ~isempty(out)
    out = out(mask(out));
end
end


%% ======================================================== regions =========

function found = regionsContaining(cat, direction, start)
%REGIONSCONTAINING  Every region whose cap contains DIRECTION, as sorted IDs.
%
%   With START -- region IDs from the previous fix -- the search walks the
%   adjacency graph toward the direction and tests a dozen centres instead of
%   all 264.  The answer is exact either way; a walk that stalls falls back to
%   the full scan.
%
%   Two regions are adjacent when their centres are at most two radii apart,
%   which is what makes the walk exact: every region containing a direction
%   lies within one radius of it, hence within two radii of any other region
%   containing it, hence in that region's adjacency list.

direction = direction(:)';
if ~cat.hasRegions
    found = [];
    return;
end

centres = cat.regionCentres;
cosRadius = cat.regionCosRadius;

if ~isempty(start)
    dots = centres(start, :) * direction';
    [best, at] = max(dots);
    current = start(at);

    while best < cosRadius
        adjacent = cat.regionAdjacent{current};
        [value, at] = max(centres(adjacent, :) * direction');
        if value <= best
            break;                     % stalled: fall through to the full scan
        end
        current = adjacent(at);
        best = value;
    end

    if best >= cosRadius
        adjacent = cat.regionAdjacent{current};
        near = adjacent(centres(adjacent, :) * direction' >= cosRadius);
        found = sort([current, near]);
        return;
    end
end

found = find(centres * direction' >= cosRadius)';
end


function selected = regionNeighbourhood(cat, regionIds, rings)
%REGIONNEIGHBOURHOOD  REGIONIDS plus every region within RINGS adjacency steps.
%
%   One ring is the rule "the regions the boresight is in, and the regions next
%   to them" -- where the boresight may have moved to since.

selected = regionIds(:)';
frontier = selected;
for ring = 1:max(0, rings)
    reached = [];
    for region = frontier
        reached = [reached, cat.regionAdjacent{region}]; %#ok<AGROW>
    end
    reached = setdiff(unique(reached), selected);
    if isempty(reached)
        break;
    end
    selected = [selected, reached]; %#ok<AGROW>
    frontier = reached;
end
selected = sort(selected);
end


function mask = subsetMask(cat, regionIds)
%SUBSETMASK  Logical vector of the stars inside a handful of regions.
mask = false(cat.nStars, 1);
for region = regionIds
    mask(cat.regionMembers{region}) = true;
end
end


%% ======================================================== detection ======

function frame = readFrame(image)
%READFRAME  A path, or a matrix, to a 2-D uint8 frame.
if ischar(image) || isstring(image)
    frame = imread(char(image));
else
    frame = image;
end

if ndims(frame) == 3 && size(frame, 3) == 3
    % Rec. 601 luma, so no Image Processing Toolbox is needed.
    frame = 0.299 * double(frame(:, :, 1)) ...
          + 0.587 * double(frame(:, :, 2)) ...
          + 0.114 * double(frame(:, :, 3));
end

if ~ismatrix(frame)
    error('startracker:image', 'image must be a 2-D frame');
end
frame = uint8(frame);
end


function [u, v, flux] = detectStars(frame, thresholdSigma)
%DETECTSTARS  Background, threshold, segmentation and centroiding.
%
%   Centroids are intensity-weighted centres of gravity over each 8-connected
%   component, on background-subtracted weights.  Pixel (row i, column j) has
%   its centre at (u, v) = (j - 0.5, i - 0.5).

minPixels = 2;
[background, noise] = backgroundAndNoise(frame);

threshold = floor(background + thresholdSigma * noise) + 1;
if threshold > 255
    u = []; v = []; flux = [];
    return;
end

bright = find(frame >= threshold);
if isempty(bright)
    u = []; v = []; flux = [];
    return;
end

label = components(size(frame), bright);
nComponents = max(label);
if nComponents == 0
    u = []; v = []; flux = [];
    return;
end

[rows, cols] = ind2sub(size(frame), bright);
weight = double(frame(bright)) - background;

% A pixel exactly at the threshold can carry a non-positive weight once the
% background is taken off; it still belongs to the component, it just does not
% pull the centroid.
total = accumarray(label, weight, [nComponents, 1]);
sumU = accumarray(label, weight .* (cols - 0.5), [nComponents, 1]);
sumV = accumarray(label, weight .* (rows - 0.5), [nComponents, 1]);
count = accumarray(label, 1, [nComponents, 1]);

keep = count >= minPixels & total > 0;
u = sumU(keep) ./ total(keep);
v = sumV(keep) ./ total(keep);
flux = total(keep);
end


function [background, noise] = backgroundAndNoise(frame)
%BACKGROUNDANDNOISE  Robust level and pixel noise from a 256-bin histogram.
%
%   The median is unaffected by the sparse bright pixels of a star field, and
%   the median absolute deviation gives a noise estimate stars cannot inflate.

histogram = accumarray(double(frame(:)) + 1, 1, [256, 1]);
total = numel(frame);
if total == 0
    background = 0;
    noise = 1;
    return;
end

half = floor(total / 2);
background = find(cumsum(histogram) > half, 1) - 1;

% Median absolute deviation, read straight off the same histogram.
values = (0:255)';
deviations = accumarray(abs(values - background) + 1, histogram, [256, 1]);
mad = find(cumsum(deviations) > half, 1) - 1;

background = double(background);
noise = max(1.4826 * mad, 0.5);
end


function label = components(frameSize, bright)
%COMPONENTS  8-connected labelling of the bright pixels, base MATLAB only.
%
%   Only the four forward neighbours are linked: the other four are the same
%   edges seen from the other end, so adding them would change nothing.

height = frameSize(1);
n = numel(bright);

position = zeros(frameSize, 'double');
position(bright) = 1:n;

[rows, cols] = ind2sub(frameSize, bright);

offsets = [1 0; -1 1; 0 1; 1 1];        % down, up-right, right, down-right
from = zeros(0, 1);
to = zeros(0, 1);
for k = 1:size(offsets, 1)
    neighbourRow = rows + offsets(k, 1);
    neighbourCol = cols + offsets(k, 2);
    inside = neighbourRow >= 1 & neighbourRow <= height ...
           & neighbourCol >= 1 & neighbourCol <= frameSize(2);

    source = find(inside);
    if isempty(source)
        continue;
    end
    target = position(sub2ind(frameSize, neighbourRow(source), neighbourCol(source)));
    linked = target > 0;

    from = [from; source(linked)];       %#ok<AGROW>
    to = [to; target(linked)];           %#ok<AGROW>
end

label = conncomp(graph(from, to, [], n))';
end


%% ======================================================== identification ==

function result = identifyPyramid(body, cat, tolerance, maxTriads, mask)
%IDENTIFYPYRAMID  Match a pyramid: a confirmed triangle plus a fourth star.
%
%   Three separations alone cannot tell a triangle from its mirror image,
%   because reflection preserves every angle.  A fourth star that must confirm
%   the same three catalogue stars resolves that and collapses the false-match
%   rate.
%
%   A solution is returned only when the confirmed assignment is unique.  Two
%   different catalogue pyramids fitting the same four detections is ambiguous,
%   and "no solution" is then the correct answer.

result = struct('success', false, 'detectionIndex', [], ...
                'catalogueIndex', [], 'reason', '');

n = size(body, 1);
if n < 4
    result.reason = 'fewer than four detections';
    return;
end

theta = angularSeparations(body);
triads = 0;
order = triadOrder(n);

for t = 1:size(order, 1)
    i = order(t, 1);
    j = order(t, 2);
    k = order(t, 3);

    if triads >= maxTriads
        result.reason = 'triad budget exhausted';
        return;
    end
    triads = triads + 1;

    found = candidateTriangles(cat, theta(i, j), theta(i, k), theta(j, k), ...
                               tolerance, mask);
    if isempty(found)
        continue;
    end

    for r = 1:n
        if r == i || r == j || r == k
            continue;
        end

        confirmed = zeros(0, 4);
        for c = 1:size(found, 1)
            fourth = confirmFourth(cat, found(c, :), ...
                                   [theta(i, r), theta(j, r), theta(k, r)], ...
                                   tolerance, mask);
            for f = fourth
                confirmed(end + 1, :) = [found(c, :), f]; %#ok<AGROW>
                if size(confirmed, 1) > 1
                    break;
                end
            end
            if size(confirmed, 1) > 1
                break;
            end
        end

        if size(confirmed, 1) == 1
            result.success = true;
            result.detectionIndex = [i, j, k, r];
            result.catalogueIndex = confirmed(1, :);
            result.reason = 'confirmed pyramid';
            return;
        end
        % More than one surviving pyramid means this fourth star does not
        % disambiguate; try the next one, then the next triad.
    end
end

result.reason = 'no confirmed pyramid';
end


function found = candidateTriangles(cat, thetaIJ, thetaIK, thetaJK, tolerance, mask)
%CANDIDATETRIANGLES  Catalogue triples whose three separations match the input.
limit = 5000;
found = zeros(0, 3);

[pairsA, pairsB] = queryPairs(cat, thetaIJ, tolerance, mask);

for p = 1:numel(pairsA)
    % The pair table stores each pair once, so both assignments must be tried:
    % detection i may sit at either end of the catalogue pair.
    ends = [pairsA(p), pairsB(p); pairsB(p), pairsA(p)];
    for e = 1:2
        a = ends(e, 1);
        b = ends(e, 2);

        fromA = neighbours(cat, a, thetaIK, tolerance, mask);
        if isempty(fromA)
            continue;
        end
        fromB = neighbours(cat, b, thetaJK, tolerance, mask);
        if isempty(fromB)
            continue;
        end

        for c = intersect(fromA, fromB)
            if c ~= a && c ~= b
                found(end + 1, :) = [a, b, c]; %#ok<AGROW>
                if size(found, 1) >= limit
                    return;
                end
            end
        end
    end
end
end


function out = confirmFourth(cat, triangle, thetaR, tolerance, mask)
%CONFIRMFOURTH  Catalogue stars at the right distance from all three vertices.
out = zeros(1, 0);

fromA = neighbours(cat, triangle(1), thetaR(1), tolerance, mask);
if isempty(fromA)
    return;
end
fromB = neighbours(cat, triangle(2), thetaR(2), tolerance, mask);
if isempty(fromB)
    return;
end
common = intersect(fromA, fromB);
if isempty(common)
    return;
end
fromC = neighbours(cat, triangle(3), thetaR(3), tolerance, mask);
if isempty(fromC)
    return;
end

out = setdiff(intersect(common, fromC), triangle);
out = reshape(out, 1, []);
end


function theta = angularSeparations(body)
%ANGULARSEPARATIONS  Angles between every pair of body vectors, in radians.
%
%   Through the cross product rather than acos of the dot product: acos loses
%   half its significant digits for the small angles that fill a 20-degree
%   field, which is exactly the regime the pair tolerance lives in.
n = size(body, 1);
theta = zeros(n);
for i = 1:n - 1
    for j = i + 1:n
        angle = atan2(norm(cross(body(i, :), body(j, :))), ...
                      dot(body(i, :), body(j, :)));
        theta(i, j) = angle;
        theta(j, i) = angle;
    end
end
end


function order = triadOrder(n)
%TRIADORDER  (i, j, k) triples ordered by increasing index gaps (Mortari).
%
%   Spreading across the brightest detections, rather than exhausting every
%   combination of the first few, reaches a clean triad far sooner when one of
%   the brightest detections turns out to be spurious.
order = zeros(nchoosek(max(n, 3), 3), 3);
count = 0;
for gapJ = 1:n - 2
    for gapK = 1:n - gapJ - 1
        for i = 1:n - gapJ - gapK
            count = count + 1;
            order(count, :) = [i, i + gapJ, i + gapJ + gapK];
        end
    end
end
order = order(1:count, :);
end


function [detectionIndex, catalogueIndex] = expandMatches(dcm, body, cat, ...
                                                          tolerance, candidates)
%EXPANDMATCHES  Grow a confirmed solution by matching the remaining detections.
%
%   Identification only has to produce enough correspondences to fix the
%   attitude.  Once a provisional attitude exists every other detection can be
%   checked directly, which better conditions QUEST.  A detection is accepted
%   only when its nearest catalogue star is inside tolerance and no other
%   catalogue star is nearly as close.

cosTolerance = cos(tolerance);

% Only catalogue stars near the boresight can land on the detector, so cull to
% the field before doing any per-detection work.  The pair table's extent is
% the full frame diagonal, so half of it is the field radius.
boresight = dcm(3, :)';
cosField = cos(cat.maxSeparationRad / 2 + 0.02);

if isempty(candidates)
    pool = (1:cat.nStars)';
else
    pool = candidates(:);
end
pool = pool(cat.vectors(pool, :) * boresight >= cosField);

if isempty(pool)
    detectionIndex = [];
    catalogueIndex = [];
    return;
end

predicted = (dcm * cat.vectors(pool, :)')';       % inertial -> body
similarity = body * predicted';                   % one row per detection

[sorted, ranking] = sort(similarity, 2, 'descend');
best = sorted(:, 1);
if size(sorted, 2) > 1
    second = sorted(:, 2);
else
    second = -2 * ones(size(best));
end

accepted = best >= cosTolerance & second < cosTolerance;
detectionIndex = find(accepted);
catalogueIndex = pool(ranking(accepted, 1));

% A catalogue star may not be claimed by two detections.
[~, firstAt] = unique(catalogueIndex, 'stable');
duplicated = true(size(catalogueIndex));
duplicated(firstAt) = false;
claimedTwice = ismember(catalogueIndex, catalogueIndex(duplicated));

detectionIndex = detectionIndex(~claimedTwice);
catalogueIndex = catalogueIndex(~claimedTwice);

detectionIndex = detectionIndex(:)';
catalogueIndex = catalogueIndex(:)';
end


%% ======================================================== attitude =======

function q = questQuaternion(body, inertial)
%QUESTQUATERNION  Optimal attitude quaternion, scalar-first, describing [BN].
%
%   QUEST solves Wahba's problem through the Davenport K-matrix: the
%   eigenvector of that 4x4 symmetric matrix belonging to its largest
%   eigenvalue is the optimal quaternion, in (vector, scalar) order.
%
%   A direct eigendecomposition is used rather than QUEST's Newton iteration on
%   the characteristic polynomial: the matrix is 4x4 and symmetric, so the
%   direct method costs nothing measurable and cannot fail to converge.

if size(body, 1) < 2
    error('startracker:quest', 'QUEST needs at least two matched vector pairs');
end

% B = sum_i b_i r_i', the attitude profile matrix.  Weights are uniform: the
% centroid accuracy is the same for every star bright enough to be detected.
profile = body' * inertial;

sigma = trace(profile);
axis = [profile(2, 3) - profile(3, 2);
        profile(3, 1) - profile(1, 3);
        profile(1, 2) - profile(2, 1)];

K = zeros(4);
K(1:3, 1:3) = profile + profile' - sigma * eye(3);
K(1:3, 4) = axis;
K(4, 1:3) = axis';
K(4, 4) = sigma;

[vectors, values] = eig(K);
[~, largest] = max(diag(values));
v = vectors(:, largest);

q = [v(4), v(1), v(2), v(3)];
q = q / norm(q);
if q(1) < 0
    q = -q;                       % one quaternion per attitude, not two
end
end


function dcm = quest(body, inertial)
%QUEST  The QUEST attitude as a direction cosine matrix [BN].
dcm = quatToDcm(questQuaternion(body, inertial));
end


function dcm = quatToDcm(q)
%QUATTODCM  Scalar-first quaternion to the direction cosine matrix [BN].
%
%   Renormalised first: a quaternion read from a file is unit-norm only to its
%   printed precision, and the matrix would otherwise not be quite orthonormal,
%   which shows up later as a spurious attitude error.
q = q(:)' / norm(q);
q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);

dcm = [q0^2 + q1^2 - q2^2 - q3^2,  2 * (q1 * q2 + q0 * q3),  2 * (q1 * q3 - q0 * q2);
       2 * (q1 * q2 - q0 * q3),  q0^2 - q1^2 + q2^2 - q3^2,  2 * (q2 * q3 + q0 * q1);
       2 * (q1 * q3 + q0 * q2),  2 * (q2 * q3 - q0 * q1),  q0^2 - q1^2 - q2^2 + q3^2];
end


function [ra, dec, roll] = dcmToRadecRoll(dcm)
%DCMTORADECROLL  [CN] to boresight right ascension, declination and roll, degrees.
%
%   Roll is measured about the boresight, with 0 putting celestial north up.
xC = dcm(1, :);
zC = dcm(3, :);

ra = mod(atan2(zC(2), zC(1)), 2 * pi);
dec = asin(max(-1, min(1, zC(3))));

y0 = -northReference(zC);
x0 = cross(y0, zC);
roll = mod(atan2(dot(xC, y0), dot(xC, x0)), 2 * pi);

ra = rad2deg(ra);
dec = rad2deg(dec);
roll = rad2deg(roll);
end


function perpendicular = northReference(boresight)
%NORTHREFERENCE  Celestial north projected perpendicular to the boresight.
reference = [0, 0, 1];
perpendicular = reference - dot(reference, boresight) * boresight;
if norm(perpendicular) < 1e-8
    reference = [1, 0, 0];       % the boresight is on the pole
    perpendicular = reference - dot(reference, boresight) * boresight;
end
perpendicular = perpendicular / norm(perpendicular);
end


function residuals = residualAngles(dcm, body, inertial)
%RESIDUALANGLES  Per-star angle between the measured and predicted direction,
%   in degrees.
predicted = (dcm * inertial')';
n = size(body, 1);
residuals = zeros(n, 1);
for i = 1:n
    residuals(i) = atan2(norm(cross(body(i, :), predicted(i, :))), ...
                         dot(body(i, :), predicted(i, :)));
end
residuals = rad2deg(residuals);
end


function body = pixelToCamera(cam, u, v)
%PIXELTOCAMERA  Pixel coordinates to unit vectors in the camera frame.
%
%   An ideal pinhole, no distortion term.  Add the correction here if your
%   optics need it and it propagates through everything else.
%
%   C is the camera frame: +Z along the boresight, +X along image columns
%   (u, right), +Y along image rows (v, down).
u = u(:);
v = v(:);
body = [(u - cam.principalPointPx(1)) / cam.focalLengthPx, ...
        (v - cam.principalPointPx(2)) / cam.focalLengthPx, ...
        ones(numel(u), 1)];
body = body ./ sqrt(sum(body .^ 2, 2));
end
