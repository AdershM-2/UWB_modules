% runSurvey.m  — Anchor self-survey: collect pairwise ranges, run classical
%               MDS, and write matlab/config/anchors.json.
%
% Usage:
%   1. Flash all anchors with Anchor.ino. Flash the tag with Tag.ino.
%   2. Connect the tag via USB serial.
%   3. Set PORT and ANCHOR_IDS below, then run this script.
%   4. The tag will range every anchor pair and report results here.
%   5. anchors.json is written automatically; run run_localization.m next.
%
% Wire protocol from tag:
%   SURVEY_BEGIN,v1,<n_pairs>
%   SURVEY,v1,<src_id>,<dst_id>,<avg_dist_mm>,<ok_samples>   (one per pair)
%   SURVEY_DONE,v1

% >>>>>>>>>> CONFIGURE <<<<<<<<<<
PORT        = 'COM5';          % serial port connected to the tag
BAUD        = 115200;
ANCHOR_IDS  = [1, 2, 3];       % decimal IDs, must match ANCHORS[] in Tag.ino
DIM         = 2;               % 2 = planar, 3 = full 3-D (needs height variation)
Z_HEIGHT_M  = 1.5;             % anchor height above floor (applied to all, 2D mode)
TIMEOUT_S   = 120;             % max seconds to wait for all pairs
% <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

fprintf('Opening %s at %u baud...\n', PORT, BAUD);
s = serialport(PORT, BAUD, 'Timeout', 2);
configureTerminator(s, 'LF');
flush(s);
pause(0.3);

% Trigger survey on the tag.
writeline(s, 'SURVEY');
fprintf('Sent SURVEY command. Waiting for results...\n\n');

% Collect pairwise distance results.
pairs = struct('src', {}, 'dst', {}, 'dist_m', {}, 'ok', {});
t0 = tic;

while toc(t0) < TIMEOUT_S
    try
        line = strtrim(char(readline(s)));
    catch
        continue;
    end
    if isempty(line); continue; end
    fprintf('  %s\n', line);

    if startsWith(line, 'SURVEY_DONE')
        break;
    elseif startsWith(line, 'SURVEY,v1,')
        tok = strsplit(line, ',');
        if numel(tok) >= 6
            src_id  = str2double(tok{3});
            dst_id  = str2double(tok{4});
            dist_mm = str2double(tok{5});
            ok_cnt  = str2double(tok{6});
            if ok_cnt > 0 && dist_mm > 0
                pairs(end+1) = struct('src', src_id, 'dst', dst_id, ...
                                      'dist_m', dist_mm/1000, 'ok', ok_cnt); %#ok<AGROW>
            end
        end
    end
end

clear s;
fprintf('\nCollected %u / %u pairs.\n', numel(pairs), ...
        numel(ANCHOR_IDS)*(numel(ANCHOR_IDS)-1)/2);

if numel(pairs) == 0
    error('No survey pairs received. Check serial port and tag firmware.');
end

% Build symmetric N×N distance matrix.
N = numel(ANCHOR_IDS);
D = zeros(N);
for k = 1:numel(pairs)
    i = find(ANCHOR_IDS == pairs(k).src,  1);
    j = find(ANCHOR_IDS == pairs(k).dst,  1);
    if ~isempty(i) && ~isempty(j)
        D(i,j) = pairs(k).dist_m;
        D(j,i) = pairs(k).dist_m;
    end
end

% Warn about missing pairs.
missing = (D == 0) & ~eye(N, 'logical');
if any(missing(:))
    [ri, ci] = find(missing);
    fprintf('WARNING: missing pairs — ');
    for k = 1:numel(ri)
        fprintf('(%u,%u) ', ANCHOR_IDS(ri(k)), ANCHOR_IDS(ci(k)));
    end
    fprintf('\n  MDS result will be degraded.\n');
end

% Classical MDS.
%   B = -0.5 * J * D² * J,  J = I - (1/N)*ones(N)
%   Eigendecompose B; top-d eigenpairs give coordinates.
D2 = D .^ 2;
J  = eye(N) - ones(N)/N;
B  = -0.5 * J * D2 * J;
[V, L]       = eig(B);
lam          = diag(L);
[lam_s, idx] = sort(lam, 'descend');
V            = V(:, idx);
lam_s(lam_s < 0) = 0;   % clamp small negatives from measurement noise

d = min(DIM, N-1);
X = V(:, 1:d) .* sqrt(lam_s(1:d))';

% Fix coordinate frame.
%   Anchor 1 at origin.
X = X - X(1,:);
%   Anchor 2 on the +X axis.
if N >= 2 && norm(X(2,:)) > 1e-6
    theta = atan2(X(2,2), X(2,1));
    c = cos(-theta); sn = sin(-theta);
    R = [c -sn; sn c];
    X = (R * X')';
end
%   Anchor 3 in the +Y half-plane.
if N >= 3 && d >= 2 && X(3,2) < 0
    X(:,2) = -X(:,2);
end

% Report.
fprintf('\nAnchor positions (MDS result):\n');
for i = 1:N
    if DIM == 2
        fprintf('  Anchor 0x%02X  id=%u : x=%.3f  y=%.3f  z=%.3f m\n', ...
                ANCHOR_IDS(i), ANCHOR_IDS(i), X(i,1), X(i,2), Z_HEIGHT_M);
    else
        fprintf('  Anchor 0x%02X  id=%u : x=%.3f  y=%.3f  z=%.3f m\n', ...
                ANCHOR_IDS(i), ANCHOR_IDS(i), X(i,1), X(i,2), X(i,3));
    end
end

% Compute auto-bounds (pad 20 % around anchor cloud).
pad = 0.5;
xv = X(:,1); yv = X(:,2);
bounds = [min(xv)-pad, max(xv)+pad, min(yv)-pad, max(yv)+pad, 0, 3];

% Build and write anchors.json.
%   Format must match AnchorConfig.fromJson():  {id, x, y, z}  per anchor.
out.dim    = DIM;
out.bounds = bounds;
out.anchors = struct('id', num2cell(ANCHOR_IDS(:)'), ...
                     'x',  num2cell(X(:,1)'), ...
                     'y',  num2cell(X(:,2)'), ...
                     'z',  num2cell(repmat(Z_HEIGHT_M, 1, N)));
if DIM == 3
    for i = 1:N
        out.anchors(i).z = X(i,3);
    end
end

outFile = fullfile(fileparts(mfilename('fullpath')), 'config', 'anchors.json');
fid = fopen(outFile, 'w');
if fid < 0
    error('Cannot write %s', outFile);
end
try
    fprintf(fid, '%s\n', jsonencode(out, 'PrettyPrint', true));
catch
    % R2020b fallback — jsonencode without PrettyPrint.
    fprintf(fid, '%s\n', jsonencode(out));
end
fclose(fid);

fprintf('\nWritten to %s\n', outFile);
fprintf('Run run_localization.m to start localization with the new anchor map.\n');
