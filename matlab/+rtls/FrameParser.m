classdef FrameParser
    %FRAMEPARSER  Parse one RTLS host-link line into a struct.
    %
    % Wire format (see firmware HostLink.h):
    %   RTLS,v1,<t_ms>,<tag_id>,<n>,<id1>,<d1_mm>,<q1>,...,<idN>,<dN_mm>,<qN>
    %        [,IMU,<qw>,<qx>,<qy>,<qz>,<ax>,<ay>,<az>]
    %
    % Returns a struct s with:
    %   s.valid  logical
    %   s.t_ms   device timestamp (ms)
    %   s.tagId  tag short address
    %   s.ids    1xN anchor ids
    %   s.dist   1xN distances (metres)
    %   s.q      1xN rx power (dBm)
    %   s.imu    [] or struct(quat=[w x y z], acc=[ax ay az])

    methods (Static)
        function s = parse(line)
            s = struct('valid', false, 't_ms', 0, 'tagId', 0, ...
                       'ids', [], 'dist', [], 'q', [], 'imu', []);
            if isempty(line); return; end
            tok = strsplit(strtrim(line), ',');
            if numel(tok) < 5 || ~strcmp(tok{1}, 'RTLS'); return; end
            % tok{2} is the version ('v1'); reserved for future changes.

            s.t_ms  = str2double(tok{3});
            s.tagId = str2double(tok{4});
            n       = round(str2double(tok{5}));
            if isnan(n) || n < 0; return; end

            need = 5 + 3*n;
            if numel(tok) < need; return; end

            ids = zeros(1, n); dist = zeros(1, n); q = zeros(1, n);
            k = 6;
            for i = 1:n
                ids(i)  = str2double(tok{k});
                dist(i) = str2double(tok{k+1}) / 1000.0;  % mm -> m
                q(i)    = str2double(tok{k+2});
                k = k + 3;
            end
            s.ids = ids; s.dist = dist; s.q = q;

            % Optional IMU tail.
            if numel(tok) >= k && strcmp(tok{k}, 'IMU') && numel(tok) >= k+7
                vals = str2double(tok(k+1:k+7));
                s.imu = struct('quat', vals(1:4), 'acc', vals(5:7));
            end

            s.valid = ~any(isnan([s.t_ms, s.tagId, ids, dist, q]));
        end
    end
end
