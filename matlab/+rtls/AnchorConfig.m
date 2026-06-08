classdef AnchorConfig < handle
    %ANCHORCONFIG  Anchor coordinates + room geometry.
    %
    % THIS IS THE ONE PLACE YOU ADD ANCHORS. The firmware only ships ranges, so
    % adding/moving an anchor is purely a host-side config change here (or in the
    % JSON file) - no reflash needed.
    %
    %   cfg = rtls.AnchorConfig();                 % built-in 3-anchor example
    %   cfg = rtls.AnchorConfig.fromJson('config/anchors.json');
    %
    %   [A, found] = cfg.coordsFor([1 2 3]);       % map ids -> coordinates

    properties
        dim    = 2          % 2 or 3
        ids    = []         % 1xK anchor short addresses (decimal)
        coords = zeros(0,3) % Kx3 [x y z] in metres (z ignored when dim==2)
        bounds = [0 5 0 5 0 3]  % [xmin xmax ymin ymax zmin zmax] for plotting
    end

    methods
        function obj = AnchorConfig(ids, coords, dim, bounds)
            if nargin == 0
                % Built-in example: 3 anchors around a ~4x4 m room, dim=2.
                obj.dim    = 2;
                obj.ids    = [1, 2, 3];
                obj.coords = [0.00, 0.00, 1.50;
                              4.00, 0.00, 1.50;
                              2.00, 4.00, 1.50];
                obj.bounds = [-0.5 4.5 -0.5 4.5 0 3];
                return;
            end
            obj.ids    = ids(:)';
            obj.coords = coords;
            if nargin >= 3 && ~isempty(dim);    obj.dim = dim; end
            if nargin >= 4 && ~isempty(bounds); obj.bounds = bounds; end
        end

        function [A, found] = coordsFor(obj, queryIds)
            %COORDSFOR  Rows of coordinates for the requested ids.
            % A is numel(queryIds) x dim; found is a logical mask of known ids.
            q = queryIds(:)';
            A = zeros(numel(q), obj.dim);
            found = false(1, numel(q));
            for i = 1:numel(q)
                idx = find(obj.ids == q(i), 1);
                if ~isempty(idx)
                    A(i, :) = obj.coords(idx, 1:obj.dim);
                    found(i) = true;
                end
            end
        end

        function c = centroid(obj)
            c = mean(obj.coords(:, 1:obj.dim), 1);
        end
    end

    methods (Static)
        function obj = fromJson(path)
            txt = fileread(path);
            j = jsondecode(txt);
            ids = zeros(1, numel(j.anchors));
            coords = zeros(numel(j.anchors), 3);
            for i = 1:numel(j.anchors)
                a = j.anchors(i);
                ids(i) = a.id;
                coords(i, :) = [a.x, a.y, getfield_default(a, 'z', 0)];
            end
            dim = getfield_default(j, 'dim', 2);
            bounds = getfield_default(j, 'bounds', [0 5 0 5 0 3]);
            obj = rtls.AnchorConfig(ids, coords, dim, bounds(:)');
        end
    end
end

function v = getfield_default(s, f, d)
    if isfield(s, f) && ~isempty(s.(f)); v = s.(f); else; v = d; end
end
