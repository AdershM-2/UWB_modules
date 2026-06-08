classdef LivePlotter < handle
    %LIVEPLOTTER  Live 2D/3D view of anchors + tracked tag.
    %
    %   plotter = rtls.LivePlotter(cfg);
    %   plotter.update(pos, info);   % pos = dim x 1, info from Multilaterator

    properties
        cfg
        fig
        ax
        hTag
        hTrail
        hEllipse
        trail = []
        trailLen = 200
    end

    methods
        function obj = LivePlotter(cfg)
            obj.cfg = cfg;
            obj.fig = figure('Name', 'UWB RTLS', 'NumberTitle', 'off');
            obj.ax = axes('Parent', obj.fig); hold(obj.ax, 'on'); grid(obj.ax, 'on');
            axis(obj.ax, 'equal');
            b = cfg.bounds;

            if cfg.dim == 2
                % Anchors
                plot(obj.ax, cfg.coords(:,1), cfg.coords(:,2), 'ks', ...
                    'MarkerFaceColor', [0.2 0.2 0.2], 'MarkerSize', 9);
                for i = 1:numel(cfg.ids)
                    text(obj.ax, cfg.coords(i,1), cfg.coords(i,2), ...
                        sprintf('  A%d', cfg.ids(i)), 'FontWeight', 'bold');
                end
                obj.hTrail = plot(obj.ax, NaN, NaN, '-', 'Color', [0 0.45 0.74]);
                obj.hTag   = plot(obj.ax, NaN, NaN, 'o', 'MarkerSize', 11, ...
                    'MarkerFaceColor', [0.85 0.1 0.1], 'MarkerEdgeColor', 'k');
                obj.hEllipse = plot(obj.ax, NaN, NaN, '-', 'Color', [0.95 0.55 0.55]);
                xlim(obj.ax, b(1:2)); ylim(obj.ax, b(3:4));
                xlabel(obj.ax, 'x (m)'); ylabel(obj.ax, 'y (m)');
            else
                plot3(obj.ax, cfg.coords(:,1), cfg.coords(:,2), cfg.coords(:,3), ...
                    'ks', 'MarkerFaceColor', [0.2 0.2 0.2], 'MarkerSize', 9);
                for i = 1:numel(cfg.ids)
                    text(obj.ax, cfg.coords(i,1), cfg.coords(i,2), cfg.coords(i,3), ...
                        sprintf('  A%d', cfg.ids(i)), 'FontWeight', 'bold');
                end
                obj.hTrail = plot3(obj.ax, NaN, NaN, NaN, '-', 'Color', [0 0.45 0.74]);
                obj.hTag   = plot3(obj.ax, NaN, NaN, NaN, 'o', 'MarkerSize', 11, ...
                    'MarkerFaceColor', [0.85 0.1 0.1], 'MarkerEdgeColor', 'k');
                xlim(obj.ax, b(1:2)); ylim(obj.ax, b(3:4)); zlim(obj.ax, b(5:6));
                xlabel(obj.ax, 'x (m)'); ylabel(obj.ax, 'y (m)'); zlabel(obj.ax, 'z (m)');
                view(obj.ax, 3);
            end
            title(obj.ax, 'UWB RTLS - waiting for data...');
        end

        function update(obj, pos, info)
            if ~all(isfinite(pos)); return; end
            obj.trail = [obj.trail, pos(:)];
            if size(obj.trail, 2) > obj.trailLen
                obj.trail = obj.trail(:, end-obj.trailLen+1:end);
            end

            if obj.cfg.dim == 2
                set(obj.hTag, 'XData', pos(1), 'YData', pos(2));
                set(obj.hTrail, 'XData', obj.trail(1,:), 'YData', obj.trail(2,:));
                if nargin >= 3 && isstruct(info) && all(isfinite(info.cov(:)))
                    [ex, ey] = obj.covEllipse(pos, info.cov);
                    set(obj.hEllipse, 'XData', ex, 'YData', ey);
                end
            else
                set(obj.hTag, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3));
                set(obj.hTrail, 'XData', obj.trail(1,:), 'YData', obj.trail(2,:), ...
                    'ZData', obj.trail(3,:));
            end

            if nargin >= 3 && isstruct(info)
                title(obj.ax, sprintf('UWB RTLS - %d anchors used, rms=%.3f m', ...
                    sum(info.used), info.rms));
            end
            drawnow limitrate;
        end
    end

    methods (Static, Access = private)
        function [ex, ey] = covEllipse(pos, C)
            % 2-sigma ellipse from the 2x2 position covariance.
            C = C(1:2, 1:2);
            [V, D] = eig((C + C')/2);
            t = linspace(0, 2*pi, 40);
            a = 2 * sqrt(max(D(1,1), 0));
            b = 2 * sqrt(max(D(2,2), 0));
            xy = V * [a*cos(t); b*sin(t)];
            ex = pos(1) + xy(1,:);
            ey = pos(2) + xy(2,:);
        end
    end
end
