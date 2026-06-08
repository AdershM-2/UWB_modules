classdef Multilaterator < handle
    %MULTILATERATOR  Overdetermined N-anchor position solve (toolbox-free).
    %
    % Minimises sum_i ( ||x - a_i|| - d_i )^2 via Levenberg-Marquardt, then
    % optionally re-solves after gating out outlier ranges (NLOS / multipath).
    % Works for any N >= dim+1, so 3,4,...,N anchors are all handled the same.
    %
    %   ml = rtls.Multilaterator(2);                 % 2D
    %   [pos, info] = ml.solve(A, d, x0);
    %     A   : N x dim anchor coordinates
    %     d   : N x 1 measured ranges (m)
    %     x0  : dim x 1 initial guess (optional; defaults to anchor centroid)

    properties
        dim      = 2
        useGating = true
        gateK     = 3.0     % robust residual gate (x MAD)
        maxIters  = 30
        rangeSigma = 0.10   % assumed range noise (m), for covariance scaling
    end

    methods
        function obj = Multilaterator(dim)
            if nargin >= 1 && ~isempty(dim); obj.dim = dim; end
        end

        function [pos, info] = solve(obj, A, d, x0)
            d = d(:);
            N = size(A, 1);
            info = struct('ok', false, 'used', true(N,1), 'rms', NaN, ...
                          'cov', NaN(obj.dim), 'iters', 0);
            if N < obj.dim + 1
                pos = NaN(obj.dim, 1);
                return;
            end
            if nargin < 4 || isempty(x0)
                x0 = mean(A, 1)';
            end

            % First solve with all anchors.
            [pos, it1] = obj.lm(A, d, x0(:));
            used = true(N, 1);

            % Robust residual gating, then re-solve if we dropped any.
            if obj.useGating && N >= obj.dim + 2
                r = obj.residuals(pos, A, d);
                med = median(r);
                mad = median(abs(r - med));
                scale = max(1.4826 * mad, 1e-3);
                used = abs(r - med) <= obj.gateK * scale;
                if sum(used) >= obj.dim + 1 && sum(used) < N
                    [pos, it2] = obj.lm(A(used,:), d(used), pos);
                    it1 = it1 + it2;
                else
                    used = true(N, 1);
                end
            end

            r = obj.residuals(pos, A(used,:), d(used));
            n = sum(used);
            info.ok    = true;
            info.used  = used;
            info.rms   = sqrt(mean(r.^2));
            info.iters = it1;
            % Covariance approximation: sigma^2 * inv(J'J).
            J = obj.jacobian(pos, A(used,:));
            H = J' * J;
            if rcond(H) > 1e-12
                info.cov = (obj.rangeSigma^2) * inv(H);  %#ok<MINV>
            end
        end
    end

    methods (Access = private)
        function [x, iters] = lm(obj, A, d, x)
            lambda = 1e-3;
            r = obj.residuals(x, A, d);
            prevCost = r' * r;
            iters = 0;
            for it = 1:obj.maxIters
                iters = it;
                J = obj.jacobian(x, A);
                H = J' * J;
                g = J' * r;
                step = -(H + lambda * diag(diag(H) + 1e-9)) \ g;
                xn = x + step;
                rn = obj.residuals(xn, A, d);
                cost = rn' * rn;
                if cost < prevCost
                    x = xn; r = rn; prevCost = cost;
                    lambda = max(lambda / 3, 1e-9);
                    if norm(step) < 1e-6; break; end
                else
                    lambda = min(lambda * 3, 1e9);
                end
            end
        end

        function r = residuals(~, x, A, d)
            r = vecnorm(x' - A, 2, 2) - d;
        end

        function J = jacobian(~, x, A)
            diff = x' - A;                 % N x dim
            rng = vecnorm(diff, 2, 2);
            rng(rng < 1e-6) = 1e-6;
            J = diff ./ rng;               % N x dim
        end
    end
end
