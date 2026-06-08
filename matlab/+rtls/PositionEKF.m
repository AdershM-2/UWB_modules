classdef PositionEKF < handle
    %POSITIONEKF  Constant-velocity Kalman filter over the multilaterated fix.
    %
    % State = [pos(dim); vel(dim)]. This is a LOOSELY coupled filter: the
    % measurement is the position from the Multilaterator. The update() signature
    % is kept generic so this can later be swapped to a TIGHTLY coupled form
    % (measurement = raw ranges) and extended with an IMU prediction step - that
    % is the natural place to fuse the BNO085.
    %
    %   ekf = rtls.PositionEKF(2);
    %   ekf.initialize([x0; y0]);
    %   [pos, vel] = ekf.step(dt, zPos);          % zPos = dim x 1 measurement
    %   [pos, vel] = ekf.step(dt, zPos, R);       % with measurement covariance

    properties
        dim       = 2
        x                       % 2*dim state
        P                       % covariance
        qAccel    = 1.0         % process accel PSD (m^2/s^3); raise if laggy
        posSigma  = 0.10        % default measurement std (m)
        initialized = false
    end

    methods
        function obj = PositionEKF(dim)
            if nargin >= 1 && ~isempty(dim); obj.dim = dim; end
            n = 2 * obj.dim;
            obj.x = zeros(n, 1);
            obj.P = eye(n);
        end

        function initialize(obj, pos0)
            obj.x = [pos0(:); zeros(obj.dim, 1)];
            obj.P = blkdiag(eye(obj.dim) * obj.posSigma^2, eye(obj.dim) * 1.0);
            obj.initialized = true;
        end

        function predict(obj, dt)
            d = obj.dim; I = eye(d); Z = zeros(d);
            F = [I, dt*I; Z, I];
            q = obj.qAccel;
            Q = q * [ (dt^3/3)*I, (dt^2/2)*I;
                      (dt^2/2)*I,  dt*I ];
            obj.x = F * obj.x;
            obj.P = F * obj.P * F' + Q;
        end

        function update(obj, z, R)
            if nargin < 3 || isempty(R)
                R = eye(obj.dim) * obj.posSigma^2;
            end
            d = obj.dim;
            H = [eye(d), zeros(d)];
            y = z(:) - H * obj.x;
            S = H * obj.P * H' + R;
            K = (obj.P * H') / S;
            obj.x = obj.x + K * y;
            n = numel(obj.x);
            obj.P = (eye(n) - K * H) * obj.P;
        end

        function [pos, vel] = step(obj, dt, z, R)
            if ~obj.initialized
                obj.initialize(z);
            else
                obj.predict(dt);
                if nargin < 4; R = []; end
                if ~isempty(z) && all(isfinite(z))
                    obj.update(z, R);
                end
            end
            pos = obj.x(1:obj.dim);
            vel = obj.x(obj.dim+1:end);
        end
    end
end
