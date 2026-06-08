classdef UwbReceiver < handle
    %UWBRECEIVER  Read RTLS sweep packets over UDP or USB serial.
    %
    %   % UDP (matches firmware UWB_HOSTLINK_UDP):
    %   rx = rtls.UwbReceiver('udp', 'LocalPort', 5005);
    %
    %   % Serial (matches firmware UWB_HOSTLINK_SERIAL):
    %   rx = rtls.UwbReceiver('serial', 'Port', 'COM5', 'Baud', 115200);
    %
    %   s = rx.next();        % blocks up to Timeout for one parsed sweep
    %   if s.valid; ...; end
    %
    % Requires the udpport/serialport interfaces (R2020b+).

    properties
        mode            % 'udp' | 'serial'
        timeout = 1.0   % seconds to wait in next()
    end
    properties (Access = private)
        dev             % udpport or serialport object
    end

    methods
        function obj = UwbReceiver(mode, varargin)
            p = inputParser;
            addParameter(p, 'LocalPort', 5005);
            addParameter(p, 'Port', 'COM5');
            addParameter(p, 'Baud', 115200);
            addParameter(p, 'Timeout', 1.0);
            parse(p, varargin{:});
            obj.mode = lower(mode);
            obj.timeout = p.Results.Timeout;

            switch obj.mode
                case 'udp'
                    obj.dev = udpport("LocalPort", p.Results.LocalPort, ...
                                      "Timeout", obj.timeout);
                    configureTerminator(obj.dev, "LF");
                case 'serial'
                    obj.dev = serialport(p.Results.Port, p.Results.Baud, ...
                                         "Timeout", obj.timeout);
                    configureTerminator(obj.dev, "LF");
                otherwise
                    error('UwbReceiver:mode', 'mode must be ''udp'' or ''serial''');
            end
        end

        function s = next(obj)
            %NEXT  Read and parse one line. Returns struct with .valid=false on timeout.
            line = '';
            try
                line = readline(obj.dev);   % respects terminator + Timeout
            catch
                % timeout / no data
            end
            if isempty(line) || (isstring(line) && strlength(line) == 0)
                s = rtls.FrameParser.parse('');
                return;
            end
            s = rtls.FrameParser.parse(char(line));
        end

        function flush(obj)
            try; flush(obj.dev); catch; end
        end

        function delete(obj)
            try; clear obj.dev; catch; end
        end
    end
end
