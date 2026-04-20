classdef HandLandMarker < handle

    properties (Constant, Access = private)
        idxLandmark double = 6 % index finger MCP
    end

    properties (Access = private)
        u
        xyPrev double = [0, 0]
    end

    methods

        function this = HandLandMarker()
            this.u = udpport("LocalPort", 5005);
            disp('Listening for hand landmarks');

        end

        function delete(this)
            if ~isempty(this.u)
                delete(this.u);
                disp('UDP connection closed.');
            end
        end

        function xy = getXY(this)
            while this.u.NumBytesAvailable == 0
                pause(0.001);
            end
            try
                raw = readline(this.u);
                coords = jsondecode(raw);
                xy = coords(this.idxLandmark, 1:2);
                this.xyPrev = xy;   % now persists because handle class
            catch
                xy = this.xyPrev;
                warning('Could not get new coordinates. Using previous coordinates.')
            end
        end

    end
end