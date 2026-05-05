classdef HandLandMarker < handle

    properties (Access = private)
        u
        xyPrev double = [0, 0]
    end

    methods

        function this = HandLandMarker()
            this.u = udpport("LocalPort", 5010);
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
                % flush messages to get most recent message
                while this.u.NumBytesAvailable > 0
                    raw = readline(this.u);
                end
                coords = jsondecode(raw);
                xy = coords(:);
                this.xyPrev = xy;
            catch
                xy = this.xyPrev;
                warning('Could not get new coordinates. Using previous coordinates.')
            end
        end

    end
end