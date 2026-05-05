classdef CytonUDP < handle

    properties (Access = private)
        udpActin
    end

    methods
        function this = CytonUDP
            this.initCytonUDP();
        end

        function delete(this)
            if ~isempty(this.udpActin)
                this.udpActin.close();
                disp('[CytonUDP] Actin UDP closed.');
            end
        end

        function moveActinCyton(this, q, g)
            % Send joint angles to Actin Viewer over UDP
            % g: value for gripper
            % q: joint angles

            if numel(q) ~= 7
                error('CytonUDP:invalidInput', 'q must have 7 elements, got %d.', numel(q));
            end

            desiredAngles = [q(:); g];
            this.udpActin.putData(typecast(double(desiredAngles), 'uint8'));
        end

    end

    methods (Access = private)
        function initCytonUDP(this)
            this.udpActin = PnetClass(8889, 8888, '127.0.0.1');
            this.udpActin.initialize();
            disp('[init_cyton_udp] Actin UDP initialized');
        end
    end

end