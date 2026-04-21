classdef CameraCalibration < handle

    properties (Constant, Access = private)

        % cardboard box dimensions
        rectWidthM  double = 0.39
        rectHeightM double = 0.285

        sampsPerCorner double = 30   % average over N frames to reduce noise
    end

    properties (Access = private)
        landmarker        % HandLandMarker instance
        cornerLabels = { 'Top-Left', 'Top-Right', 'Bottom-Right', 'Bottom-Left' }
        nCorners double
    end

    properties (SetAccess = private)

        % MediaPipe coordinates collected at each corner
        mpCornerCoords  double

        % Computed scaling factors
        scaleX double
        scaleY double

        isCalibrated logical = false
    end

    methods
        function this = CameraCalibration(landmarker)
            this.landmarker   = landmarker;
            this.nCorners = length(this.cornerLabels);
            this.mpCornerCoords    = zeros(this.nCorners, 2);
        end

        function this = runCalibration(this)

            % Collect MediaPipe coordinates at each corner
            for i = 1:this.nCorners

                % Block until user clicks OK
                this.waitForUser(this.cornerLabels{i});

                samples = zeros(this.sampsPerCorner, 2);
                for s = 1 : this.sampsPerCorner
                    samples(s, :) = this.landmarker.getXY();
                end
                this.mpCornerCoords(i, :) = mean(samples, 1);
            end

            % Compute scaling factors
            this = this.computeScaling();
            this.isCalibrated = true;

            fprintf('Scale X: %.4f m per MP unit\n', this.scaleX);
            fprintf('Scale Y: %.4f m per MP unit\n', this.scaleY);
        end

        function [xMeters, yMeters] = toMeters(this, mpXY)
            % Convert MediaPipe [0,1] coordinates to meters
            if ~this.isCalibrated
                error('Camera and cyton are not calibrated');
            end
            xMeters = mpXY(1) * this.scaleX;
            yMeters = mpXY(2) * this.scaleY;
        end
    end

    methods (Access = private)

        function this = computeScaling(this)

            % get mean width and height
            mpWidthTop    = abs(this.mpCornerCoords(2,1) - this.mpCornerCoords(1,1));
            mpWidthBottom = abs(this.mpCornerCoords(3,1) - this.mpCornerCoords(4,1));
            mpWidth       = mean([mpWidthTop, mpWidthBottom]);

            mpHeightLeft  = abs(this.mpCornerCoords(4,2) - this.mpCornerCoords(1,2));
            mpHeightRight = abs(this.mpCornerCoords(3,2) - this.mpCornerCoords(2,2));
            mpHeight      = mean([mpHeightLeft, mpHeightRight]);

            % Validate — warn if edges are inconsistent (camera not level)
            widthDiff  = abs(mpWidthTop  - mpWidthBottom) / mpWidth;
            heightDiff = abs(mpHeightLeft - mpHeightRight) / mpHeight;
            if widthDiff > 0.05 || heightDiff > 0.05
                warning('Rerun calibration.');
            end

            this.scaleX = this.rectWidthM / mpWidth;
            this.scaleY = this.rectHeightM / mpHeight;
        end

        function waitForUser(~, cornerLabel)
            msg = sprintf('Move your finger to the %s corner.\n\nClick OK when ready to collect samples.', cornerLabel);
            h   = msgbox(msg, 'Calibration', 'help', 'modal');
            uiwait(h);   % blocks until user closes the dialog
        end

    end
end