%%% Problem 7
%% Cyton Arm Class
% Philip Deaton

classdef CytonEpsilon1500

    properties (Constant, Access = private)

        %%%%% Arm parameters %%%%%

        % link lengths in meters
        l01 double = 0.08 % base
        l double = ...
            [ 0.135 ...     % q1 -> q2
            ; 0.125 ... % q2 -> q3
            ; 0.125 ... % q3 -> q4
            ; 0.135 ... % q4 -> q5
            ; 0 ...     % q5 -> q6
            ; 0 ...     % q6 -> q7
            ; 0.09 ...  % q7 -> end effector
            ]

        % joint configs: type and limit
        baseAngle double = 0;
        qCfg cell = ...
            { struct('type', "roll", 'lim', 300) ...    % shoulder roll
            ; struct('type', "pitch", 'lim', 220) ...   % shoulder pitch
            ; struct('type', "yaw", 'lim', 220) ...     % shoulder yaw
            ; struct('type', "pitch", 'lim', 220) ...   % elbow pitch
            ; struct('type', "yaw", 'lim', 220) ...     % wrist yaw
            ; struct('type', "pitch", 'lim', 220) ...   % wrist pitch
            ; struct('type', "roll", 'lim', 300) ...    % wrist roll
            }

    end % constant private properties

    properties (Access = private)
        nJoints     double
        qD          double % link offsets
        qA          double % link length
        qAlpha      double % twist angles
        thetaOffset double % joint variable offset
        qLims       double % All scan limits
    end % private properties

    properties (GetAccess = public)
        % Cyton arm
        robot
    end % public properties

    methods

        %%%%%%%%%% Constructor %%%%%%%%%%

        function this = CytonEpsilon1500()

            % joint parameters
            this = this.setJointParameters();

            % build arm
            this = this.buildArm();
        end

    end % public methods

    %%%%%%%%%% Arm Initialization %%%%%%%%%%

    methods (Access = private)

        %%%% Joint Parameters %%%%%

        % d, a, alpha, and theta offset
        function this = setJointParameters(this)

            % number of joints
            this = this.setNJoints();

            % joint types
            jointTypes = this.getJointTypes();

            % link lengths
            this = this.setLinkLengths(jointTypes);

            % joint angles
            this = this.setJointAngles(jointTypes);

            % joint limits
            this = this.setCenteredLimits();

        end

        % Set number of joints
        function this = setNJoints(this)
            assert(...
                length(this.l) == length(this.qCfg), ...
                'Unequal number of joints and links!'...
                );
            this.nJoints = length(this.l);
        end

        % joint types
        function jointTypes = getJointTypes(this)
            jointTypes = cellfun(@(cfg) cfg.type, this.qCfg);
        end

        %%%% Link Parameters %%%%

        function this = setLinkLengths(this, jointTypes)
            this = this.setD(jointTypes);
            this = this.setA(jointTypes);
        end

        % d
        function this = setD(this, jointTypes)
            this.qD = zeros(this.nJoints, 1);
            idx = jointTypes == "roll";
            this.qD(idx) = this.l(idx);
        end

        % a
        function this = setA(this, jointTypes)
            this.qA = zeros(this.nJoints, 1);
            idx = jointTypes ~= "roll";
            this.qA(idx) = this.l(idx);
        end

        %%%% Joint Angle Parameters %%%%

        function this = setJointAngles(this, jointTypes)
            this = this.setTwistAngles(jointTypes);
            this = this.setThetaOffsets(jointTypes);
        end

        % twist angles
        function this = setTwistAngles(this, jointTypes)
            thetaZDesired = this.getThetaZDesired(jointTypes);
            alphaDiffs = diff(thetaZDesired);
            this.qAlpha = [alphaDiffs; 0];
        end

        function thetaZDesired = getThetaZDesired(this, jointTypes)
            % map joint type to desired angle of z-axis
            % roll: z = 0
            % pitch: z = pi/2
            % yaw: z = 0 (since serial link)
            thetaZDesired = zeros(this.nJoints, 1);
            idx = jointTypes == "pitch";
            thetaZDesired(idx) = 3*pi/2;
        end

        % Theta offsets
        function this = setThetaOffsets(this, jointTypes)
            this.thetaOffset = zeros(this.nJoints, 1);
            for i = 2 : this.nJoints-1
                this.thetaOffset(i) = this.getThetaOffset ...
                    ( jointTypes(i-1)... % prev
                    , jointTypes(i)... % curr
                    , jointTypes(i+1)... % next
                    );
            end
        end

        function offset = getThetaOffset(~, typePrev, typeCurr, typeNext)
            if "roll" == typePrev && "pitch" == typeCurr
                offset = 3*pi/2;
            elseif "pitch" == typeCurr && "roll" == typeNext
                offset = pi/2;
            else
                offset = 0;
            end
        end

        %%%% joint limits %%%%
        function this = setCenteredLimits(this)
            % get limits
            lims = cellfun(@(q) q.lim, this.qCfg);

            % center limits around 0 in radians
            this.qLims = [-lims, lims] / 2 * pi / 180;
        end

        %%%%% Build Arm %%%%%
        function this = buildArm(this)
            q = this.getJoints();
            base = this.getBase();
            this.robot = SerialLink...
                ( q...
                , 'base', base...
                , 'name', 'Cyton Epsilon 1500' ...
                );
        end

        function q = getJoints(this)
            q(this.nJoints) = Revolute();
            for i = 1 : this.nJoints
                q(i) = Revolute...
                    ( 'd', this.qD(i)...
                    , 'a', this.qA(i)...
                    , 'alpha', this.qAlpha(i)...
                    , 'offset', this.thetaOffset(i)...
                    , 'qlim', this.qLims(i, :)...
                    );
            end
        end

        function base = getBase(this)
            base = ETS3.Tz(this.l01);
            base = base.fkine;
        end
    end % private methods - initialization

    %%%%%%%%%% Kinematics %%%%%%%%%%

    methods(Access = public)

        % teach
        function teach(this)
            this.robot.teach;
        end

        % Forward Kinematics
        function T = fkine(this, q)
            q = this.getVerifiedJointAngles(q);
            T = this.robot.fkine(q);
            this.plotForwardKinematics(q);
        end

        % Inverse Kinematics
        function q = ikine(this, p)
            T = SE3(p);
            q = this.robot.ikine(T);

            % plot
            % this.fkine(q);
        end

    end % public kinematics methods

    methods (Access = private)

        % joint limits
        function q = getVerifiedJointAngles(this, q)

            % Check for correct number of joints
            n = length(this.qLims);
            assert(length(q)==n, 'Input has incorrect number of joint angles');

            % bound joint angles
            q = max(this.qLims(:,1)', min(this.qLims(:,2)', q));

            % warn if any angles are out of bounds
            idxTooLow = q == this.qLims(:,1)';
            idxTooHigh = q == this.qLims(:,2)';
            if any(idxTooLow)
                warning('Joints %s clamped to lower bound', num2str(find(idxTooLow)'));
            end
            if any(idxTooHigh)
                warning('Joints %s clamped to upper bound', num2str(find(idxTooHigh)'));
            end
        end

        % Plot forward kinematics
        function plotForwardKinematics(this, q)
            figure;
            this.robot.plot(q);
        end

    end % private kinematics methods

end % class
