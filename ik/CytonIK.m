classdef CytonIK

    properties(Constant, Access = private)

        % joint angle actin offset
        actinOffset double = [pi/2; pi/2; 0; 0; 0; pi/2; 0];

        % joint limits
        qMin double = [-2.6; -2.6; -2.6; -2.6; -2.6; 0; -2.6];
        qMax double = [ 2.6;  2.6;  2.6;  0;  2.6;  2.6;  2.6];

        % gains
        lambda   double = 0.005;   % DLS damping
        kGainPos double = 0.8;
        kNull    double = 0.03;    % null-space gain

        qRefMatlab = zeros(7,1);

    end

    properties(Access = private)

        % robot struct
        robot struct
        % jacobian
        jac double

        % normalization references
        xyzInputRef  double    % first input received
        xyzRobotInit double    % robot's position at start from FK
        isInitialized   logical = false

    end

    properties(SetAccess = private)
        % current joint values
        qMatlab double
    end

    properties
        qActin double
    end

    methods
        function this = CytonIK(qInitial)

            % initialize robot
            this = this.setRobotStruct();

            % initialize joint variables
            this.qActin  = qInitial(:);
            this = this.setMatlabJoints();

        end

        function this = updateIK(this, xyz)
            if numel(xyz) ~= 3
                error('CytonIK: xyz must have 3 elements.');
            end

            % References on first pass
            if ~this.isInitialized
                this.xyzInputRef  = xyz(:);
                this.xyzRobotInit = this.getRobotPosition();
                this.isInitialized = true;
            end

            % target = robot init + displacement from input init
            target = this.xyzRobotInit + (xyz(:) - this.xyzInputRef);

            this = this.setNumericJacobian();
            e    = this.computeTaskError(target);
            Jinv = this.computeDampedPseudoinverse();
            dq   = this.computeJointUpdate(Jinv, e);

            this.qMatlab = this.qMatlab + dq;
            this = this.setActinJoints();
            this = this.setMatlabJoints();
        end

    end

    methods (Access = private)

        %%%%%%%%%% Setup %%%%%%%%%

        function this = setRobotStruct(this)
            r = CytonEpsilon1500Robot();
            this.robot = r.robot;
        end

        % actin joints
        function this = setActinJoints(this)
            this.qActin = this.qMatlab(:) + this.actinOffset;
            this = this.clamp_cyton_joints();
        end

        % MATLAB joints
        function this = setMatlabJoints(this)
            this.qMatlab = this.qActin(:) - this.actinOffset;
        end

        %%%%%%%%%% Kinematic Helpers %%%%%%%%%%

        function [A, T] = getKinematics(this)
            theta = this.qMatlab;
            robotStruct = this.robot;

            numJoints = robotStruct.numJoints;
            d = robotStruct.d;
            a = robotStruct.a;
            alpha = robotStruct.alpha;

            A = repmat({eye(4)},[1 numJoints]);

            for i = 1:numJoints
                A{i} = this.DH(a(i),alpha(i),d(i),theta(i));
            end

            if nargout > 1
                T = cell(size(A));
                T{1} = A{1};

                for i = 2:numJoints
                    T{i} = T{i-1}*A{i};
                end
            end

        end

        function A = DH(~, linkLength,linkTwist,linkOffset,jointAngle)
            c_theta = cos(jointAngle);
            s_theta = sin(jointAngle);
            c_alpha = cos(linkTwist);
            s_alpha = sin(linkTwist);

            A = [
                c_theta -s_theta*c_alpha  s_theta*s_alpha   linkLength*c_theta;
                s_theta  c_theta*c_alpha -c_theta*s_alpha   linkLength*s_theta;
                0        s_alpha          c_alpha           linkOffset;
                0        0                0                 1;];
        end

        function xyz = getRobotPosition(this)
            [~, T] = this.getKinematics();
            xyz = T{end}(1:3, 4);
        end

        function this = setNumericJacobian(this)

            robotStruct = this.robot;
            [~, T] = this.getKinematics();

            % z
            z = cell(robotStruct.numJoints, 1);
            z{1} = [0 0 1]';
            for i = 2 : length(z)
                z{i} = T{i - 1}(1:3,3);
            end

            % joint centers
            o = cell(robotStruct.numJoints + 1, 1);
            o{1} = [0 0 0]';
            for i = 2 : length(o)
                o{i} = T{i - 1}(1:3,4);
            end

            % Jacobian
            oc = o{end};
            J = cell(robotStruct.numJoints, 1);
            for i = 1 : length(J)
                J{i} = cross(z{i}, (oc - o{i}));
            end

            Jpos = J{1};
            for i = 2:length(J)
                Jpos = [Jpos, J{i}];
            end

            this.jac = Jpos;
        end

        function this = clamp_cyton_joints(this)
            this.qActin = min(this.qActin, this.qMax);
            this.qActin = max(this.qActin, this.qMin);
        end

        %%%%%%%%%% ikine helpers %%%%%%%%%%

        function e = computeTaskError(this, xyz)
            [~, T] = this.getKinematics();
            pCurr = T{end}(1:3, 4);
            e = this.kGainPos * (xyz - pCurr);
        end

        function Jinv = computeDampedPseudoinverse(this)
            Jp   = this.jac;
            JpT  = Jp';
            Jinv = JpT / (Jp * JpT + this.lambda^2 * eye(3));
        end

        function dq = computeJointUpdate(this, Jinv, e)
            dqTask = Jinv * e;
            Jp     = this.jac;
            N      = eye(7) - Jinv * Jp;
            dqNull = N * this.kNull * (this.qRefMatlab - this.qMatlab);
            dq     = dqTask + dqNull;

            maxStep = 0.05;
            if max(abs(dq)) > maxStep
                dq = dq * (maxStep / max(abs(dq)));
            end
        end
    end

end