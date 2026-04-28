classdef CytonIK

    properties(Constant, Access = private)
        % joint angle actin offset
        actinOffset double = [pi/2; pi/2; 0; 0; 0; pi/2; 0];

        % joint limits
        % qMin double = [-2.6; -2.6; -2.6; -2.6; -2.6; -2.6; -2.6];
        % qMax double = [ 2.6;  2.6;  2.6;  2.6;  2.6;  2.6;  2.6];

        qMin double = [-2.6; -2.6; -2.6; -2.6; -2.6; 0; -2.6];
        qMax double = [ 2.6;  2.6;  2.6;  0;  2.6;  2.6;  2.6];

        % Tracking gains
        lambda double = 0.005;   % DLS damping
        kGainPos double = 0.5;     % proportional gain on position error (tune for speed vs stability)
        kGainOri double = 0.5;
        kNull double = 0.03;    % null-space gain

        qRefMatlab = zeros(7,1);

    end

    properties(Access = private)

        % robot struct
        robot struct
        % jacobian
        jac double

        % normalization references
        xyzRPYInputRef  double    % first input received
        xyzRPYRobotInit double    % robot's pose at start (from FK)
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

        function this = updateIK(this, xyzRPY)
            if numel(xyzRPY) ~= 6
                error('CytonIK: xyzRPY must have 6 elements.');
            end

            % On first call, record references
            if ~this.isInitialized
                this.xyzRPYInputRef  = xyzRPY(:);
                this.xyzRPYRobotInit = this.getRobotPose();
                this.isInitialized   = true;
            end

            % Orientation target: robot init + angular displacement from input init
            % Wrap to [-pi, pi] to prevent windup
            deltaRPY   = wrapToPi(xyzRPY(4:6) - this.xyzRPYInputRef(4:6));
            targetRPY  = this.xyzRPYRobotInit(4:6) + deltaRPY;

            % Position target: robot init + displacement from input init
            x = xyzRPY(1);
            y = xyzRPY(2);
            z = xyzRPY(3);
            roll = xyzRPY(4);
            pitch = xyzRPY(5);
            yaw = xyzRPY(6);

            Txy = transl(x, y, 0);
            Tz = transl(0, 0, z);
            R = rpy2tr(roll, 0, 0);

            % translation
            Txyz = Txy * R * Tz;
            xyzRPY(1) = Txyz(1,4);
            xyzRPY(2) = Txyz(2,4);
            xyzRPY(3) = Txyz(3,4);

            targetXYZ = this.xyzRPYRobotInit(1:3) + (xyzRPY(1:3) - this.xyzRPYInputRef(1:3));

            target = [targetXYZ; targetRPY];

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
            % Get kinematics for the Elbow Manipulator
            % theta is a list of joint angles
            % robot_struct has four fields, which comprise the DH convention for
            % describing a robot: (1) numJoints, (2) d, (3) a, (4) alpha
            %
            % Returns: A as a cell array of transformation matrices between joints
            % Returns: T as a cell array of transformation matrices in global
            %       coordinates

            theta = this.qMatlab;
            robotStruct = this.robot;

            % grab info from the struct
            numJoints = robotStruct.numJoints;
            d = robotStruct.d;
            a = robotStruct.a;
            alpha = robotStruct.alpha;

            % initialize each transformation matrix
            A = repmat({eye(4)},[1 numJoints]);

            % Ref pg 77 Spong for DH convention
            % Ai = DH_transformation(a_i,alpha_i,d_i,theta_i)
            % Ai = DH_transformation(linkLength,linkTwist,linkOffset,jointAngle)
            % Ai = DH_transformation(Tx,Rx,Tz,Rz)
            % Ai = Rot_z * Trans_z * Trans_x * Rot_x

            for i = 1:numJoints
                A{i} = this.DH(a(i),alpha(i),d(i),theta(i));
            end

            % compute the global transformation matrices if output is size 2
            if nargout > 1
                % initialize T and then initialize T{1}
                T = cell(size(A));
                T{1} = A{1};

                % post-multiply each transformation matrix
                for i = 2:numJoints
                    T{i} = T{i-1}*A{i};
                end
            end

        end

        function A = DH(~, linkLength,linkTwist,linkOffset,jointAngle)
            % Compute homogeneous transformation A given Denavit-Hartenberg parameters:
            % linkLength    = a_i
            % linkTwist     = alpha_i
            % linkOffset    = d_i
            % jointAngle    = theta_i

            % pre-computing the cosine and sine is more efficient
            c_theta = cos(jointAngle);
            s_theta = sin(jointAngle);

            % pre-computing the cosine and sine is more efficient
            c_alpha = cos(linkTwist);
            s_alpha = sin(linkTwist);

            A = [
                c_theta -s_theta*c_alpha  s_theta*s_alpha   linkLength*c_theta;
                s_theta  c_theta*c_alpha -c_theta*s_alpha   linkLength*s_theta;
                0        s_alpha          c_alpha           linkOffset;
                0        0                0                 1;];
        end

        function xyzRPY = getRobotPose(this)
            [~, T] = this.getKinematics();

            % Extract position
            p = T{end}(1:3, 4);

            % Extract RPY (XYZ convention) from rotation matrix
            R = T{end}(1:3, 1:3);
            roll  = atan2(R(3,2), R(3,3));
            pitch = atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2));
            yaw   = atan2(R(2,1), R(1,1));

            xyzRPY = [p; roll; pitch; yaw];
        end

        function this = setNumericJacobian(this)

            robotStruct = this.robot;

            % Kinematics for the Elbow Robot
            [~, T] = this.getKinematics();

            % Define components for Jacobian:
            % z-axis is just the 3rd column of the transforms
            z = cell(robotStruct.numJoints, 1);
            z{1} = [0 0 1]';
            for loop = 2 : length(z)
                z{loop} = T{loop - 1}(1:3,3);
            end

            % The joint centers are just the 4th column from each transformation
            % matrix to the base
            o = cell(robotStruct.numJoints + 1, 1);
            o{1} = [0 0 0]';
            for loop = 2 : length(o)
                o{loop} = T{loop - 1}(1:3,4);
            end

            % Per Eq. 4.64, J is the Geometric Jacobian
            oc = o{end};
            J = cell(robotStruct.numJoints, 1);
            for loop = 1 : length(J)
                J{loop} = cross(z{loop}, (oc-o{loop}));
            end

            % construct the top half of the Jacobian, that corresponds to endpoint
            J_endpoint = J{1};
            J_orientation = z{1};
            for loop = 2:length(J)
                J_endpoint = [J_endpoint, J{loop}];
                J_orientation = [J_orientation, z{loop}];
            end

            % stack the two jacobian components
            this.jac = [J_endpoint; J_orientation];
        end

        function this = clamp_cyton_joints(this)
            % Clamp joint values to safe limits
            this.qActin = min(this.qActin, this.qMax);
            this.qActin = max(this.qActin, this.qMin);
        end

        %%%%%%%%%% ikine helpers %%%%%%%%%%

        function e = computeTaskError(this, xyzRPY)
            [~, T] = this.getKinematics();

            % Position error (3x1)
            pCurr  = T{end}(1:3, 4);
            eTrans = xyzRPY(1:3) - pCurr;

            % Orientation error (3x1)
            % Build desired rotation matrix from input RPY (XYZ convention)
            r = xyzRPY(4);  p = xyzRPY(5);  y = xyzRPY(6);
            Rd = [cos(y)*cos(p), cos(y)*sin(p)*sin(r)-sin(y)*cos(r), cos(y)*sin(p)*cos(r)+sin(y)*sin(r);
                sin(y)*cos(p), sin(y)*sin(p)*sin(r)+cos(y)*cos(r), sin(y)*sin(p)*cos(r)-cos(y)*sin(r);
                -sin(p),        cos(p)*sin(r),                       cos(p)*cos(r)];

            % Current rotation matrix
            Rc = T{end}(1:3, 1:3);

            % Orientation error via skew-symmetric part of R_error = Rc' * Rd
            % eOri = vex(R_error - R_error') / 2, expressed in world frame as Rc * eOri_body
            Re = Rc' * Rd;
            eOri_body = 0.5 * [Re(3,2) - Re(2,3);
                Re(1,3) - Re(3,1);
                Re(2,1) - Re(1,2)];
            eOri = Rc * eOri_body;  % rotate error into world frame

            % Stack into 6x1 task error
            e = [this.kGainPos * eTrans; this.kGainOri * eOri];
        end

        function Jinv = computeDampedPseudoinverse(this)
            % Full 6x7 Jacobian — all rows
            Jp  = this.jac;   % 6x7
            JpT = Jp';
            Jinv = JpT / (Jp * JpT + this.lambda^2 * eye(6));  % 7x6
        end

        function dq = computeJointUpdate(this, Jinv, e)
            dqTask = Jinv * e;   % kGain already applied in e
            Jp     = this.jac;
            N      = eye(7) - Jinv * Jp;
            dqNull = N * this.kNull * (this.qRefMatlab - this.qMatlab);
            dq     = dqTask + dqNull;

            % Hard cap: no joint moves more than maxStep radians per frame
            maxStep = 0.05;   % ~3 degrees per frame
            if max(abs(dq)) > maxStep
                dq = dq * (maxStep / max(abs(dq)));
            end
        end
    end

end