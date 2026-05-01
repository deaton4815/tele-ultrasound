% instantiate cyton arm

cytonArm = CytonEpsilon1500();

startingJointPosition = [-pi/2, -0.55, 0, -1.15, 0, -1.33, 0];

rotation = rpy2r(pi, 0, -pi);

% get xyz of starting position
startingHTM = cytonArm.fkine(startingJointPosition);
% startingHTM = startingHTM.T
% Load hand coordinate data
load('hand_data.mat');

cytonHTM = startingHTM;
handOrigin = [0.5, 0.5];
for i = 1 : length(data)
    dxy = data(i, :) - handOrigin;
    cytonHTM.t(1) = startingHTM.t(1) + dxy(1);
    cytonHTM.t(2) = startingHTM.t(2) + dxy(2);

    % get joint positions
    cytonHTMUpdate = cytonHTM.T;

    % get ikine
    % Target position (x, y, z)
    targetPos = [0.2; 0.1; 0.5];

    % Inside your loop:

    % Forward kinematics
    [A, T] = get_kinematics(matlabAngles(q), robotStruct);
    currentPos = T(1:3, 4, end);   % Current XYZ position

    % Position error
    error = targetPos - currentPos;

    % Gain (speed multiplier)
    K = 5;
    vXYZ = K * error;   % Velocity proportional to error

    % Jacobian
    J = numeric_jacobian(matlabAngles(q), robotStruct);

    % Use only position part (top 3 rows)
    Jpos = J(1:3, :);

    % Pseudoinverse
    Jinv = pinv(Jpos);

    % Joint velocity
    q_dot = Jinv * vXYZ;

    % Integrate
    q = q + q_dot * dt;



    pause(0.01);
end

