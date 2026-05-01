matlabAngles = @(x) [x(1) - pi / 2, x(2) - pi /2, x(3), x(4), x(5), x(6) - pi / 2, x(7)];
robotStruct = link_constants_cyton();

% Target position (x, y, z)
targetPos = [0.2; 0.1; 0.5];

% Inside your loop:
q = [-pi/2, -0.55, 0, -1.15, 0, -1.33, 0];

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