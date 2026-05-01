% Target position (x, y, z)
targetPos = [0.2, 0.1, 0.5]';
matlabAngles = @(x) [x(1) - pi / 2, x(2) - pi /2, x(3), x(4), x(5), x(6) - pi / 2, x(7)];
% Inside your loop:
[A, T] = get_kinematics(matlabAngles(q), robotStruct);
% currentPos = T(1:3, 4, end); % Get current XYZ from Forward Kinematics
currentPos = T{7}(1:3, 4);
% Calculate position error
error = targetPos - currentPos;
% Define a gain (speed multiplier)
K = 5;
vXYZ = K * error; % The further away, the faster it moves
% Standard IK steps
J = numeric_jacobian(matlabAngles(q), robotStruct);
Jinv = pinv(J(1:3,:));
q_dot = Jinv * vXYZ;
q = q + (q_dot * dt);
disp(q)