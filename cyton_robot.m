% Add MiniVIE to path (provides PnetClass)
currDir = cd;
cd('C:\GitHub\MiniVIE');
MiniVIE.configurePath();
cd(currDir);

% To Actin Viewer
udpActin = PnetClass(8889, 8888, '127.0.0.1');
udpActin.initialize();
% q = [0, 0, 0, 0, 0, 0, 0];  % Home
q = [pi/4, pi/4, 0, pi/2, 0, 0, 0];  % 7 joint angles in radians
gripper = 0.01;                         % gripper distance in meters

% Actin: 8x float64 (doubles), angles in RADIANS
udpActin.putData(typecast([q, gripper], 'uint8'));