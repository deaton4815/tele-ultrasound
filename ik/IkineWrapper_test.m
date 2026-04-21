% ikine Wrapper
function IkineWrapper_test(isActin)

if nargin < 1
    isActin = false;
end

if isActin
    currDir = cd;
    cd('C:\GitHub\MiniVIE');
    MiniVIE.configurePath();
    cd(currDir);
    udp = CytonUDP();
end

q_initial_actin = [-pi/2, -0.55, 0, -1.15, 0, -1.33, 0];  % Initialize joint angles for the actuator
z_fixed = 0.5;                   % Fixed z-coordinate for the IK calculation
roll_fixed = 0;
pitch_fixed = 0;
yaw_fixed = 0;
gripper_fixed = 0.01;
dt = .1;                        % Time step for the loop
ik  = CytonIK(q_initial_actin);

xy = [0.5, 0.5];
while true
    % xy = xy*0.5;
% get z from force sensor
    % get rotation from myoband
    ik = ik.updateIK([xy(1); xy(2); z_fixed; roll_fixed; pitch_fixed; yaw_fixed]);
    disp(ik.qActin);
    if isActin
        udp.moveActinCyton(ik.qActin, gripper_fixed);
    end
    pause(dt);
end

end
