% ikine Wrapper

function IkineWrapper(isActin)

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

q_initial_actin = zeros(7, 1);  % Initialize joint angles for the actuator
z_fixed = 0.5;                   % Fixed z-coordinate for the IK calculation
roll_fixed = 0;
pitch_fixed = 0;
yaw_fixed = 0;
gripper_fixed = 0.01;
dt = 0.1;                        % Time step for the loop

lm  = HandLandMarker();
ik  = CytonIK(q_initial_actin);


while true
    xy = lm.getXY();           % blocks until data arrives
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
