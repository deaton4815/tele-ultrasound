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
% q_initial_actin = zeros(1,7);
q_initial_actin = [.33, -0.74, 0, -1.51, 0, 0.768, 0];  % Initialize joint angles for the actuator
z_fixed = 0.5;                   % Fixed z-coordinate for the IK calculation
%roll_fixed = 0;
roll_myo = 0;
pitch_fixed = 0;
yaw_fixed = 0;
gripper_fixed = 0.01;
dt = 0.1;                        % Time step for the loop

hMyo = init_myo();
hMyo.getData();

disp('Get Neutrual Position');
neutral_gyro = [0, 0, 0];
for i = 1:1000
    neutral_gyro = neutral_gyro + hMyo.Gyroscope(:)';
end
neutral_gyro = neutral / 1000;
disp('stop');
tPrev = tic;

lm  = HandLandMarker();
ik  = CytonIK(q_initial_actin);

if isActin
    udp.moveActinCyton(ik.qActin, gripper_fixed);
end

cal  = CameraCalibration(lm);
cal  = cal.runCalibration();

msg = sprintf('Place your hand to the center of the  box to start the procedure');
h   = msgbox(msg, 'Ultrasound', 'help', 'modal');
uiwait(h);   % blocks until user closes the dialog

while true
    xy = lm.getXY();           % blocks until data arrives
    [xM, yM] = cal.toMeters(xy);
    % get z from force sensor
    % get rotation from myoband
    hMyo.getData();
    dt_myo = toc(tPrev);
    tPrev = tic;
    roll_myo = orientation(hyo, roll_myo, neurtal_gyro, dt_myo);
    % ik = ik.updateIK([xM; yM; z_fixed; roll_fixed; pitch_fixed; yaw_fixed]);
    ik = ik.updateIK([xM; yM; z_fixed; roll_myo; pitch_fixed; yaw_fixed]);
    disp(ik.qActin);
    if isActin
        udp.moveActinCyton(ik.qActin, gripper_fixed);
    end
    pause(dt);
end

end
