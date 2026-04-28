function main_cyton_control()
%% Main script: Hand IK + Myo gripper/rotation together

clear; clc;

%%% Force sensor init %%%%
aOperator = arduino('COM5', 'Uno');
aRobot = arduino('COM7', 'Uno');

%% MiniVIE path
currDir = cd;
cd('C:\GitHub\MiniVIE');
MiniVIE.configurePath();
cd(currDir);

%% Main robot initial state
q_initial_actin = [.33, -0.74, 0, -1.51, 0, 0.768, 0];

z = 0.5;

roll_cmd  = 0;
pitch_cmd = 0;
yaw_cmd   = 0;

dt = 0.1;

%% Gripper settings
gripMin = 0.00;
gripMax = 0.03;
gripStep = 0.003;
grip = 0.01;

%% Myo classifier setup
myDataFilename = 'WIMBISH.trainingData';

hData = PatternRecognition.TrainingData();
hData.loadTrainingData(myDataFilename);

hLda = SignalAnalysis.Lda;
hLda.initialize(hData);
hLda.train();
hLda.computeError();

classNames = hLda.getClassNames;

disp('Classifier trained.');
disp('Available classes:');
disp(classNames);

%% Initialize Myo
hMyo = Inputs.MyoUdp.getInstance();
hMyo.initialize();
disp('Myo initialized.');

%% Initialize hand landmark / IK / calibration
lm = HandLandMarker();
ik = CytonIK(q_initial_actin);

cal = CameraCalibration(lm);
cal = cal.runCalibration();

%% Initialize Actin / Cyton UDP
udp = CytonUDP();

udp.moveActinCyton(ik.qActin, grip);
pause(1);

%% Start message
msg = sprintf('Place your hand in the center of the box to start the procedure');
h = msgbox(msg, 'Ultrasound', 'help', 'modal');
uiwait(h);

%% Control gains
gyroDeadband = 8;
yawGain = 0.010;

gLimLow = -90;
gLimHigh = 90;

disp('Stop')

%% Start/stop form
StartStopForm([]);

tLast = tic;

disp('Running hand IK + Myo gripper/rotation together.');
disp('Close StartStopForm to stop.');

%%% Neutral gripper %%%
disp('Get Neutral Position')
neutral_gyro = [0, 0, 0];
for i = 1:1000
    neutral_gyro = neutral_gyro + hMyo.Gyroscope(:)';
end
neutral_gyro = neutral_gyro / 1000;
g = neutral_gyro(1);
g_rad = 0;

%%%% Voltage and Z %%%%%
zPID = ForcePid();

while StartStopForm
    drawnow;

    loopDt = toc(tLast);
    tLast = tic;

    if loopDt <= 0 || loopDt > 0.2
        loopDt = dt;
    end

    %% -----------------------------
    % 1. Get hand position
    %% -----------------------------
    xy = lm.getXY();
    [xM, yM] = cal.toMeters(xy);

    %% -----------------------------
    % 2. Myo EMG controls gripper
    %% -----------------------------
    % emgData = hMyo.getData(hLda.NumSamplesPerWindow, 1:8);
    % features2D = hLda.extractfeatures(emgData);
    % [classDecision, ~] = hLda.classify(reshape(features2D', [], 1));
    % className = classNames{classDecision};
    % 
    % if contains(lower(className), 'open')
    %     grip = min(grip + gripStep, gripMax);
    % else
    %     grip = max(grip - gripStep, gripMin);
    % end

    %% -----------------------------
    % 3. Myo gyro controls rotation
    %% -----------------------------
    % hMyo.getData();
    % 
    % gyroDeg = hMyo.Gyroscope(:);
    % gyroDeg(abs(gyroDeg) < gyroDeadband) = 0;
    % 
    % yaw_cmd = yaw_cmd + yawGain * gyroDeg(3) * loopDt;
    % 
    % yaw_cmd = max(min(yaw_cmd, pi), -pi);


    %%%%%%%% Get force diff %%%%%%%%
    vOperator = readVoltage(aOperator, 'A0');
    vRobot = readVoltage(aRobot, 'A0');
    vErr = vOperator - vRobot;

    zUpdate = zPID.update(vErr, loopDt);
    z = z - zUpdate;

    %% -----------------------------
    % 4. IK update
    %% -----------------------------
    ik = ik.updateIK([xM; yM; z; roll_cmd; pitch_cmd; yaw_cmd]);

    hMyo.getData();
    gyroDeg = hMyo.Gyroscope(:)';
    kB = 3;
    
    % Subtract neutral position for zeroing
    g = g + gyroDeg(1) - neutral_gyro(1);
    g = max(min(g, gLimHigh), gLimLow);
    
    % deadband to reduce drift
    if abs(g) < 1.0
        g = 0;
    end

    g_rad = deg2rad(g);

    % Calculate new position
    raw_q7 = ik.qActin(7) + g_rad * loopDt; 
    
    q_7 = atan2(sin(raw_q7), cos(raw_q7));

    ik.qActin(7) = q_7*kB;

    %% -----------------------------
    % 5. Send one combined command
    %% -----------------------------
    udp.moveActinCyton(ik.qActin, grip);

    % fprintf('\rGrip=%.3f | Class=%s | xyz=[%.2f %.2f %.2f] | yaw=%.2f | gyroZ=%.1f', ...
    %     grip, className, xM, yM, z_fixed, yaw_cmd, gyroDeg(3));

    pause(0.1);
end

disp(' ');
disp('TeleUltrasoundMain stopped.');

end