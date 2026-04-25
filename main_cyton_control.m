function main_cyton_control()
%% Main script: Hand IK + Myo gripper/rotation together

clear; clc;

%% MiniVIE path
currDir = cd;
cd('C:\GitHub\MiniVIE');
MiniVIE.configurePath();
cd(currDir);

%% Main robot initial state
q_initial_actin = [.33, -0.74, 0, -1.51, 0, 0.768, 0];

z_fixed = 0.5;

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

%% Start/stop form
StartStopForm([]);

tLast = tic;

disp('Running hand IK + Myo gripper/rotation together.');
disp('Close StartStopForm to stop.');

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
    emgData = hMyo.getData(hLda.NumSamplesPerWindow, 1:8);
    features2D = hLda.extractfeatures(emgData);
    [classDecision, ~] = hLda.classify(reshape(features2D', [], 1));
    className = classNames{classDecision};

    if contains(lower(className), 'open')
        grip = min(grip + gripStep, gripMax);
    else
        grip = max(grip - gripStep, gripMin);
    end

    %% -----------------------------
    % 3. Myo gyro controls rotation
    %% -----------------------------
    hMyo.getData();

    gyroDeg = hMyo.Gyroscope(:);
    gyroDeg(abs(gyroDeg) < gyroDeadband) = 0;

    yaw_cmd = yaw_cmd + yawGain * gyroDeg(3) * loopDt;

    yaw_cmd = max(min(yaw_cmd, pi), -pi);

    %% -----------------------------
    % 4. IK update
    %% -----------------------------
    ik = ik.updateIK([xM; yM; z_fixed; roll_cmd; pitch_cmd; yaw_cmd]);

    %% -----------------------------
    % 5. Send one combined command
    %% -----------------------------
    udp.moveActinCyton(ik.qActin, grip);

    fprintf('\rGrip=%.3f | Class=%s | xyz=[%.2f %.2f %.2f] | yaw=%.2f | gyroZ=%.1f', ...
        grip, className, xM, yM, z_fixed, yaw_cmd, gyroDeg(3));

    pause(dt);
end

disp(' ');
disp('TeleUltrasoundMain stopped.');

end