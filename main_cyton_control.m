function main_cyton_control(myDataFilename)
%%%%% Main script: Hand IK + Myo gripper/rotation together %%%%%

%%%% Inputs %%%%%
if nargin < 1
myDataFilename = 'Training Data/NEW_USER_20260501_211053.trainingData';
end

%% Force sensor init
aOperator = arduino('COM7', 'Uno');
aRobot = arduino('COM5', 'Uno');

%% Main robot initial state
q_initial_actin = [0.89, -0.57, 0, -1.71, 0, 0.768, 0];

z = 0.5;
roll_cmd  = 0;
pitch_cmd = 0;
yaw_cmd   = 0;

tPause = 0.1;

%% Gripper settings
gripMin = 0.005;
gripMax = 0.01;
gripStep = 0.003;
grip = 0.005;

%% Myo classifier setup

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
disp('Calibration Complete!');

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

gripperControlTime = 10;   % seconds
tStart = tic;
tLast  = tic;

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

v0_Operator = readVoltage(aOperator, 'A0');
v0_Robot = readVoltage(aRobot, 'A0');

vOperatorPrev = 0;
while StartStopForm
    drawnow;

    elapsedTime = toc(tStart);
    loopDt = toc(tLast);
    tLast = tic;

    if loopDt <= 0 || loopDt > 0.2
        loopDt = tPause;
    end

    %% -----------------------------
    % 1. Get hand position
    %% -----------------------------
    xy = lm.getXY();
    [xM, yM] = cal.toMeters(xy);

    %% -----------------------------
    % 2. Myo EMG controls gripper
    %% -----------------------------
    if elapsedTime < gripperControlTime
        emgData = hMyo.getData(hLda.NumSamplesPerWindow, 1:8);
        features2D = hLda.extractfeatures(emgData);
        [classDecision, ~] = hLda.classify(reshape(features2D', [], 1));
        className = classNames{classDecision};
        
        if contains(lower(className), 'open')
            grip = min(grip + gripStep, gripMax);
        else
            grip = max(grip - gripStep, gripMin);
        end
        udp.moveActinCyton(q_initial_actin, grip);

    else
        grip = gripMin;
        %%%%%%%% Get force diff %%%%%%%%
        vOperator = readVoltage(aOperator, 'A0') - v0_Operator;
        vRobot = readVoltage(aRobot, 'A0') - v0_Robot;
        vErr = vOperator - vRobot;

        disp(readVoltage(aOperator, 'A0'));
    
        zUpdate = zPID.update(vErr, loopDt);
        z = z - zUpdate;

        if vOperator - vOperatorPrev < 0
            z = z + 0.05;
        end
    
        %% -----------------------------
        % 4. IK update
        %% -----------------------------
        ik = ik.updateIK([xM; yM; z; roll_cmd; pitch_cmd; yaw_cmd]);
    
        % if vOperator < 0.1
        %     z = min(0.1, z + 0.02);
        % end
        % 
        hMyo.getData();
        dt = toc(tLast);
        tLast = tic;
        q_7 = orientation(hMyo, ik.qActin, neutral_gyro, dt);
        ik.qActin(7) = q_7;
    
        %% -----------------------------
        % 5. Send one combined command
        %% -----------------------------
        udp.moveActinCyton(ik.qActin, grip);
    end

    pause(0.1);
end

disp(' ');
disp('TeleUltrasoundMain stopped.');

end