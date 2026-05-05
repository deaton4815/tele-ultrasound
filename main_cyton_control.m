function main_cyton_control(myDataFilename)
%%%%% Main script: Hand IK + Myo gripper/rotation together %%%%%

%%%% Inputs %%%%%
if nargin < 1
    myDataFilename = 'Training Data/grippertraining.trainingData';
end

%% Force sensor init
aOperator = arduino('COM7', 'Uno');
aRobot = arduino('COM5', 'Uno');

%% Main robot initial state
q_initial_actin = [0.89, -0.57, 0, -1.71, 0, 0.768, 0];

z = 0.5;

tPause = 0.1;

%% Gripper settings
gripMin = 0.007;
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

disp('Stop')

%% Start/stop form
StartStopForm([]);

gripperControlTime = 20;   % seconds
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

%% Key input figure for manual z control
hFig = figure('Name', 'Z Control  (↑↓ keys)', ...
    'KeyPressFcn', @(src,evt) zKeyHandler(src, evt), ...
    'MenuBar', 'none', 'ToolBar', 'none');
setappdata(hFig, 'zOffset', 0);

text(0.5, 0.5, {'↑  z up', '↓  z down'}, ...
    'Units','normalized', 'HorizontalAlignment','center', 'FontSize', 14);
axis off;

%%%% Voltage and Z %%%%%
zPID = ForcePid();

v0_Operator = readVoltage(aOperator, 'A0');
v0_Robot = readVoltage(aRobot, 'A0');

vErrAll = []; % store all errors
count = 0;
while StartStopForm
    count = count + 1;
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
        disp(className);

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
        vOperator = min(vOperator, 1);
        vRobot = min(vRobot, 1);
        vErr = vOperator - vRobot;

        vErrAll(count) = vErr; % store all force errors
        
        
        disp("Operator = ");
        disp(readVoltage(aOperator, 'A0'));

        disp("Robot = ");
        disp(readVoltage(aRobot, 'A0'));

        zUpdate = zPID.update(vErr, loopDt);
        z = z - zUpdate;

        z = z + getappdata(hFig, 'zOffset');  % add manual bias on top
        setappdata(hFig, 'zOffset', 0);       % clear after applying

        %% -----------------------------
        % 4. IK update
        %% -----------------------------
        ik = ik.updateIK([xM; yM; z; roll_cmd; pitch_cmd; yaw_cmd]);

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

save('vErrAll.mat', 'vErrAll'); % save all force errors for processing

disp(' ');
disp('TeleUltrasoundMain stopped.');

end

function zKeyHandler(hFig, event)
zStep = 0.01;
offset = getappdata(hFig, 'zOffset');
switch event.Key
    case 'uparrow'
        offset = offset + zStep;
        fprintf('zOffset = %.4f\n', offset);
    case 'downarrow'
        offset = offset - zStep;
        fprintf('zOffset = %.4f\n', offset);
    case 'r'
        offset = 0;           % reset bias back to zero
        disp('zOffset reset');
end
setappdata(hFig, 'zOffset', offset);
end