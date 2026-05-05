function MyoGripRotationWrapper()
%% controlling arm gripper and rotation from myoband
% First 10 seconds control gripper.
% After 10 seconds, EMG is ignored and gyro controls rotation.

clear; clc;

%% MiniVIE path
currDir = cd;
cd('C:\GitHub\MiniVIE');
MiniVIE.configurePath();
cd(currDir);

%% Load training data
myDataFilename = 'WIMBISH.trainingData';

hData = PatternRecognition.TrainingData();
hData.loadTrainingData(myDataFilename);

hLda = SignalAnalysis.Lda;
hLda.initialize(hData);
hLda.train();
hLda.computeError();

disp('Classifier trained.');

%% Initialize Myo
hMyo = Inputs.MyoUdp.getInstance();
hMyo.initialize();

disp('Myo initialized.');

%% Actin Viewer connection
udpActin = PnetClass(8889, 8888, '127.0.0.1');
udpActin.initialize();

disp('Actin UDP initialized.');

%% Robot state
q_home = [0, 0.6, 0, 1.2, 0, 0.4, 0];
q = q_home;

%% Gripper
gripMin = 0.00;
gripMax = 0.03;
gripStep = 0.003;
grip = gripMin;

%% Send initial pose
udpActin.putData(typecast([q, grip], 'uint8'));
pause(1);

%% Class names
classNames = hLda.getClassNames;
disp('Available classes:');
disp(classNames);

%% Timing
gripperControlTime = 10;
tStart = tic;
tLast  = tic;

disp('First 10 seconds: ONLY gripper control.');
disp('After 10 seconds: gripper closes, EMG ignored, gyro controls joints.');

%% Main loop
StartStopForm([]);

while StartStopForm
    drawnow;

    elapsedTime = toc(tStart);
    dt = toc(tLast);
    tLast = tic;

    if dt <= 0 || dt > 0.2
        dt = 0.02;
    end

    if elapsedTime < gripperControlTime
        %% Phase 1 - only gripper control
        q = q_home;

        emgData = hMyo.getData(hLda.NumSamplesPerWindow, 1:8);
        features2D = hLda.extractfeatures(emgData);
        [classDecision, voteDecision] = hLda.classify(reshape(features2D', [], 1)); %#ok<ASGLU>
        className = classNames{classDecision};

        if contains(lower(className), 'open')
            grip = min(grip + gripStep, gripMax);
        else
            grip = max(grip - gripStep, gripMin);
        end

        modeString = 'GRIPPER';

        fprintf('t=%5.1f \nMode=%8s \nClass=%16s Grip=%.3f q4=%.2f q7=%.2f\n', ...
            elapsedTime, modeString, className, grip, q(4), q(7));

    else
        %% Phase 2 - gyro rotation only
        hMyo.getData();

        gyroDeg = hMyo.Gyroscope(:);
        gyroDeg(abs(gyroDeg) < 8) = 0;

        JOINT_B = 7;
        kB = 0.070;

        q(JOINT_B) = q(JOINT_B) + kB * gyroDeg(3) * dt;

        q = min(max(q, -pi * ones(size(q))), pi * ones(size(q)));

        grip = gripMin;

        fprintf('\r[%5.1f] Grip=%6.3f | qA=%6.2f qB=%6.2f | G=[%6.1f %6.1f %6.1f]', ...
            elapsedTime, grip, q(4), q(7), ...
            gyroDeg(1), gyroDeg(2), gyroDeg(3));
    end

    %% Send command every loop
    udpActin.putData(typecast([q, grip], 'uint8'));
end

disp(' ');
disp('MyoGripRotationWrapper stopped.');

try
    udpActin.close();
catch
    disp('Actin UDP already closed.');
end

end