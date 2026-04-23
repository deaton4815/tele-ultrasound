%% controlling arm gripper and rotation from myoband
% first 10 seconds control gripper, then it just rotates based on Gyro data
% the motion of rotate confuses the gripper, so it's best to keep these
% separate

clear; clc;

% MiniVIE path
currDir = cd;
cd('C:\GitHub\MiniVIE');
MiniVIE.configurePath();
cd(currDir);

% Load training data
myDataFilename = 'NEW_USER_20260420_181346.trainingData';

hData = PatternRecognition.TrainingData();
hData.loadTrainingData(myDataFilename);

hLda = SignalAnalysis.Lda;
hLda.initialize(hData);
hLda.train();
hLda.computeError();

disp('Classifier trained.');

% Initialize Myo
hMyo = Inputs.MyoUdp.getInstance();
hMyo.initialize();

disp('Myo initialized.');

% Actin Viewer connection
udpActin = PnetClass(8889, 8888, '127.0.0.1');
udpActin.initialize();

disp('Actin UDP initialized.');

% Robot state
q_home = [0, 0.6, 0, 1.2, 0, 0.4, 0]; % generic 'bent' position
q = q_home;                

% gripper
gripMin = 0.00;
gripMax = 0.03;
gripStep = 0.003;
grip = gripMin;

% send initial pose
udpActin.putData(typecast([q, grip], 'uint8'));
pause(1);

% Class names
classNames = hLda.getClassNames;
disp('Available classes:');
disp(classNames);

% Timing
gripperControlTime = 10;   % seconds
tStart = tic;
tLast  = tic;

imuOnlyStarted = false;

disp('First 10 seconds: ONLY gripper control.');
disp('After 10 seconds: gripper closes, EMG ignored, gyro controls joints.');

% Main loop
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
        % Phase 1 - only gripper control
        % Keep all joints fixed
        q = q_home;

        emgData = hMyo.getData(hLda.NumSamplesPerWindow,1:8);
        features2D = hLda.extractfeatures(emgData);
        [classDecision, voteDecision] = hLda.classify(reshape(features2D',[],1));
        className = classNames{classDecision};

        if contains(lower(className), 'open')
            grip = min(grip + gripStep, gripMax);
        else
            grip = max(grip - gripStep, gripMin);
        end

        modeString = 'GRIPPER';

        fprintf('t=%5.1f Mode=%8s Class=%16s Grip=%.3f q4=%.2f q7=%.2f\n', ...
            elapsedTime, modeString, className, grip, q(4), q(7));

    else
        hMyo.getData();
        gyroDeg = hMyo.Gyroscope(:);
        gyroDeg(abs(gyroDeg) < 8) = 0;
        
        % testing out joints to rotate, settled on just rotating gripper
        %JOINT_A = 5;   % forearm / elbow-like joint
        JOINT_B = 7;   % wrist / probe rotation joint
        
        % Stronger gains so motion is obvious
        kA = 0.030;
        kB = 0.070;
        
        %q(JOINT_A) = q(JOINT_A) + kA * gyroDeg(2) * dt;
        q(JOINT_B) = q(JOINT_B) + kB * gyroDeg(3) * dt;
        
        % Generic clamp
        q = min(max(q, -pi*ones(size(q))), pi*ones(size(q)));
        
        grip = gripMin;
        
        fprintf('t=%5.1f Grip=%.3f qA=%.2f qB=%.2f GYRO=[%.1f %.1f %.1f]\n', ...
            elapsedTime, grip, q(JOINT_B), ...
            gyroDeg(1), gyroDeg(2), gyroDeg(3));
    end

    % send command every loop
    udpActin.putData(typecast([q, grip], 'uint8'));
end