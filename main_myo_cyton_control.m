clc; clear; close all;

% user settings
useAction = true;
useUnity = false;
useMyo = false;
sendToRobot = true;

chIds = [1 5];
wristFlexThreshold = 0.10;
wristExtendThreshold = 0.05;
sampleWindow = 100;

dt = 0.01;
jointSpeed = 3.0; 
gripSpeed = 0.5;

% robot state
qHome = zeros(1, 7);
q = qHome;
gripper = 0.0; % close

% cannot figure out a way for this sim motors to match robot, so this home
% state is hard coded
qSimHome = [0 -1.55 0 0 0 -1.57 0];

gripperMin = 0.0;
gripperMax = 1.0;

jointList = 1:7;
jointIdx = 1;
activeJoint = jointList(jointIdx);

% mini vie path
currDir = cd;
cd('C:\GitHub\MiniVIE');
MiniVIE.configurePath();
cd(currDir);

% hrilabs path
addpath(genpath('C:\GitHub\hrilabs'));

% myo init
hMyo = [];
if useMyo
    hMyo = init_myo();
end

if sendToRobot
    [udpAction, udpUnity] = init_cyton_udp(useAction, useUnity);
    send_cyton_command(udpAction, udpUnity, q, gripper, useAction, useUnity);
end 

% robot model
robotStruct = link_constants_cyton();

% main figure
hFig = figure('Name','Myo to Cyton Control', ...
    'Position',[100 100 1300 800], ...
    'WindowKeyPressFcn', @keyDownHandler, ...
    'WindowKeyReleaseFcn', @keyUpHandler);

setappdata(hFig, 'jointIdx', jointIdx);
setappdata(hFig, 'jointMoveCmd', 0);
setappdata(hFig, 'gripperCmd', 0);
setappdata(hFig, 'stopRequested', false);
setappdata(hFig, 'resetRequested', false);

sgtitle('Myo to Cyton Teleoperation System');

% robot subplot
subplot(2,2,[1 3]);
handlesRobot = plot_cyton_robot();
view(-35,25);
grid on;
axis([-0.6 0.6 -0.6 0.6 -0.2 1.0]);
daspect([1 1 1]);
title('Cyton Robot');

% emg subplot
subplot(2,2,2);
emgData = zeros(sampleWindow, length(chIds));
plot(emgData);
title('Filtered EMG Channels');
grid on;

% control state subplot
subplot(2,2,4);
axis([0 1 0 1]);
set(gca, 'XTick', [], 'YTick', []);
title('Control State');

drawnow;

% initialize robot
[A, ~] = get_kinematics(q + qSimHome, robotStruct);
update_cyton_robot(handlesRobot, A);

disp('hold arrow keys for smooth motion');
disp('n/p switch joints, up/down move, o/c gripper, r reset, q quit');

while ishandle(hFig)
    drawnow;

    if getappdata(hFig,'stopRequested')
        break;
    end

    if getappdata(hFig,'resetRequested')
        q = qHome;
        gripper = 0;
        setappdata(hFig,'resetRequested',false);
    end

    jointIdx = getappdata(hFig,'jointIdx');
    activeJoint = jointList(jointIdx);

    moveCmd = getappdata(hFig,'jointMoveCmd');
    gripCmd = getappdata(hFig,'gripperCmd');

    % get myo data
    if useMyo
        emgData = hMyo.getFilteredData(sampleWindow, chIds);
        [motion, rmsData] = classify_myo_motion( ...
            emgData, wristFlexThreshold, wristExtendThreshold);

        if strcmpi(motion,'flex')
            moveCmd = 1;
        elseif strcmpi(motion,'extend')
            moveCmd = -1;
        else
            moveCmd = 0;
        end
    else
        emgData = zeros(sampleWindow, length(chIds));
        rmsData = [0 0];
    end

    % apply joint movement continuously while key is held
    if moveCmd ~= 0
        q(activeJoint) = q(activeJoint) + jointSpeed * dt * moveCmd;
    end

    % apply gripper continuously while key is held
    if gripCmd ~= 0
        gripper = gripper + gripSpeed * dt * gripCmd;
        gripper = min(max(gripper,gripperMin),gripperMax);
    end

    % update robot
    [A, ~] = get_kinematics(q + qSimHome, robotStruct);
    update_cyton_robot(handlesRobot,A);

    % send current pose to robot
    if sendToRobot
        send_cyton_command(udpAction, udpUnity, q, gripper, useAction, useUnity);
    end

    % redraw emg subplot
    subplot(2,2,2);
    cla;
    plot(emgData, 'LineWidth', 1.2);
    title('Filtered EMG Channels');
    xlabel('Sample');
    ylabel('EMG');
    legend(arrayfun(@(x) sprintf('Ch %d', x), chIds, 'UniformOutput', false));
    grid on;

    % redraw control state
    subplot(2,2,4);
    cla;
    axis([0 1 0 1]);
    set(gca,'XTick',[],'YTick',[]);
    title('Control State');
    text(0.05,0.85,sprintf('Active Joint: %d',activeJoint),'FontSize',14);
    text(0.05,0.68,sprintf('q (joint angles): %.2f',q(activeJoint)),'FontSize',14);
    text(0.05,0.51,sprintf('Gripper: %.2f (0=closed, 1=opened)',gripper),'FontSize',14);
    if useMyo
        text(0.05,0.34,sprintf('Motion: %s | RMS = [%.3f %.3f]', ...
            motion, rmsData(1), rmsData(2)),'FontSize',14);
    end
    text(0.05,0.17,'hold up/down to move | n/p switch','FontSize',11);

    pause(dt);
end

disp('stopped');

function keyDownHandler(src,event)
switch lower(event.Key)
    case 'n'
        idx = getappdata(src,'jointIdx');
        setappdata(src,'jointIdx',min(idx+1,7));
    case 'p'
        idx = getappdata(src,'jointIdx');
        setappdata(src,'jointIdx',max(idx-1,1));
    case 'uparrow'
        setappdata(src,'jointMoveCmd',1);
    case 'downarrow'
        setappdata(src,'jointMoveCmd',-1);
    case 'o'
        setappdata(src,'gripperCmd',1);
    case 'c'
        setappdata(src,'gripperCmd',-1);
    case 'r'
        setappdata(src,'resetRequested',true);
    case 'q'
        setappdata(src,'stopRequested',true);
end
end

function keyUpHandler(src,event)
switch lower(event.Key)
    case {'uparrow','downarrow'}
        setappdata(src,'jointMoveCmd',0);
    case {'o','c'}
        setappdata(src,'gripperCmd',0);
end
end