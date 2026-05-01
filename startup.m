addpath(genpath('Training Data'));
addpath(genpath('media_pipe_hands'));
addpath(genpath('ik'));
addpath(genpath('gripper_rotation'));
addpath(genpath('fsr'));
addpath(genpath('Arduino'));

%% MiniVIE path
currDir = cd;
cd('C:\GitHub\MiniVIE');
MiniVIE.configurePath();
cd(currDir);