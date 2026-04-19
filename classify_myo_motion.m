function [motion, rmsData] = classify_myo_motion(emgData, flexThresh, extendThresh)
% Classify EMG into flex / extend / rest

rmsData = rms(emgData);

if rmsData(1) > flexThresh
    motion = 'flex';
elseif rmsData(2) > extendThresh
    motion = 'extend';
else
    motion = 'rest';
end
end
