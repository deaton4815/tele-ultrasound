function plot_control_state(axState, motion, rmsData, activeJoint, q)

cla(axState);
axis(axState, [0 1 0 1]);
set(axState, 'XTick', [], 'YTick', []);
title(axState, 'Control State');

text(0.1, 0.6, sprintf('Motion: %s | RMS = [%.3f %.3f]', ...
    motion, rmsData(1), rmsData(2)), ...
    'FontSize', 14, 'Parent', axState);

text(0.1, 0.4, sprintf('Active Joint: %d', activeJoint), ...
    'FontSize', 14, 'Parent', axState);

text(0.1, 0.2, sprintf('q(%d): %.3f rad', activeJoint, q(activeJoint)), ...
    'FontSize', 14, 'Parent', axState);

drawnow limitrate;
end