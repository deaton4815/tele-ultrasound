function plot_emg_channels(axEMG, emgData, chIds)
% Redraw EMG subplot each update

    cla(axEMG);
    plot(axEMG, emgData, 'LineWidth', 1.2);
    xlabel(axEMG, 'Sample');
    ylabel(axEMG, 'EMG');
    title(axEMG, 'Filtered EMG Channels');
    legend(axEMG, arrayfun(@(x) sprintf('Ch %d', x), chIds, 'UniformOutput', false));
    grid(axEMG, 'on');
    
    drawnow limitrate;
end