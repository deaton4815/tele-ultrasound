function dispFSR()

a1 = arduino('COM5', 'Uno');
% a2 = arduino('COM7', 'Uno');

while true
    voltage1 = readVoltage(a1, 'A0');
    % voltage2 = readVoltage(a2, 'A0');
    fprintf('Voltage1: %.3f V\n', voltage1);
    % fprintf('Voltage2: %.3f V\n', voltage2);
    pause(0.05);
end
end