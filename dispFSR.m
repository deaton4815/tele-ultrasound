function dispFSR()

a = arduino('COM11', 'Uno');

while true
    voltage = readVoltage(a, 'A0');
    fprintf('Voltage: %.3f V\n', voltage);
    pause(0.05);
end
end