% Actin Viewer connection
clear; clc;

udpActin = PnetClass(8889, 8888, '127.0.0.1');
udpActin.initialize();

disp('Actin UDP initialized.');

q = [0, -0.7, 0, -0.7, 0, -0.7, 0];

hMyo = Inputs.MyoUdp.getInstance();
hMyo.initialize();
hMyo.getData();

udpActin.putData(typecast([q, 0.01], 'uint8'));

disp('Get Neutral Position')
neutral_gyro = [0, 0, 0];
for i = 1:1000
    neutral_gyro = neutral_gyro + hMyo.Gyroscope(:)';
end
neutral_gyro = neutral_gyro / 1000;
disp('Stop')

StartStopForm([]);
tPrev = tic;

while StartStopForm
    drawnow;

    %% gripper 
    % grip = gripper_control(hMyo, 'USER_20260421_183507.trainingData');
    % udpActin.putData(typecast([q, grip], 'uint8'));

    %% rotation
    hMyo.getData();
    dt = toc(tPrev);
    tPrev = tic;
    q_7 = orientation(hMyo, q, neutral_gyro, dt);
    q(7) = q_7;
    udpActin.putData(typecast([q, 0.01], 'uint8'));
end