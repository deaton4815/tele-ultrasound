clear; clc;

currDir = cd;
cd('C:\GitHub\MiniVIE');
MiniVIE.configurePath();
cd(currDir);

udpActin = PnetClass(8889, 8888, '127.0.0.1');
udpActin.initialize();

q = [0 0 0 0 0 0 0];
grip = 0.00;

udpActin.putData(typecast([q, grip], 'uint8'));
pause(1);

for j = 1:7
    fprintf('Testing joint %d\n', j);

    q = [0 0 0 0 0 0 0];
    udpActin.putData(typecast([q, grip], 'uint8'));
    pause(1);

    q(j) = 0.6;
    udpActin.putData(typecast([q, grip], 'uint8'));
    pause(1.5);

    q(j) = -0.6;
    udpActin.putData(typecast([q, grip], 'uint8'));
    pause(1.5);

    q(j) = 0;
    udpActin.putData(typecast([q, grip], 'uint8'));
    pause(1);
end

udpActin.close();