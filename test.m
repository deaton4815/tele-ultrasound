hMyo = Inputs.MyoUdp.getInstance();
hMyo.initialize();
hMyo.getData();

StartStopForm([]);
tPrev = tic;

while StartStopForm
    drawnow;

    neutral_gyro = [0, 0, 0];
    for i = 1:1000
        neutral_gyro = neutral_gyro + hMyo.Gyroscope(:)';
    end
    neutral_gyro = neutral_gyro / 1000;
    g = neutral_gyro(1);
    g_rad = 0;
    
    disp("neutral")
    disp(neutral_gyro(1))
    hMyo.getData();
    gyroDeg = hMyo.Gyroscope(:)';
    kB = 3;
    
    % Subtract neutral position for zeroing
    g = g + gyroDeg(1) - neutral_gyro(1);
    disp(g)
    
    % deadband to reduce drift
    if abs(g) < 1.0
        g = 0;
    end
    
    g_rad = deg2rad(g);
    
    
    % Calculate new position
    raw_q7 = g_rad * loopDt; 
    disp(raw_q7)
    
    q_7 = atan2(sin(raw_q7), cos(raw_q7));
    disp(q_7)
end