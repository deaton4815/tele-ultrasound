function q_7 = orientation(hMyo, q, neutral_gyro, dt)
    gyroDeg = hMyo.Gyroscope(:)';
    kB = 1.5;
    
    % Subtract neutral position for zeroing
    g = gyroDeg(1) - neutral_gyro(1);
    % disp(g)
    
    % deadband to reduce drift
    if abs(g) < 1.0
        g = 0;
    end

    g_rad = deg2rad(g);

    % Calculate new position
    raw_q7 = q(7) + g_rad * dt; 
    
    q_7 = atan2(sin(raw_q7), cos(raw_q7));
    % disp(q_7)

end