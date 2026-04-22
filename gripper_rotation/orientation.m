%function q_7 = orientation(hMyo, q, neutral_gyro, dt)
function q_7 = orientation(hMyo, roll_in, neutral_gyro, dt)
    gyroDeg = hMyo.Gyroscope(:)';
    kB = 1.5;
    
    % Subtract neutral position for zeroing
    g = gyroDeg(1) - neutral_gyro(1);
    
    % deadband to reduce drift
    if abs(g) < 1.0
        g = 0;
    end

    g_rad = deg2rad(g);

    % Calculate new position
    %raw_q7 = q(7) + g_rad * dt; 
    raw_roll = roll_in + g_rad * dt; 
    
    %q_7 = atan2(sin(raw_q7), cos(raw_q7));
    q_7 = atan2(sin(raw_roll), cos(raw_roll));

end