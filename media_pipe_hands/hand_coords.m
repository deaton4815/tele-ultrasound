u = udpport("LocalPort", 5005);

disp('Listening for hand landmarks...');

while true
    if u.NumBytesAvailable > 0
        try
            raw = readline(u);
            if strlength(raw) > 0
                landmarks = jsondecode(raw);  % 21x3 matrix [x, y, z]
                disp(landmarks);
            end
        catch e
            disp('Read error, skipping frame');
        end
    end
    pause(0.001);
end