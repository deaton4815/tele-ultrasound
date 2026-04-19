clear u
u = udpport("LocalPort", 5005);

disp('Collecting data...');

num_samples = 500;  % adjust as needed
data = zeros(num_samples, 2);  % [x, y] per sample

for i = 1:num_samples
    while u.NumBytesAvailable == 0
        pause(0.001);  % wait for data
    end
    try
        raw = readline(u);
        coords = jsondecode(raw);
        data(i, :) = coords(6, 1:2);
    catch
        if i > 1
            warning('colelct storage failed')
            data(i, :) = data(i-1, :);  % repeat last sample if bad read
        end
    end
end

save('hand_data.mat', 'data');
disp('Done! Saved to hand_data.mat');