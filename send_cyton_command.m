function send_cyton_command(udpActin, udpUnity, q, gripper, useActin, useUnity)
% Send Cyton command to Actin and/or Unity

desiredAngles = [q, gripper];

if useActin && ~isempty(udpActin)
    % Actin expects radians as doubles
    udpActin.putData(typecast(desiredAngles, 'uint8'));
end

if useUnity && ~isempty(udpUnity)
    % Unity expects degrees as float32 (single)
    udpUnity.putData(typecast(single([rad2deg(q), gripper]), 'uint8'));
end
end
