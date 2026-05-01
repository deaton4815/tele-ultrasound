function isSafe = safe_cyton_pose(T, q)
    % Basic safety checks for Cyton pose

    pEE = T{end}(1:3,4);

    % Workspace box
    xMin = 0.05;  xMax = 0.45;
    yMin = -0.30; yMax = 0.30;
    zMin = 0.05;  zMax = 0.50;
    
    insideWorkspace = ...
        pEE(1) >= xMin && pEE(1) <= xMax && ...
        pEE(2) >= yMin && pEE(2) <= yMax && ...
        pEE(3) >= zMin && pEE(3) <= zMax;
    
    % Base exclusion zone
    baseRadius = 0.10;
    baseHeight = 0.12;
    r = sqrt(pEE(1)^2 + pEE(2)^2);
    
    outsideBaseZone = ~(r < baseRadius && pEE(3) < baseHeight);
    
    % Final decision
    isSafe = insideWorkspace && outsideBaseZone;
end