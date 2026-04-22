function runPython()
    % 1. Setup Paths
    disp('Initializing Python environment...');
    targetDir = 'C:\GitHub\hrilabs\gp2\tele-ultrasound\media_pipe_hands';
    venvLib = 'C:\GitHub\hrilabs\gp2\tele-ultrasound\.venv\Lib\site-packages';
    
    if count(py.sys.path, venvLib) == 0
        insert(py.sys.path, int32(0), venvLib);
    end
    if count(py.sys.path, targetDir) == 0
        insert(py.sys.path, int32(0), targetDir);
    end
    cleanupObj = onCleanup(@() cleanupFunction());

    try
        fprintf('Starting Hand Tracking \n');
        cd(targetDir);

        pyrunfile("main.py");
        
    catch ME
        fprintf('Process interrupted: %s\n', ME.message);
    end
end

function cleanupFunction()
    disp('Stop button detected. Cleaning up resources...');
    
    evalin('base', 'clear classes'); 
    fprintf('Python engine reset. Camera should now be free.\n');
end